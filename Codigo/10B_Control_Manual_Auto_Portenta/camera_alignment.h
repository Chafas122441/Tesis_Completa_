/**
 * @file camera_alignment.h
 * @brief Calibracion por AprilTags y seleccion de candidatos visuales.
 * Transforma la imagen a coordenadas de banda y valida observaciones sucesivas.
 */
#ifndef CAMERA_ALIGNMENT_H
#define CAMERA_ALIGNMENT_H

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "camera_packet.h"
#include "camera_piece_classes.h"

/* Coordenadas y reloj del MISMO ultimo cuadro coherente. observedMs es la
   recepcion local del primer paquete del cuadro, NO el instante de exposicion
   de HUSKYLENS; sincronizar encoder exige validar ademas la latencia real. */
struct CameraPieceTarget {
  uint8_t id = 0u;
  float robotY = 0.0f;
  float worldLongitudinalMm = 0.0f;
  uint32_t observedMs = 0u;
};

/*
  Estado de vision de la etapa 10. No mueve el robot: solo valida la
  calibracion y publica un destino Y una vez que tres cuadros coherentes
  describen la misma pieza. Todas las decisiones de armado permanecen en 10B.
*/
class CameraAlignment {
 public:
  enum class State : uint8_t {
    OFF = 0u, WAIT_TAG_MODE, COLLECTING_TAGS, SOLVING,
    WAIT_PIECE_MODE, READY, FAULT
  };

  enum class Observation : uint8_t {
    NONE = 0u, EMPTY, REJECTED, MULTIPLE, TRACKING,
    UNSTABLE, ACCEPTED, LATCHED
  };

  void beginCalibration() {
    clearCalibration();
    // Una calibracion explicita solo enlaza la sesion al recibir OFF confirmado.
    hasRemoteSession_ = false;
    remoteSession_ = 0u;
    state_ = State::WAIT_TAG_MODE;
    // OFF confirmado distingue recalibracion deliberada de retransmision UDP.
    resettingMode_ = true;
    pendingCommand_ = CAMERA_COMMAND_OFF;
    modeRequestStartedMs_ = millis();
    fault_[0] = '\0';
  }

  void turnOff() {
    resettingMode_ = false;
    state_ = State::OFF;
    pendingCommand_ = CAMERA_COMMAND_OFF;
    clearPendingTargetForArm();
    clearObservation();
    modeRequestStartedMs_ = 0u;
  }

  /* Un ARM explicito nunca consume una pieza detectada antes del ARM. El
     siguiente destino necesita tres cuadros completos nuevos. */
  void clearPendingTargetForArm() {
    clearTarget();
    clearPieceWindow();
    pieceLatched_ = false;
    emptyFrames_ = 0u;
    frameExpected_ = 0u;
    frameReceived_ = 0u;
    frameInvalid_ = true;
    hasCompletePieceFrame_ = false;
    memset(frameSlot_, 0, sizeof(frameSlot_));
    clearObservation();
  }

  // A completed cycle discards buffered detections but does NOT unlatch a
  // visible old piece. Five genuinely empty frames still release that latch.
  // The next target needs three new complete frames after returning to wait.
  void clearPendingTargetForNextCycle() {
    const bool wasLatched = pieceLatched_;
    const uint8_t empty = emptyFrames_;
    clearPendingTargetForArm();
    pieceLatched_ = wasLatched;
    emptyFrames_ = empty;
  }

  bool consumeCommand(uint16_t &command) {
    if (pendingCommand_ == kNoCommand) return false;
    command = pendingCommand_;
    pendingCommand_ = kNoCommand;
    return true;
  }

  void onPacket(const CameraPacket &packet) {
    lastPacketMs_ = millis();
    hasPacket_ = true;
    if (resettingMode_) {
      // Primero OFF: READY/FAULT/cuadros atrasados no pueden enlazar otra sesion.
      const bool confirmedOff = packet.type == CAMERA_TYPE_ACK &&
          packet.value == CAMERA_COMMAND_OFF && packet.algorithm == -1 &&
          packet.session != 0u &&
          (packet.flags & (CAMERA_FLAG_MODE_READY | CAMERA_FLAG_SETTLING)) == 0u;
      if (confirmedOff) {
        remoteSession_ = packet.session;
        hasRemoteSession_ = true;
        lastAlgorithm_ = packet.algorithm;
        lastFlags_ = packet.flags;
        resettingMode_ = false;
        pendingCommand_ = CAMERA_COMMAND_TAG_MODE;
        modeRequestStartedMs_ = millis();
      }
      return;
    }
    if (hasRemoteSession_ && remoteSession_ != packet.session &&
        state_ != State::OFF) {
      remoteSession_ = packet.session;
      setFault("Sesion de camara cambio; recalibrar");
      return;
    }
    remoteSession_ = packet.session;
    hasRemoteSession_ = true;
    lastAlgorithm_ = packet.algorithm;
    lastFlags_ = packet.flags;
    if (packet.type == CAMERA_TYPE_FAULT) {
      setFault("FAULT remoto de HUSKYLENS");
      return;
    }
    const bool ready = (packet.flags & CAMERA_FLAG_MODE_READY) != 0u;
    if (state_ == State::WAIT_TAG_MODE && ready &&
        packet.algorithm == CAMERA_COMMAND_TAG_MODE) {
      state_ = State::COLLECTING_TAGS;
      modeRequestStartedMs_ = 0u;
    } else if (state_ == State::WAIT_PIECE_MODE && ready &&
               packet.algorithm == CAMERA_COMMAND_PIECE_MODE) {
      state_ = State::READY;
      modeRequestStartedMs_ = 0u;
    }
    if (packet.type == CAMERA_TYPE_ACK || packet.type == CAMERA_TYPE_STATUS) {
      return;
    }
    if (packet.type != CAMERA_TYPE_FRAME) return;
    if (packet.algorithm == CAMERA_COMMAND_TAG_MODE) recordRawTagPacket(packet);
    if (state_ == State::COLLECTING_TAGS) collectTag(packet);
    else if (state_ == State::READY) collectPiece(packet);
  }

  void service() {
    expireTarget();
    if ((state_ == State::WAIT_TAG_MODE ||
         state_ == State::WAIT_PIECE_MODE) &&
        modeRequestStartedMs_ != 0u &&
        millis() - modeRequestStartedMs_ > kModeChangeTimeoutMs) {
      setFault("Timeout 60 s al cambiar modo");
    }
    if (pieceCount_ > 0u && millis() - pieceFirstMs_ > 600u) {
      clearPieceWindow();
      setObservation(Observation::NONE, 0u, 0.0f, 0u, 0u, 0u);
    }
  }

  bool consumeTarget(uint8_t &id, float &robotY) {
    expireTarget();
    if (!targetReady_) return false;
    targetReady_ = false;
    id = targetId_;
    robotY = targetRobotY_;
    return true;
  }

  bool consumePieceTarget(CameraPieceTarget &target) {
    expireTarget();
    if (!targetReady_ || !ready()) return false;
    target = pieceTarget_;
    targetReady_ = false;
    return true;
  }

  State state() const { return state_; }
  bool awaitingOffAck() const { return resettingMode_; }
  void invalidateRemoteSession() {
    clearCalibration();
    setFault("Sesion de camara cambio; recalibrar");
  }
  bool calibrated() const { return calibrated_; }
  bool ready() const { return state_ == State::READY && calibrated_; }
  uint8_t calibrationSamples(uint8_t id) const {
    return id < 4u ? tagCount_[id] : 0u;
  }
  float reprojectionMm() const { return maxReprojectionMm_; }
  uint8_t emptyFrames() const { return emptyFrames_; }
  bool pieceLatched() const { return pieceLatched_; }
  bool targetPending() const {
    return targetReady_ && millis() - pieceTarget_.observedMs <= kTargetTtlMs;
  }
  uint32_t targetAgeMs() const {
    return targetReady_ ? millis() - pieceTarget_.observedMs : 0xFFFFFFFFu;
  }
  uint32_t lastCompleteFrameAgeMs() const {
    return hasCompletePieceFrame_ ? millis() - lastCompletePieceFrameMs_
                                  : 0xFFFFFFFFu;
  }
  Observation observation() const { return observation_; }
  uint8_t observedPieceId() const { return observedPieceId_; }
  float observedRobotY() const { return observedRobotY_; }
  uint8_t observationSamples() const { return observationSamples_; }
  uint8_t detectedObjects() const { return detectedObjects_; }
  uint8_t eligibleObjects() const { return eligibleObjects_; }
  uint32_t tagFramesSeen() const { return tagFramesSeen_; }
  uint32_t tagObjectsSeen() const { return tagObjectsSeen_; }
  uint32_t invalidTagContents() const { return invalidTagContents_; }
  uint8_t lastRawTagCount() const { return lastRawTagCount_; }
  int16_t lastRawLearnedId() const { return lastRawLearnedId_; }
  int16_t lastRawTagContent() const { return lastRawTagContent_; }
  int16_t lastRawTagX() const { return lastRawTagX_; }
  int16_t lastRawTagY() const { return lastRawTagY_; }
  uint32_t observationAgeMs() const {
    return observationMs_ == 0u ? 0xFFFFFFFFu : millis() - observationMs_;
  }
  uint32_t remoteSession() const { return remoteSession_; }
  int16_t algorithm() const { return lastAlgorithm_; }
  uint16_t flags() const { return lastFlags_; }
  uint32_t lastPacketAgeMs() const {
    return hasPacket_ ? millis() - lastPacketMs_ : 0xFFFFFFFFu;
  }
  const char *fault() const { return fault_[0] == '\0' ? "NONE" : fault_; }
  const char *stateName() const {
    switch (state_) {
      case State::OFF: return "OFF";
      case State::WAIT_TAG_MODE: return "WAIT_TAG";
      case State::COLLECTING_TAGS: return "CAL_TAGS";
      case State::SOLVING: return "SOLVING";
      case State::WAIT_PIECE_MODE: return "WAIT_PIECE";
      case State::READY: return "READY";
      default: return "FAULT";
    }
  }
  const char *observationName() const {
    switch (observation_) {
      case Observation::NONE: return "SIN_DATOS";
      case Observation::EMPTY: return "VACIO";
      case Observation::REJECTED: return "RECHAZADA";
      case Observation::MULTIPLE: return "MULTIPLE";
      case Observation::TRACKING: return "MUESTREANDO";
      case Observation::UNSTABLE: return "INESTABLE";
      case Observation::ACCEPTED: return "ACEPTADA";
      default: return "ENCLAVADA";
    }
  }

 private:
  static constexpr uint16_t kNoCommand = 0xFFFFu;
  static constexpr uint8_t kSamples = 5u;
  static constexpr float kJitterPx = 4.0f;
  static constexpr float kMaxReprojectionMm = 0.5f;
  static constexpr uint32_t kModeChangeTimeoutMs = 60000u;
  bool resettingMode_ = false;
  static constexpr uint32_t kTargetTtlMs = 600u;
  /* La banda mueve la coordenada longitudinal durante los tres cuadros. No
     usar los 10 mm transversales para exigir una pieza longitudinalmente
     inmovil. Este limite conservador no calibra velocidad ni orientacion. */
  static constexpr float kMaxLongitudinalSpreadMm = 110.0f;
  static constexpr float kWorldX[4] = {-165.0f, 165.0f, -165.0f, 165.0f};
  static constexpr float kWorldY[4] = {-117.5f, -117.5f, 117.5f, 117.5f};

  State state_ = State::OFF;
  uint16_t pendingCommand_ = kNoCommand;
  uint32_t lastPacketMs_ = 0u;
  bool hasPacket_ = false;
  uint32_t modeRequestStartedMs_ = 0u;
  uint32_t remoteSession_ = 0u;
  bool hasRemoteSession_ = false;
  int16_t lastAlgorithm_ = 0;
  uint16_t lastFlags_ = 0u;
  float tagX_[4][kSamples]{};
  float tagY_[4][kSamples]{};
  uint8_t tagCount_[4]{};
  uint32_t lastTagFrame_[4]{};
  float homography_[8]{};
  float cameraOriginX_ = 0.0f;
  float cameraOriginY_ = 0.0f;
  bool calibrated_ = false;
  float maxReprojectionMm_ = NAN;
  char fault_[48]{};

  uint8_t pieceId_ = 0u;
  float pieceY_[3]{};
  float pieceLongitudinal_[3]{};
  uint8_t pieceCount_ = 0u;
  uint32_t pieceFirstMs_ = 0u;
  uint32_t lastPieceFrame_ = 0u;
  bool hasPieceFrame_ = false;
  uint32_t frameReceivedMs_ = 0u;
  uint32_t lastCompletePieceFrameMs_ = 0u;
  bool hasCompletePieceFrame_ = false;
  uint8_t frameExpected_ = 0u;
  uint8_t frameReceived_ = 0u;
  bool frameInvalid_ = false;
  bool frameSlot_[10]{};
  int16_t frameId_[10]{};
  int16_t frameX_[10]{};
  int16_t frameY_[10]{};
  bool pieceLatched_ = false;
  uint8_t emptyFrames_ = 0u;
  bool targetReady_ = false;
  uint8_t targetId_ = 0u;
  float targetRobotY_ = 0.0f;
  CameraPieceTarget pieceTarget_{};
  Observation observation_ = Observation::NONE;
  uint8_t observedPieceId_ = 0u;
  float observedRobotY_ = 0.0f;
  uint8_t observationSamples_ = 0u;
  uint8_t detectedObjects_ = 0u;
  uint8_t eligibleObjects_ = 0u;
  uint32_t observationMs_ = 0u;
  uint32_t lastRawTagFrame_ = 0u;
  uint32_t tagFramesSeen_ = 0u;
  uint32_t tagObjectsSeen_ = 0u;
  uint32_t invalidTagContents_ = 0u;
  uint8_t lastRawTagCount_ = 0u;
  int16_t lastRawLearnedId_ = -1;
  int16_t lastRawTagContent_ = -1;
  int16_t lastRawTagX_ = 0;
  int16_t lastRawTagY_ = 0;

  void setFault(const char *message) {
    resettingMode_ = false;
    pendingCommand_ = kNoCommand;
    state_ = State::FAULT;
    calibrated_ = false;
    clearPendingTargetForArm();
    strncpy(fault_, message, sizeof(fault_) - 1u);
    fault_[sizeof(fault_) - 1u] = '\0';
    clearObservation();
    modeRequestStartedMs_ = 0u;
  }

  void clearCalibration() {
    memset(tagX_, 0, sizeof(tagX_));
    memset(tagY_, 0, sizeof(tagY_));
    memset(tagCount_, 0, sizeof(tagCount_));
    memset(lastTagFrame_, 0, sizeof(lastTagFrame_));
    calibrated_ = false;
    maxReprojectionMm_ = NAN;
    clearPieceWindow();
    pieceLatched_ = false;
    emptyFrames_ = 0u;
    clearTarget();
    hasPieceFrame_ = false;
    hasCompletePieceFrame_ = false;
    frameExpected_ = 0u;
    frameReceived_ = 0u;
    frameInvalid_ = false;
    memset(frameSlot_, 0, sizeof(frameSlot_));
    clearObservation();
    lastRawTagFrame_ = 0u;
    tagFramesSeen_ = 0u;
    tagObjectsSeen_ = 0u;
    invalidTagContents_ = 0u;
    lastRawTagCount_ = 0u;
    lastRawLearnedId_ = -1;
    lastRawTagContent_ = -1;
    lastRawTagX_ = 0;
    lastRawTagY_ = 0;
  }

  void recordRawTagPacket(const CameraPacket &packet) {
    if (packet.frame != lastRawTagFrame_) {
      lastRawTagFrame_ = packet.frame;
      ++tagFramesSeen_;
      lastRawTagCount_ = packet.count;
    }
    if ((packet.flags & CAMERA_FLAG_EMPTY_FRAME) != 0u || packet.count == 0u)
      return;
    ++tagObjectsSeen_;
    lastRawLearnedId_ = packet.learnedId;
    lastRawTagContent_ = packet.tagContent;
    lastRawTagX_ = packet.xCenter;
    lastRawTagY_ = packet.yCenter;
    if (packet.tagContent < 0 || packet.tagContent > 3)
      ++invalidTagContents_;
  }

  void clearObservation() {
    observation_ = Observation::NONE;
    observedPieceId_ = 0u;
    observedRobotY_ = 0.0f;
    observationSamples_ = 0u;
    detectedObjects_ = 0u;
    eligibleObjects_ = 0u;
    observationMs_ = 0u;
  }

  void setObservation(Observation observation, uint8_t id, float robotY,
                      uint8_t samples, uint8_t detected, uint8_t eligible) {
    observation_ = observation;
    observedPieceId_ = id;
    observedRobotY_ = robotY;
    observationSamples_ = samples;
    detectedObjects_ = detected;
    eligibleObjects_ = eligible;
    observationMs_ = millis();
  }

  void clearPieceWindow() {
    pieceId_ = 0u;
    pieceCount_ = 0u;
    pieceFirstMs_ = 0u;
  }

  void clearTarget() {
    targetReady_ = false;
    targetId_ = 0u;
    targetRobotY_ = 0.0f;
    pieceTarget_ = CameraPieceTarget{};
  }

  void expireTarget() {
    if (targetReady_ && millis() - pieceTarget_.observedMs > kTargetTtlMs)
      clearTarget();
  }

  void resetFrameAssembly(uint32_t frame, uint8_t expected) {
    lastPieceFrame_ = frame;
    hasPieceFrame_ = true;
    frameReceivedMs_ = millis();
    frameExpected_ = expected;
    frameReceived_ = 0u;
    frameInvalid_ = false;
    memset(frameSlot_, 0, sizeof(frameSlot_));
  }

  static float median5(const float values[kSamples]) {
    float sorted[kSamples];
    memcpy(sorted, values, sizeof(sorted));
    for (uint8_t i = 1u; i < kSamples; ++i) {
      const float value = sorted[i];
      int j = static_cast<int>(i) - 1;
      while (j >= 0 && sorted[j] > value) {
        sorted[j + 1] = sorted[j];
        --j;
      }
      sorted[j + 1] = value;
    }
    return sorted[2];
  }

  static bool solve8(float a[8][9], float result[8]) {
    for (uint8_t column = 0u; column < 8u; ++column) {
      uint8_t pivot = column;
      for (uint8_t row = column + 1u; row < 8u; ++row) {
        if (fabsf(a[row][column]) > fabsf(a[pivot][column])) pivot = row;
      }
      if (fabsf(a[pivot][column]) < 1.0e-6f) return false;
      if (pivot != column) {
        for (uint8_t k = column; k < 9u; ++k) {
          const float temporary = a[column][k];
          a[column][k] = a[pivot][k];
          a[pivot][k] = temporary;
        }
      }
      const float divisor = a[column][column];
      for (uint8_t k = column; k < 9u; ++k) a[column][k] /= divisor;
      for (uint8_t row = 0u; row < 8u; ++row) {
        if (row == column) continue;
        const float factor = a[row][column];
        for (uint8_t k = column; k < 9u; ++k)
          a[row][k] -= factor * a[column][k];
      }
    }
    for (uint8_t i = 0u; i < 8u; ++i) result[i] = a[i][8];
    return true;
  }

  bool pixelToWorld(float px, float py, float &x, float &y) const {
    const float denominator = homography_[6] * px + homography_[7] * py + 1.0f;
    if (!isfinite(denominator) || fabsf(denominator) < 1.0e-6f) return false;
    x = (homography_[0] * px + homography_[1] * py + homography_[2]) /
        denominator;
    y = (homography_[3] * px + homography_[4] * py + homography_[5]) /
        denominator;
    return isfinite(x) && isfinite(y);
  }

  bool solveHomography() {
    float px[4];
    float py[4];
    for (uint8_t id = 0u; id < 4u; ++id) {
      if (tagCount_[id] < kSamples) return false;
      px[id] = median5(tagX_[id]);
      py[id] = median5(tagY_[id]);
      float maxDistance = 0.0f;
      for (uint8_t sample = 0u; sample < kSamples; ++sample) {
        const float dx = tagX_[id][sample] - px[id];
        const float dy = tagY_[id][sample] - py[id];
        maxDistance = fmaxf(maxDistance, sqrtf(dx * dx + dy * dy));
      }
      if (maxDistance > kJitterPx) {
        setFault("Jitter AprilTag mayor de 4 px");
        return false;
      }
    }
    const uint8_t order[4] = {0u, 1u, 3u, 2u};
    float twiceArea = 0.0f;
    for (uint8_t i = 0u; i < 4u; ++i) {
      const uint8_t a = order[i];
      const uint8_t b = order[(i + 1u) % 4u];
      twiceArea += px[a] * py[b] - py[a] * px[b];
    }
    if (fabsf(twiceArea) * 0.5f < 5000.0f) {
      setFault("Cuadrilatero AprilTag demasiado pequeno");
      return false;
    }
    float matrix[8][9]{};
    for (uint8_t i = 0u; i < 4u; ++i) {
      const uint8_t row = i * 2u;
      matrix[row][0] = px[i]; matrix[row][1] = py[i]; matrix[row][2] = 1.0f;
      matrix[row][6] = -kWorldX[i] * px[i];
      matrix[row][7] = -kWorldX[i] * py[i];
      matrix[row][8] = kWorldX[i];
      matrix[row + 1u][3] = px[i]; matrix[row + 1u][4] = py[i];
      matrix[row + 1u][5] = 1.0f;
      matrix[row + 1u][6] = -kWorldY[i] * px[i];
      matrix[row + 1u][7] = -kWorldY[i] * py[i];
      matrix[row + 1u][8] = kWorldY[i];
    }
    if (!solve8(matrix, homography_)) {
      setFault("Homografia degenerada");
      return false;
    }
    maxReprojectionMm_ = 0.0f;
    for (uint8_t i = 0u; i < 4u; ++i) {
      float x = 0.0f;
      float y = 0.0f;
      if (!pixelToWorld(px[i], py[i], x, y)) return false;
      const float dx = x - kWorldX[i];
      const float dy = y - kWorldY[i];
      maxReprojectionMm_ = fmaxf(maxReprojectionMm_, sqrtf(dx * dx + dy * dy));
    }
    if (maxReprojectionMm_ > kMaxReprojectionMm) {
      setFault("Reproyeccion mayor de 0.5 mm");
      return false;
    }
    if (!pixelToWorld(320.0f, 240.0f, cameraOriginX_, cameraOriginY_)) {
      setFault("Centro optico no proyectable");
      return false;
    }
    calibrated_ = true;
    state_ = State::WAIT_PIECE_MODE;
    pendingCommand_ = CAMERA_COMMAND_PIECE_MODE;
    modeRequestStartedMs_ = millis();
    return true;
  }

  void collectTag(const CameraPacket &packet) {
    if (packet.tagContent < 0 || packet.tagContent > 3) return;
    const uint8_t id = static_cast<uint8_t>(packet.tagContent);
    if (packet.frame == lastTagFrame_[id] || tagCount_[id] >= kSamples) return;
    lastTagFrame_[id] = packet.frame;
    const uint8_t sample = tagCount_[id]++;
    tagX_[id][sample] = static_cast<float>(packet.xCenter);
    tagY_[id][sample] = static_cast<float>(packet.yCenter);
    if (tagCount_[0] >= kSamples && tagCount_[1] >= kSamples &&
        tagCount_[2] >= kSamples && tagCount_[3] >= kSamples) {
      state_ = State::SOLVING;
      solveHomography();
    }
  }

  void evaluatePieceFrame() {
    hasCompletePieceFrame_ = true;
    lastCompletePieceFrameMs_ = frameReceivedMs_;
    uint8_t candidateIndex = 0u;
    uint8_t candidates = 0u;
    uint8_t detected = 0u;
    for (uint8_t i = 0u; i < frameExpected_; ++i) {
      if (!frameSlot_[i]) continue;
      ++detected;
      /* Toda la imagen puede iniciar una deteccion. La homografia y las
         guardias fisicas posteriores siguen rechazando destinos invalidos. */
      if (frameId_[i] == 1 || frameId_[i] == 2) {
        candidateIndex = i;
        ++candidates;
      }
    }
    if (candidates == 0u) {
      /* Una pieza visible con ID no valido no equivale a banda vacia. Solo
         cinco cuadros sin NINGUN objeto permiten aceptar otra pieza. */
      if (detected == 0u && emptyFrames_ < 255u) ++emptyFrames_;
      if (detected != 0u) emptyFrames_ = 0u;
      if (detected == 0u && emptyFrames_ >= 5u) {
        pieceLatched_ = false;
        clearPieceWindow();
        clearTarget();
        setObservation(detected == 0u ? Observation::EMPTY
                                     : Observation::REJECTED,
                       0u, 0.0f, 0u, detected, 0u);
      } else if (pieceLatched_) {
        setObservation(Observation::LATCHED, targetId_, targetRobotY_, 3u,
                       detected, 0u);
      } else {
        clearPieceWindow();
        setObservation(detected == 0u ? Observation::EMPTY
                                     : Observation::REJECTED,
                       0u, 0.0f, 0u, detected, 0u);
      }
      return;
    }
    emptyFrames_ = 0u;
    if (candidates != 1u) {
      clearPieceWindow();
      setObservation(Observation::MULTIPLE, 0u, 0.0f, 0u,
                     detected, candidates);
      return;
    }
    if (pieceLatched_) {
      setObservation(Observation::LATCHED, targetId_, targetRobotY_, 3u,
                     detected, candidates);
      return;
    }
    float worldX = 0.0f;
    float worldY = 0.0f;
    if (!pixelToWorld(static_cast<float>(frameX_[candidateIndex]),
                      static_cast<float>(frameY_[candidateIndex]),
                      worldX, worldY)) {
      setObservation(Observation::REJECTED, 0u, 0.0f, 0u,
                     detected, candidates);
      return;
    }
    const float cameraX = worldX - cameraOriginX_;
    const float longitudinal = worldY - cameraOriginY_;
    if (cameraX < -145.0f || cameraX > 145.0f) {
      setObservation(Observation::REJECTED, 0u, 0.0f, 0u,
                     detected, candidates);
      return;
    }
    const float robotY = cameraX + 37.0f;
    if (!isfinite(robotY) || !isfinite(longitudinal)) {
      setObservation(Observation::REJECTED, 0u, 0.0f, 0u,
                     detected, candidates);
      return;
    }
    const uint32_t now = millis();
    const uint8_t id = static_cast<uint8_t>(frameId_[candidateIndex]);
    if (pieceCount_ == 0u || pieceId_ != id || now - pieceFirstMs_ > 600u) {
      clearPieceWindow();
      pieceId_ = id;
      pieceFirstMs_ = now;
    }
    pieceY_[pieceCount_] = robotY;
    pieceLongitudinal_[pieceCount_] = longitudinal;
    ++pieceCount_;
    setObservation(Observation::TRACKING, id, robotY, pieceCount_,
                   detected, candidates);
    if (pieceCount_ < 3u) return;
    const float minimum = fminf(pieceY_[0], fminf(pieceY_[1], pieceY_[2]));
    const float maximum = fmaxf(pieceY_[0], fmaxf(pieceY_[1], pieceY_[2]));
    const float longitudinalMinimum = fminf(pieceLongitudinal_[0],
        fminf(pieceLongitudinal_[1], pieceLongitudinal_[2]));
    const float longitudinalMaximum = fmaxf(pieceLongitudinal_[0],
        fmaxf(pieceLongitudinal_[1], pieceLongitudinal_[2]));
    if (maximum - minimum > 10.0f || longitudinalMaximum -
        longitudinalMinimum > kMaxLongitudinalSpreadMm) {
      clearPieceWindow();
      setObservation(Observation::UNSTABLE, id, robotY, 0u,
                     detected, candidates);
      return;
    }
    targetId_ = id;
    targetRobotY_ = (pieceY_[0] + pieceY_[1] + pieceY_[2]) / 3.0f;
    pieceTarget_.id = id;
    pieceTarget_.robotY = robotY;
    pieceTarget_.worldLongitudinalMm = longitudinal;
    pieceTarget_.observedMs = frameReceivedMs_;
    targetReady_ = true;
    pieceLatched_ = true;
    setObservation(Observation::ACCEPTED, id, targetRobotY_, 3u,
                   detected, candidates);
    clearPieceWindow();
  }

  void collectPiece(const CameraPacket &packet) {
    /* No completar parcialmente un cuadro de mas de diez objetos ni convertir
       un paquete EMPTY inconsistente en evidencia de banda despejada. */
    if (packet.algorithm != CAMERA_COMMAND_PIECE_MODE) {
      clearPieceWindow();
      return;
    }
    if (packet.count > 10u ||
        ((packet.flags & CAMERA_FLAG_EMPTY_FRAME) != 0u && packet.count != 0u)) {
      resetFrameAssembly(packet.frame, 0u);
      frameInvalid_ = true;
      clearPieceWindow();
      setObservation(Observation::REJECTED, 0u, 0.0f, 0u, 0u, 0u);
      return;
    }
    if (hasPieceFrame_ && packet.frame == lastPieceFrame_ && frameInvalid_)
      return;
    if (hasPieceFrame_ && packet.frame == lastPieceFrame_ &&
        packet.count != frameExpected_) {
      frameInvalid_ = true;
      clearPieceWindow();
      setObservation(Observation::REJECTED, 0u, 0.0f, 0u, 0u, 0u);
      return;
    }
    if ((packet.flags & CAMERA_FLAG_EMPTY_FRAME) != 0u || packet.count == 0u) {
      if (!hasPieceFrame_ || lastPieceFrame_ != packet.frame) {
        resetFrameAssembly(packet.frame, 0u);
        evaluatePieceFrame();
      }
      return;
    }
    if (!hasPieceFrame_ || packet.frame != lastPieceFrame_)
      resetFrameAssembly(packet.frame, packet.count);
    if (packet.index >= frameExpected_ || packet.index >= 10u) {
      frameInvalid_ = true;
      clearPieceWindow();
      setObservation(Observation::REJECTED, 0u, 0.0f, 0u, 0u, 0u);
      return;
    }
    if (frameSlot_[packet.index]) return;
    frameSlot_[packet.index] = true;
    frameId_[packet.index] = packet.learnedId;
    frameX_[packet.index] = packet.xCenter;
    frameY_[packet.index] = packet.yCenter;
    ++frameReceived_;
    if (frameReceived_ >= frameExpected_) evaluatePieceFrame();
  }
};

constexpr float CameraAlignment::kWorldX[4];
constexpr float CameraAlignment::kWorldY[4];

#endif
