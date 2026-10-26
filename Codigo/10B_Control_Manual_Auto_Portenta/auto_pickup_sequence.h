/**
 * @file auto_pickup_sequence.h
 * @brief Maquina de estados de recogida, traslado, descarga y retorno.
 * Explicita permisos, vencimientos y transiciones sin rearme automatico.
 */
#ifndef AUTO_PICKUP_SEQUENCE_H
#define AUTO_PICKUP_SEQUENCE_H

#include <cmath>
#include <stdint.h>
#include "gripper_auto_calibration.h"
#include "auto_pickup_height.h"
#include "auto_box_calibration.h"
#include "auto_gate_encoder.h"

// Pure cycle coordinator. It never writes GPIO/UART or rearms a robot.
// Continuous repetition requires an explicit caller advance after verified return.
// The caller owns fresh gamepad/session/stream/lease checks, path IK/FK/RAW
// validation, and the physical Nucleo checkpoint. Model arrival is NOT enough
// to call markCartesianVerified(). RC ACK and dwell are temporal command
// guards only: these three RC servos provide no measured-position feedback.
namespace AutoPickup {

struct Pose { float x, y, z; };
struct RcPose { uint16_t z, x, palo; };

enum class State : uint8_t {
  IDLE, PREPARE, INITIAL_LIFT, WAIT_PIECE, ALIGN_ABOVE, LOWER_PICKUP, WAIT_ARRIVAL,
  CLOSE_GATE, CARRY_X, LIFT, TRANSFER, LOWER_DROP, OPEN_ROTATE, DUMP_X,
  RESET_X, LIFT_RETURN, RESET_POSE, RETURN_XY, RETURN_HOME, COMPLETE,
  ABORTED,
  // Append stages to preserve the diagnostic values of existing states.
  CLOSE_RETURN_GATE, OPEN_AT_ORIGIN, PICKUP_POSE
};

enum class Kind : uint8_t { HOLD, CARTESIAN, RC, WAIT_ENCODER, COMPLETE, FAULT };
enum class Fault : uint8_t {
  NONE, CONFIGURATION, BAD_PIECE, STALE_CAMERA, STALE_ENCODER,
  ENCODER_REVERSED, PIECE_LATE, BAD_ENCODER, STAGE_TIMEOUT, EXTERNAL_STOP
};

struct Config {
  float transitZMm = NAN;       // No default height or guessed route.
  float waitZMm = NAN;          // AUTO ready pose, distinct from safety HOME/loaded transit.
  bool combinedApproach = false;
  bool combinedLoadedTransfer = false; // one XYZ route from pickup to final box
  bool twoDirectLoadedMoves = false; // LIFT, physical PASS, then final box XYZ
  bool returnXyAtDropZ = false; // centre XY at deposit Z, then descend at centre
  uint16_t waitTiltUs = GRIP_X_PICKUP_US; // legacy callers retain their pickup tilt
  bool pickupPoseDuringApproach = false; // empty approach only; final descent waits for RC ACK+settle
  bool rightLiftFirst = false; // only an explicitly configured lateral zone
  float rightLiftThresholdY = NAN; // robot +Y is the camera's right side
  float approachClearanceMm = 20.0f; // model guard, not certified mechanical clearance
  float pickupXMm = NAN;        // Captured trial X is not a camera calibration.
  float cameraLongitudinalSign = NAN;
  float robotBeltXSign = NAN;
  float arrivalToleranceMm = NAN;
  float reverseToleranceMm = NAN;
  bool routeConfirmed = false;
  bool mappingConfirmed = false;
  bool rcTimingConfirmed = false;

  bool supervisedTrial = false;
  bool continuous = false; // same live ARM/STREAM session; never recovery from STOP
  float closeLeadMm = 0.0f; // tunable advance before geometric arrival
  bool countBasedGate = false; // legacy callers retain their mm-based interface
  double encoderCountsPerMm = AutoGateEncoder::COUNTS_PER_MM;
  // HOME is below the recorded drop height. Supporting HOME as a transit
  // plane therefore requires explicit consent to ASCEND at the box, not an
  // implicit substitution that claims to lower there.
  bool dropMayBeAboveTransitConfirmed = false;
  uint32_t rcSettleMs = GRIP_OPEN_SETTLE_MS;
  uint32_t dumpSettleMs = 500u;
  uint32_t stageTimeoutMs = 15000u;
  uint32_t cartesianStageTimeoutMs = 15000u; // separate bounded feedback budget
};

struct Piece {
  uint8_t id = 0u;
  float robotYMm = NAN;
  // Caller associates the encoder count with a fresh camera frame, computes
  // longitudinal world-coordinate/datum correction and pickup X correction.
  // This is NOT a fixed 500 mm wait and is NOT a 500/speed timer.
  float distanceToArrivalMm = NAN;
  uint32_t observedAtMs = 0u;
  int32_t encoderAtFrame = 0;
  float closeLeadMm = NAN;  
};

struct Action {
  Kind kind = Kind::HOLD;
  uint32_t token = 0u;
  Pose cartesian = {0.0f, 0.0f, 0.0f};
  RcPose rc = {0u, 0u, 0u};
  bool concurrentRc = false;
};

class Sequence {
 public:
  static constexpr uint32_t CAMERA_MAX_AGE_MS = 600u;

  static bool configValid(const Config& c) {
    return (c.supervisedTrial ||
            (c.routeConfirmed && c.mappingConfirmed && c.rcTimingConfirmed)) &&
      std::isfinite(c.transitZMm) &&
      std::isfinite(c.waitZMm) && c.waitZMm > GRIP_PICKUP_Z_MM &&
      c.waitTiltUs >= 500u && c.waitTiltUs <= 2500u &&
      (!c.pickupPoseDuringApproach || c.combinedApproach) &&
      (!c.twoDirectLoadedMoves || c.combinedLoadedTransfer) &&
      (!c.rightLiftFirst || (std::isfinite(c.rightLiftThresholdY) &&
        c.rightLiftThresholdY >= 0.0f && c.rightLiftThresholdY <= 280.0f)) &&
      c.waitZMm <= -180.0f &&
      std::isfinite(c.approachClearanceMm) && c.approachClearanceMm >= 20.0f &&
      c.approachClearanceMm <= 100.0f &&
      c.waitZMm > AutoPickupHeight::clearanceZBetween(-295.0f, 280.0f,
                                                     c.approachClearanceMm) &&
      c.transitZMm > GRIP_PICKUP_Z_MM &&
      (c.transitZMm >= AutoBoxCalibration::MAX_DEPOSIT_Z_MM || c.dropMayBeAboveTransitConfirmed) &&
      c.transitZMm <= -180.0f && std::isfinite(c.pickupXMm) &&
      c.pickupXMm >= -270.0f && c.pickupXMm <= 270.0f &&
      (c.cameraLongitudinalSign == -1.0f || c.cameraLongitudinalSign == 1.0f) &&
      (c.robotBeltXSign == -1.0f || c.robotBeltXSign == 1.0f ||
       (c.robotBeltXSign == 0.0f && c.pickupXMm == 0.0f)) &&
      std::isfinite(c.arrivalToleranceMm) && c.arrivalToleranceMm > 0.0f &&
      c.arrivalToleranceMm <= 15.0f && std::isfinite(c.reverseToleranceMm) &&
      c.reverseToleranceMm >= 0.0f &&
      c.reverseToleranceMm < c.arrivalToleranceMm &&
      std::isfinite(c.closeLeadMm) && c.closeLeadMm >= 0.0f &&
      c.closeLeadMm <= 400.0f &&
      std::isfinite(c.encoderCountsPerMm) && c.encoderCountsPerMm > 0.0 &&
      c.rcSettleMs >= GRIP_OPEN_SETTLE_MS && c.rcSettleMs <= 5000u &&
      c.dumpSettleMs >= GRIP_OPEN_SETTLE_MS && c.dumpSettleMs <= 5000u &&
      c.stageTimeoutMs > c.dumpSettleMs &&
      c.stageTimeoutMs > c.rcSettleMs && c.stageTimeoutMs <= 30000u &&
      c.cartesianStageTimeoutMs > c.rcSettleMs && c.cartesianStageTimeoutMs <= 30000u;
  }

  bool begin(const Config& config, uint32_t now) {
    // Initial start requires the caller's explicit ARM workflow. Repetition
    // uses continueAfterComplete(), with the caller's live safety guards.
    if (active()) return false;
    config_ = config;
    fault_ = Fault::NONE;
    piece_ = Piece();
    gateTarget_ = AutoGateEncoder::Target();
    effectiveCloseLeadMm_ = config_.closeLeadMm;
    pickupZMm_ = NAN;
    approachZMm_ = NAN;
    box_ = 0u;
    completedCycles_ = 0u;
    if (!configValid(config_)) {
      abort(Fault::CONFIGURATION);
      return false;
    }
    enter(State::PREPARE, now);
    return true;
  }

  bool continueAfterComplete(uint32_t now) {
    // Only a physically verified complete route may repeat. No new ARM,
    // no checkpoint shortcut and absolutely no restart from an abort.
    if (!config_.continuous || state_ != State::COMPLETE || fault_ != Fault::NONE)
      return false;
    ++completedCycles_;
    piece_ = Piece();
    gateTarget_ = AutoGateEncoder::Target();
    effectiveCloseLeadMm_ = config_.closeLeadMm;
    pickupZMm_ = NAN;
    approachZMm_ = NAN;
    box_ = 0u;
    maximumAdvanceMm_ = 0.0f;
    enter(State::WAIT_PIECE, now); // already prepared at origin/AUTO wait height
    return true;
  }

  uint32_t completedCycles() const { return completedCycles_; }

  bool acceptPiece(const Piece& piece, uint32_t now, float signedAdvanceMm) {
    if (state_ != State::WAIT_PIECE) return false;
    const float lead = std::isnan(piece.closeLeadMm) ? config_.closeLeadMm : piece.closeLeadMm;
    unsigned box = AutoBoxCalibration::COUNT;
    for (unsigned i = 0u; i < AutoBoxCalibration::COUNT; ++i) {
      if (piece.id == AutoBoxCalibration::PIECE_ID[i]) box = i;
    }
    if (box == AutoBoxCalibration::COUNT || !std::isfinite(piece.robotYMm) ||
        piece.robotYMm < -295.0f || piece.robotYMm > 280.0f ||
        !std::isfinite(piece.distanceToArrivalMm) ||
        !std::isfinite(lead) || lead < 0.0f || lead > 400.0f ||
        piece.distanceToArrivalMm <= config_.arrivalToleranceMm + lead) {
      abort(Fault::BAD_PIECE);
      return false;
    }
    if (static_cast<uint32_t>(now - piece.observedAtMs) > CAMERA_MAX_AGE_MS) {
      abort(Fault::STALE_CAMERA);
      return false;
    }
    const float pickupY = AutoPickupHeight::clampPickupY(piece.robotYMm);
    const float pickupZ = AutoPickupHeight::targetZ(pickupY);
    const float approachZ = pickupApproachZ(config_, pickupY);
    if (!std::isfinite(pickupZ) || pickupZ >= config_.transitZMm ||
        !std::isfinite(approachZ) || approachZ <= pickupZ ||
        (config_.combinedApproach && approachZ >= config_.waitZMm)) {
      abort(Fault::BAD_PIECE);
      return false;
    }
    piece_ = piece;
    piece_.robotYMm = pickupY; // freeze the bounded lateral target with this piece
    effectiveCloseLeadMm_ = lead;
    if (config_.countBasedGate && !AutoGateEncoder::target(piece.encoderAtFrame,
        piece.distanceToArrivalMm, lead, config_.arrivalToleranceMm, gateTarget_,
        config_.encoderCountsPerMm)) {
      abort(Fault::BAD_ENCODER);
      return false;
    }
    pickupZMm_ = pickupZ; // frozen with this accepted piece, never camera-chased
    approachZMm_ = approachZ;
    box_ = box;
    maximumAdvanceMm_ = 0.0f;
    if (!checkEncoder(true, signedAdvanceMm)) return false;
    enter(config_.waitTiltUs != GRIP_X_PICKUP_US && !config_.pickupPoseDuringApproach
      ? State::PICKUP_POSE : State::ALIGN_ABOVE, now);
    return true;
  }

  void tickEncoder(uint32_t now, bool encoderFresh, int32_t currentCounts) {
    const float advance = gateTarget_.valid ? static_cast<float>(AutoGateEncoder::advanceMm(
      gateTarget_.frameCount,currentCounts,gateTarget_.countsPerMm)) : 0.0f;
    tick(now,encoderFresh,advance,gateTarget_.reached(currentCounts));
  }

  void tick(uint32_t now, bool encoderFresh, float signedAdvanceMm, bool gateReached = false) {
    if (!active() || state_ == State::WAIT_PIECE) return;
    if (beforeArrival() && !checkEncoder(encoderFresh, signedAdvanceMm)) return;
    if (static_cast<uint32_t>(now - enteredAtMs_) >
        (action_.kind == Kind::CARTESIAN ? config_.cartesianStageTimeoutMs : config_.stageTimeoutMs)) {
      abort(Fault::STAGE_TIMEOUT);
      return;
    }
    if (state_ == State::WAIT_ARRIVAL &&
        (config_.countBasedGate ? gateReached :
         signedAdvanceMm >= piece_.distanceToArrivalMm - effectiveCloseLeadMm_ -
                            config_.arrivalToleranceMm)) {
      enter(State::CLOSE_GATE, now);
      return;
    }
    if (action_.kind != Kind::RC || !rcTargetAcked_) return;
    const uint32_t dwell = state_ == State::DUMP_X ?
      config_.dumpSettleMs : config_.rcSettleMs;
    if (static_cast<uint32_t>(now - rcAckAtMs_) < dwell) return;
    switch (state_) {
      case State::PREPARE: enter(State::INITIAL_LIFT, now); break;
      case State::PICKUP_POSE: enter(State::ALIGN_ABOVE, now); break;
      case State::CLOSE_GATE: enter(State::CARRY_X, now); break;
      case State::CARRY_X:
        enter(config_.twoDirectLoadedMoves ? State::LIFT :
          (config_.combinedLoadedTransfer &&
              !(config_.rightLiftFirst && piece_.robotYMm >= config_.rightLiftThresholdY)
          ? State::TRANSFER : State::LIFT), now); break;
      case State::OPEN_ROTATE: enter(State::DUMP_X, now); break;
      case State::DUMP_X: enter(State::RESET_X, now); break;
      case State::RESET_X: enter(State::CLOSE_RETURN_GATE, now); break;
      case State::CLOSE_RETURN_GATE:
        enter(config_.returnXyAtDropZ || sameDropPlane()
          ? State::RESET_POSE : State::LIFT_RETURN, now); break;
      case State::RESET_POSE: enter(State::RETURN_XY, now); break;
      case State::OPEN_AT_ORIGIN: enter(State::COMPLETE, now); break;
      default: break;
    }
  }

  bool markCartesianVerified(uint32_t token, uint32_t now) {
    if (action_.kind != Kind::CARTESIAN || token != action_.token) return false;
    if (action_.concurrentRc && !rcTargetSettled(now)) return false;
    if (static_cast<uint32_t>(now - enteredAtMs_) > config_.cartesianStageTimeoutMs) {
      abort(Fault::STAGE_TIMEOUT);
      return false;
    }
    switch (state_) {
      case State::INITIAL_LIFT: enter(State::WAIT_PIECE, now); break;
      case State::ALIGN_ABOVE: enter(State::LOWER_PICKUP, now); break;
      case State::LOWER_PICKUP: enter(State::WAIT_ARRIVAL, now); break;
      case State::LIFT: enter(State::TRANSFER, now); break;
      case State::TRANSFER:
        enter(config_.combinedLoadedTransfer || sameDropPlane()
          ? State::OPEN_ROTATE : State::LOWER_DROP, now); break;
      case State::LOWER_DROP: enter(State::OPEN_ROTATE, now); break;
      case State::LIFT_RETURN: enter(State::RESET_POSE, now); break;
      case State::RETURN_XY:
        enter(config_.returnXyAtDropZ || !config_.continuous
          ? State::RETURN_HOME : State::OPEN_AT_ORIGIN, now); break;
      case State::RETURN_HOME: enter(State::OPEN_AT_ORIGIN, now); break;
      default: return false;
    }
    return true;
  }

  bool markRcTargetAck(uint32_t token, uint32_t now) {
    if ((action_.kind != Kind::RC && !action_.concurrentRc) || token != action_.token) return false;
    if (static_cast<uint32_t>(now - enteredAtMs_) >
        (action_.kind == Kind::CARTESIAN ? config_.cartesianStageTimeoutMs : config_.stageTimeoutMs)) {
      abort(Fault::STAGE_TIMEOUT);
      return false;
    }
    // Repeated lease ACKs must not push the settle deadline forward forever.
    if (!rcTargetAcked_) { rcTargetAcked_ = true; rcAckAtMs_ = now; }
    return true;
  }

  bool rcTargetSettled(uint32_t now) const {
    return rcTargetAcked_ && static_cast<uint32_t>(now - rcAckAtMs_) >= config_.rcSettleMs;
  }

  void abort(Fault reason = Fault::EXTERNAL_STOP) {
    fault_ = reason == Fault::NONE ? Fault::EXTERNAL_STOP : reason;
    state_ = State::ABORTED;
    rcTargetAcked_ = false;
    gateTarget_ = AutoGateEncoder::Target();
    nextToken();
    action_.kind = Kind::FAULT;
    action_.concurrentRc = false;
  }

  State state() const { return state_; }
  Fault fault() const { return fault_; }
  const Action& action() const { return action_; }
  const Piece& piece() const { return piece_; }
  const AutoGateEncoder::Target& gateTarget() const { return gateTarget_; }
  float closeLeadMm() const { return effectiveCloseLeadMm_; }
  float pickupZMm() const { return pickupZMm_; }
  float approachZMm() const { return approachZMm_; }
  static float pickupApproachZ(const Config& c, float robotY) {
    return c.combinedApproach
      ? AutoPickupHeight::clearanceZBetween(0.0f, robotY, c.approachClearanceMm)
      : c.transitZMm;
  }
  uint32_t stateAgeMs(uint32_t now) const { return now - enteredAtMs_; }
  bool active() const {
    return state_ != State::IDLE && state_ != State::ABORTED && state_ != State::COMPLETE;
  }

  static const char* stateName(State s) {
    switch (s) {
      case State::IDLE: return "IDLE";
      case State::WAIT_PIECE: return "WAIT_PIECE";
      case State::PREPARE: return "PREPARE";
      case State::INITIAL_LIFT: return "INITIAL_LIFT";
      case State::ALIGN_ABOVE: return "ALIGN_ABOVE";
      case State::LOWER_PICKUP: return "LOWER_PICKUP";
      case State::WAIT_ARRIVAL: return "WAIT_ARRIVAL";
      case State::CLOSE_GATE: return "CLOSE_GATE";
      case State::CARRY_X: return "CARRY_X";
      case State::LIFT: return "LIFT";
      case State::TRANSFER: return "TRANSFER";
      // This approach may ascend in an explicitly confirmed HOME-transit
      // configuration; its destination is now the selected box's own Z.
      case State::LOWER_DROP: return "APPROACH_DROP";
      case State::OPEN_ROTATE: return "OPEN_ROTATE";
      case State::DUMP_X: return "DUMP_X";
      case State::RESET_X: return "RESET_X";
      case State::CLOSE_RETURN_GATE: return "CLOSE_RETURN_GATE";
      case State::OPEN_AT_ORIGIN: return "OPEN_AT_ORIGIN";
      case State::LIFT_RETURN: return "LIFT_RETURN";
      case State::RESET_POSE: return "RESET_POSE";
      case State::RETURN_XY: return "RETURN_XY";
      case State::RETURN_HOME: return "RETURN_HOME";
      case State::PICKUP_POSE: return "PICKUP_POSE";
      case State::COMPLETE: return "COMPLETE";
      case State::ABORTED: return "ABORTED";
      default: return "INVALID";
    }
  }

 private:
  bool sameDropPlane() const {
    return std::fabs(config_.transitZMm - AutoBoxCalibration::TARGET_MM[box_][2]) <= 0.001f;
  }

  bool beforeArrival() const {
    return state_ == State::ALIGN_ABOVE ||
      state_ == State::LOWER_PICKUP || state_ == State::WAIT_ARRIVAL;
  }

  bool checkEncoder(bool fresh, float advance) {
    if (!fresh) { abort(Fault::STALE_ENCODER); return false; }
    if (!std::isfinite(advance)) { abort(Fault::BAD_ENCODER); return false; }
    if (advance < maximumAdvanceMm_ - config_.reverseToleranceMm) {
      abort(Fault::ENCODER_REVERSED); return false;
    }
    if (advance > piece_.distanceToArrivalMm + config_.arrivalToleranceMm) {
      abort(Fault::PIECE_LATE); return false;
    }
    if (advance > maximumAdvanceMm_) maximumAdvanceMm_ = advance;
    return true;
  }

  void nextToken() {
    ++action_.token;
    if (action_.token == 0u) ++action_.token;
  }

  void enter(State next, uint32_t now) {
    state_ = next;
    enteredAtMs_ = now;
    rcTargetAcked_ = false;
    nextToken();
    action_.kind = Kind::HOLD;
    action_.concurrentRc = false;
    const float boxDropZ = AutoBoxCalibration::TARGET_MM[box_][2];
    // TWO_DIRECT uses two coordinate endpoints after pickup: first raise at
    // the frozen pickup X/Y to the selected box Z, then move to the box at
    // that same Z. Legacy routes retain the configured transit plane.
    const Pose pickupHigh = {config_.pickupXMm, piece_.robotYMm,
      config_.twoDirectLoadedMoves ? boxDropZ : config_.transitZMm};
    const Pose boxHigh = {AutoBoxCalibration::TARGET_MM[box_][0],
                         AutoBoxCalibration::TARGET_MM[box_][1], config_.transitZMm};
    switch (next) {
      case State::PREPARE:
      case State::OPEN_AT_ORIGIN:
        setRc(GRIP_Z_PICKUP_US, config_.waitTiltUs, GRIP_PUSH_WAIT_US); break;
      case State::PICKUP_POSE:
        setRc(GRIP_Z_PICKUP_US, GRIP_X_PICKUP_US, GRIP_PUSH_WAIT_US); break;
      case State::RESET_POSE:
        // Restore Z at the box, but retain the raised scoop and closed gate
        // throughout the Cartesian return. Never open beside the boxes.
        setRc(GRIP_Z_PICKUP_US, GRIP_X_CARRY_US, GRIP_PUSH_CLOSED_US); break;
      case State::INITIAL_LIFT:
        setCartesian({0.0f, 0.0f, config_.waitZMm}); break;
      case State::ALIGN_ABOVE:
        // Y and Z advance together to a frozen clearance point; the final
        // descent remains vertical with its separate physical checkpoint.
        setCartesian({config_.pickupXMm, piece_.robotYMm, approachZMm_});
        if (config_.pickupPoseDuringApproach && config_.waitTiltUs != GRIP_X_PICKUP_US) {
          action_.concurrentRc = true;
          action_.rc = {GRIP_Z_PICKUP_US, GRIP_X_PICKUP_US, GRIP_PUSH_WAIT_US};
        }
        break;
      case State::LIFT:
        setCartesian(pickupHigh); break;
      case State::LOWER_PICKUP:
        setCartesian({config_.pickupXMm, piece_.robotYMm, pickupZMm_}); break;
      case State::WAIT_ARRIVAL: action_.kind = Kind::WAIT_ENCODER; break;
      case State::CLOSE_GATE:
        setRc(GRIP_Z_PICKUP_US, GRIP_X_PICKUP_US, GRIP_PUSH_CLOSED_US); break;
      case State::CARRY_X:
        setRc(GRIP_Z_PICKUP_US, GRIP_X_CARRY_US, GRIP_PUSH_CLOSED_US); break;
      case State::TRANSFER:
        setCartesian(config_.combinedLoadedTransfer
          ? Pose{boxHigh.x,boxHigh.y,boxDropZ} : boxHigh); break;
      case State::LIFT_RETURN:
        setCartesian(boxHigh); break;
      case State::LOWER_DROP:
        setCartesian({boxHigh.x, boxHigh.y, AutoBoxCalibration::TARGET_MM[box_][2]}); break;
      case State::OPEN_ROTATE:
      case State::RESET_X:
        setRc(GRIP_Z_DROP_US, GRIP_X_CARRY_US, GRIP_PUSH_OPEN_US); break;
      case State::DUMP_X:
        setRc(GRIP_Z_DROP_US, AutoBoxCalibration::DUMP_X_US, GRIP_PUSH_OPEN_US); break;
      case State::CLOSE_RETURN_GATE:
        // RESET_X already completed its final command ACK and settle first.
        setRc(GRIP_Z_DROP_US, GRIP_X_CARRY_US, GRIP_PUSH_CLOSED_US); break;
      case State::RETURN_XY:
        setCartesian({0.0f, 0.0f, config_.returnXyAtDropZ ? boxDropZ : config_.waitZMm}); break;
      case State::RETURN_HOME:
        setCartesian({0.0f, 0.0f, config_.returnXyAtDropZ && config_.continuous
          ? config_.waitZMm : -287.0f}); break;
      case State::COMPLETE: action_.kind = Kind::COMPLETE; break;
      default: break;
    }
  }

  void setCartesian(const Pose& target) { action_.kind = Kind::CARTESIAN; action_.cartesian = target; }
  void setRc(uint16_t z, uint16_t x, uint16_t palo) { action_.kind = Kind::RC; action_.rc = {z, x, palo}; }

  Config config_;
  Piece piece_;
  AutoGateEncoder::Target gateTarget_;
  float effectiveCloseLeadMm_ = 0.0f;
  float pickupZMm_ = NAN;
  float approachZMm_ = NAN;
  State state_ = State::IDLE;
  Fault fault_ = Fault::NONE;
  Action action_;
  unsigned box_ = 0u;
  uint32_t enteredAtMs_ = 0u;
  uint32_t rcAckAtMs_ = 0u;
  bool rcTargetAcked_ = false;
  float maximumAdvanceMm_ = 0.0f;
  uint32_t completedCycles_ = 0u;
};

} // namespace AutoPickup
#endif
