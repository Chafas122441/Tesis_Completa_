/**
 * @file nucleo_stream_link.h
 * @brief Gestor UART de sesion, confirmaciones, permisos y telemetria Nucleo.
 * Conserva historial del encoder y no confirma movimiento fisico por un ACK.
 */
#ifndef NUCLEO_STREAM_LINK_H
#define NUCLEO_STREAM_LINK_H

#include <Arduino.h>
#include "delta_stream_protocol.h"
#include "gripper_limits.h"

class NucleoStreamLink {
 public:
  explicit NucleoStreamLink(HardwareSerial &port) : port_(port) {}

  void begin(uint32_t baud = 115200u) {
    port_.begin(baud);
    startNewSession();
  }

  void service() {
    readIncomingBytes();
    const uint32_t now = millis();
    if (pointActive_ && pointExpired(now)) {
      pointActive_=false;pointEvent_=true;pointResult_=DELTA_RESULT_FEEDBACK_FAILED;
    }
    if (checkpointActive_ && now - checkpointStartedMs_ > checkpointTimeoutMs_) {
      checkpointActive_ = false;
      checkpointEvent_ = true;
      checkpointResult_ = DELTA_RESULT_FEEDBACK_FAILED;
      memset(checkpointMeasuredRaw_, 0, sizeof(checkpointMeasuredRaw_));
    }
    if (recoveryActive_ && now - recoveryStartedMs_ > DELTA_FAULT_RECOVERY_TIMEOUT_MS) {
      recoveryActive_ = false;
      recoveryEvent_ = true;
      recoveryResult_ = DELTA_RESULT_FEEDBACK_FAILED;
    }
    if (waitingResponse_ && now - pendingSentMs_ >= responseTimeoutMs()) {
      if (pendingAttempts_ < MAX_ATTEMPTS) {
        ++pendingAttempts_;
        ++retries_;
        transmit(pendingFrame_);
        pendingSentMs_ = now;
        Serial.print(F("[STREAM RETRY] type="));
        Serial.print(pendingFrame_.type);
        Serial.print(F(" seq="));
        Serial.print(pendingFrame_.sequence);
        Serial.print(F(" attempt="));
        Serial.println(pendingAttempts_);
      } else {
        Serial.println(F("[STREAM LINK FAULT] Sin ACK despues de 3 intentos."));
        ++linkFaults_;
        invalidateLink();
        nextHelloMs_ = now + RECONNECT_DELAY_MS;
      }
    }
    if (!ready_ && !waitingResponse_ && now >= nextHelloMs_) startNewSession();
    if (pointActive_ && ready_ && !waitingResponse_ && now-pointLeaseMs_>=80u) {
      const uint16_t token[3]={pointSequence_,0u,0u};
      if (sendCommand(DELTA_MSG_POINT_KEEPALIVE,token,0u,pointOption_)) pointLeaseMs_=now;
    }
    if (ready_ && !waitingResponse_ &&
        now - lastCommandSentMs_ >= HEARTBEAT_PERIOD_MS) {
      sendSimple(DELTA_MSG_HEARTBEAT);
    }
    if (ready_ && now - lastValidResponseMs_ > LINK_TIMEOUT_MS) {
      Serial.println(F("[STREAM LINK FAULT] 750 ms sin respuesta valida."));
      ++linkFaults_;
      invalidateLink();
      nextHelloMs_ = now + RECONNECT_DELAY_MS;
    }
  }

  bool ready() const { return ready_; }
  bool waitingResponse() const { return waitingResponse_; }
  bool armed() const { return flag(DELTA_FLAG_ARMED); }
  bool homeValid() const { return flag(DELTA_FLAG_HOME_VALID); }
  bool busy() const { return flag(DELTA_FLAG_BUSY); }
  bool dryRun() const { return flag(DELTA_FLAG_DRY_RUN); }
  bool streamActive() const { return flag(DELTA_FLAG_STREAM_ACTIVE); }
  bool remoteFault() const { return flag(DELTA_FLAG_FAULT); }
  uint32_t sessionGeneration() const { return sessionGeneration_; }
  bool gripperStateFresh() const {
    return ready_ && gripperStateKnown_ && millis() - gripperStateMs_ <= 1000u;
  }
  const uint16_t *gripperStateUs() const { return gripperStateUs_; }
  uint16_t gripperStateChannel() const { return gripperStateChannel_; }
  uint8_t gripperStateFlags() const { return gripperStateFlags_; }
  const uint16_t *remoteRaw() const { return remoteRaw_; }
  // ACK/STATUS describe the nominal command. A physical PASS is a separate
  // origin for DIRECT planning; it must never become a hold command.
  const uint16_t *directOriginRaw() const { return directOriginRaw_; }
  bool directOriginKnown() const {
    return directOriginKnown_ && ready_ && armed() && homeValid() &&
      streamActive() && !remoteFault() && !dryRun();
  }
  uint32_t validResponses() const { return validResponses_; }
  uint32_t crcErrors() const { return crcErrors_; }
  uint32_t sessionErrors() const { return sessionErrors_; }
  uint32_t unexpectedResponses() const { return unexpectedResponses_; }
  uint32_t retries() const { return retries_; }
  uint32_t linkFaults() const { return linkFaults_; }
  uint32_t setpointAcks() const { return setpointAcks_; }
  // Nucleo ACKs only after its motor TX. Receipt-time pacing is conservative
  // even when calculation/UART/ACK latency varies. Fine orders keep their bounds.
  static constexpr uint32_t COARSE_ACK_GUARD_MS = 110u;
  static constexpr uint16_t COARSE_MIN_MOVE_MS = 200u;
  uint32_t streamAckAgeMs() const {
    return streamAckClockKnown_ ? millis() - lastStreamAckMs_ : UINT32_MAX;
  }
  bool coarseDispatchReady() const {
    return ready_ && !waitingResponse_ && armed() && homeValid() && streamActive() &&
      !remoteFault() && !dryRun() && !checkpointActive_ && streamAckClockKnown_ &&
      streamAckAgeMs() >= COARSE_ACK_GUARD_MS;
  }
  bool approachDispatchReady() const {
    return ready_ && !waitingResponse_ && armed() && homeValid() && streamActive() &&
      !remoteFault() && !dryRun() && !checkpointActive_ && streamAckClockKnown_ &&
      streamAckAgeMs() >= DELTA_APPROACH_PERIOD_MS;
  }
  uint8_t remoteFlags() const { return remoteFlags_; }
  bool encoderFresh() const {
    return ready_ && encoderKnown_ && millis() - encoderReceivedMs_ <= 3500u;
  }
  bool encoderFastFresh() const {
    return ready_ && encoderKnown_ && encoderIntervalMs_ <= 200u &&
      millis() - encoderReceivedMs_ <= 200u;
  }
  uint32_t encoderReceivedMs() const { return encoderReceivedMs_; }
  // Diagnostics only: never extend a sample's lease or change freshness.
  bool encoderKnown() const { return encoderKnown_; }
  uint32_t encoderAgeMs() const {
    return encoderKnown_ ? millis() - encoderReceivedMs_ : UINT32_MAX;
  }
  uint16_t encoderIntervalMs() const { return encoderIntervalMs_; }
  uint16_t encoderSequence() const { return encoderSequence_; }
  // Interpolate receipt-time samples only. Never substitute current counts
  // for an older camera observation, or extrapolate beyond the history.
  bool encoderPositionAt(uint32_t receiptMs, int32_t &position,
                         uint32_t maximumGapMs = 150u) const {
    if (!encoderFastFresh() || encoderHistoryCount_ < 2u ||
        maximumGapMs == 0u || maximumGapMs > 150u) return false;
    for (uint8_t i = 1u; i < encoderHistoryCount_; ++i) {
      const EncoderSample &older = encoderHistory_[i - 1u];
      const EncoderSample &newer = encoderHistory_[i];
      const uint32_t gap = newer.ms - older.ms;
      const uint32_t offset = receiptMs - older.ms;
      if (!gap || gap > maximumGapMs || offset > gap) continue;
      const int32_t delta = static_cast<int32_t>(
        static_cast<uint32_t>(newer.position) - static_cast<uint32_t>(older.position));
      position = static_cast<int32_t>(static_cast<uint32_t>(older.position) +
        static_cast<uint32_t>((static_cast<int64_t>(delta) * offset) / gap));
      return true;
    }
    return false;
  }
  bool checkpointActive() const { return checkpointActive_; }
  uint16_t checkpointSequence() const { return checkpointSequence_; }
  bool consumeCheckpoint(DeltaResultCode &result, uint16_t measuredRaw[3]) {
    if (!checkpointEvent_) return false;
    checkpointEvent_ = false;
    result = checkpointResult_;
    memcpy(measuredRaw, checkpointMeasuredRaw_, sizeof(checkpointMeasuredRaw_));
    return true;
  }
  int32_t encoderPosition() const { return encoderPosition_; }
  float beltSpeedMmS() const { return beltSpeedTenths_ * 0.1f; }
  uint16_t beltSpeedTenths() const { return beltSpeedTenths_; }
  bool feedbackSnapshotKnown() const {
    return ready_ && feedbackKnown_ && homeValid() && !remoteFault();
  }
  const uint16_t *measuredRawSnapshot() const { return feedbackRaw_; }
  uint32_t feedbackAgeMs() const {
    return feedbackBaseAge_ + (millis() - feedbackReceivedMs_);
  }

  bool consumeCompletion(uint8_t &type, DeltaResultCode &result) {
    if (!completionEvent_) return false;
    completionEvent_ = false;
    type = completedCommandType_;
    result = completedResult_;
    return true;
  }

  bool sendHome() {
    // IK de (0,0,-287): RAW410. No cambia el cero de calibracion angular.
    const uint16_t raw[3] = {410u, 410u, 410u};
    return sendCommand(DELTA_MSG_HOME, raw, 2500u, 0u);
  }
  bool recoveryActive() const { return recoveryActive_; }
  bool sendFaultRecovery() {
    if (!ready() || !remoteFault() || armed() || busy() || streamActive() ||
        dryRun() || recoveryActive_ || recoveryEvent_) return false;
    const uint16_t raw[3] = {0u, 0u, 0u};
    if (!sendCommand(DELTA_MSG_FAULT_RECOVER, raw, 0u, DELTA_FAULT_RECOVERY_MAGIC)) return false;
    recoverySequence_ = pendingFrame_.sequence;
    recoveryStartedMs_ = millis();
    recoveryActive_ = true;
    return true;
  }
  bool consumeFaultRecovery(DeltaResultCode &result) {
    if (!recoveryEvent_) return false;
    recoveryEvent_ = false;
    result = recoveryResult_;
    return true;
  }
  bool sendArm() { return sendSimple(DELTA_MSG_ARM); }
  bool sendGripper(uint8_t type, uint16_t channel, uint16_t us) {
    const uint16_t raw[3] = {channel, us, 0u};
    return sendCommand(type, raw, 0u, GRIP_MAGIC);
  }
  bool sendDisarm() {
    if (!sendSimple(DELTA_MSG_DISARM)) return false;
    cancelCheckpoint();
    cancelPoint();
    return true;
  }
  bool sendGripperGroup(uint8_t type, const uint16_t us[3], bool runtime = false,
                        bool automatic = false) {
    if (automatic && !runtime) return false;
    return sendCommand(type, us, 0u, automatic ? GRIP_AUTO_MAGIC :
                       (runtime ? GRIP_STREAM_MAGIC : GRIP_MAGIC));
  }
  bool sendStatusRequest() { return sendSimple(DELTA_MSG_STATUS_REQUEST); }
  bool sendGripperStateRequest() {
    const uint16_t raw[3] = {0u, 0u, 0u};
    gripperStateKnown_ = false;
    return sendCommand(DELTA_MSG_GRIP_STATE_REQUEST, raw, 0u, GRIP_MAGIC);
  }
  bool sendStreamBegin(uint16_t option) {
    if (!sendCommand(DELTA_MSG_STREAM_BEGIN, remoteRaw_, 0u, option)) return false;
    directOriginKnown_ = false;
    return true;
  }
  bool sendStreamEnd(uint16_t option) {
    const uint16_t policy = option & DELTA_CHECKPOINT_POLICY_MASK;
    if ((policy && !(option & DELTA_AUTO_STOP_POLICY)) ||
        (policy && policy != DELTA_CHECKPOINT_AUTO9 && policy != DELTA_CHECKPOINT_BOX15 &&
         policy != DELTA_CHECKPOINT_AUTO15_REC30) ||
        ((option & DELTA_AUTO_STOP_POLICY) && (!(option & DELTA_STREAM_OPTION_AUTOMATIC) ||
         (option & DELTA_STREAM_OPTION_MODE_MASK) != DELTA_STREAM_OPTION_XYZ_MANUAL))) return false;
    if (!sendCommand(DELTA_MSG_STREAM_END, remoteRaw_, 0u, option)) return false;
    cancelCheckpoint();
    cancelPoint();
    return true;
  }
  bool sendStreamCheckpoint(uint16_t option,bool recovery200=false,bool stepFallback=false,
                            bool centerCapability=false,bool centerObservation=false,
                            bool finePickup20=false,bool repeat5=false,bool approach5=false) {
    if(approach5 && !repeat5)return false;
    if(repeat5 && (!finePickup20 || !centerCapability || !recovery200 ||
        !stepFallback || centerObservation))return false;
    if(finePickup20 && (centerObservation ||
        !(option & DELTA_STREAM_OPTION_AUTOMATIC) ||
        (option & DELTA_STREAM_OPTION_MODE_MASK)!=DELTA_STREAM_OPTION_XYZ_MANUAL ||
        (!centerCapability && (recovery200 || stepFallback ||
          (option & ~(DELTA_STREAM_OPTION_MODE_MASK | DELTA_STREAM_OPTION_RANGE_MASK |
                       DELTA_STREAM_OPTION_AUTOMATIC))))))return false;
    if(stepFallback && !recovery200)return false;
    if(centerCapability && (!recovery200 || !stepFallback || centerObservation))return false;
    if(centerObservation && (recovery200 || stepFallback ||
        (option & DELTA_CHECKPOINT_POLICY_MASK)!=DELTA_CHECKPOINT_OBSERVE ||
        (option & DELTA_AUTO_STOP_POLICY)))return false;
    const uint16_t allowed = DELTA_STREAM_OPTION_MODE_MASK | DELTA_STREAM_OPTION_RANGE_MASK |
      DELTA_STREAM_OPTION_AUTOMATIC | DELTA_CHECKPOINT_POLICY_MASK | DELTA_AUTO_STOP_POLICY |
      DELTA_LOCAL_POINT_CAPABILITY | DELTA_DIRECT_ENDPOINT_CAPABILITY |
      DELTA_DIRECT_TRANSACTION_CAPABILITY | DELTA_DIRECT_VALIDATED_PATH_CAPABILITY |
      DELTA_DIRECT_LIFT_DELAY_CAPABILITY | DELTA_DIRECT_PROFILE250_CAPABILITY |
      DELTA_DIRECT_RETRY_CAPABILITY;
    if (option & (0xFFFFu ^ allowed)) return false;
    const uint16_t boxPermission = option & DELTA_CHECKPOINT_POLICY_MASK;
    const uint16_t directCapabilities = option &
      (DELTA_DIRECT_ENDPOINT_CAPABILITY | DELTA_DIRECT_TRANSACTION_CAPABILITY);
    const bool auto15 = boxPermission == DELTA_CHECKPOINT_AUTO15_REC30;
    const bool finalRead = boxPermission == DELTA_DIRECT_FINAL_READ_CAPABILITY ||
                          boxPermission == DELTA_ENDPOINT_RETRY_CAPABILITY;
    const bool observe = boxPermission == DELTA_CHECKPOINT_OBSERVE || finalRead;
    if (recovery200 && (boxPermission != DELTA_ENDPOINT_RETRY_CAPABILITY ||
        !(option & DELTA_AUTO_STOP_POLICY))) return false;
    if (finalRead && (!(option & DELTA_AUTO_STOP_POLICY) ||
        !(option & DELTA_DIRECT_PROFILE250_CAPABILITY))) return false;
    if (boxPermission && (boxPermission & (boxPermission - 1u)) && !auto15 && !observe) return false;
    const bool boxTolerance = boxPermission != 0u;
    if (directCapabilities && directCapabilities !=
        (DELTA_DIRECT_ENDPOINT_CAPABILITY | DELTA_DIRECT_TRANSACTION_CAPABILITY)) return false;
    if ((option & DELTA_DIRECT_VALIDATED_PATH_CAPABILITY) && directCapabilities !=
        (DELTA_DIRECT_ENDPOINT_CAPABILITY | DELTA_DIRECT_TRANSACTION_CAPABILITY)) return false;
    if ((option & DELTA_DIRECT_LIFT_DELAY_CAPABILITY) &&
        !(option & DELTA_DIRECT_VALIDATED_PATH_CAPABILITY)) return false;
    if ((option & DELTA_DIRECT_PROFILE250_CAPABILITY) &&
        !(option & DELTA_DIRECT_LIFT_DELAY_CAPABILITY)) return false;
    if ((option & DELTA_DIRECT_RETRY_CAPABILITY) &&
        !(option & DELTA_DIRECT_PROFILE250_CAPABILITY)) return false;
    if ((option & (DELTA_LOCAL_POINT_CAPABILITY | DELTA_DIRECT_ENDPOINT_CAPABILITY |
                   DELTA_DIRECT_TRANSACTION_CAPABILITY | DELTA_DIRECT_VALIDATED_PATH_CAPABILITY |
                   DELTA_DIRECT_LIFT_DELAY_CAPABILITY | DELTA_DIRECT_PROFILE250_CAPABILITY)) &&
        !(option & DELTA_AUTO_STOP_POLICY)) return false;
    if ((option & DELTA_AUTO_STOP_POLICY) && ((boxPermission && !auto15 && !observe) ||
        !(option & DELTA_STREAM_OPTION_AUTOMATIC) ||
        (option & DELTA_STREAM_OPTION_MODE_MASK) != DELTA_STREAM_OPTION_XYZ_MANUAL)) return false;
    if (boxTolerance && (!(option & DELTA_STREAM_OPTION_AUTOMATIC) ||
        (option & DELTA_STREAM_OPTION_MODE_MASK) != DELTA_STREAM_OPTION_XYZ_MANUAL)) return false;
    if (checkpointActive_ || checkpointEvent_ || !armed() || !streamActive() ||
        !homeValid() || remoteFault() || dryRun()) return false;
    if (!sendCommand((uint8_t)(approach5 ? DELTA_MSG_APPROACH_STEPS_PROBE :
        repeat5 ? DELTA_MSG_DIRECT_REPEAT5_PROBE :
        finePickup20 ? (centerCapability ? DELTA_MSG_PICKUP_FINE20_PROBE :
        DELTA_MSG_PICKUP_FINE20_CHECKPOINT) : centerCapability ? DELTA_MSG_CENTER_RECOVERY_PROBE :
        centerObservation ? DELTA_MSG_CENTER_OBSERVE :
        stepFallback ? DELTA_MSG_ENDPOINT_STEP_PROBE : recovery200 ? DELTA_MSG_ENDPOINT_REC200_PROBE :
        DELTA_MSG_STREAM_CHECKPOINT), remoteRaw_, 0u, option)) return false;
    directOriginKnown_ = false;
    checkpointSequence_ = pendingFrame_.sequence;
    checkpointObservationRequested_ = observe && !(option & DELTA_AUTO_STOP_POLICY);
    checkpointRequestsDirectRetry_ = (option & DELTA_DIRECT_RETRY_CAPABILITY) != 0u;
    memcpy(checkpointTargetRaw_, remoteRaw_, sizeof(checkpointTargetRaw_));
    checkpointTargetGeneration_ = targetGeneration_;
    checkpointToleranceRaw_ = DELTA_CHECKPOINT_PRECISION_RAW;
    if (!(option & DELTA_AUTO_STOP_POLICY)) {
      if (auto15) checkpointToleranceRaw_ = DELTA_AUTO15_TOLERANCE_RAW;
      else if (boxPermission == DELTA_CHECKPOINT_AUTO9)
        checkpointToleranceRaw_ = DELTA_CHECKPOINT_AUTO9_TOLERANCE_RAW;
      else if (boxPermission == DELTA_CHECKPOINT_BOX15)
        checkpointToleranceRaw_ = DELTA_CHECKPOINT_BOX15_TOLERANCE_RAW;
      else if (boxTolerance) checkpointToleranceRaw_ = DELTA_CHECKPOINT_BOX_TOLERANCE_RAW;
    }
    checkpointStartedMs_ = millis();
    checkpointTimeoutMs_ = finePickup20 && !centerCapability ? DELTA_PICKUP_FINE_TIMEOUT_MS :
      (option & DELTA_STREAM_OPTION_AUTOMATIC)
      ? DELTA_AUTO_CHECKPOINT_TIMEOUT_MS : CHECKPOINT_TIMEOUT_MS;
    checkpointActive_ = true;
    return true;
  }
  bool sendStreamSetpoint(const uint16_t raw[3], uint16_t moveTimeMs,
                          uint16_t option,bool approach5=false) {
    if (pointActive_) return false;
    if(approach5 && ((option & DELTA_STREAM_SETPOINT_COARSE) ||
        !(option & DELTA_STREAM_OPTION_AUTOMATIC) ||
        (option & DELTA_STREAM_OPTION_MODE_MASK)!=DELTA_STREAM_OPTION_XYZ_MANUAL ||
        moveTimeMs<DELTA_APPROACH_MOVE_MS || !approachDispatchReady()))return false;
    if ((option & DELTA_STREAM_SETPOINT_COARSE) &&
        (moveTimeMs < COARSE_MIN_MOVE_MS || !coarseDispatchReady())) return false;
    if (!sendCommand(static_cast<uint8_t>(approach5 ? DELTA_MSG_APPROACH_SETPOINT : DELTA_MSG_STREAM_SETPOINT),
                     raw, moveTimeMs, option)) return false;
    if (memcmp(raw, remoteRaw_, sizeof(remoteRaw_)) != 0) ++targetGeneration_;
    directOriginKnown_ = false; // a fresh DIRECT barrier must follow any setpoint
    return true;
  }

  bool pointActive() const {return pointActive_;}
  void cancelPoint() {
    pointActive_=pointEvent_=false;pointStartType_=0u;directOriginKnown_=false;
    directRetryHolding_=directRetryReady_=directRetryUsed_=directRetryResumeSent_=false;
  }
  bool directRetryReady() const { return pointActive_ && directRetryReady_; }
  const uint16_t *directRetryOriginRaw() const { return directRetryOrigin_; }
  bool sendDirectRetry(const uint16_t goal[3], uint16_t duration, uint16_t option) {
    if (!directRetryReady() || !directRetryHolding_ || !directRetryAllowed_ ||
        !armed() || !homeValid() || !streamActive() || remoteFault() || dryRun() ||
        option!=pointOption_ || memcmp(goal,pointGoal_,sizeof(pointGoal_)) ||
        duration<pointDurationMs_ || duration<DELTA_DIRECT_PROFILE250_MIN_DURATION_MS ||
        duration<deltaDirectMinimumMs(directRetryOrigin_,goal) ||
        duration>DELTA_DIRECT_MAX_DURATION_MS ||
        millis()-directRetryReceivedMs_>DELTA_DIRECT_RETRY_ORIGIN_MAX_AGE_MS) return false;
    if (!sendCommand(DELTA_MSG_DIRECT_RETRY,goal,duration,option)) return false;
    directRetryResumeSent_=true;
    directRetryReady_=false; // ACK finishes the hold; duplicate READY cannot resume twice
    pointDurationMs_=duration;
    return true;
  }
  uint16_t directRetryDurationFloor() const { return (uint16_t)pointDurationMs_; }
  bool sendPoint(const uint16_t goal[3],uint16_t duration,uint16_t option) {
    return sendPointLike(DELTA_MSG_POINT_START,goal,duration,option);
  }
  bool sendDirect(const uint16_t goal[3],uint16_t duration,uint16_t option) {
    if (!directOriginKnown() || !deltaDirectMinimumMs(directOriginRaw_,goal) ||
        duration<deltaDirectMinimumMs(directOriginRaw_,goal) ||
        duration>DELTA_DIRECT_MAX_DURATION_MS) return false;
    return sendPointLike(DELTA_MSG_DIRECT_START,goal,duration,option);
  }
  bool sendValidatedDirect(const uint16_t goal[3],uint16_t duration,uint16_t option,
                           bool delayedLiftProbe=false,bool finalReadTransit=false) {
    if (!directOriginKnown() || !deltaDirectMinimumMs(directOriginRaw_,goal) ||
        duration<deltaDirectMinimumMs(directOriginRaw_,goal) ||
        duration>DELTA_DIRECT_MAX_DURATION_MS) return false;
    if (finalReadTransit && !delayedLiftProbe) return false;
    return sendPointLike((uint8_t)(finalReadTransit ? DELTA_MSG_DIRECT_FINAL_READ_START :
                         delayedLiftProbe ? DELTA_MSG_DIRECT_LIFT_START :
                         DELTA_MSG_DIRECT_PATH_START),goal,duration,option);
  }
  bool sendEndpointRetry(const uint16_t nominal[3],uint16_t duration,uint16_t option,bool steps=false) {
    uint16_t effective[3];
    if (!directOriginKnown() || !deltaEndpointRetryPlan(directOriginRaw_,nominal,effective) ||
        duration<DELTA_DIRECT_PROFILE250_MIN_DURATION_MS ||
        duration<deltaDirectMinimumMs(directOriginRaw_,effective)) return false;
    if(steps && duration<deltaEndpointStepBudget(directOriginRaw_,effective))return false;
    return sendPointLike((uint8_t)(steps?DELTA_MSG_ENDPOINT_STEP_RETRY:DELTA_MSG_ENDPOINT_RETRY),nominal,duration,option);
  }

 private:
  bool sendPointLike(uint8_t type,const uint16_t goal[3],uint16_t duration,uint16_t option) {
    if (pointActive_ || pointEvent_ || checkpointActive_ || !armed() || !homeValid() ||
        !streamActive() || dryRun() || remoteFault() ||
        ((remoteFlags_ & DELTA_FLAG_FINE_REQUIRED) && type != DELTA_MSG_DIRECT_START &&
          type != DELTA_MSG_DIRECT_PATH_START && type != DELTA_MSG_DIRECT_LIFT_START &&
          type != DELTA_MSG_DIRECT_FINAL_READ_START && type != DELTA_MSG_ENDPOINT_RETRY && type != DELTA_MSG_ENDPOINT_STEP_RETRY) ||
        !(option & DELTA_STREAM_OPTION_AUTOMATIC)) return false;
    if (!sendCommand(type,goal,duration,option)) return false;
    directOriginKnown_ = false;
    pointActive_=true;pointStartedMs_=pointLeaseMs_=millis();pointDurationMs_=duration;
    directRetryHolding_=directRetryReady_=directRetryUsed_=directRetryResumeSent_=false;
    pointSequence_=pendingFrame_.sequence;pointOption_=option;pointStartType_=type;
    memcpy(pointGoal_,goal,sizeof(pointGoal_));return true;
  }

 public:
  bool consumePoint(DeltaResultCode &result) {
    if (!pointEvent_) return false;
    pointEvent_=false;result=pointResult_;return true;
  }

  void printStatus() const {
    Serial.print(F("[STREAM LINK] ready=")); Serial.print(ready_);
    Serial.print(F(" wait=")); Serial.print(waitingResponse_);
    Serial.print(F(" flags=0x")); Serial.print(remoteFlags_, HEX);
    Serial.print(F(" dry=")); Serial.print(dryRun());
    Serial.print(F(" home=")); Serial.print(homeValid());
    Serial.print(F(" armed=")); Serial.print(armed());
    Serial.print(F(" stream=")); Serial.print(streamActive());
    Serial.print(F(" raw=")); Serial.print(remoteRaw_[0]);
    Serial.print(','); Serial.print(remoteRaw_[1]);
    Serial.print(','); Serial.print(remoteRaw_[2]);
    Serial.print(F(" valid/crc/session/unexpected/retry/fault/sp_ack="));
    Serial.print(validResponses_); Serial.print('/');
    Serial.print(crcErrors_); Serial.print('/');
    Serial.print(sessionErrors_); Serial.print('/');
    Serial.print(unexpectedResponses_); Serial.print('/');
    Serial.print(retries_); Serial.print('/');
    Serial.print(linkFaults_); Serial.print('/');
    Serial.println(setpointAcks_);
    Serial.print(F("[UART TIMING] reply_last_ms=")); Serial.print(lastReplyMs_);
    Serial.print(F(" reply_max_ms=")); Serial.println(maxReplyMs_);
    Serial.print(F("[STREAM CADENCE] coarse_ack_guard_ms=")); Serial.print(COARSE_ACK_GUARD_MS);
    Serial.print(F(" ack_age_ms="));
    if (streamAckClockKnown_) Serial.println(streamAckAgeMs());
    else Serial.println(F("SIN_ACK_STREAM"));
    Serial.print(F("[ENCODER LINK] known=")); Serial.print(encoderKnown());
    Serial.print(F(" age_ms="));
    if (encoderKnown()) Serial.print(encoderAgeMs());
    else Serial.print(F("SIN_DATO"));
    Serial.print(F(" dt_ms=")); Serial.print(encoderIntervalMs());
    Serial.print(F(" seq=")); Serial.print(encoderSequence());
    Serial.print(F(" fast=")); Serial.println(encoderFastFresh());
  }

 private:
  static constexpr uint32_t HEARTBEAT_PERIOD_MS = 250u;
  static constexpr uint32_t RESPONSE_TIMEOUT_MS = 60u;
  static constexpr uint32_t DIRECT_RESPONSE_TIMEOUT_MS = 220u;
  static constexpr uint32_t LINK_TIMEOUT_MS = 750u;
  static constexpr uint32_t RECONNECT_DELAY_MS = 1000u;
  static constexpr uint8_t MAX_ATTEMPTS = 3u;
  static constexpr uint32_t CHECKPOINT_TIMEOUT_MS = DELTA_CHECKPOINT_TIMEOUT_MS;

  HardwareSerial &port_;
  uint32_t session_ = 0u;
  uint16_t sequence_ = 0u;
  bool ready_ = false;
  uint32_t sessionGeneration_ = 0u;
  bool gripperStateKnown_ = false;
  uint16_t gripperStateUs_[3] = {};
  uint16_t gripperStateChannel_ = 0u;
  uint8_t gripperStateFlags_ = DELTA_FLAG_LINK_STALE;
  uint32_t gripperStateMs_ = 0u;
  bool waitingResponse_ = false;
  uint8_t pendingAttempts_ = 0u;
  uint32_t pendingSentMs_ = 0u;
  uint32_t pendingFirstSentMs_ = 0u;
  uint32_t lastReplyMs_ = 0u;
  uint32_t maxReplyMs_ = 0u;
  uint32_t lastCommandSentMs_ = 0u;
  uint32_t lastValidResponseMs_ = 0u;
  uint32_t nextHelloMs_ = 0u;
  DeltaLinkFrame pendingFrame_ = {};
  uint8_t remoteFlags_ = DELTA_FLAG_LINK_STALE;
  uint16_t remoteRaw_[3] = {0u, 0u, 0u};
  uint16_t directOriginRaw_[3] = {0u, 0u, 0u};
  bool directOriginKnown_ = false;
  uint32_t targetGeneration_ = 0u, checkpointTargetGeneration_ = 0u;
  bool pointActive_=false,pointEvent_=false;
  bool directRetryAllowed_=false, checkpointRequestsDirectRetry_=false;
  bool checkpointObservationRequested_=false;
  bool directRetryHolding_=false,directRetryReady_=false,directRetryUsed_=false;
  bool directRetryResumeSent_=false;
  uint32_t directRetryStartedMs_=0u,directRetryReceivedMs_=0u;
  uint16_t directRetryOrigin_[3]={};
  uint8_t pointStartType_=0u;
  DeltaResultCode pointResult_=DELTA_RESULT_OK;
  uint16_t pointGoal_[3]={},pointSequence_=0u,pointOption_=0u;
  uint32_t pointStartedMs_=0u,pointLeaseMs_=0u,pointDurationMs_=0u;
  bool completionEvent_ = false;
  uint8_t completedCommandType_ = 0u;
  DeltaResultCode completedResult_ = DELTA_RESULT_OK;
  uint8_t rxBytes_[DELTA_LINK_FRAME_SIZE] = {};
  size_t rxIndex_ = 0u;
  uint32_t validResponses_ = 0u;
  uint32_t crcErrors_ = 0u;
  uint32_t sessionErrors_ = 0u;
  uint32_t unexpectedResponses_ = 0u;
  uint32_t retries_ = 0u;
  uint32_t linkFaults_ = 0u;
  uint32_t setpointAcks_ = 0u;
  bool streamAckClockKnown_ = false;
  uint32_t lastStreamAckMs_ = 0u;
  uint16_t lastStateSequence_ = 0u;
  bool stateSequenceKnown_ = false;
  bool encoderKnown_ = false;
  uint16_t encoderSequence_ = 0u;
  uint32_t encoderReceivedMs_ = 0u;
  int32_t encoderPosition_ = 0;
  uint16_t beltSpeedTenths_ = 0u;
  uint16_t encoderIntervalMs_ = 0u;
  struct EncoderSample { uint32_t ms; int32_t position; };
  EncoderSample encoderHistory_[16] = {};
  uint8_t encoderHistoryCount_ = 0u;
  bool checkpointActive_ = false;
  bool checkpointEvent_ = false;
  uint16_t checkpointSequence_ = 0u;
  uint16_t checkpointTargetRaw_[3] = {};
  uint16_t checkpointToleranceRaw_ = DELTA_CHECKPOINT_PRECISION_RAW;
  uint16_t checkpointMeasuredRaw_[3] = {};
  uint32_t checkpointStartedMs_ = 0u;
  uint32_t checkpointTimeoutMs_ = CHECKPOINT_TIMEOUT_MS;
  bool recoveryActive_ = false, recoveryEvent_ = false;
  uint16_t recoverySequence_ = 0u;
  uint32_t recoveryStartedMs_ = 0u;
  DeltaResultCode recoveryResult_ = DELTA_RESULT_FEEDBACK_FAILED;
  DeltaResultCode checkpointResult_ = DELTA_RESULT_OK;
  bool feedbackKnown_ = false;
  uint16_t feedbackRaw_[3] = {};
  uint32_t feedbackReceivedMs_ = 0u;
  uint32_t feedbackBaseAge_ = 0u;

  bool flag(uint8_t mask) const { return (remoteFlags_ & mask) != 0u; }
  uint32_t responseTimeoutMs() const {
    /* DIRECT prepara y relee hasta tres HTS antes del ACK. No retransmitir la
       misma orden mientras esa transaccion valida sigue en curso. */
    return (pendingFrame_.type == DELTA_MSG_DIRECT_START ||
            pendingFrame_.type == DELTA_MSG_DIRECT_PATH_START ||
            pendingFrame_.type == DELTA_MSG_DIRECT_LIFT_START ||
            pendingFrame_.type == DELTA_MSG_DIRECT_FINAL_READ_START ||
            pendingFrame_.type == DELTA_MSG_ENDPOINT_RETRY ||
            pendingFrame_.type == DELTA_MSG_ENDPOINT_STEP_RETRY ||
            pendingFrame_.type == DELTA_MSG_DIRECT_RETRY)
      ? DIRECT_RESPONSE_TIMEOUT_MS : RESPONSE_TIMEOUT_MS;
  }
  uint16_t nextSequence() {
    ++sequence_;
    if (sequence_ == 0u) ++sequence_;
    return sequence_;
  }

  void startNewSession() {
    cancelPoint();
    directRetryAllowed_=false;
    streamAckClockKnown_ = false;
    ++sessionGeneration_;
    gripperStateKnown_ = false;
    encoderKnown_ = feedbackKnown_ = false;
    encoderHistoryCount_ = 0u;
    cancelCheckpoint();
    stateSequenceKnown_ = false;
    session_ = micros() ^ (millis() << 16) ^ 0x57EA4D21u;
    if (session_ == 0u) session_ = 1u;
    sequence_ = 0u;
    ready_ = false;
    completionEvent_ = false;
    remoteFlags_ = DELTA_FLAG_LINK_STALE;
    DeltaLinkFrame hello = {};
    hello.type = DELTA_MSG_HELLO;
    hello.session = session_;
    hello.sequence = nextSequence();
    queueAndTransmit(hello);
    Serial.print(F("[STREAM LINK] HELLO v3 session=0x"));
    Serial.println(session_, HEX);
  }

  void invalidateLink() {
    cancelPoint();
    directRetryAllowed_=false;
    streamAckClockKnown_ = false;
    recoveryActive_ = false;
    recoveryEvent_ = false;
    gripperStateKnown_ = false;
    ready_ = false;
    waitingResponse_ = false;
    pendingAttempts_ = 0u;
    completionEvent_ = false;
    remoteFlags_ = DELTA_FLAG_LINK_STALE;
    encoderKnown_ = false;
    encoderHistoryCount_ = 0u;
    cancelCheckpoint();
  }

  void cancelCheckpoint() {
    checkpointActive_ = checkpointEvent_ = false;
    directOriginKnown_ = false;
    checkpointToleranceRaw_ = DELTA_CHECKPOINT_PRECISION_RAW;
  }

  bool pointExpired(uint32_t now) const {
    return directRetryHolding_
      ? now-directRetryStartedMs_>DELTA_DIRECT_RETRY_TIMEOUT_MS
      : now-pointStartedMs_>pointDurationMs_+2000u;
  }

  bool sendSimple(uint8_t type) {
    const uint16_t raw[3] = {0u, 0u, 0u};
    return sendCommand(type, raw, 0u, 0u);
  }

  bool sendCommand(uint8_t type, const uint16_t raw[3], uint16_t moveTimeMs,
                   uint16_t requestStatus) {
    if (waitingResponse_ || (type != DELTA_MSG_HELLO && !ready_)) return false;
    DeltaLinkFrame frame = {};
    frame.type = type;
    frame.session = session_;
    frame.sequence = nextSequence();
    memcpy(frame.raw, raw, sizeof(frame.raw));
    frame.moveTimeMs = moveTimeMs;
    frame.status = requestStatus;
    queueAndTransmit(frame);
    return true;
  }

  void queueAndTransmit(DeltaLinkFrame frame) {
    deltaPrepareFrame(&frame);
    pendingFrame_ = frame;
    waitingResponse_ = true;
    pendingAttempts_ = 1u;
    pendingSentMs_ = millis();
    pendingFirstSentMs_ = pendingSentMs_; // Retries must not reset total latency.
    lastCommandSentMs_ = pendingSentMs_;
    transmit(pendingFrame_);
  }

  void transmit(const DeltaLinkFrame &frame) {
    const size_t written = port_.write(
      reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
    port_.flush();
    if (written != sizeof(frame)) {
      Serial.print(F("[STREAM TX ERROR] bytes=")); Serial.println(written);
    }
  }

  void readIncomingBytes() {
    while (port_.available() > 0) {
      const uint8_t value = static_cast<uint8_t>(port_.read());
      if (rxIndex_ == 0u) {
        if (value == DELTA_LINK_SOF_A) rxBytes_[rxIndex_++] = value;
      } else if (rxIndex_ == 1u) {
        if (value == DELTA_LINK_SOF_B) rxBytes_[rxIndex_++] = value;
        else {
          rxIndex_ = value == DELTA_LINK_SOF_A ? 1u : 0u;
          if (rxIndex_ == 1u) rxBytes_[0] = value;
        }
      } else {
        rxBytes_[rxIndex_++] = value;
        if (rxIndex_ == DELTA_LINK_FRAME_SIZE) {
          DeltaLinkFrame frame{};
          memcpy(&frame, rxBytes_, sizeof(frame));
          rxIndex_ = 0u;
          processResponse(frame);
        }
      }
    }
  }

  void processResponse(const DeltaLinkFrame &frame) {
    if (!deltaFrameValid(&frame)) { ++crcErrors_; return; }
    if (frame.session != session_) { ++sessionErrors_; return; }
    if (frame.type == DELTA_MSG_DIRECT_RETRY_STATE) {
      const uint8_t flags=deltaStatusFlags(frame.status);
      const uint8_t required=DELTA_FLAG_ARMED|DELTA_FLAG_HOME_VALID|
        DELTA_FLAG_STREAM_ACTIVE|DELTA_FLAG_BUSY;
      const DeltaResultCode result=deltaStatusResult(frame.status);
      if (!ready_ || !pointActive_ || !directRetryAllowed_ ||
          (pointStartType_!=DELTA_MSG_DIRECT_LIFT_START &&
           pointStartType_!=DELTA_MSG_DIRECT_FINAL_READ_START) || frame.sequence!=pointSequence_ ||
          pointExpired(millis()) || (flags & required)!=required ||
          (flags & (DELTA_FLAG_FAULT|DELTA_FLAG_LINK_STALE|DELTA_FLAG_DRY_RUN))) {
        ++unexpectedResponses_;return;
      }
      if (result==DELTA_RESULT_BUSY && frame.moveTimeMs==0u && !directRetryUsed_) {
        directRetryHolding_=directRetryUsed_=true;
        directRetryStartedMs_=millis();directRetryReady_=false;
      } else if (result==DELTA_RESULT_OK && frame.moveTimeMs==7u &&
                 directRetryHolding_ && !directRetryReady_ && !directRetryResumeSent_) {
        for (uint8_t i=0u;i<3u;++i)
          if (frame.raw[i]<205u || frame.raw[i]>700u) {++unexpectedResponses_;return;}
        memcpy(directRetryOrigin_,frame.raw,sizeof(directRetryOrigin_));
        directRetryReady_=true;directRetryReceivedMs_=millis();
      } else {++unexpectedResponses_;return;}
      // Unsolicited state NEVER satisfies a concurrent ACK or refreshes a lease.
      return;
    }
    if (frame.type == DELTA_MSG_CHECKPOINT_RESULT) {
      if (!ready_ || !checkpointActive_ || frame.sequence != checkpointSequence_ ||
          frame.moveTimeMs > 7u) { ++unexpectedResponses_; return; }
      DeltaResultCode result = deltaStatusResult(frame.status);
      if (result != DELTA_RESULT_OK && result != DELTA_RESULT_OBSERVED && result != DELTA_RESULT_FEEDBACK_FAILED) {
        ++unexpectedResponses_; return;
      }
      // service() drains RX before its periodic timeout check. A buffered
      // result must not turn an already expired verification into permission.
      // Unsigned elapsed time also preserves this deadline across millis wrap.
      const bool expired = millis() - checkpointStartedMs_ > checkpointTimeoutMs_;
      if (expired) result = DELTA_RESULT_FEEDBACK_FAILED;
      if (result == DELTA_RESULT_OK || result == DELTA_RESULT_OBSERVED) {
        const uint8_t flags = deltaStatusFlags(frame.status);
        const uint8_t required = DELTA_FLAG_ARMED | DELTA_FLAG_HOME_VALID |
          DELTA_FLAG_STREAM_ACTIVE;
        bool reached = frame.moveTimeMs == 7u && (flags & required) == required &&
          (checkpointObservationRequested_ == (result == DELTA_RESULT_OBSERVED)) &&
          checkpointTargetGeneration_ == targetGeneration_ &&
          memcmp(remoteRaw_, checkpointTargetRaw_, sizeof(remoteRaw_)) == 0 &&
          !(flags & (DELTA_FLAG_FAULT | DELTA_FLAG_BUSY | DELTA_FLAG_DRY_RUN |
                     DELTA_FLAG_LINK_STALE));
        for (uint8_t i = 0u; i < 3u; ++i) {
          const int difference = static_cast<int>(frame.raw[i]) - checkpointTargetRaw_[i];
          if (checkpointObservationRequested_)
            reached = reached && frame.raw[i] >= 215u && frame.raw[i] <= 690u;
          else reached = reached && difference >= -static_cast<int>(checkpointToleranceRaw_) &&
                         difference <= static_cast<int>(checkpointToleranceRaw_);
        }
        if (!reached) result = DELTA_RESULT_FEEDBACK_FAILED;
      }
      checkpointActive_ = false;
      checkpointEvent_ = true;
      checkpointResult_ = result;
      if (result == DELTA_RESULT_OK || result == DELTA_RESULT_OBSERVED) {
        lastStreamAckMs_ = millis();
        if (checkpointRequestsDirectRetry_) directRetryAllowed_=true;
        streamAckClockKnown_ = true; // final fresh PASS may follow a correction TX
        memcpy(directOriginRaw_, frame.raw, sizeof(directOriginRaw_));
        directOriginKnown_ = true;
      }
      if (expired) memset(checkpointMeasuredRaw_, 0, sizeof(checkpointMeasuredRaw_));
      else memcpy(checkpointMeasuredRaw_, frame.raw, sizeof(checkpointMeasuredRaw_));
      return; // Never clears a concurrent command's waitingResponse or health.
    }
    if (frame.type == DELTA_MSG_FAULT_RECOVERY_RESULT) {
      if (!ready_ || !recoveryActive_ || frame.sequence != recoverySequence_) {
        ++unexpectedResponses_; return;
      }
      const uint8_t flags = deltaStatusFlags(frame.status);
      bool valid = deltaStatusResult(frame.status) == DELTA_RESULT_OK && frame.moveTimeMs == 7u &&
        !(flags & (DELTA_FLAG_ARMED | DELTA_FLAG_HOME_VALID | DELTA_FLAG_BUSY | DELTA_FLAG_FAULT |
                   DELTA_FLAG_STREAM_ACTIVE | DELTA_FLAG_DRY_RUN | DELTA_FLAG_LINK_STALE)) &&
        millis() - recoveryStartedMs_ <= DELTA_FAULT_RECOVERY_TIMEOUT_MS;
      for (uint8_t i = 0u; i < 3u; ++i)
        valid = valid && frame.raw[i] >= 195u && frame.raw[i] <= 700u;
      recoveryActive_ = false;
      recoveryEvent_ = true;
      recoveryResult_ = valid ? DELTA_RESULT_OK : DELTA_RESULT_FEEDBACK_FAILED;
      if (valid) {
        remoteFlags_ = flags;
        feedbackKnown_ = false; // not a HOME/ARM reference
      }
      return; // does not satisfy any heartbeat or concurrent command ACK
    }
    // This solicited snapshot is deliberately separate from DELTA remoteRaw_.
    if (frame.type == DELTA_MSG_GRIP_STATE) {
      if (!ready_ || !waitingResponse_ ||
          pendingFrame_.type != DELTA_MSG_GRIP_STATE_REQUEST ||
          frame.sequence != pendingFrame_.sequence) {
        ++unexpectedResponses_;
        return;
      }
      DeltaResultCode result = deltaStatusResult(frame.status);
      bool valid = result == DELTA_RESULT_OK && frame.moveTimeMs <= 4u;
      for (uint8_t i = 0u; i < 3u; ++i)
        valid = valid && frame.raw[i] >= GRIP_MIN_US[i] && frame.raw[i] <= GRIP_MAX_US[i];
      gripperStateKnown_ = valid;
      if (valid) {
        memcpy(gripperStateUs_, frame.raw, sizeof(gripperStateUs_));
        gripperStateChannel_ = frame.moveTimeMs;
        gripperStateFlags_ = deltaStatusFlags(frame.status);
        gripperStateMs_ = millis();
        lastStateSequence_ = frame.sequence;
        stateSequenceKnown_ = true;
        remoteFlags_ = gripperStateFlags_;
      } else if (result == DELTA_RESULT_OK) result = DELTA_RESULT_BAD_FRAME;
      ++validResponses_;
      lastValidResponseMs_ = millis();
      lastReplyMs_ = millis() - pendingFirstSentMs_;
      if (lastReplyMs_ > maxReplyMs_) maxReplyMs_ = lastReplyMs_;
      waitingResponse_ = false;
      pendingAttempts_ = 0u;
      completedCommandType_ = DELTA_MSG_GRIP_STATE_REQUEST;
      completedResult_ = result;
      completionEvent_ = true;
      return;
    }
    /* These telemetry frames do not satisfy an ACK or renew command health. */
    if (frame.type == DELTA_MSG_ENCODER) {
      if (!ready_ || frame.status != 1u || frame.moveTimeMs == 0u) return;
      if (encoderKnown_ &&
          static_cast<int16_t>(frame.sequence - encoderSequence_) <= 0) return;
      encoderSequence_ = frame.sequence;
      encoderPosition_ = (int32_t)((uint32_t)frame.raw[0] |
                                  ((uint32_t)frame.raw[1] << 16));
      beltSpeedTenths_ = frame.raw[2];
      encoderIntervalMs_ = frame.moveTimeMs;
      encoderReceivedMs_ = millis();
      encoderKnown_ = true;
      if (encoderIntervalMs_ > 200u) encoderHistoryCount_ = 0u;
      else {
        if (encoderHistoryCount_ &&
            encoderReceivedMs_ == encoderHistory_[encoderHistoryCount_ - 1u].ms) {
          encoderHistory_[encoderHistoryCount_ - 1u].position = encoderPosition_;
        } else {
          if (encoderHistoryCount_ == 16u) {
            memmove(encoderHistory_, encoderHistory_ + 1u,
                    15u * sizeof(EncoderSample));
            --encoderHistoryCount_;
          }
          encoderHistory_[encoderHistoryCount_++] = {encoderReceivedMs_, encoderPosition_};
        }
      }
      return;
    }
    if (frame.type == DELTA_MSG_FEEDBACK) {
      if (!ready_ || (stateSequenceKnown_ &&
          static_cast<int16_t>(frame.sequence - lastStateSequence_) < 0)) return;
      memcpy(feedbackRaw_, frame.raw, sizeof(feedbackRaw_));
      feedbackKnown_ = frame.status == 7u;
      feedbackBaseAge_ = frame.moveTimeMs;
      feedbackReceivedMs_ = millis();
      return;
    }
    // Un ACK atrasado o una trama de comando no puede cambiar home/busy/raw.
    const bool expectedStatus = waitingResponse_ &&
      pendingFrame_.type == DELTA_MSG_STATUS_REQUEST &&
      frame.sequence == pendingFrame_.sequence;
    const bool matchingReply = waitingResponse_ &&
      frame.sequence == pendingFrame_.sequence &&
      (frame.type == DELTA_MSG_ACK || frame.type == DELTA_MSG_NACK ||
       (frame.type == DELTA_MSG_STATUS && expectedStatus));
    if (!matchingReply && frame.type != DELTA_MSG_STATUS) {
      ++unexpectedResponses_;
      return;
    }
    if (frame.type == DELTA_MSG_STATUS && stateSequenceKnown_ &&
        static_cast<int16_t>(frame.sequence - lastStateSequence_) < 0) {
      ++unexpectedResponses_;
      return;
    }
    lastStateSequence_ = frame.sequence;
    stateSequenceKnown_ = true;
    ++validResponses_;
    lastValidResponseMs_ = millis();
    remoteFlags_ = deltaStatusFlags(frame.status);
    if (!armed() || !homeValid() || !streamActive() || remoteFault() || dryRun())
      directOriginKnown_ = false;
    DeltaResultCode result = deltaStatusResult(frame.status);
    if (frame.type == DELTA_MSG_NACK && result == DELTA_RESULT_OK)
      result = DELTA_RESULT_BAD_FRAME;
    if (memcmp(remoteRaw_, frame.raw, sizeof(remoteRaw_)) != 0)
      directOriginKnown_ = false;
    memcpy(remoteRaw_, frame.raw, sizeof(remoteRaw_));

    if (frame.type == DELTA_MSG_NACK && result == DELTA_RESULT_BAD_SESSION) {
      Serial.println(F("[STREAM LINK] BAD_SESSION; nueva sesion."));
      invalidateLink();
      nextHelloMs_ = millis();
      return;
    }
    if (frame.type == DELTA_MSG_STATUS && !expectedStatus) return;
    if (!waitingResponse_ || frame.sequence != pendingFrame_.sequence) {
      ++unexpectedResponses_;
      return;
    }

    const uint8_t completedType = pendingFrame_.type;
    if (pointActive_ && (completedType==pointStartType_ ||
                        completedType==DELTA_MSG_DIRECT_RETRY ||
                        completedType==DELTA_MSG_POINT_KEEPALIVE)) {
      if (frame.type!=DELTA_MSG_ACK || result!=DELTA_RESULT_OK ||
          !armed() || !homeValid() || !streamActive() || remoteFault() || dryRun() ||
          pointExpired(millis())) {
        pointActive_=false;pointEvent_=true;
        pointResult_=result==DELTA_RESULT_OK ? DELTA_RESULT_FEEDBACK_FAILED : result;
      } else if (completedType==DELTA_MSG_DIRECT_RETRY) {
        directRetryHolding_=false;pointStartedMs_=millis();
      } else if (completedType==DELTA_MSG_POINT_KEEPALIVE && !busy() && !directRetryHolding_) {
        pointActive_=false;pointEvent_=true;
        pointResult_=memcmp(frame.raw,pointGoal_,sizeof(pointGoal_))==0
          ? DELTA_RESULT_OK : DELTA_RESULT_FEEDBACK_FAILED;
      }
    }
    lastReplyMs_ = millis() - pendingFirstSentMs_;
    if (lastReplyMs_ > maxReplyMs_) maxReplyMs_ = lastReplyMs_;
    waitingResponse_ = false;
    pendingAttempts_ = 0u;
    if (frame.type == DELTA_MSG_ACK && result == DELTA_RESULT_OK) {
      if (completedType == DELTA_MSG_STREAM_BEGIN || completedType == DELTA_MSG_STREAM_SETPOINT ||
          completedType == DELTA_MSG_APPROACH_SETPOINT) {
        lastStreamAckMs_ = millis();
        streamAckClockKnown_ = true; // includes fine/hold ACKs, not just coarse
      } else if (completedType == DELTA_MSG_STREAM_END || completedType == DELTA_MSG_DISARM) {
        streamAckClockKnown_ = false;
      }
    }
    if (completedType == DELTA_MSG_HELLO && frame.type == DELTA_MSG_ACK &&
        result == DELTA_RESULT_OK) {
      ready_ = true;
      Serial.print(F("[STREAM LINK] READY v3 dry_run="));
      Serial.println(dryRun());
    }
    if ((completedType == DELTA_MSG_STREAM_CHECKPOINT || completedType == DELTA_MSG_ENDPOINT_REC200_PROBE || completedType == DELTA_MSG_ENDPOINT_STEP_PROBE ||
         completedType == DELTA_MSG_CENTER_RECOVERY_PROBE || completedType == DELTA_MSG_CENTER_OBSERVE ||
         completedType == DELTA_MSG_PICKUP_FINE20_PROBE || completedType == DELTA_MSG_PICKUP_FINE20_CHECKPOINT ||
         completedType == DELTA_MSG_DIRECT_REPEAT5_PROBE || completedType == DELTA_MSG_APPROACH_STEPS_PROBE) &&
        result != DELTA_RESULT_OK) {
      checkpointActive_ = false;
      checkpointEvent_ = true;
      checkpointResult_ = result;
      memset(checkpointMeasuredRaw_, 0, sizeof(checkpointMeasuredRaw_));
    }
    if (completedType == DELTA_MSG_FAULT_RECOVER && result != DELTA_RESULT_OK) {
      recoveryActive_ = false;
      recoveryEvent_ = true;
      recoveryResult_ = result;
    }
    if (completedType != DELTA_MSG_POINT_START && completedType != DELTA_MSG_DIRECT_START &&
        completedType != DELTA_MSG_DIRECT_PATH_START && completedType != DELTA_MSG_DIRECT_LIFT_START &&
        completedType != DELTA_MSG_DIRECT_FINAL_READ_START &&
        completedType != DELTA_MSG_ENDPOINT_RETRY &&
        completedType != DELTA_MSG_ENDPOINT_STEP_RETRY &&
        completedType != DELTA_MSG_DIRECT_RETRY &&
        completedType != DELTA_MSG_POINT_KEEPALIVE &&
        completedType != DELTA_MSG_HEARTBEAT &&
        completedType != DELTA_MSG_HELLO) {
      completedCommandType_ = completedType;
      completedResult_ = result;
      completionEvent_ = true;
    }
    if ((completedType == DELTA_MSG_STREAM_SETPOINT || completedType == DELTA_MSG_APPROACH_SETPOINT) &&
        frame.type == DELTA_MSG_ACK && result == DELTA_RESULT_OK) {
      ++setpointAcks_;
    } else if (completedType != DELTA_MSG_POINT_KEEPALIVE && completedType != DELTA_MSG_HEARTBEAT &&
               !((completedType == DELTA_MSG_GRIP_SET ||
                  completedType == DELTA_MSG_GRIP_GROUP_SET) &&
                 frame.type == DELTA_MSG_ACK && result == DELTA_RESULT_OK)) {
      if (frame.type == DELTA_MSG_ACK) Serial.print(F("[ACK] "));
      else if (frame.type == DELTA_MSG_NACK) Serial.print(F("[NACK] "));
      else Serial.print(F("[STATUS RESPONSE] "));
      Serial.print(F("type=")); Serial.print(completedType);
      Serial.print(F(" seq=")); Serial.print(frame.sequence);
      Serial.print(F(" result=")); Serial.print(static_cast<unsigned>(result));
      Serial.print(F(" flags=0x")); Serial.println(remoteFlags_, HEX);
      if ((completedType == DELTA_MSG_STREAM_SETPOINT || completedType == DELTA_MSG_APPROACH_SETPOINT) &&
          frame.type == DELTA_MSG_NACK) {
        Serial.print(F("[STREAM TX REJECT] move_ms=")); Serial.print(pendingFrame_.moveTimeMs);
        Serial.print(F(" option=0x")); Serial.print(pendingFrame_.status, HEX);
        Serial.print(F(" ack_age_ms=")); Serial.println(streamAckAgeMs());
      }
    }
  }
};

#endif
