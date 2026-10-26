/**
 * @file auto_pickup_config.h
 * @brief Parametros de la secuencia automatica, tiempos y perfiles de movimiento.
 * Aproximacion incremental y descenso fino preceden a elevacion y traslado
 * directos. Las tolerancias de aceptacion son distintas de las envolventes
 * de recuperacion. Los parametros no certifican seguridad mecanica.
 */
#ifndef AUTO_PICKUP_CONFIG_H
#define AUTO_PICKUP_CONFIG_H
#include "auto_pickup_sequence.h"
#include "auto_stop_policy.h"
#include "auto_speed_profile.h"
#include "delta_stream_protocol.h"
#include "gripper_limits.h"


namespace AutoPickupSetup {
constexpr bool SUPERVISED_TRIAL_ENABLED = true;
constexpr uint32_t GAMEPAD_RUNTIME_MAX_AGE_MS = 500u; // AUTO only; preparation/manual retain their guards
constexpr float WAIT_Z_MM = -284.94f; // latest origin capture RAW408/408/408; not safety HOME
constexpr bool COMBINED_APPROACH_ENABLED = true;
constexpr bool PICKUP_POSE_DURING_APPROACH_ENABLED = true; // X1270->1510 during empty DLS approach
constexpr bool COMBINED_LOADED_TRANSFER_ENABLED = true;
constexpr bool LOCAL_POINT_TRANSIT_ENABLED = true; // B/C capability required before PWM

constexpr bool TWO_DIRECT_LOADED_MOVES_ENABLED = true;
constexpr bool RETURN_XY_AT_DROP_Z_ENABLED = true;
constexpr bool ENDPOINT_RETRY_ENABLED = true; // new paired B/C contract, before PWM
constexpr uint16_t WAIT_TILT_US = 1270u;  
constexpr bool POSITION_MOVES_ALL_STAGES = true;
constexpr bool PICKUP_APPROACH_DIRECT_ENABLED = false;
constexpr bool LOWER_PICKUP_DIRECT_ENABLED = false;
constexpr bool PICKUP_APPROACH_STEP5_ENABLED = true;
// Only ALIGN_ABOVE; the final descent/manual retain 0.90 deg and <=4 RAW.
constexpr float APPROACH_JOINT_STEP_DEG = 1.20f;
// Explicit transit-only policy: missing RX and slow intermediate progress are advisory.
// The real final checkpoint still gates every following stage.
constexpr bool TRANSIT_FINAL_READ_ENABLED = true;
inline bool finalReadTransit(AutoPickup::State state) {
  return TRANSIT_FINAL_READ_ENABLED &&
    (state == AutoPickup::State::LIFT || state == AutoPickup::State::TRANSFER ||
     state == AutoPickup::State::LOWER_DROP || state == AutoPickup::State::LIFT_RETURN ||
     state == AutoPickup::State::RETURN_XY || state == AutoPickup::State::RETURN_HOME);
}

constexpr bool DIRECT_PROFILE250_ENABLED = true;
constexpr bool DIRECT_COORDINATED_RETRY_ENABLED = true; // only with PROFILE250
inline bool positionMove(AutoPickup::State state);

constexpr bool POST_PICKUP_LIFT_DELAYED_PROBE_ENABLED = true;

constexpr bool POST_PICKUP_TRANSFER_DELAYED_PROBE_ENABLED = true;
// Apply the same bounded startup profile when returning from the deposit.
// LIFT_RETURN may be only 300 ms: keep its existing short-move timing/profile.
constexpr bool POST_DEPOSIT_RETURN_DELAYED_PROBE_ENABLED = true;
inline bool postPickupDelayedProbe(AutoPickup::State state) {
  if (DIRECT_PROFILE250_ENABLED) return positionMove(state);
  return (POST_PICKUP_LIFT_DELAYED_PROBE_ENABLED && state==AutoPickup::State::LIFT) ||
         (POST_PICKUP_TRANSFER_DELAYED_PROBE_ENABLED && state==AutoPickup::State::TRANSFER) ||
         (POST_DEPOSIT_RETURN_DELAYED_PROBE_ENABLED &&
          (state==AutoPickup::State::RETURN_XY || state==AutoPickup::State::RETURN_HOME));
}
constexpr uint16_t EMPTY_POSITION_RAW_PER_SECOND = 120u;
constexpr uint16_t LOADED_POSITION_RAW_PER_SECOND = 120u;
constexpr float POSITION_APPROACH_MARGIN_MM = 2.0f;
constexpr float POSITION_CLEARANCE_QUANTIZATION_MM = 0.5f;
constexpr float POSITION_PICKUP_MAX_ORIGIN_OFFSET_MM = 12.0f;
constexpr float POSITION_BOX_MAX_ORIGIN_OFFSET_MM = 25.0f;
constexpr uint16_t ROUTE_PREFLIGHT_SAMPLES = 64u;
static_assert(EMPTY_POSITION_RAW_PER_SECOND == DELTA_DIRECT_RAW_PER_SECOND &&
              LOADED_POSITION_RAW_PER_SECOND == DELTA_DIRECT_RAW_PER_SECOND,
              "All endpoint stages retain the operator's 120 RAW/s ceiling");
constexpr bool RIGHT_LIFT_FIRST_ENABLED = true;
// Adjustable trial boundary: midpoint between the measured centre and right
// pickup anchors (46.44 / 139.51). +Y, NOT robot X. No measured box clearance.
constexpr float RIGHT_LIFT_THRESHOLD_Y_MM = 92.975f;
constexpr float APPROACH_CLEARANCE_MM = 20.0f; // provisional clearance above crossed pickup knots
constexpr float TRANSIT_Z_MM = -220.01f; // unchanged loaded transit; small box final rise 7.14 mm
constexpr bool DROP_ABOVE_TRANSIT_CONFIRMED = true; // C final local rise 21.81 mm; full route preflight still required
constexpr bool CONTINUOUS_ENABLED = true; // until menu/cancel/loss/fault, not auto-rearm
static_assert(TRANSIT_Z_MM >= AutoBoxCalibration::MAX_DEPOSIT_Z_MM ||
              DROP_ABOVE_TRANSIT_CONFIRMED,
              "A deposit above transit requires an explicit local approach");
constexpr float PICKUP_X_MM = 0.0f; // pickup at robot X=0; gate line measured below
constexpr float CAMERA_LONGITUDINAL_SIGN = 1.0f;  
constexpr float ROBOT_BELT_X_SIGN = 0.0f; // not needed at pickupX=0; NOT an axis sign
constexpr float ARRIVAL_TOLERANCE_MM = 10.0f; // trial window, not a measurement
constexpr float REVERSE_TOLERANCE_MM = 2.0f;
constexpr float CLOSE_LEAD_MM = 20.0f;  
constexpr float GATE_LINE_FROM_IMAGE_CENTRE_MM = 420.0f;
// Gate close offsets. Sign: + closes LATER, - closes EARLIER. They shift the
// predicted arrival, so the close trigger and the PIECE_LATE guard move together.
// Tune one family at a time: a constant error at any belt speed is mm; an
// error that grows with belt speed is ms (converted with the belt speed at
// piece acceptance). If the gate hits the piece: +10..+20 mm. If still late:
// -10..-20 mm. Each 10 mm is ~0.13 s at 79 mm/s.
constexpr float GATE_CLOSE_OFFSET_MM = -15.0f;         // all pieces, geometry
constexpr float GATE_CLOSE_OFFSET_MS = 0.0f;         // all pieces, time (x belt speed)
constexpr float GATE_CLOSE_OFFSET_PIECE_C_MM = 0.0f; // camera id 1 (pieza_C) only
constexpr float GATE_CLOSE_OFFSET_SMALL_MM = 0.0f;   // camera id 2 (pieza_chica) only
// The 10D experimental 71 mm lead was fitted against the 500 mm datum; with the
// measured 420 mm line it would double-compensate and close on the piece.
constexpr bool GATE_EXPERIMENTAL_LEAD_ENABLED = false;
static_assert(GATE_LINE_FROM_IMAGE_CENTRE_MM >= 200.0f &&
              GATE_LINE_FROM_IMAGE_CENTRE_MM <= 800.0f,
              "Gate line must stay a plausible measured belt distance");
static_assert(GATE_CLOSE_OFFSET_MM >= -150.0f && GATE_CLOSE_OFFSET_MM <= 150.0f &&
              GATE_CLOSE_OFFSET_PIECE_C_MM >= -150.0f && GATE_CLOSE_OFFSET_PIECE_C_MM <= 150.0f &&
              GATE_CLOSE_OFFSET_SMALL_MM >= -150.0f && GATE_CLOSE_OFFSET_SMALL_MM <= 150.0f,
              "Gate mm offsets are corrections, not a new datum (+-150 mm)");
static_assert(GATE_CLOSE_OFFSET_MS >= -1000.0f && GATE_CLOSE_OFFSET_MS <= 1000.0f,
              "Gate ms offset is a correction (+-1000 ms)");
// Cartesian trial limits by route segment; tune here, not in MANUAL/BANDA.
constexpr float APPROACH_SPEED_MM_S = 150.0f;
constexpr float LOADED_TRANSFER_SPEED_MM_S = 150.0f;
constexpr float EMPTY_RETURN_SPEED_MM_S = 150.0f;
constexpr float FINAL_APPROACH_SPEED_MM_S = 40.0f;
constexpr float FINAL_APPROACH_DISTANCE_MM = 20.0f;
constexpr uint16_t RC_RAMP_STEP_US = 150u;  
constexpr uint16_t CARRY_X_RAMP_STEP_US = 25u; // only loaded scoop tilt1510->1090, about510ms
constexpr uint32_t RC_RAMP_PERIOD_MS = 30u; // AUTO only; fixed 80 ms lease renewal unchanged
// Same cadence for all AUTO RC ramps; final commands and dwells unchanged.
constexpr uint32_t GATE_CLOSE_RAMP_PERIOD_MS = 30u;
constexpr uint32_t RC_SETTLE_MS = 300u; // after exact final RC command ACK
constexpr uint32_t DUMP_SETTLE_MS = 500u;
// Extra hold after final Cartesian ACK, before the physical checkpoint starts.
// Only loaded box arrival/discharge height; do not delay pickup/gate timing.
// DIRECT completion is followed by the receiver's 300 ms physical-read settle.
constexpr uint32_t LOADED_CARTESIAN_SETTLE_MS = 0u;
constexpr uint32_t STAGE_TIMEOUT_MS = 15000u;
constexpr uint32_t CARTESIAN_STAGE_TIMEOUT_MS = 30000u;
constexpr bool MAPPING_CONFIRMED = false;
constexpr bool ROUTE_CONFIRMED = false;
constexpr bool RC_TIMING_CONFIRMED = false;
constexpr double ENCODER_COUNTS_PER_MM = AutoGateEncoder::COUNTS_PER_MM; // already x4
static_assert(LOADED_CARTESIAN_SETTLE_MS <= 5000u &&
              LOADED_CARTESIAN_SETTLE_MS + DELTA_AUTO_CHECKPOINT_TIMEOUT_MS < CARTESIAN_STAGE_TIMEOUT_MS,
              "Loaded settle must be 0..5000 ms and leave the shared checkpoint budget");
static_assert(RC_RAMP_STEP_US > 0u && RC_RAMP_STEP_US <= GRIP_AUTO_MAX_STEP_US,
              "RC_RAMP_STEP_US must respect the explicit AUTO protocol bound");
static_assert(RC_RAMP_PERIOD_MS >= 25u && RC_RAMP_PERIOD_MS <= 80u,
              "RC_RAMP_PERIOD_MS must be 25..80 ms; do not extend the lease");
static_assert(GATE_CLOSE_RAMP_PERIOD_MS >= GRIP_GROUP_ACK_GUARD_MS &&
              GATE_CLOSE_RAMP_PERIOD_MS <= RC_RAMP_PERIOD_MS,
              "Gate closure must respect group ACK guard and existing ramp bound");
static_assert(APPROACH_SPEED_MM_S > 0.0f && APPROACH_SPEED_MM_S <= 150.0f &&
              LOADED_TRANSFER_SPEED_MM_S > 0.0f && LOADED_TRANSFER_SPEED_MM_S <= 150.0f &&
              EMPTY_RETURN_SPEED_MM_S > 0.0f && EMPTY_RETURN_SPEED_MM_S <= 150.0f,
              "AUTO trial cruise limits must be 0..150 mm/s (exclusive zero)");
static_assert(FINAL_APPROACH_SPEED_MM_S > 0.0f &&
              FINAL_APPROACH_SPEED_MM_S <= APPROACH_SPEED_MM_S &&
              FINAL_APPROACH_SPEED_MM_S <= LOADED_TRANSFER_SPEED_MM_S &&
              FINAL_APPROACH_SPEED_MM_S <= EMPTY_RETURN_SPEED_MM_S &&
              FINAL_APPROACH_DISTANCE_MM > 0.0f && FINAL_APPROACH_DISTANCE_MM <= 100.0f,
              "AUTO final limit/zone must be positive and no faster than cruise");

inline AutoSpeedProfile::Config speedProfileConfig() {
  return {APPROACH_SPEED_MM_S, LOADED_TRANSFER_SPEED_MM_S, EMPTY_RETURN_SPEED_MM_S,
          FINAL_APPROACH_SPEED_MM_S, FINAL_APPROACH_DISTANCE_MM};
}

inline bool positionStage(AutoPickup::State state) {
  switch (state) {
    case AutoPickup::State::INITIAL_LIFT:
    case AutoPickup::State::ALIGN_ABOVE:
    case AutoPickup::State::LOWER_PICKUP:
    case AutoPickup::State::LIFT:
    case AutoPickup::State::TRANSFER:
    case AutoPickup::State::LOWER_DROP:
    case AutoPickup::State::LIFT_RETURN:
    case AutoPickup::State::RETURN_XY:
    case AutoPickup::State::RETURN_HOME: return true;
    default: return false;
  }
}

inline bool positionMove(AutoPickup::State state) {
  if (state == AutoPickup::State::ALIGN_ABOVE && !PICKUP_APPROACH_DIRECT_ENABLED)
    return false;
  if (state == AutoPickup::State::LOWER_PICKUP && !LOWER_PICKUP_DIRECT_ENABLED)
    return false;
  return POSITION_MOVES_ALL_STAGES ? positionStage(state) :
    (TWO_DIRECT_LOADED_MOVES_ENABLED &&
     (state == AutoPickup::State::LIFT || state == AutoPickup::State::TRANSFER));
}

inline bool verticalPositionStage(AutoPickup::State state) {
  return state == AutoPickup::State::LOWER_PICKUP || state == AutoPickup::State::LIFT ||
    state == AutoPickup::State::LOWER_DROP || state == AutoPickup::State::LIFT_RETURN ||
    (RETURN_XY_AT_DROP_Z_ENABLED && state == AutoPickup::State::RETURN_HOME);
}

inline bool fastApproach(AutoPickup::State state) {
  return PICKUP_APPROACH_STEP5_ENABLED && state==AutoPickup::State::ALIGN_ABOVE &&
         !positionMove(state);
}

inline uint16_t positionDurationFloor(AutoPickup::State state) {
  // 300 ms is a shared, nonblocking read window, never a UART call timeout.
  const bool postPickup=state==AutoPickup::State::LIFT ||
    state==AutoPickup::State::TRANSFER || state==AutoPickup::State::LOWER_DROP ||
    state==AutoPickup::State::LIFT_RETURN || state==AutoPickup::State::RETURN_XY ||
    state==AutoPickup::State::RETURN_HOME;
  return PICKUP_APPROACH_STEP5_ENABLED && postPickup
    ? DELTA_POST_PICKUP_MIN_DURATION_MS : DELTA_DIRECT_PROFILE250_MIN_DURATION_MS;
}

inline float verticalOriginCapMm(AutoPickup::State state) {
  return state == AutoPickup::State::LIFT_RETURN || state == AutoPickup::State::LOWER_DROP
    ? POSITION_BOX_MAX_ORIGIN_OFFSET_MM : POSITION_PICKUP_MAX_ORIGIN_OFFSET_MM;
}

inline uint16_t positionRawPerSecond(AutoPickup::State) {
  return DELTA_DIRECT_RAW_PER_SECOND;
}

inline float positionCruiseMmS(AutoPickup::State state) {
  return state == AutoPickup::State::LOWER_PICKUP ? FINAL_APPROACH_SPEED_MM_S :
    AutoSpeedProfile::cruiseLimit(state, speedProfileConfig());
}

inline uint16_t precheckFlags(AutoPickup::State state) {
  if (state == AutoPickup::State::INITIAL_LIFT || state == AutoPickup::State::ALIGN_ABOVE ||
      state == AutoPickup::State::LOWER_PICKUP || state == AutoPickup::State::LIFT)
    return 0u;
  // LIFT still starts from the precise physical pickup, never from a loose CP.
  return DELTA_CHECKPOINT_OBSERVE;
}

inline uint32_t cartesianSettleMs(AutoPickup::State state) {
  return state == AutoPickup::State::TRANSFER || state == AutoPickup::State::LOWER_DROP
    ? LOADED_CARTESIAN_SETTLE_MS : 0u;
}

inline bool centerRecovery(AutoPickup::State state) {
  return ENDPOINT_RETRY_ENABLED &&
    (state==AutoPickup::State::RETURN_XY || state==AutoPickup::State::RETURN_HOME);
}
inline uint16_t endpointRecoveryTolerance(AutoPickup::State state) {
  return centerRecovery(state) ? DELTA_ENDPOINT_CENTER_BRIDGE_RAW :
    DELTA_ENDPOINT_RETRY_TOLERANCE_RAW;
}

inline uint16_t checkpointRequestFlags(AutoPickup::State state) {
  // Observe transit/box pose without target-precision correction. Centre,
  // pickup approach/descent and the independent real HOME remain precise.
  switch (state) {
    case AutoPickup::State::TRANSFER:
    case AutoPickup::State::LOWER_DROP:
    case AutoPickup::State::LIFT:
    case AutoPickup::State::LIFT_RETURN:
      return DELTA_CHECKPOINT_OBSERVE;
    default: return 0u;
  }
}

inline bool finePickupCorrection(AutoPickup::State state) {
  return state==AutoPickup::State::ALIGN_ABOVE || state==AutoPickup::State::LOWER_PICKUP;
}
static_assert(DELTA_PICKUP_FINE_TIMEOUT_MS < CARTESIAN_STAGE_TIMEOUT_MS,
              "Pickup correction retains the finite30s Cartesian stage deadline");

inline bool checkpointResultAccepted(AutoPickup::State state, DeltaResultCode result) {
  return result == (checkpointRequestFlags(state) == DELTA_CHECKPOINT_OBSERVE
    ? DELTA_RESULT_OBSERVED : DELTA_RESULT_OK);
}

inline uint32_t rcRampPeriodMs(AutoPickup::State state) {
  return state == AutoPickup::State::CLOSE_GATE
    ? GATE_CLOSE_RAMP_PERIOD_MS : RC_RAMP_PERIOD_MS;
}

inline uint16_t rcRampStepUs(AutoPickup::State state,uint8_t axis) {
  return state==AutoPickup::State::CARRY_X && axis==1u
    ? CARRY_X_RAMP_STEP_US : RC_RAMP_STEP_US;
}

inline AutoPickup::Config configuration() {
  AutoPickup::Config c;
  c.transitZMm = TRANSIT_Z_MM;
  c.waitZMm = WAIT_Z_MM;
  c.combinedApproach = COMBINED_APPROACH_ENABLED;
  c.pickupPoseDuringApproach = PICKUP_POSE_DURING_APPROACH_ENABLED &&
    !PICKUP_APPROACH_DIRECT_ENABLED; // DIRECT does not allow changed RC commands in flight
  c.combinedLoadedTransfer = COMBINED_LOADED_TRANSFER_ENABLED;
  c.twoDirectLoadedMoves = TWO_DIRECT_LOADED_MOVES_ENABLED;
  c.returnXyAtDropZ = RETURN_XY_AT_DROP_Z_ENABLED;
  c.waitTiltUs = WAIT_TILT_US;
  c.rightLiftFirst = RIGHT_LIFT_FIRST_ENABLED;
  c.rightLiftThresholdY = RIGHT_LIFT_THRESHOLD_Y_MM;
  c.approachClearanceMm = APPROACH_CLEARANCE_MM +
    (POSITION_MOVES_ALL_STAGES ? POSITION_APPROACH_MARGIN_MM : 0.0f);
  c.pickupXMm = PICKUP_X_MM;
  c.cameraLongitudinalSign = CAMERA_LONGITUDINAL_SIGN;
  c.robotBeltXSign = ROBOT_BELT_X_SIGN;
  c.dropMayBeAboveTransitConfirmed = DROP_ABOVE_TRANSIT_CONFIRMED;
  c.arrivalToleranceMm = ARRIVAL_TOLERANCE_MM;
  c.reverseToleranceMm = REVERSE_TOLERANCE_MM;
  c.mappingConfirmed = MAPPING_CONFIRMED;
  c.routeConfirmed = ROUTE_CONFIRMED;
  c.rcTimingConfirmed = RC_TIMING_CONFIRMED;
  c.supervisedTrial = SUPERVISED_TRIAL_ENABLED;
  c.continuous = CONTINUOUS_ENABLED;
  c.closeLeadMm = CLOSE_LEAD_MM;
  c.countBasedGate = true;
  c.encoderCountsPerMm = ENCODER_COUNTS_PER_MM;
  c.rcSettleMs = RC_SETTLE_MS;
  c.dumpSettleMs = DUMP_SETTLE_MS;
  c.stageTimeoutMs = STAGE_TIMEOUT_MS;
  c.cartesianStageTimeoutMs = CARTESIAN_STAGE_TIMEOUT_MS;
  return c;
}

inline float pickupApproachZ(float robotY) {
  return AutoPickup::Sequence::pickupApproachZ(configuration(), robotY);
}

inline float distanceToArrival(float worldLongitudinalMm) {
  // Geometry only: measured image-centre -> gate line at robot X=0.
  // A point downstream in the image is closer; downstream pickup X is farther.
  return GATE_LINE_FROM_IMAGE_CENTRE_MM - CAMERA_LONGITUDINAL_SIGN * worldLongitudinalMm
         + ROBOT_BELT_X_SIGN * PICKUP_X_MM;
}

inline float gateCloseOffsetMm(uint8_t pieceId, float beltSpeedMmS) {
  const float classMm = pieceId == 1u ? GATE_CLOSE_OFFSET_PIECE_C_MM :
                        pieceId == 2u ? GATE_CLOSE_OFFSET_SMALL_MM : 0.0f;
  const float timeMm = std::isfinite(beltSpeedMmS) && beltSpeedMmS > 0.0f
    ? beltSpeedMmS * GATE_CLOSE_OFFSET_MS * 0.001f : 0.0f;
  return GATE_CLOSE_OFFSET_MM + classMm + timeMm;
}
inline float distanceToArrival(float worldLongitudinalMm, uint8_t pieceId,
                               float beltSpeedMmS) {
  return distanceToArrival(worldLongitudinalMm) + gateCloseOffsetMm(pieceId, beltSpeedMmS);
}
inline float advanceMm(int32_t atFrame, int32_t current) {
  return static_cast<float>(AutoGateEncoder::advanceMm(atFrame,current,ENCODER_COUNTS_PER_MM));
}
}
#endif
