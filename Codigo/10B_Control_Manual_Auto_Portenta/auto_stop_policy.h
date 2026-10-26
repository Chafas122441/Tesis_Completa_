/**
 * @file auto_stop_policy.h
 * @brief Politica de cancelacion y retencion del mecanismo en automatico.
 */
#ifndef AUTO_STOP_POLICY_H
#define AUTO_STOP_POLICY_H
#include "auto_pickup_sequence.h"
#include "delta_stream_protocol.h"
namespace AutoStopPolicy {
// STOP remains STOP. Flags authorize bounded verification, never another
// route, gate release, ARM or resume. Geometry is model clearance only.
inline uint16_t requestFlags(AutoPickup::State state, float y, float z) {
  const uint16_t precise = DELTA_AUTO_STOP_POLICY;
  if (!std::isfinite(y) || !std::isfinite(z) || y < -295.0f || y > 280.0f ||
      z < -367.0f || z > -180.0f) return precise;
  if (z < AutoPickupHeight::targetZ(y) + 20.0f) return precise;
  switch (state) {
    // These states require the box's physical checkpoint PASS first.
    case AutoPickup::State::OPEN_ROTATE:
    case AutoPickup::State::DUMP_X:
    case AutoPickup::State::RESET_X:
    case AutoPickup::State::CLOSE_RETURN_GATE:
    case AutoPickup::State::PREPARE:
    case AutoPickup::State::INITIAL_LIFT:
    case AutoPickup::State::WAIT_PIECE:
    case AutoPickup::State::LIFT:
    case AutoPickup::State::TRANSFER:
    case AutoPickup::State::LIFT_RETURN:
    case AutoPickup::State::RESET_POSE:
    case AutoPickup::State::RETURN_XY:
    case AutoPickup::State::RETURN_HOME:
    case AutoPickup::State::OPEN_AT_ORIGIN:
    case AutoPickup::State::COMPLETE:
      return precise | DELTA_CHECKPOINT_AUTO15_REC30;
    default: return precise; // pickup/approach/unknown/aborted: +/-5
  }
}
}
#endif
