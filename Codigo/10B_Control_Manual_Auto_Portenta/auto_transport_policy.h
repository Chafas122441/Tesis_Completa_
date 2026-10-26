/**
 * @file auto_transport_policy.h
 * @brief Cadencia y limites de los setpoints de traslado incremental.
 * El envio se coordina con las confirmaciones del enlace UART.
 */
#ifndef AUTO_TRANSPORT_POLICY_H
#define AUTO_TRANSPORT_POLICY_H
#include <math.h>
#include <stdint.h>
#include "auto_pickup_sequence.h"
#include "delta_stream_protocol.h"

// Only the interior of a loaded XY transfer. No larger endpoint tolerance.
namespace AutoTransport {
constexpr float FINE_ZONE_MM = 60.0f;
constexpr uint32_t MODEL_PERIOD_MS = 100u;
// Minimum scheduling interval, PLUS the link's 110 ms guard from the latest
// accepted STREAM ACK (fine/coarse/hold). Never catch up delayed ACKs.
constexpr uint32_t DISPATCH_PERIOD_MS = 110u;
constexpr uint16_t MOVE_TIME_MS = 200u;
constexpr int MAX_STEP_RAW = 8;
static_assert(MAX_STEP_RAW * 1000u / MODEL_PERIOD_MS == 80u,
              "Coarse transport must keep the 80 RAW/s model limit");
inline bool coarse(AutoPickup::State state, float distance, bool moving,
                   float departureDistance) {
  return moving && state == AutoPickup::State::TRANSFER &&
         isfinite(distance) && distance > FINE_ZONE_MM &&
         isfinite(departureDistance) && departureDistance > 20.0f;
}
}
#endif
