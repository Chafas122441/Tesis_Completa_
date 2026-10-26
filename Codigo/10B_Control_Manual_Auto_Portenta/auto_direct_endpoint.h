/**
 * @file auto_direct_endpoint.h
 * @brief Validacion de recorridos directos y recuperaciones de destino.
 * Comprueba geometria y limites antes de autorizar los comandos articulares.
 */
#ifndef AUTO_DIRECT_ENDPOINT_H
#define AUTO_DIRECT_ENDPOINT_H

#include <cmath>
#include <cstring>
#include <stdint.h>
#include "auto_pickup_sequence.h"
#include "delta_stream_protocol.h"

/* Preflight for exactly one long HTS interpolation. The receiver gets only
   the final RAW endpoint and duration. Sampling here validates the same
   straight line in joint/RAW space that the servo interpolation will follow;
   it does not generate runtime setpoints. */
namespace AutoDirectEndpoint {

constexpr uint16_t SAMPLES = 128u;

struct Profile {
  uint16_t goal[3] = {0u, 0u, 0u};
  uint16_t durationMs = 0u;
};

inline bool eligible(AutoPickup::State state) {
  return state == AutoPickup::State::LIFT || state == AutoPickup::State::TRANSFER;
}

template<class ToTheta, class Evaluate, class Validate>
bool prepare(const uint16_t start[3], const uint16_t goal[3], Profile &profile,
             float cruiseMmS, ToTheta toTheta, Evaluate evaluate, Validate validate,
             uint16_t samples = SAMPLES) {
  if (!start || !goal || !std::isfinite(cruiseMmS) || cruiseMmS <= 0.0f ||
      cruiseMmS > 150.0f || samples < 8u || samples > SAMPLES) return false;
  const uint16_t rawDuration = deltaDirectMinimumMs(start, goal);
  if (!rawDuration) return false;

  float a[3], b[3];
  toTheta(start, a);
  toTheta(goal, b);
  float previous[3] = {NAN, NAN, NAN};
  float longestSegmentMm = 0.0f;
  for (uint16_t sample = 0u; sample <= samples; ++sample) {
    const float u = static_cast<float>(sample) / static_cast<float>(samples);
    float q[3], xyz[3];
    for (uint8_t axis = 0u; axis < 3u; ++axis)
      q[axis] = a[axis] + (b[axis] - a[axis]) * u;
    if (!evaluate(q, xyz) || !validate(xyz, previous, sample)) return false;
    if (sample) {
      const float dx = xyz[0] - previous[0];
      const float dy = xyz[1] - previous[1];
      const float dz = xyz[2] - previous[2];
      const float segmentMm = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (!std::isfinite(segmentMm)) return false;
      if (segmentMm > longestSegmentMm) longestSegmentMm = segmentMm;
    }
    std::memcpy(previous, xyz, sizeof(previous));
  }
  /* Joint-linear motion does not have constant TCP speed. Size the single
     servo interpolation from the fastest sampled portion, then round upward
     to the 50 ms protocol grid. The RAW-rate floor and Cartesian ceiling are
     both enforced; this remains exactly one transmitted endpoint. */
  uint32_t cartesianDuration = static_cast<uint32_t>(std::ceil(
    longestSegmentMm * static_cast<float>(samples) * 1000.0f / cruiseMmS));
  cartesianDuration = ((cartesianDuration + 49u) / 50u) * 50u;
  uint32_t duration = rawDuration > cartesianDuration ? rawDuration : cartesianDuration;
  if (duration < DELTA_DIRECT_MIN_DURATION_MS) duration = DELTA_DIRECT_MIN_DURATION_MS;
  if (duration > DELTA_DIRECT_MAX_DURATION_MS) return false;
  std::memcpy(profile.goal, goal, sizeof(profile.goal));
  profile.durationMs = static_cast<uint16_t>(duration);
  return true;
}

} // namespace AutoDirectEndpoint

#endif
