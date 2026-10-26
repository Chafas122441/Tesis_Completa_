/**
 * @file belt_estimator.h
 * @brief Estimacion de rapidez de banda desde cuentas X4 y tiempo transcurrido.
 * Utiliza la calibracion empirica del rodillo y evita division por cero.
 */
#ifndef BELT_ESTIMATOR_H
#define BELT_ESTIMATOR_H
#include <stdint.h>
/* Empirical X4 calibration: 24246 counts / 743.5 mm. Wheel D=49 mm.
   PPR is not known; do not substitute a guessed counts/revolution value. */
#define BELT_WHEEL_DIAMETER_MM 49u
static inline uint16_t BeltSpeedTenths(int32_t delta, uint32_t elapsedMs) {
  if (!elapsedMs) return 0u;
  int64_t magnitude = delta;
  if (magnitude < 0) magnitude = -magnitude;
  const uint64_t numerator = (uint64_t)magnitude * 7435u * 1000u;
  const uint64_t denominator = (uint64_t)24246u * elapsedMs;
  const uint64_t speed = (numerator + denominator / 2u) / denominator;
  return speed > 65535u ? 65535u : (uint16_t)speed;
}
#endif
