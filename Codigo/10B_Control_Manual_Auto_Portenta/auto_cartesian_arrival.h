/**
 * @file auto_cartesian_arrival.h
 * @brief Criterios de llegada y coherencia del estado cartesiano automatico.
 * La estimacion por FK no sustituye la verificacion fisica del actuador.
 */
#ifndef AUTO_CARTESIAN_ARRIVAL_H
#define AUTO_CARTESIAN_ARRIVAL_H
#include <cmath>

// Reference planning only. Reaching this envelope never substitutes for the
// Nucleo's physical checkpoint PASS, nor measures the mechanism's velocity.
namespace AutoCartesianArrival {
inline float nextSpeed(float distance, float previousSpeed, float maximumSpeed,
                       float acceleration, float deceleration, float dt,
                       float positionTolerance) {
  if (!std::isfinite(distance) || !std::isfinite(previousSpeed) ||
      !std::isfinite(maximumSpeed) || !std::isfinite(acceleration) ||
      !std::isfinite(deceleration) || !std::isfinite(dt) ||
      !std::isfinite(positionTolerance) || distance < 0.0f ||
      previousSpeed < 0.0f || maximumSpeed <= 0.0f || acceleration <= 0.0f ||
      deceleration <= 0.0f || dt <= 0.0f || positionTolerance < 0.0f) return NAN;
  if (distance == 0.0f) return 0.0f;

  // Reserve one complete command interval before the continuous braking
  // distance: v*dt + v*v/(2*a) <= distance. sqrt(2*a*d) alone crosses short
  // waypoints at 50 ms and can settle into a direction-reversing limit cycle.
  // Rationalized form avoids cancellation close to zero distance.
  const float decelerationStep = deceleration * dt;
  const float brakingSpeed = (2.0f * deceleration * distance) /
    (std::sqrt(decelerationStep * decelerationStep +
               2.0f * deceleration * distance) + decelerationStep);
  const float desired = distance <= positionTolerance ? 0.0f
    : std::fmin(maximumSpeed, brakingSpeed);
  const float rate = desired < previousSpeed ? deceleration : acceleration;
  const float change = std::fmax(-rate * dt,
                                std::fmin(rate * dt, desired - previousSpeed));
  // An inherited velocity outside the braking envelope must never advance the
  // reference past the waypoint. Cap position increment, not elapsed ACK time.
  // Normal approaches brake beforehand; this last cap also handles a replaced
  // or already-near target without pretending that physical motion has stopped.
  return std::fmax(0.0f, std::fmin(distance / dt,
                 std::fmin(maximumSpeed, previousSpeed + change)));
}
}
#endif
