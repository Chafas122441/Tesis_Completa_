/**
 * @file auto_pickup_height.h
 * @brief Perfil de altura de recogida en funcion del lateral Y del robot.
 * Interpola referencias de mando dentro de limites locales, mantiene X=0 y
 * rechaza valores no finitos. Las coordenadas no son una medicion del filo.
 */
#ifndef AUTO_PICKUP_HEIGHT_H
#define AUTO_PICKUP_HEIGHT_H
#include <cmath>
#include "gripper_auto_calibration.h"


namespace AutoPickupHeight {

constexpr float LEFT_Y_MM = -38.45f;
constexpr float LEFT_Z_MM = -336.42f;
constexpr float FAR_LEFT_Y_MM = LEFT_Y_MM; // compatibility alias, not another capture
constexpr float FAR_LEFT_Z_MM = LEFT_Z_MM;
constexpr float CENTER_Y_MM = 0.0f;
constexpr float BELT_CENTER_Y_MM = 27.88f;
constexpr float BELT_CENTER_Z_MM = -330.49f;
// Robot Y=0 is an interpolated compatibility value, not a fourth measurement.
constexpr float CENTER_Z_MM = LEFT_Z_MM + (CENTER_Y_MM-LEFT_Y_MM) /
  (BELT_CENTER_Y_MM-LEFT_Y_MM) * (BELT_CENTER_Z_MM-LEFT_Z_MM);
constexpr float RIGHT_Y_MM = 116.53f;
constexpr float RIGHT_Z_MM = -319.56f;
static_assert(LEFT_Y_MM < CENTER_Y_MM && LEFT_Y_MM < BELT_CENTER_Y_MM,
              "Pickup limits and three measured knots must be ordered");
static_assert(CENTER_Y_MM < BELT_CENTER_Y_MM && BELT_CENTER_Y_MM < RIGHT_Y_MM,
              "Positive-Y interpolation knots must be ordered");

inline bool calibrated(float robotY) {
  return std::isfinite(robotY) && robotY >= FAR_LEFT_Y_MM && robotY <= RIGHT_Y_MM;
}

// Local AUTO pickup reach only. Does NOT clamp camera data, manual motion,
// box targets, or an invalid/non-finite observation into a valid command.
inline float clampPickupY(float robotY) {
  return std::isfinite(robotY) ? std::fmax(LEFT_Y_MM,std::fmin(RIGHT_Y_MM,robotY)) : NAN;
}

inline float targetZ(float robotY) {
  if (!std::isfinite(robotY)) return NAN; // invalid input must fail closed
  // Piecewise-linear interpolation through the three new captures, no old
  // low-belt knots or extrapolation beyond the local pickup boundaries.
  if (robotY <= LEFT_Y_MM) return LEFT_Z_MM;
  if (robotY >= RIGHT_Y_MM) return RIGHT_Z_MM;
  if (robotY > BELT_CENTER_Y_MM) {
    const float t = (robotY - BELT_CENTER_Y_MM) / (RIGHT_Y_MM - BELT_CENTER_Y_MM);
    return BELT_CENTER_Z_MM + t * (RIGHT_Z_MM - BELT_CENTER_Z_MM);
  }
  const float t = (robotY - LEFT_Y_MM) / (BELT_CENTER_Y_MM - LEFT_Y_MM);
  return LEFT_Z_MM + t * (BELT_CENTER_Z_MM - LEFT_Z_MM);
}
// Highest interpolated pickup reference crossed by a straight lateral segment.
// A piecewise-linear profile reaches its maximum at endpoints or a knot.
// This models command clearance only; it does not measure the scoop/belt TCP.
inline float clearanceZBetween(float fromY, float toY, float clearanceMm) {
  if (!std::isfinite(fromY) || !std::isfinite(toY) ||
      !std::isfinite(clearanceMm) || clearanceMm < 20.0f || clearanceMm > 100.0f)
    return NAN;
  const float lo = std::fmin(fromY, toY), hi = std::fmax(fromY, toY);
  float highest = std::fmax(targetZ(fromY), targetZ(toY));
  const float knots[] = {FAR_LEFT_Y_MM, LEFT_Y_MM, CENTER_Y_MM,
                         BELT_CENTER_Y_MM, RIGHT_Y_MM};
  for (float y : knots) {
    if (y >= lo && y <= hi) highest = std::fmax(highest, targetZ(y));
  }
  return highest + clearanceMm;
}
} // namespace AutoPickupHeight
#endif
