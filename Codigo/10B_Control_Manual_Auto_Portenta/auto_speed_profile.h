/**
 * @file auto_speed_profile.h
 * @brief Seleccion de rapidez y perfil segun la etapa automatica.
 */
#ifndef AUTO_SPEED_PROFILE_H
#define AUTO_SPEED_PROFILE_H
#include <cmath>
#include "auto_pickup_sequence.h"

// Supervised reference-planning limits only, not measured mechanism speeds.
// They do not change acceleration, joint/RAW guards, ACKs or physical PASS.
namespace AutoSpeedProfile {
struct Config {
  float approachMmS;
  float loadedTransferMmS;
  float emptyReturnMmS;
  float finalMmS;
  float finalZoneMm;
};

inline bool valid(const Config& c) {
  return std::isfinite(c.approachMmS) && c.approachMmS > 0.0f &&
    c.approachMmS <= 150.0f && std::isfinite(c.loadedTransferMmS) &&
    c.loadedTransferMmS > 0.0f && c.loadedTransferMmS <= 150.0f &&
    std::isfinite(c.emptyReturnMmS) && c.emptyReturnMmS > 0.0f &&
    c.emptyReturnMmS <= 150.0f && std::isfinite(c.finalMmS) &&
    c.finalMmS > 0.0f && c.finalMmS <= c.approachMmS &&
    c.finalMmS <= c.loadedTransferMmS && c.finalMmS <= c.emptyReturnMmS &&
    std::isfinite(c.finalZoneMm) && c.finalZoneMm > 0.0f && c.finalZoneMm <= 100.0f;
}

inline float cruiseLimit(AutoPickup::State state, const Config& c) {
  if (!valid(c)) return NAN;
  switch (state) {
    case AutoPickup::State::TRANSFER: return c.loadedTransferMmS;
    case AutoPickup::State::LIFT_RETURN:
    case AutoPickup::State::RETURN_XY:
    case AutoPickup::State::RETURN_HOME: return c.emptyReturnMmS;
    default: return c.approachMmS;
  }
}

inline const char* name(AutoPickup::State state) {
  switch (state) {
    case AutoPickup::State::TRANSFER: return "LOADED_TRANSFER";
    case AutoPickup::State::LIFT_RETURN:
    case AutoPickup::State::RETURN_XY:
    case AutoPickup::State::RETURN_HOME: return "EMPTY_RETURN";
    default: return "APPROACH";
  }
}

inline float maximumSpeed(float cruise, float distance, const Config& c,
                          float deceleration, float dt) {
  if (!valid(c) || !std::isfinite(cruise) || cruise <= 0.0f || cruise > 150.0f ||
      !std::isfinite(distance) || distance < 0.0f ||
      !std::isfinite(deceleration) || deceleration <= 0.0f ||
      !std::isfinite(dt) || dt <= 0.0f) return NAN;
  if (distance <= c.finalZoneMm) return std::fmin(cruise, c.finalMmS);

  // Begin braking BEFORE the final zone, continuously rather than changing
  // the limit from 120/150 directly to 40 at the 20 mm boundary. The envelope
  // reserves one full command interval: v*dt + (v*v-final*final)/(2*a)
  // <= distance-finalZone + final*dt. Its discrete decrease is <= a*dt
  // during normal forward approaches. The rationalized form avoids loss of
  // precision near the zone boundary. Actual endpoint braking remains owned
  // by AutoCartesianArrival::nextSpeed() in the production controller.
  const float outsideZone = distance - c.finalZoneMm;
  const float base = c.finalMmS + deceleration * dt;
  const float envelope = c.finalMmS + (2.0f * deceleration * outsideZone) /
    (std::sqrt(base * base + 2.0f * deceleration * outsideZone) + base);
  return std::fmin(cruise, envelope);
}
}
#endif
