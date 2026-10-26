/**
 * @file auto_local_point.h
 * @brief Preparacion de objetivos locales y comprobaciones de trayectoria.
 */
#ifndef AUTO_LOCAL_POINT_H
#define AUTO_LOCAL_POINT_H
#include "auto_joint_ptp.h"
#include "local_point_profile.h"
#include <stdlib.h>

namespace AutoLocalPoint {
inline bool eligible(AutoPickup::State state) {
  return state==AutoPickup::State::TRANSFER || state==AutoPickup::State::INITIAL_LIFT ||
    state==AutoPickup::State::LIFT_RETURN || state==AutoPickup::State::RETURN_XY ||
    state==AutoPickup::State::RETURN_HOME;
}
// LIFT (right-side exit) and the short escape from the belt remain vertical
// Cartesian control. A joint-space arc is not a vertical straight line.
template<class ToTheta,class Evaluate>
bool prepare(const uint16_t start[3],const uint16_t goal[3],DeltaPointProfile &profile,
             float jointRadS,float cruise,float finalSpeed,float finalZone,
             float acceleration,float deceleration,ToTheta toTheta,Evaluate evaluate) {
  float a[3],b[3];toTheta(start,a);toTheta(goal,b);
  AutoJointPtp::Plan check;
  if (!check.begin(a,b,jointRadS,cruise,finalSpeed,finalZone,acceleration,deceleration,evaluate)) return false;
  uint32_t duration=check.durationMs();
  const uint16_t minimum=deltaPointMinimumMs(start,goal);
  if (duration<minimum) duration=minimum;
  if (duration>DELTA_POINT_MAX_DURATION_MS || !deltaPointInit(&profile,start,goal,(uint16_t)duration)) return false;
  uint16_t previous[3]={start[0],start[1],start[2]};
  for (uint16_t t=0u;t<=profile.durationMs;t=(uint16_t)(t+50u)) {
    uint16_t raw[3];float q[3],p[3];
    for (uint8_t i=0u;i<3u;++i) {
      raw[i]=deltaPointRaw(&profile,i,t);
      if (abs((int)raw[i]-(int)previous[i])>4) return false;
      previous[i]=raw[i];
    }
    toTheta(raw,q);
    if (!evaluate(q,p)) return false; // quantized FK/Jacobian/clearance, not only endpoints
  }
  return true;
}
}
#endif
