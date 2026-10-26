/**
 * @file local_point_profile.h
 * @brief Limites y perfiles compartidos para objetivos articulares locales.
 */
#ifndef LOCAL_POINT_PROFILE_H
#define LOCAL_POINT_PROFILE_H
#include <stdint.h>
#include <math.h>

/* Shared deterministic RAW-space endpoint profile. A single host destination
   is NOT a long HTS MOVE. Nucleo emits at most one <=4 RAW batch per 50 ms,
   with 100 ms interpolation; never catches up after a delayed loop. The host
   must preflight FK/workspace/clearance along THIS curve before starting it. */
#define DELTA_POINT_PERIOD_MS 50u
#define DELTA_POINT_SLICE_MS 100u
#define DELTA_POINT_MAX_DURATION_MS 12000u
#define DELTA_POINT_MAX_STEP_RAW 4u
#define DELTA_POINT_RAW_PER_SECOND 60u /* 14.4 deg/s < existing 18 deg/s */

typedef struct {
  uint16_t start[3], goal[3], durationMs, elapsedMs;
} DeltaPointProfile;

static inline float deltaPointBlend(float u) {
  return u*u*u*(10.0f+u*(-15.0f+6.0f*u));
}
static inline uint16_t deltaPointRaw(const DeltaPointProfile *p, uint8_t axis,
                                     uint16_t elapsedMs) {
  if (elapsedMs >= p->durationMs) return p->goal[axis];
  const float s=deltaPointBlend((float)elapsedMs/(float)p->durationMs);
  return (uint16_t)floorf((float)p->start[axis]+
    ((float)p->goal[axis]-(float)p->start[axis])*s+0.5f);
}
static inline uint16_t deltaPointMinimumMs(const uint16_t start[3],const uint16_t goal[3]) {
  uint16_t largest=0u;
  for (uint8_t i=0u;i<3u;++i) {
    const uint16_t d=start[i]>goal[i] ? start[i]-goal[i] : goal[i]-start[i];
    if (d>largest) largest=d;
  }
  /* max derivative of quintic is 1.875; integer ceil to 50 ms */
  const uint32_t ms=((uint32_t)largest*1875u+DELTA_POINT_RAW_PER_SECOND-1u)/DELTA_POINT_RAW_PER_SECOND;
  return (uint16_t)(ms<50u ? 50u : ((ms+49u)/50u)*50u);
}
static inline int deltaPointInit(DeltaPointProfile *p,const uint16_t start[3],
                                 const uint16_t goal[3],uint16_t durationMs) {
  if (!p || !start || !goal || !durationMs || durationMs%50u ||
      durationMs>DELTA_POINT_MAX_DURATION_MS || durationMs<deltaPointMinimumMs(start,goal)) return 0;
  for (uint8_t i=0u;i<3u;++i) {p->start[i]=start[i];p->goal[i]=goal[i];}
  p->durationMs=durationMs;p->elapsedMs=0u;
  return 1;
}
#endif
