/**
 * @file auto_joint_ptp.h
 * @brief Planificacion de desplazamientos articulares punto a punto.
 * Calcula tiempos y comprueba las posturas intermedias mediante FK e IK.
 */
#ifndef AUTO_JOINT_PTP_H
#define AUTO_JOINT_PTP_H
#include <cmath>
#include <cstdint>
#include "auto_pickup_sequence.h"

// Legacy: one IK endpoint and synchronized joint-space curve. Combined mode:
// continuous geometric XYZ with per-sample IK/FK. Neither is a long HTS MOVE:
// the caller still sends short, ACK-gated STREAM segments and physical PASS.
namespace AutoJointPtp {
constexpr unsigned SAMPLES = 128u;
inline bool eligible(AutoPickup::State state) {
  return state == AutoPickup::State::TRANSFER;
}
inline float blend(float u) {
  return u*u*u*(10.0f+u*(-15.0f+6.0f*u));
}
struct Sample {
  float theta[3]; uint32_t elapsedMs; bool complete;
  bool cartesian = false;
  float xyz[3] = {};
};
class Plan {
 public:
  void cancel() { active_=combined_=false; elapsedMs_=durationMs_=0u; }
  bool active() const { return active_; }
  bool combined() const { return combined_; }
  bool complete() const { return active_ && elapsedMs_>=durationMs_; }
  uint32_t durationMs() const { return durationMs_; }
  uint32_t elapsedMs() const { return elapsedMs_; }

  // evaluate checks the ACTUAL FK curve, RAW/Jacobian/workspace/clearance, not
  // a Cartesian chord. It must be bounded/nonblocking and returns model XYZ.
  // Dense finite differences bound the numerical profile conservatively; the
  // caller checks every dispatched sample again. No collision claim is made.
  template<class Evaluate>
  bool begin(const float start[3],const float goal[3],float jointRadS,
             float cruiseMmS,float finalMmS,float finalZoneMm,
             float acceleration,float deceleration,Evaluate evaluate) {
    cancel();
    if (!start || !goal || !std::isfinite(jointRadS) || jointRadS<=0.0f ||
        !std::isfinite(cruiseMmS) || cruiseMmS<=0.0f || cruiseMmS>150.0f ||
        !std::isfinite(finalMmS) || finalMmS<=0.0f || finalMmS>cruiseMmS ||
        !std::isfinite(finalZoneMm) || finalZoneMm<=0.0f ||
        !std::isfinite(acceleration) || acceleration<=0.0f ||
        !std::isfinite(deceleration) || deceleration<=0.0f) return false;
    float largest=0.0f,goalPose[3];
    for(unsigned a=0u;a<3u;++a) {
      if(!std::isfinite(start[a]) || !std::isfinite(goal[a])) return false;
      start_[a]=start[a]; delta_[a]=goal[a]-start[a];
      largest=std::fmax(largest,std::fabs(delta_[a]));
    }
    if(!evaluate(goal,goalPose)) return false;
    float seconds=std::fmax(0.05f,1.875f*largest/jointRadS);
    float previous[3]={},beforePrevious[3]={};
    for(unsigned i=0u;i<=SAMPLES;++i) {
      const float u=static_cast<float>(i)/SAMPLES;
      const float s=blend(u);
      float q[3],p[3];
      for(unsigned a=0u;a<3u;++a)q[a]=start_[a]+delta_[a]*s;
      if(!evaluate(q,p)) { cancel();return false; }
      float distance2=0.0f,velocity2=0.0f,acceleration2=0.0f;
      for(unsigned a=0u;a<3u;++a) {
        if(!std::isfinite(p[a])) {cancel();return false;}
        const float d=goalPose[a]-p[a];distance2+=d*d;
        if(i) {const float v=(p[a]-previous[a])*SAMPLES;velocity2+=v*v;}
        if(i>1u) {const float acc=(p[a]-2.0f*previous[a]+beforePrevious[a])*SAMPLES*SAMPLES;acceleration2+=acc*acc;}
      }
      // The +2 mm numerical buffer also covers a sample straddling the zone.
      const float limit=std::sqrt(distance2)<=finalZoneMm+2.0f ? finalMmS : cruiseMmS;
      seconds=std::fmax(seconds,1.25f*std::sqrt(velocity2)/limit);
      const float rate=std::fmin(acceleration,deceleration);
      seconds=std::fmax(seconds,std::sqrt(1.5f*std::sqrt(acceleration2)/rate));
      for(unsigned a=0u;a<3u;++a) {beforePrevious[a]=previous[a];previous[a]=p[a];}
    }
    if(!std::isfinite(seconds) || seconds>25.0f) {cancel();return false;}
    durationMs_=static_cast<uint32_t>(std::ceil(seconds*20.0f))*50u;
    active_=true;
    return true;
  }

  // Single continuous loaded XYZ route. Z starts vertically to clear the
  // entire crossed belt profile; it keeps moving while XY blends in. There
  // is no waypoint stop, intermediate token, or physical checkpoint here.
  // evaluate resolves IK and checks the ACTUAL FK/RAW/Jacobian envelope.
  template<class Evaluate>
  bool beginCombined(const float start[3],const float goal[3],float clearanceZ,
                     float jointRadS,float cruiseMmS,float finalMmS,float finalZoneMm,
                     float acceleration,float deceleration,Evaluate evaluate) {
    cancel();
    if (!start || !goal || !std::isfinite(clearanceZ) ||
        !std::isfinite(jointRadS) || jointRadS<=0.0f ||
        !std::isfinite(cruiseMmS) || cruiseMmS<=0.0f || cruiseMmS>150.0f ||
        !std::isfinite(finalMmS) || finalMmS<=0.0f || finalMmS>cruiseMmS ||
        !std::isfinite(finalZoneMm) || finalZoneMm<=0.0f ||
        !std::isfinite(acceleration) || acceleration<=0.0f ||
        !std::isfinite(deceleration) || deceleration<=0.0f) return false;
    for (unsigned a=0u;a<3u;++a) {
      if (!std::isfinite(start[a]) || !std::isfinite(goal[a])) return false;
      xyzStart_[a]=start[a]; xyzGoal_[a]=goal[a];
    }
    clearanceZ_=std::fmax(clearanceZ,start[2]);
    if (goal[2]<=start[2] || goal[2]<=clearanceZ_) return false;
    liftFraction_=(clearanceZ_-start[2])/(goal[2]-start[2]);
    combined_=true;
    float previous[3]={},previousQ[3]={},direction[SAMPLES+1u][3]={};
    float cap[SAMPLES+1u]={};
    arc_[0]=time_[0]=0.0f;
    for (unsigned i=0u;i<=SAMPLES;++i) {
      float p[3],q[3];
      combinedPosition(static_cast<float>(i)/SAMPLES,p);
      if (!pointSafe(p) || !evaluate(p,q)) {cancel();return false;}
      float distance2=0.0f,segment2=0.0f,largestJoint=0.0f;
      for (unsigned a=0u;a<3u;++a) {
        if (!std::isfinite(q[a])) {cancel();return false;}
        const float d=goal[a]-p[a];distance2+=d*d;
        if (i) {
          const float delta=p[a]-previous[a];segment2+=delta*delta;
          largestJoint=std::fmax(largestJoint,std::fabs(q[a]-previousQ[a]));
        }
      }
      cap[i]=0.80f*(std::sqrt(distance2)<=finalZoneMm+2.0f ? finalMmS : cruiseMmS);
      if (i) {
        const float length=std::sqrt(segment2);
        if (!std::isfinite(length) || length<=0.0f) {cancel();return false;}
        arc_[i]=arc_[i-1u]+length;
        const float jointCap=largestJoint>0.0f ? 0.75f*jointRadS*length/largestJoint : cruiseMmS;
        cap[i]=std::fmin(cap[i],jointCap);cap[i-1u]=std::fmin(cap[i-1u],jointCap);
        for(unsigned a=0u;a<3u;++a)direction[i][a]=(p[a]-previous[a])/length;
      }
      for(unsigned a=0u;a<3u;++a) {previous[a]=p[a];previousQ[a]=q[a];}
    }
    // Time-reparameterize the geometric curve locally. Scaling a whole
    // nested quintic by its single worst joint derivative made valid left
    // pickups take >22s. Local arc caps retain the SAME physical limits.
    const float curvatureBudget=0.30f*std::fmin(acceleration,deceleration);
    for(unsigned i=1u;i<SAMPLES;++i) {
      float turn2=0.0f;
      for(unsigned a=0u;a<3u;++a) {
        const float d=direction[i+1u][a]-direction[i][a];turn2+=d*d;
      }
      const float curvature=2.0f*std::sqrt(turn2)/(arc_[i+1u]-arc_[i-1u]);
      if(curvature>0.0f) {
        const float v=std::sqrt(curvatureBudget/curvature);
        cap[i-1u]=std::fmin(cap[i-1u],v);cap[i]=std::fmin(cap[i],v);
        cap[i+1u]=std::fmin(cap[i+1u],v);
      }
    }
    velocity_[0]=0.0f;
    for(unsigned i=1u;i<=SAMPLES;++i)
      velocity_[i]=std::fmin(cap[i],std::sqrt(velocity_[i-1u]*velocity_[i-1u]+
        0.60f*acceleration*(arc_[i]-arc_[i-1u])));
    velocity_[SAMPLES]=0.0f;
    for(unsigned i=SAMPLES;i>0u;--i)
      velocity_[i-1u]=std::fmin(velocity_[i-1u],std::sqrt(velocity_[i]*velocity_[i]+
        0.60f*deceleration*(arc_[i]-arc_[i-1u])));
    for(unsigned i=1u;i<=SAMPLES;++i) {
      const float totalSpeed=velocity_[i]+velocity_[i-1u];
      if(!std::isfinite(totalSpeed) || totalSpeed<=0.0f) {cancel();return false;}
      time_[i]=time_[i-1u]+2.0f*(arc_[i]-arc_[i-1u])/totalSpeed;
    }
    const float seconds=time_[SAMPLES];
    if (!std::isfinite(seconds) || seconds>25.0f) {cancel();return false;}
    durationMs_=static_cast<uint32_t>(std::ceil(seconds*20.0f))*50u;
    active_=true;
    return true;
  }

  bool pointSafe(const float p[3]) const {
    if (!combined_ || !p) return false;
    constexpr float epsilon=0.001f; // float/FK rounding only; not physical clearance
    for (unsigned a=0u;a<3u;++a) {
      if (!std::isfinite(p[a]) ||
          p[a]<std::fmin(xyzStart_[a],xyzGoal_[a])-epsilon ||
          p[a]>std::fmax(xyzStart_[a],xyzGoal_[a])+epsilon) return false;
    }
    if (p[2]<clearanceZ_-epsilon &&
        (std::fabs(p[0]-xyzStart_[0])>epsilon ||
         std::fabs(p[1]-xyzStart_[1])>epsilon)) return false;
    return true;
  }
  bool next(uint32_t intervalMs,Sample& out) const {
    if(!active_ || (intervalMs!=50u && intervalMs!=100u)) return false;
    out.elapsedMs=elapsedMs_+intervalMs;
    if(out.elapsedMs>durationMs_)out.elapsedMs=durationMs_;
    const float s=blend(static_cast<float>(out.elapsedMs)/durationMs_);
    out.cartesian=combined_;
    if (combined_) {
      const float t=static_cast<float>(out.elapsedMs)/durationMs_*time_[SAMPLES];
      unsigned i=1u;
      while(i<SAMPLES && time_[i]<t)++i;
      const float localTime=std::fmax(0.0f,t-time_[i-1u]);
      const float segmentTime=time_[i]-time_[i-1u];
      const float acceleration=(velocity_[i]-velocity_[i-1u])/segmentTime;
      const float localDistance=velocity_[i-1u]*localTime+0.5f*acceleration*localTime*localTime;
      const float fraction=std::fmax(0.0f,std::fmin(1.0f,localDistance/(arc_[i]-arc_[i-1u])));
      combinedPosition((static_cast<float>(i-1u)+fraction)/SAMPLES,out.xyz);
      for(unsigned a=0u;a<3u;++a)out.theta[a]=NAN; // caller must resolve and validate IK
    } else {
      for(unsigned a=0u;a<3u;++a)out.theta[a]=start_[a]+delta_[a]*s;
    }
    out.complete=out.elapsedMs==durationMs_;
    return true;
  }
  // Only the ACK of the corresponding, still-authorized segment may commit.
  bool commit(uint32_t elapsedMs) {
    if(!active_ || elapsedMs<elapsedMs_ || elapsedMs>durationMs_)return false;
    elapsedMs_=elapsedMs;return true;
  }
 private:
  void combinedPosition(float s,float out[3]) const {
    const float lateral=s<=liftFraction_ ? 0.0f
      : blend((s-liftFraction_)/(1.0f-liftFraction_));
    for(unsigned a=0u;a<2u;++a)out[a]=xyzStart_[a]+(xyzGoal_[a]-xyzStart_[a])*lateral;
    out[2]=xyzStart_[2]+(xyzGoal_[2]-xyzStart_[2])*s;
  }
  float start_[3]={},delta_[3]={};
  float xyzStart_[3]={},xyzGoal_[3]={},clearanceZ_=0.0f,liftFraction_=0.0f;
  float arc_[SAMPLES+1u]={},time_[SAMPLES+1u]={},velocity_[SAMPLES+1u]={};
  uint32_t elapsedMs_=0u,durationMs_=0u;
  bool active_=false,combined_=false;
};
}
#endif
