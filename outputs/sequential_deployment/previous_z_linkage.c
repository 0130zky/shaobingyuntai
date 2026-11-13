#include "z_linkage.h"
#include "motion.h"
#include "gimbal_config.h"
#include <math.h>
#include <string.h>
static const float nodes[3][2]={
 {SENTRY_Z_OPEN_FOLD_RAW,SENTRY_Z_OPEN_PITCH_RAW},
 {SENTRY_Z_MID_FOLD_RAW,SENTRY_Z_MID_PITCH_RAW},
 {SENTRY_Z_FOLDED_FOLD_RAW,SENTRY_Z_FOLDED_PITCH_RAW}
};
float ZLink_Level(float raw)
{
 raw=Sentry_Clamp(raw,nodes[0][0],nodes[2][0]);
 unsigned i=raw<=nodes[1][0]?0:1;
 return nodes[i][1]+(raw-nodes[i][0])*(nodes[i+1][1]-nodes[i][1])/(nodes[i+1][0]-nodes[i][0]);
}
float ZLink_Inverse(float raw)
{
 raw=Sentry_Clamp(raw,nodes[2][1],nodes[0][1]);
 unsigned i=raw>=nodes[1][1]?0:1;
 return nodes[i][0]+(raw-nodes[i][1])*(nodes[i+1][0]-nodes[i][0])/(nodes[i+1][1]-nodes[i][1]);
}
float ZLink_OperatingLow(const ZConfig *c)
{return fmaxf(c->fold_low+c->margin,ZLink_Inverse(c->pitch_high-c->margin));}
float ZLink_OperatingHigh(const ZConfig *c)
{return fminf(c->fold_high-c->margin,ZLink_Inverse(c->pitch_low+c->margin));}
int ZLink_ConfigValid(const ZConfig *c)
{
 for(unsigned i=0;i<3;i++) {
  if(nodes[i][0]<c->fold_low || nodes[i][0]>c->fold_high ||
     nodes[i][1]<c->pitch_low || nodes[i][1]>c->pitch_high)return 0;
  if(i && (nodes[i][0]<=nodes[i-1][0] || nodes[i][1]>=nodes[i-1][1]))return 0;
 }
 return c->pitch_speed>0 && c->fold_speed>0 && c->accel>0 && c->margin>0 &&
   ZLink_OperatingLow(c)<ZLink_OperatingHigh(c) && c->level_tolerance<c->track_tolerance;
}
void ZLink_Reset(ZLinkage *z){memset(z,0,sizeof(*z));}
static void Align(ZLinkage *z,uint32_t now,int from_open)
{
 z->mode=Z_LEVELING;z->align_started=now;z->settled=0;z->align_from_open=(uint8_t)from_open;
 z->fold.goal=z->fold.p;z->fold.v=0;
}
static void Track(ZTrajectory *a,float measured,float goal,float speed,float low,float high,
                  const ZConfig *c,float dt)
{
 a->goal=Sentry_Clamp(goal,low,high);
 a->goal=Sentry_Clamp(a->goal,fmaxf(low,measured-c->error),fminf(high,measured+c->error));
 Sentry_Move(&a->p,&a->v,a->goal,speed,c->accel,dt);
 a->p=Sentry_Clamp(a->p,fmaxf(low,measured-c->error),fminf(high,measured+c->error));
 if((a->p<=low && a->v<0)||(a->p>=high && a->v>0))a->v=0;
}
int ZLink_Step(ZLinkage *z,const ZConfig *c,float fold,float pitch,float vf,float vp,
               float left,float right,float dt,uint32_t now)
{
 if(!ZLink_ConfigValid(c)||!isfinite(fold)||!isfinite(pitch)||!isfinite(vf)||!isfinite(vp)||
    !isfinite(dt)||dt<=0||dt>.02f||fold<c->fold_low||fold>c->fold_high||
    pitch<c->pitch_low||pitch>c->pitch_high)return 1;
 const float low=ZLink_OperatingLow(c),high=ZLink_OperatingHigh(c);
 float level=ZLink_Level(fold),err=fabsf(pitch-level);
 int still=fabsf(vf)<=c->still_rate && fabsf(vp)<=c->still_rate;
 int at_open=fold<=low+c->end_tolerance;
 if(!z->initialized) {
  z->fold=(ZTrajectory){fold,0,fold};z->pitch=(ZTrajectory){pitch,0,pitch};
  z->initialized=1;
  if(at_open && still)z->mode=Z_OPEN;
  else if(err<=c->level_tolerance)z->mode=fold>=high-c->end_tolerance?Z_FOLDED:Z_LINKED;
  else Align(z,now,0);
 }
 if(z->mode==Z_OPEN && !at_open) {
  z->fold.p=z->fold.goal=fold;Align(z,now,0);
 }
 if(z->mode==Z_OPEN) {
  if(left<0)Align(z,now,1); /* No fold motion until actual Pitch levels. */
  else {
   /* Fold is held; only the fully open state accepts the right stick. */
   z->fold.v=0;
   Sentry_Jog(&z->pitch.p,&z->pitch.v,&z->pitch.goal,pitch,right,1,c->pitch_speed,
     c->accel,c->pitch_low+c->margin,c->pitch_high-c->margin,c->error,c->slow,dt);
   return 0;
  }
 }
 if(z->mode==Z_LEVELING) {
  if(z->align_from_open && left>=0) {
   z->mode=Z_OPEN;z->pitch.goal=z->pitch.p;z->pitch.v=0;return 0;
  }
  Track(&z->pitch,pitch,level,c->pitch_speed,c->pitch_low,c->pitch_high,c,dt);
  z->fold.v=0;
  if(err<=c->level_tolerance && still) {
   if(!z->settled)z->settled=now;
   if(now-z->settled>=c->settle_ms) {
    z->mode=Z_LINKED;z->settled=0;z->align_from_open=0;
   }
  } else z->settled=0;
  if(z->mode==Z_LEVELING && now-z->align_started>=c->align_timeout_ms)return 2;
  return 0; /* Even the transition tick cannot advance Fold. */
 }
 if(err>c->track_tolerance) {
  Align(z,now,0);Track(&z->pitch,pitch,level,c->pitch_speed,c->pitch_low,c->pitch_high,c,dt);return 0;
 }
 /* Advance a shared path coordinate. Both position commands must describe
  * the same pose on this tick; independent ramps produce transient tilt.
  * Actual feedback above still pauses the path when either drive lags. */
 float dir=-left;float distance=dir>0?high-fold:fold-low;
 float rate=dir*c->fold_speed*Sentry_Clamp(distance/c->slow,0,1);
 rate=Sentry_Clamp(rate,-sqrtf(2*c->accel*fmaxf(distance,0)),sqrtf(2*c->accel*fmaxf(distance,0)));
 if(dir*(z->fold.goal-fold)<0)z->fold.goal=fold;
 z->fold.goal+=rate*dt;
 /* An initial pose on a manually captured hard endpoint can be held there.
  * Only an inward request may leave it; never jump on enable to a soft bound. */
 float dyn_low=fminf(low,fold),dyn_high=fmaxf(high,fold);
 ZTrajectory previous=z->fold;
 Track(&z->fold,fold,z->fold.goal,c->fold_speed,dyn_low,dyn_high,c,dt);
 float next_pitch=ZLink_Level(z->fold.p);
 /* Do not advance the common coordinate beyond Pitch speed or following limits. */
 if(fabsf(next_pitch-z->pitch.p)>c->pitch_speed*dt+.01f ||
    fabsf(next_pitch-pitch)>c->error) {
  z->fold=previous;z->fold.goal=z->fold.p;z->fold.v=0;
  Align(z,now,0);
  Track(&z->pitch,pitch,level,c->pitch_speed,c->pitch_low,c->pitch_high,c,dt);
  return 0;
 }
 unsigned segment=z->fold.p<=nodes[1][0]?0:1;
 float slope=(nodes[segment+1][1]-nodes[segment][1])/(nodes[segment+1][0]-nodes[segment][0]);
 z->pitch=(ZTrajectory){next_pitch,slope*z->fold.v,next_pitch};
 if((z->fold.p<=dyn_low && z->fold.v<0)||(z->fold.p>=dyn_high && z->fold.v>0))z->fold.v=0;
 z->mode=fold>=high-c->end_tolerance && still?Z_FOLDED:Z_LINKED;
 if(at_open && err<=c->level_tolerance && still && left>=0) {
  if(!z->open_settled)z->open_settled=now;
  if(now-z->open_settled>=c->settle_ms) {
   z->mode=Z_OPEN;z->fold.goal=z->fold.p;z->fold.v=0;
   z->pitch.goal=z->pitch.p;z->pitch.v=0;
  }
 } else z->open_settled=0;
 return 0;
}
