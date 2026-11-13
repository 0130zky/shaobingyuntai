#include "z_linkage.h"
#include "motion.h"
#include "gimbal_config.h"
#include <math.h>
#include <string.h>
float ZLink_OperatingLow(const ZConfig *c){return c->fold_low+c->margin;}
float ZLink_OperatingHigh(const ZConfig *c){return c->fold_high-c->margin;}
int ZLink_ConfigValid(const ZConfig *c)
{
 return isfinite(c->fold_low)&&isfinite(c->fold_high)&&isfinite(c->pitch_low)&&isfinite(c->pitch_high)&&
  c->fold_speed>0 && c->pitch_speed>0 && c->accel>0 && c->error>0 && c->slow>0 && c->margin>0 &&
  c->pitch_high-c->pitch_low>2*c->margin && ZLink_OperatingLow(c)<ZLink_OperatingHigh(c) &&
  SENTRY_SEQ_PREP_PITCH_RAW>=c->pitch_low && SENTRY_SEQ_PREP_PITCH_RAW<=c->pitch_high &&
  SENTRY_SEQ_FOLD_PITCH_RAW>=c->pitch_low &&
  SENTRY_SEQ_FOLD_PITCH_RAW<=c->pitch_high &&
  SENTRY_SEQ_OPEN_PITCH_RAW>=c->pitch_low && SENTRY_SEQ_OPEN_PITCH_RAW<=c->pitch_high &&
  c->level_tolerance>0 && c->level_tolerance<c->track_tolerance && c->settle_ms>0 && c->align_timeout_ms>c->settle_ms;
}
void ZLink_Reset(ZLinkage *z){memset(z,0,sizeof(*z));}
static void Hold(ZTrajectory *a){a->goal=a->p;a->v=0;}
static void Begin(ZLinkage *z,ZMode mode,uint32_t now)
{
 Hold(&z->fold);Hold(&z->pitch);
 z->mode=mode;z->align_started=now;z->settled=0;
}
static void Track(ZTrajectory *a,float measured,float goal,float speed,float low,float high,
                  const ZConfig *c,float dt)
{
 a->goal=Sentry_Clamp(goal,low,high);
 a->goal=Sentry_Clamp(a->goal,fmaxf(low,measured-c->error),fminf(high,measured+c->error));
 Sentry_Move(&a->p,&a->v,a->goal,speed,c->accel,dt);
 a->p=Sentry_Clamp(a->p,fmaxf(low,measured-c->error),fminf(high,measured+c->error));
 if((a->p<=low&&a->v<0)||(a->p>=high&&a->v>0))a->v=0;
}
static int Settled(ZLinkage *z,int ready,uint32_t now,const ZConfig *c)
{
 if(!ready){z->settled=0;return 0;}
 if(!z->settled)z->settled=now;
 return now-z->settled>=c->settle_ms;
}
int ZLink_Step(ZLinkage *z,const ZConfig *c,float fold,float pitch,float vf,float vp,
               float left,float right,float dt,uint32_t now)
{
 if(!ZLink_ConfigValid(c)||!isfinite(fold)||!isfinite(pitch)||!isfinite(vf)||!isfinite(vp)||
    !isfinite(left)||!isfinite(right)||!isfinite(dt)||dt<=0||dt>.02f||
    fold<0||fold>65535||pitch<0||pitch>65535)return 1;
 float low=ZLink_OperatingLow(c),high=ZLink_OperatingHigh(c);
 int at_open=fold<=low+c->end_tolerance;
 int still=fabsf(vf)<=c->still_rate&&fabsf(vp)<=c->still_rate;
 if(!z->initialized){
  z->fold=(ZTrajectory){fold,0,fold};z->pitch=(ZTrajectory){pitch,0,pitch};z->initialized=1;
  z->mode=at_open?Z_OPEN:fold>=high-c->end_tolerance?Z_FOLDED:Z_HOLD;
 }
 /* Optional opening gate; current user-selected configuration enables it. */
 if(left>0 && !SENTRY_SEQ_UNFOLD_ALLOWED){Begin(z,Z_HOLD,now);return 0;}
 if(left<0 && z->mode==Z_FOLDED && fabsf(pitch-SENTRY_SEQ_FOLD_PITCH_RAW)>c->level_tolerance)
  Begin(z,Z_LEVELING,now);
 if(left<0 && z->mode!=Z_LEVELING && z->mode!=Z_LINKED && z->mode!=Z_FOLD_PITCH && z->mode!=Z_FOLDED)
  Begin(z,Z_LEVELING,now);
 else if(left>0 && z->mode!=Z_UNFOLDING && z->mode!=Z_OPEN_LEVELING && z->mode!=Z_OPEN)
  Begin(z,Z_UNFOLDING,now);
 if(left==0 && (z->mode==Z_LEVELING || z->mode==Z_LINKED || z->mode==Z_UNFOLDING || z->mode==Z_FOLD_PITCH))
  Begin(z,Z_HOLD,now);
 if(z->mode==Z_LEVELING || z->mode==Z_FOLD_PITCH){
  int final=z->mode==Z_FOLD_PITCH;
  /* Final Pitch is forbidden until actual Fold has reached the folded end. */
  if(final && fold<c->fold_high-c->end_tolerance)return 1;
  float target=final?SENTRY_SEQ_FOLD_PITCH_RAW:SENTRY_SEQ_PREP_PITCH_RAW;
  Track(&z->pitch,pitch,target,c->pitch_speed,0,65535,c,dt);
  if(Settled(z,fabsf(pitch-target)<=c->level_tolerance &&
      fabsf(z->pitch.p-target)<=c->level_tolerance && still,now,c))
   Begin(z,final?Z_FOLDED:Z_LINKED,now);
  else if(now-z->align_started>=c->align_timeout_ms)return 2;
  return 0;
 }
 if(z->mode==Z_LINKED || z->mode==Z_UNFOLDING){
  /* A held Pitch that drifts must not be carried through the mechanism. */
  if(fabsf(pitch-z->pitch.p)>c->track_tolerance){
   Hold(&z->fold);
   if(!z->settled)z->settled=now;
   if(now-z->settled>=c->align_timeout_ms)return 2;
   return 0;
  }
  z->settled=0;
  float dyn_low=fminf(low,fold),dyn_high=fmaxf(high,fold);
  Sentry_Jog(&z->fold.p,&z->fold.v,&z->fold.goal,fold,left,-1,c->fold_speed,c->accel,
    dyn_low,dyn_high,c->error,c->slow,dt);
  /* Wait for actual Fold and its commanded trajectory, not just a soft target. */
  int reached=z->mode==Z_UNFOLDING?
    (fold<=low+c->end_tolerance && z->fold.p<=low+c->end_tolerance):
    (fold>=c->fold_high-c->end_tolerance && z->fold.p>=high-c->end_tolerance);
  if(reached && still && fabsf(z->fold.v)<c->still_rate){
   /* Separate arrival dwell from the drift timer. */
   if(now-z->align_started>=c->settle_ms){
    if(z->mode==Z_UNFOLDING)Begin(z,Z_OPEN_LEVELING,now);
    else Begin(z,Z_FOLD_PITCH,now);
   }
  }else z->align_started=now;
  return 0;
 }
 if(z->mode==Z_OPEN_LEVELING){
  float target=SENTRY_SEQ_OPEN_PITCH_RAW;
  Track(&z->pitch,pitch,target,c->pitch_speed,0,65535,c,dt);
  if(Settled(z,fabsf(pitch-target)<=c->level_tolerance && fabsf(z->pitch.p-target)<=c->level_tolerance && still,now,c))
   Begin(z,Z_OPEN,now);
  else if(now-z->align_started>=c->align_timeout_ms)return 2;
  return 0;
 }
 if(z->mode==Z_OPEN){
  /* Preserve the manually captured boundary pose on enable. Only a nonzero
   * right stick starts jogging into the soft operating range. */
  if(right!=0)Sentry_Jog(&z->pitch.p,&z->pitch.v,&z->pitch.goal,pitch,right,1,c->pitch_speed,
   c->accel,0,65535,c->error,c->slow,dt);
  else Hold(&z->pitch);
 }
 return 0;
}
