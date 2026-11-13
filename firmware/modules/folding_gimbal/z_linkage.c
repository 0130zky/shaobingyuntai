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
  c->fold_speed>0 && c->pitch_speed>0 && c->fold_accel>0 && c->pitch_accel>0 && c->error>0 && c->slow>0 && c->margin>0 &&
  c->pitch_high-c->pitch_low>2*c->margin && ZLink_OperatingLow(c)<ZLink_OperatingHigh(c) &&
  SENTRY_SEQ_FOLD_PITCH_RAW>=c->pitch_low && SENTRY_SEQ_FOLD_PITCH_RAW<=c->pitch_high &&
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
static void Track(ZTrajectory *a,float measured,float goal,float speed,float accel,const ZConfig *c,float dt)
{
 /* Only the protocol domain and relative following error constrain commands.
  * Captured endpoints are destinations, not position-limit interlocks. */
 a->goal=Sentry_Clamp(goal,fmaxf(0,measured-c->error),fminf(65535,measured+c->error));
 Sentry_Track(&a->p,&a->v,goal,measured,speed,accel,0,65535,c->error,dt);
}
int ZLink_Step(ZLinkage *z,const ZConfig *c,float fold,float pitch,float vf,float vp,
               float left,float right,float dt,uint32_t now)
{
 if(!ZLink_ConfigValid(c)||!isfinite(fold)||!isfinite(pitch)||!isfinite(vf)||!isfinite(vp)||
    !isfinite(left)||!isfinite(right)||!isfinite(dt)||dt<=0||dt>.02f||
    fold<0||fold>65535||pitch<0||pitch>65535)return 1;
 float low=ZLink_OperatingLow(c),high=ZLink_OperatingHigh(c);
 if(!z->initialized){
  z->fold=(ZTrajectory){fold,0,fold};z->pitch=(ZTrajectory){pitch,0,pitch};z->initialized=1;
  z->mode=fabsf(fold-low)<=c->end_tolerance?Z_OPEN:
    fabsf(fold-high)<=c->end_tolerance?Z_FOLDED:Z_HOLD;
 }
 int request=left<0?-1:left>0?1:0;
 if(request>0&&!SENTRY_SEQ_UNFOLD_ALLOWED)request=0;
 if(request!=z->request){
  z->request=(int8_t)request;
  Begin(z,request<0?Z_LINKED:request>0?Z_UNFOLDING:Z_HOLD,now);
 }
 if(z->mode==Z_LINKED||z->mode==Z_UNFOLDING){
  int closing=z->mode==Z_LINKED;
  float fold_target=closing?high:low;
  float pitch_target=closing?SENTRY_SEQ_FOLD_PITCH_RAW:SENTRY_SEQ_OPEN_PITCH_RAW;
  /* Both axes start on this tick, each with its own speed/feedback ramp. */
  Track(&z->fold,fold,fold_target,c->fold_speed,c->fold_accel,c,dt);
  Track(&z->pitch,pitch,pitch_target,c->pitch_speed,c->pitch_accel,c,dt);
  int reached=fabsf(fold-fold_target)<=c->end_tolerance &&
    fabsf(pitch-pitch_target)<=c->level_tolerance &&
    fabsf(z->fold.p-fold_target)<=c->end_tolerance &&
    fabsf(z->pitch.p-pitch_target)<=c->level_tolerance &&
    fabsf(vf)<=c->still_rate && fabsf(vp)<=c->still_rate;
  if(reached){
   if(!z->settled)z->settled=now;
   if(now-z->settled>=c->settle_ms)Begin(z,closing?Z_FOLDED:Z_OPEN,now);
  }else z->settled=0;
  if((z->mode==Z_LINKED||z->mode==Z_UNFOLDING)&&now-z->align_started>=c->align_timeout_ms)return 2;
  return 0;
 }
 if(z->mode==Z_OPEN){
  Sentry_Jog(&z->pitch.p,&z->pitch.v,&z->pitch.goal,pitch,right,1,c->pitch_speed,
   c->pitch_accel,0,65535,c->error,c->slow,dt);
 }
 return 0;
}
