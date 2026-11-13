"""Actual RC channel mapping and manual trajectory: amplitude/speed proportionality."""
from pathlib import Path
import subprocess,tempfile
r=Path(__file__).resolve().parents[1]
code=r'''
#include "motion.h"
#include "sentry_remote.h"
#include "gimbal_config.h"
#include <math.h>
#include <assert.h>
#include <stdio.h>
static const float counts=65535.0f/25;
static float mapped(float fraction){
 float raw=(SENTRY_STICK_DEADZONE+(1-SENTRY_STICK_DEADZONE)*fabsf(fraction))*660;
 sentry_remote.channel[1]=(uint16_t)lroundf(1024+copysignf(raw,fraction));
 return SentryRemote_Stick(1);
}
static float simulate(float stick){
 float p=32768,v=0,goal=p,measured=p,sum=0;
 float accel=SENTRY_PITCH_ACCEL*counts,speed=SENTRY_PITCH_SPEED*counts;
 float maximum=fminf(speed,sqrtf(2*accel*131.07f));
 for(int i=0;i<200;i++){
  float previous=p,previous_v=v;
  Sentry_Jog(&p,&v,&goal,measured,stick,1,speed,accel,0,65535,131.07f,262.14f,.003f);
  assert(fabsf(v-previous_v)<=accel*.003f+.01f);
  assert(goal==p&&fabsf(p-measured)<=131.08f);
  measured=p;if(i>=100)sum+=p-previous;
 }
 float observed=sum/(100*.003f);
 assert(fabsf(observed-stick*maximum)<=maximum*.02f);
 // Release decelerates without a queued position target.
 for(int i=0;i<400;i++){
  Sentry_Jog(&p,&v,&goal,measured,0,1,speed,accel,0,65535,131.07f,262.14f,.003f);measured=p;
 }
 assert(fabsf(v)<.01f&&goal==p);
 return observed;
}
int main(void){
 sentry_remote.channel[1]=1024;assert(SentryRemote_Stick(1)==0);
 sentry_remote.channel[1]=1057;assert(SentryRemote_Stick(1)==0);
 float quarter=mapped(.25f),half=mapped(.5f),full=mapped(1);
 assert(quarter>.24f&&quarter<.26f&&half>.49f&&half<.51f&&full==1);
 float q=simulate(quarter),h=simulate(half),f=simulate(full);
 assert(fabsf(q/f-.25f)<.02f&&fabsf(h/f-.5f)<.02f);
 assert(simulate(mapped(-.25f))<0&&simulate(mapped(-.5f))<0&&simulate(mapped(-1))<0);
 // Physical following saturation bounds both the position lead and velocity output.
 float p=32768,v=0,goal=p;
 for(int i=0;i<1000;i++)Sentry_Jog(&p,&v,&goal,32768,1,1,SENTRY_PITCH_SPEED*counts,SENTRY_PITCH_ACCEL*counts,0,65535,131.07f,262.14f,.003f);
 assert(fabsf(p-32768)<=131.08f&&fabsf(v)<.01f);
 puts("PASS manual: rescaled dead zone, 25/50/100 percent speed ratios in both directions, ramp, release, no backlog and following saturation");
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td);(p/'test.c').write_text(code)
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I'+str(r/'modules/folding_gimbal'),'-I'+str(r/'robots/sentry'),str(p/'test.c'),str(r/'modules/folding_gimbal/motion.c'),str(r/'modules/folding_gimbal/sentry_remote.c'),'-lm','-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
