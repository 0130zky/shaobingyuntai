"""Sequential folding order, physical following interlocks and cancellation."""
from pathlib import Path
import tempfile, subprocess
r=Path(__file__).resolve().parents[1]
code=r'''
#include "z_linkage.h"
#include "gimbal_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static const float counts=65535.0f/25.0f;
static ZConfig c;
static ZLinkage z;static float f,p,vf,vp;static unsigned now=1000;
static void step(float command,float right){
 now+=3;assert(!ZLink_Step(&z,&c,f,p,vf,vp,command,right,.003f,now));
 assert(z.fold.p>=0&&z.fold.p<=65535&&z.pitch.p>=0&&z.pitch.p<=65535);
 f=z.fold.p;p=z.pitch.p;vf=z.fold.v;vp=z.pitch.v;
}
int main(void){
 c=(ZConfig){38800,45628,29762,36039,64,SENTRY_FOLD_SPEED*counts,SENTRY_PITCH_SPEED*counts,
  SENTRY_FOLD_ACCEL*counts,SENTRY_PITCH_ACCEL*counts,131.07f,262.14f,96,128,80,96,100,20000};
 assert(ZLink_ConfigValid(&c));
 f=38890;p=34370;step(0,0);assert(z.mode==Z_OPEN);
 float initial_f=f,initial_p=p;step(-1,1);
 // No prerequisite phase: both references advance in the same first control tick.
 assert(z.mode==Z_LINKED&&f>initial_f&&p<initial_p);
 int overlap=0;
 for(int i=0;i<14000;i++){
  float before=f,pp=p;step(-1,1);
  if(f>before+.001f&&p<pp-.001f)overlap=1;
  assert(z.mode==Z_LINKED||z.mode==Z_FOLDED);
  if(z.mode==Z_FOLDED)break;
 }
 assert(overlap&&z.mode==Z_FOLDED&&fabsf(p-29763)<2&&fabsf(f-ZLink_OperatingHigh(&c))<2);
 initial_f=f;initial_p=p;step(1,-1);
 assert(z.mode==Z_UNFOLDING&&f<initial_f&&p>initial_p);
 overlap=0;
 for(int i=0;i<14000;i++){
  float before=f,pp=p;step(1,-1);
  if(f<before-.001f&&p>pp+.001f)overlap=1;
  assert(z.mode==Z_UNFOLDING||z.mode==Z_OPEN);
  if(z.mode==Z_OPEN)break;
 }
 assert(overlap&&z.mode==Z_OPEN&&fabsf(p-34370)<2);
 // Holding S2 middle preserves right-stick control after opening, without restarting.
 float before=p;for(int i=0;i<300;i++)step(1,-1);assert(z.mode==Z_OPEN&&p<before-50);
 // A reversal changes both destinations immediately and respects the per-tick ramp.
 step(-1,0);step(1,0);assert(z.mode==Z_UNFOLDING);
 // Neutral cancels movement and holds both references.
 step(0,0);float held_f=f,held_p=p;
 for(int i=0;i<100;i++)step(0,0);assert(f==held_f&&p==held_p&&z.mode==Z_HOLD);
 // Arriving Fold alone must not mark the whole operation finished; Pitch keeps advancing.
 ZLink_Reset(&z);f=ZLink_OperatingHigh(&c);p=34000;vf=vp=0;
 step(-1,0);assert(z.mode==Z_LINKED&&fabsf(f-ZLink_OperatingHigh(&c))<.01f&&p<34000);
 // A physically stuck Pitch never blocks Fold from starting, but cannot complete.
 ZLink_Reset(&z);now+=3;float start=38890;
 assert(!ZLink_Step(&z,&c,start,34370,0,0,-1,0,.003f,now));
 assert(z.fold.p>start&&z.pitch.p<34370);
 now+=20001;assert(ZLink_Step(&z,&c,start,34370,0,0,-1,0,.003f,now)==2);
 // No historical-position interlock or clamped jump on a neutral enable.
 const float poses[3][2]={{29245,33057},{45629,29761},{42000,36040}};
 for(unsigned i=0;i<3;i++){
  ZLink_Reset(&z);now+=3;assert(!ZLink_Step(&z,&c,poses[i][0],poses[i][1],0,0,0,0,.003f,now));
  assert(z.fold.p==poses[i][0]&&z.pitch.p==poses[i][1]);
 }
 // Changing one acceleration must not alter the other axis's first ramp step.
 ZConfig isolated=c;isolated.fold_accel=.5f*counts;isolated.pitch_accel=1.5f*counts;
 ZLink_Reset(&z);now+=3;
 assert(!ZLink_Step(&z,&isolated,38890,34370,0,0,-1,0,.003f,now));
 float fold_v=z.fold.v,pitch_v=z.pitch.v;
 assert(fabsf(fold_v-isolated.fold_accel*.003f)<.001f);
 assert(fabsf(pitch_v+isolated.pitch_accel*.003f)<.001f);
 isolated.fold_accel*=4;ZLink_Reset(&z);now+=3;
 assert(!ZLink_Step(&z,&isolated,38890,34370,0,0,-1,0,.003f,now));
 assert(fabsf(z.fold.v-4*fold_v)<.001f&&z.pitch.v==pitch_v);
 isolated.pitch_accel*=2;ZLink_Reset(&z);now+=3;
 assert(!ZLink_Step(&z,&isolated,38890,34370,0,0,-1,0,.003f,now));
 assert(fabsf(z.fold.v-4*fold_v)<.001f&&fabsf(z.pitch.v-2*pitch_v)<.001f);
 puts("PASS parallel: same-tick start in both directions, overlapping motion, independent arrival, both-axis completion, reversal, S2 persistence, cancel, timeout and outside-range hold");
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td);(p/'test.c').write_text(code)
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I'+str(r/'modules/folding_gimbal'),'-I'+str(r/'robots/sentry'),str(p/'test.c'),str(r/'modules/folding_gimbal/z_linkage.c'),str(r/'modules/folding_gimbal/motion.c'),'-lm','-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
