"""Automatic reference must not stop at the moving feedback-window endpoint."""
from pathlib import Path
import subprocess, tempfile
r = Path(__file__).resolve().parents[1]
code = r'''
#include "motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static const float dt=.003f,error=.05f;
static float prescribed(float speed,float accel){
 float p=0,v=0,measured=0,sum=0;
 for(int i=0;i<300;i++){
  Sentry_Track(&p,&v,10,measured,speed,accel,-12.5f,12.5f,error,dt);
  assert(fabsf(p-measured)<=error+1e-5f);
  if(i>=100){assert(v>2.9f);sum+=v;}
  measured+=3*dt;
 }
 return sum/200;
}
static void equal_manual(float speed,float accel,float sign){
 float p=0,v=0,m=0,jp=0,jv=0,jgoal=0;
 // Far from the final destination, automatic and full-stick share a ramp.
 for(int i=0;i<80;i++){
  Sentry_Track(&p,&v,10*sign,m,speed,accel,-12.5f,12.5f,error,dt);
  Sentry_Jog(&jp,&jv,&jgoal,m,sign,1,speed,accel,-12.5f,12.5f,error,.1f,dt);
  assert(fabsf(p-jp)<1e-5f&&fabsf(v-jv)<1e-5f);
  m=p;
 }
}
int main(void){
 float a=prescribed(800,800),b=prescribed(1200,800),c=prescribed(1200,1200);
 assert(fabsf(a-3)<.01f&&fabsf(b-3)<.01f&&fabsf(c-3)<.01f);
 printf("prescribed 3 rad/s feedback: 800/800=%.3f, 1200/800=%.3f, 1200/1200=%.3f rad/s reference velocity\n",a,b,c);
 equal_manual(1200,1200,1);equal_manual(1200,1200,-1);
 equal_manual(1,2,1);equal_manual(1,2,-1);
 // Speed and acceleration changes take effect below the common following cap.
 float p=0,v=0;
 Sentry_Track(&p,&v,1,0,1,2,-12.5f,12.5f,error,dt);assert(fabsf(v-2*dt)<1e-6f);
 for(int i=0;i<1000;i++){
  Sentry_Track(&p,&v,1,p,.2f,2,-12.5f,12.5f,error,dt);
  assert(v<=.20001f&&p<=1);
 }
 for(int i=0;i<1000;i++)Sentry_Track(&p,&v,1,p,.2f,2,-12.5f,12.5f,error,dt);
 assert(p==1&&v==0);
 // Reversal ramps through zero; a blocked motor keeps a bounded reference.
 p=0;v=.2f;
 Sentry_Track(&p,&v,-1,0,1,2,-12.5f,12.5f,error,dt);
 assert(fabsf(v-(.2f-2*dt))<1e-6f);
 p=v=0;
 for(int i=0;i<1000;i++)Sentry_Track(&p,&v,1,0,1200,1200,-12.5f,12.5f,error,dt);
 assert(fabsf(p-error)<1e-6f&&v==0);
 puts("PASS automatic: no moving-window velocity reset, manual parity, speed/acceleration response, exact arrival, reversal and bounded blocked feedback");
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td); (p/'test.c').write_text(code)
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I'+str(r/'modules/folding_gimbal'),str(p/'test.c'),str(r/'modules/folding_gimbal/motion.c'),'-lm','-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
