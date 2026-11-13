#!/usr/bin/env python3
"""Run production DM control with the real PID calculation/limits on a host clock."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
controller=(root/'Components/Controller/controller.c').read_text()
header=(root/'Components/Controller/controller.h').read_text()
port=(root/'Components/Devices/dial_port_dm.c').read_text()
def function(source,signature):
    start=source.index(signature+'\n{')
    return source[start:source.index('\n}',start)+2]
code=r'''
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include "dial_config.h"
#include "controller.h"
#define absf fabsf
static uint32_t tick=1000;
uint32_t HAL_GetTick(void){return tick;}
'''
code+=r'''
static PID_t s_pid_angle,s_pid_speed;
static struct {float angle_deg,speed_dps,out;} s_fb;
static uint8_t s_active;
'''+port[port.index('typedef struct {'):port.index('static dial_terminal_t s_terminal;')+len('static dial_terminal_t s_terminal;')]+ '\n'+function(port,'void Dial_Port_SetTarget(float dial_deg, float dt_s)').replace('void Dial_Port_SetTarget(', 'void Dial_Port_SetTargetDt(')+'\n'+function(port,'uint8_t Dial_Port_Settled(void)')+'\n'+function(port,'void Dial_Port_ClearIntegral(void)')+'\n'+function(port,'void Dial_Port_Stop(void)')+r'''
void reset(float angle,float speed,float angle_i,float speed_i){
 memset(&s_pid_angle,0,sizeof s_pid_angle);memset(&s_pid_speed,0,sizeof s_pid_speed);
 s_pid_angle.Kp=DIAL_DM_ANGLE_KP;s_pid_angle.Ki=DIAL_DM_ANGLE_KI;
 s_pid_angle.MaxOut=DIAL_DM_ANGLE_MAX;s_pid_angle.IntegralLimit=DIAL_DM_ANGLE_I_LIMIT;
 s_pid_angle.Improve=Integral_Limit;s_pid_angle.Iout=angle_i;
 s_pid_speed.Kp=DIAL_DM_SPEED_KP;s_pid_speed.Ki=DIAL_DM_SPEED_KI;
 s_pid_speed.MaxOut=DIAL_DM_SPEED_MAX;s_pid_speed.IntegralLimit=DIAL_DM_SPEED_I_LIMIT;
 s_pid_speed.Improve=Integral_Limit;s_pid_speed.Iout=speed_i;
 s_fb.angle_deg=angle;s_fb.speed_dps=speed;s_active=0;tick=1000;
 memset(&s_terminal,0,sizeof s_terminal);
}
#define Dial_Port_SetTarget(target) Dial_Port_SetTargetDt(target,.003f)
int main(void){
 PID_t integral={0};integral.Ki=2;integral.MaxOut=1000;integral.IntegralLimit=1000;
 integral.Improve=Integral_Limit;
 PID_CalculateDt(&integral,0,10,.005f);assert(fabsf(integral.Iout-.1f)<.000001f);
 PID_CalculateDt(&integral,0,10,.001f);assert(fabsf(integral.Iout-.12f)<.000001f);
 PID_CalculateDt(&integral,0,10,NAN);assert(integral.Output==0 && fabsf(integral.Iout-.12f)<.000001f);
 PID_Init(&integral,1000,1000,10,0,0,.001f,OutputFilter);
 PID_CalculateDt(&integral,0,10,.001f);assert(fabsf(integral.Output-50)<.0001f);
 PID_CalculateDt(&integral,0,10,.003f);assert(fabsf(integral.Output-87.5f)<.0001f);


 /* Recorded crossing of target539: old stored position integral still drives forward. */
 reset(539.06134f,44.9132f,70,0);Dial_Port_SetTarget(539);
 assert(s_fb.out<0 && s_active); /* must brake, not continue pushing past goal */
 reset(-539.06134f,-44.9132f,-70,0);Dial_Port_SetTarget(-539);
 assert(s_fb.out>0 && s_active);
 /* A speed integral of the wrong sign must not cancel that braking. */
 reset(539.2f,20,70,.20f);Dial_Port_SetTarget(539);assert(s_fb.out<0);
 reset(-539.2f,-20,-70,-.20f);Dial_Port_SetTarget(-539);assert(s_fb.out>0);
 /* Keep same-sign loaded push capability before the goal. */
 reset(530,0,70,.02f);Dial_Port_SetTarget(539);assert(s_fb.out>.18f);
 /* Recorded terminal stalls get bounded output immediately, not after integrating. */
 const float stalls[]={.913f,2.493f,3.715f};
 for(unsigned i=0;i<sizeof(stalls)/sizeof(stalls[0]);i++){
  reset(539-stalls[i],0,0,0);Dial_Port_SetTarget(539);assert(s_fb.out>.14f);
  const float initial=s_fb.out;
  for(int j=0;j<10000;j++){tick+=3;Dial_Port_SetTarget(539);}
  assert(fabsf(s_fb.out-initial)<.00001f && !Dial_Port_Settled());
  assert(s_pid_angle.Iout==0 && s_pid_speed.Iout==0);
  reset(-539+stalls[i],0,0,0);Dial_Port_SetTarget(-539);assert(s_fb.out<-.14f);
 }
 /* Exact zero error cannot leave the previous position-loop output frozen. */
 reset(539,20,70,.20f);s_pid_angle.Output=70;Dial_Port_SetTarget(539);
 assert(s_pid_angle.Output==0 && s_fb.out<0);
 /* Motion and torque limits are retained at a full 45-degree step. */
 reset(0,0,0,0);Dial_Port_SetTarget(45);
 assert(s_fb.out>0 && s_fb.out<=1 && s_pid_angle.Output<=DIAL_DM_ANGLE_MAX);
 reset(0,0,0,0);Dial_Port_SetTarget(-45);assert(s_fb.out<0 && s_fb.out>=-1);
 /* Fast motion/braking must never receive a forward stiction kick. */
 reset(538,100,70,.20f);Dial_Port_SetTarget(539);
 assert(s_fb.out<0 && s_pid_angle.Iout==0 && s_pid_speed.Iout==0);
 /* Precision arrival uses real elapsed time; 3deg below the old 4deg band is not done. */
 reset(42,0,0,0);
 for(int j=0;j<100;j++){tick+=3;Dial_Port_SetTarget(45);}
 assert(!Dial_Port_Settled());
 reset(44.7f,.14f,70,.20f);Dial_Port_SetTarget(45);
 for(unsigned j=0;j<DIAL_DM_SETTLE_MS-1;j++){tick++;Dial_Port_SetTarget(45);}
 assert(!Dial_Port_Settled());tick++;Dial_Port_SetTarget(45);assert(Dial_Port_Settled());
 const float hold=s_fb.out;
 for(int j=0;j<10000;j++){tick+=3;Dial_Port_SetTarget(45);assert(fabsf(s_fb.out-hold)<.000001f);}
 assert(s_pid_angle.Iout==0 && s_pid_speed.Iout==0 && hold!=0);
 /* Deadband hysteresis retains hold for tiny drift, then resumes correction. */
 s_fb.angle_deg=44.3f;tick+=3;Dial_Port_SetTarget(45);assert(Dial_Port_Settled());
 assert(fabsf(s_fb.out)<.03f);
 s_fb.angle_deg=43.9f;assert(!Dial_Port_Settled());tick+=3;Dial_Port_SetTarget(45);
 assert(!Dial_Port_Settled() && s_fb.out>.14f);
 /* Time and settling streak survive HAL wrap, but any speed excursion restarts the hold. */
 reset(44.8f,0,0,0);tick=0xfffffff0U;Dial_Port_SetTarget(45);
 for(int j=0;j<40;j++){tick++;Dial_Port_SetTarget(45);}
 s_fb.speed_dps=3;tick++;Dial_Port_SetTarget(45);
 s_fb.speed_dps=0;tick++;Dial_Port_SetTarget(45);
 for(unsigned j=0;j<DIAL_DM_SETTLE_MS-1;j++){tick++;Dial_Port_SetTarget(45);}assert(!Dial_Port_Settled());tick++;Dial_Port_SetTarget(45);
 assert(Dial_Port_Settled());tick+=11;assert(!Dial_Port_Settled());
 tick++;Dial_Port_SetTarget(45);assert(!Dial_Port_Settled());
 for(unsigned j=0;j<DIAL_DM_SETTLE_MS;j++){tick++;Dial_Port_SetTarget(45);}
 assert(Dial_Port_Settled());
 Dial_Port_ClearIntegral();assert(!Dial_Port_Settled());
 /* New targets, manual reverse and disabled/reconnected control cannot inherit readiness. */
 reset(45,0,0,0);Dial_Port_SetTarget(45);
 for(unsigned j=0;j<DIAL_DM_SETTLE_MS;j++){tick++;Dial_Port_SetTarget(45);}
 assert(Dial_Port_Settled());
 tick++;Dial_Port_SetTarget(90);assert(!Dial_Port_Settled() && s_fb.out>0);
 tick++;Dial_Port_SetTarget(0);assert(!Dial_Port_Settled() && s_fb.out<0);
 Dial_Port_Stop();assert(!Dial_Port_Settled() && s_fb.out==0 && !s_terminal.valid);
 /* Synthetic Coulomb-friction regression, not a hardware accuracy model.
    Exercises both directions from recorded stall distances and full 45deg moves. */
 const float starts[]={.913f,2.493f,3.715f,45.0f};
 const float friction[]={.08f,.13f};
 for(int dir=-1;dir<=1;dir+=2)for(unsigned f=0;f<2;f++)for(unsigned k=0;k<4;k++){
  reset(0,0,0,0);const float target=dir*starts[k];int done=0;
  for(int j=0;j<500;j++){
   tick+=3;Dial_Port_SetTarget(target);assert(fabsf(s_fb.out)<=1);
   float v=s_fb.speed_dps,u=s_fb.out;
   if(fabsf(v)<.5f && fabsf(u)<=friction[f])v=0;
   else {
    const float before=v;
    const float drag=copysignf(.06f,fabsf(v)>.01f?v:u);
    v+=(10000*(u-drag)-1.5f*v)*.003f;
    if(before*v<0 && fabsf(u)<=friction[f])v=0;
   }
   s_fb.speed_dps=v;s_fb.angle_deg+=v*.003f;
   assert(fabsf(s_fb.angle_deg)<=fabsf(target)+DIAL_OVERRUN_FAULT_DEG);
   if(Dial_Port_Settled()){done=1;break;}
  }
  if(!done)fprintf(stderr,"unsettled: start=%f friction=%f dir=%d angle=%f speed=%f\n",starts[k],friction[f],dir,s_fb.angle_deg,s_fb.speed_dps);
  assert(done);
 }
 puts("PASS: crossing brake, recorded stiction stalls, no terminal integral creep, timed precision hold, hysteresis, target/stop reset, HAL wrap, torque limits and bidirectional friction model");
}
'''
with tempfile.TemporaryDirectory(prefix='dm-control-') as tmp:
    p=Path(tmp)
    (p/'test.c').write_text(code)
    (p/'dial_config.h').write_text((root/'robots/example/dial_config.h').read_text())
    (p/'app_RM_Motor.h').write_text('/* Configuration-only stub. */\n')
    subprocess.run(['cc','-std=c11','-I',str(p),'-I',str(root/'Components/Controller'),str(p/'test.c'),str(root/'Components/Controller/controller.c'),'-lm','-o',str(p/'test')],check=True)
    subprocess.run([str(p/'test')],check=True)
