#!/usr/bin/env python3
"""Exercise production single-stroke bounds and jam rollback without motor hardware."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'modules/shooter/dial_control.c').read_text()
header = (root / 'modules/shooter/shooter_internal.h').read_text()
end = header.index('}Dail_state_t;') + len('}Dail_state_t;')
state_type = header[header.rfind('typedef struct', 0, end):end]


def function(signature):
    start = source.index(signature)
    return source[start:source.index('\n}', start)+2]


code = r'''
#include <stdint.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "dial_config.h"
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define absf fabsf
enum {check_yse,check_no};
/* module_task.c 里的端点标定转发，本测试不链接云台实现。 */
static void Gimbal_RequestLimit(uint8_t endpoint) { (void)endpoint; }
'''
code += 'static volatile int16_t s_phase_pending; static volatile uint8_t s_phase_requested; static volatile int16_t s_phase_reply; static volatile uint8_t s_phase_reply_type, s_phase_reply_pending;\n'
code += state_type + r'''
typedef struct {uint8_t link_ok;} booster_interface_t;
typedef struct {float angle_deg,speed_dps;uint8_t fb_ok;} dial_fb_t;
static struct {Dail_state_t Dail_state;booster_interface_t Booster_Interface;int16_t Dial_Shoot;} BOOSTER;
static struct {uint8_t Error_code;} RC_Data;
static dial_fb_t feedback;
static int enabled,stopped;
static uint32_t tick;
static float target;
static uint8_t s_ready_tracking,s_wheels_ready;
static uint8_t port_settled=1;
uint8_t Dial_Port_Settled(void){return port_settled;}
uint32_t HAL_GetTick(void){return tick;}
uint8_t RC_LinkFresh(void){return 1;}
uint8_t RC_MasterEnabled(void){return enabled;}
uint8_t Shooter_ControlEnabled(void){return enabled;}
uint8_t Shooter_CommandEnabled(void){return enabled;}
void Shooter_ResetReadiness(void){s_ready_tracking=s_wheels_ready=0;}
const dial_fb_t *Dial_Port_Get(void){return &feedback;}
void Dial_Port_Stop(void){stopped++;}
void Dial_Port_ClearIntegral(void){}
static float control_dt=.003f;
float Shooter_ControlDt(void){return control_dt;}
void Dial_Port_SetTarget(float t,float dt_s){
  assert(dt_s>0);
  Dail_state_t *st=&BOOSTER.Dail_state;
  assert(fabsf(st->Dial_Move_End-st->Dial_Move_Start)<=45.001f);
  assert(t>=fminf(st->Dial_Move_Start,st->Dial_Move_End)-.001f);
  assert(t<=fmaxf(st->Dial_Move_Start,st->Dial_Move_End)+.001f);
  target=t;
}
float Booster_DialTarget(void);
'''
for signature in ('uint8_t Booster_DialJammed(void)', 'uint8_t Booster_DialArrived(void)',
                  'static float Dial_StrokeClamp(', 'float Booster_DialTarget(void)',
                  'void Booster_RequestShot(int8_t n)', 'void Booster_RequestDialPhase(int16_t centidegrees)', 'void Booster_DialCtrl(void)'):
    code += function(signature) + '\n'
module_source = (root / 'Application/module_task.c').read_text()
rx_start = module_source.index('void FrictionScope_RxByte(uint8_t byte)')
code += module_source[rx_start:module_source.index('\n}', rx_start)+2] + '\n'
code += r'''
void reset(float angle){
  memset(&BOOSTER,0,sizeof BOOSTER);memset(&RC_Data,0,sizeof RC_Data);
  feedback=(dial_fb_t){angle,0,1};enabled=1;stopped=0;tick=1000;
  Booster_DialCtrl();
}
void cycle(void){tick+=3;Booster_DialCtrl();}
void stall(void){feedback.speed_dps=0;for(int i=0;i<DIAL_JAM_MS/3;i++)cycle();}
int main(void){
  uint8_t command[6]={0xa6,0x5a,1,0x38,0xff,0}; /* -200 centidegrees */
  for(int i=0;i<5;i++)command[5]^=command[i];
  for(int i=0;i<6;i++)FrictionScope_RxByte(command[i]);
  assert(s_phase_requested && s_phase_pending==-200);s_phase_requested=0;
  command[5]^=1;
  for(int i=0;i<6;i++)FrictionScope_RxByte(command[i]);
  assert(!s_phase_requested);
  Booster_RequestDialPhase(1001);assert(!s_phase_requested);
  reset(0);assert(Booster_DialArrived());
  /* Even in the old 4deg band, the DM port must explicitly finish precision settling. */
  port_settled=0;assert(!Booster_DialArrived());
  Booster_RequestShot(1);assert(Booster_DialTarget()==0);
  port_settled=1;cycle();assert(Booster_DialArrived());
  Booster_RequestShot(6);assert(BOOSTER.Dial_Shoot==1);assert(Booster_DialTarget()==45);
  for(int i=0;i<50;i++)Booster_RequestShot(1);
  assert(Booster_DialTarget()==45); /* no stacking, even across feedback movement */
  feedback.angle_deg=20;feedback.speed_dps=100;cycle();Booster_RequestShot(1);
  assert(Booster_DialTarget()==45);
  BOOSTER.Dial_Shoot=99;cycle();assert(Booster_DialTarget()==45);
  feedback.angle_deg=44.5f;feedback.speed_dps=0;port_settled=0;cycle();
  assert(BOOSTER.Dail_state.Dial_Move_Active && !Booster_DialArrived());
  Booster_RequestShot(1);assert(Booster_DialTarget()==45);
  port_settled=1;cycle();assert(Booster_DialArrived());
  Booster_RequestShot(1);assert(BOOSTER.Dail_state.Dial_Move_Start==45);
  assert(Booster_DialTarget()==90); /* previous arrival error must not shift the fixed target grid */
  Booster_RequestShot(1);assert(Booster_DialTarget()==90);

  /* 3:1 external gearing: 135 output-shaft degrees equals 45 dial degrees. */
  assert(DIAL_DM_RATIO_GEAR==3.0f);
  assert(fabsf((135.0f*3.14159265f/180.0f)*DIAL_DEG_PER_RAD-45.0f)<.001f);
  reset(0);
  for(int i=1;i<=40;i++){
    Booster_RequestShot(1);
    assert(BOOSTER.Dail_state.Dial_Move_Start==45.0f*(i-1));
    assert(Booster_DialTarget()==45.0f*i);
    feedback.angle_deg=45.0f*i-.5f;feedback.speed_dps=0;cycle();
    assert(Booster_DialArrived());
  }
  for(int i=39;i>=0;i--){
    Booster_RequestShot(-1);assert(Booster_DialTarget()==45.0f*i);
    feedback.angle_deg=45.0f*i+.5f;cycle();assert(Booster_DialArrived());
  }

  reset(0);Booster_RequestShot(1);feedback.angle_deg=5;stall();
  assert(Booster_DialJammed());assert(Booster_DialTarget()==0); /* cannot retreat to -17.5 */
  feedback.angle_deg=0;cycle();assert(Booster_DialJammed());assert(stopped>0);
  int stop_before=stopped;
  for(int i=0;i<500;i++){Booster_RequestShot(1);cycle();assert(Booster_DialTarget()==0);}
  assert(stopped>stop_before);assert(BOOSTER.Dail_state.Dial_Jam_Retry==1);
  Booster_RequestShot(-1);cycle();assert(!Booster_DialJammed());assert(Booster_DialTarget()==0);
  Booster_RequestShot(-127);assert(Booster_DialTarget()==-45);
  Booster_RequestShot(-1);assert(Booster_DialTarget()==-45); /* reverse also cannot stack */

  reset(0);Booster_RequestShot(1);feedback.angle_deg=40;stall();
  assert(Booster_DialJammed());assert(fabsf(Booster_DialTarget()-17.5f)<.001f);
  stop_before=stopped;tick+=DIAL_JAM_ROLLBACK_TIMEOUT_MS;Booster_DialCtrl();
  assert(stopped>stop_before && BOOSTER.Dail_state.Dial_Jam_Hold);
  Booster_RequestShot(-1);assert(!Booster_DialJammed());
  assert(fabsf(BOOSTER.Dail_state.Dial_Move_End-BOOSTER.Dail_state.Dial_Move_Start)<=45);

  reset(0);Booster_RequestShot(-1);feedback.angle_deg=-10;stall();
  assert(Booster_DialJammed());assert(Booster_DialTarget()==0); /* reverse jam returns inside original range */
  /* Recorded 13:11:41 false lock: 5534.0278deg vs 5534deg target at 4.9dps. */
  reset(5489);BOOSTER.Dail_state.Dial_Phase_Deg=-1;
  BOOSTER.Dail_state.Dial_Move_Start=5489;
  BOOSTER.Dail_state.Dial_Move_End=5534;
  BOOSTER.Dail_state.Dial_Move_Active=1;
  feedback.angle_deg=5534.0278f;feedback.speed_dps=4.9f;
  cycle();assert(!Booster_DialJammed());assert(Booster_DialArrived());
  assert(Booster_DialTarget()==5534);
  Booster_RequestShot(1);assert(Booster_DialTarget()==5579);
  assert(BOOSTER.Dail_state.Dial_Move_End-BOOSTER.Dail_state.Dial_Move_Start==45);
  feedback.angle_deg=5579.02f;feedback.speed_dps=0;cycle();
  assert(!Booster_DialJammed() && Booster_DialArrived());

  reset(0);Booster_RequestShot(-1);feedback.angle_deg=-45.028f;feedback.speed_dps=-4.9f;
  cycle();assert(!Booster_DialJammed() && Booster_DialArrived());
  assert(Booster_DialTarget()==-45);
  for(int i=0;i<50;i++) {
    feedback.angle_deg=-45.0f+((i%2)?0.02f:-0.02f);feedback.speed_dps=0;cycle();
    assert(!Booster_DialJammed() && Booster_DialTarget()==-45);
  }
  /* Recorded 13:26:10: normal low-speed holding 0.103deg beyond target must not lock. */
  reset(89);BOOSTER.Dail_state.Dial_Phase_Deg=-1;
  BOOSTER.Dail_state.Dial_Move_Start=89;BOOSTER.Dail_state.Dial_Move_End=134;
  BOOSTER.Dail_state.Dial_Move_Active=0;
  feedback.angle_deg=134.10324f;feedback.speed_dps=3.218f;
  cycle();assert(!Booster_DialJammed() && Booster_DialArrived());
  for(int i=0;i<1200;i++) {
    feedback.angle_deg=134.0f+((i%2)?0.18f:-0.18f);
    feedback.speed_dps=(i%2)?3.2f:-3.2f;cycle();
    assert(!Booster_DialJammed() && Booster_DialTarget()==134);
  }
  Booster_RequestShot(1);assert(Booster_DialTarget()==179);
  assert(BOOSTER.Dail_state.Dial_Move_End-BOOSTER.Dail_state.Dial_Move_Start==45);
  /* Continuous recorded waveform at 13:39:09: crossing 8dps is not a hard fault. */
  reset(449);BOOSTER.Dail_state.Dial_Phase_Deg=-1;
  BOOSTER.Dail_state.Dial_Move_Start=449;BOOSTER.Dail_state.Dial_Move_End=494;
  BOOSTER.Dail_state.Dial_Move_Active=0;
  const float hold_angle[]={493.9413f,493.9559f,493.9705f,493.9850f,494.0142f,
                            494.0506f,494.0724f,494.1089f,494.1380f,494.1453f,494.1307f};
  const float hold_speed[]={.4198f,1.5391f,2.9383f,4.0576f,6.2962f,
                            8.2551f,8.8147f,9.0946f,7.9752f,6.0164f,.6996f};
  for(unsigned i=0;i<sizeof(hold_angle)/sizeof(hold_angle[0]);i++) {
    feedback.angle_deg=hold_angle[i];feedback.speed_dps=hold_speed[i];cycle();
    assert(!Booster_DialJammed() && Booster_DialTarget()==494);
    if(fabsf(hold_speed[i])>DIAL_HOLD_SPD_DPS) {
      assert(!Booster_DialArrived());Booster_RequestShot(1);assert(Booster_DialTarget()==494);
    }
  }
  assert(Booster_DialArrived());Booster_RequestShot(1);assert(Booster_DialTarget()==539);
  assert(BOOSTER.Dail_state.Dial_Move_End-BOOSTER.Dail_state.Dial_Move_Start==45);
  /* 正常刹车过冲不再锁存：1.2° 过冲既不算越界，也不该让位置环失控。 */
  reset(0);Booster_RequestShot(1);feedback.angle_deg=46.2f;feedback.speed_dps=0;
  cycle();assert(!Booster_DialJammed() && !BOOSTER.Dail_state.Dial_Jam_Hold);
  assert(!Booster_DialArrived());         /* 1.2deg overshoot needs precision correction */
  feedback.angle_deg=45.5f;cycle();assert(Booster_DialArrived());
  Booster_RequestShot(1);assert(Booster_DialTarget()==90);
  reset(0);Booster_RequestShot(1);feedback.angle_deg=45.2f;feedback.speed_dps=50;
  cycle();assert(!Booster_DialJammed() && !Booster_DialArrived());
  Booster_RequestShot(1);assert(Booster_DialTarget()==45); /* still settling: no new shot */
  feedback.angle_deg=45.02f;feedback.speed_dps=0;cycle();
  assert(Booster_DialArrived());Booster_RequestShot(1);assert(Booster_DialTarget()==90);
  reset(0);Booster_RequestShot(-1);feedback.angle_deg=-45.2f;feedback.speed_dps=-50;
  cycle();assert(!Booster_DialJammed() && !Booster_DialArrived());
  feedback.angle_deg=-45.02f;feedback.speed_dps=0;cycle();
  assert(Booster_DialArrived());Booster_RequestShot(-1);assert(Booster_DialTarget()==-90);

  /* 冲过终点 5° 但没到故障门槛：只允许位置环回拉，不能判卡弹/锁存。 */
  reset(0);Booster_RequestShot(1);feedback.angle_deg=50;feedback.speed_dps=0;
  for(int i=0;i<DIAL_JAM_MS/3;i++)cycle();
  assert(!Booster_DialJammed() && !BOOSTER.Dail_state.Dial_Jam_Hold);
  assert(!Booster_DialArrived());

  /* 10° 故障门槛：9.9° 短暂停留不锁；回到区间内计时清零。 */
  reset(0);Booster_RequestShot(1);feedback.angle_deg=54.9f;feedback.speed_dps=0;
  for(int i=0;i<20;i++)cycle();assert(!Booster_DialJammed());
  feedback.angle_deg=45;feedback.speed_dps=0;cycle();
  assert(!Booster_DialJammed() && Booster_DialArrived());

  /* 真正冲出 10° 以上并持续 20ms：仍然锁存（方向/反馈失控）。 */
  reset(0);Booster_RequestShot(1);feedback.angle_deg=56.0f;feedback.speed_dps=0;
  for(int i=0;i<DIAL_OVERRUN_MS/3+1;i++)cycle();
  assert(Booster_DialJammed() && stopped>0 && BOOSTER.Dail_state.Dial_Jam_Hold);
  assert(Booster_DialTarget()==45);stop_before=stopped;
  feedback.angle_deg=45;feedback.speed_dps=0;cycle();assert(stopped>stop_before);
  Booster_RequestShot(1);assert(Booster_DialTarget()==45);
  /* Operator reverse still clears a real overrun without stacking a forward stroke. */
  feedback.angle_deg=56.0f;
  Booster_RequestShot(-1);cycle();assert(!Booster_DialJammed() && Booster_DialTarget()==0);
  reset(0);Booster_RequestShot(-1);feedback.angle_deg=-56.0f;feedback.speed_dps=0;
  for(int i=0;i<DIAL_OVERRUN_MS/3+1;i++)cycle();
  assert(Booster_DialJammed() && stopped>0 && BOOSTER.Dail_state.Dial_Jam_Hold);
  assert(Booster_DialTarget()==-45);
  reset(0);Booster_RequestShot(1);feedback.angle_deg=15;feedback.speed_dps=100;cycle();
  Booster_RequestShot(-1);assert(Booster_DialTarget()==0);
  assert(BOOSTER.Dail_state.Dial_Move_Start==45);

  reset(0);Booster_RequestShot(1);enabled=0;cycle();assert(stopped>0);
  Booster_RequestShot(1);assert(!BOOSTER.Dail_state.Dial_Move_Active);
  enabled=1;feedback.angle_deg=20;cycle();assert(Booster_DialTarget()==0);
  feedback.fb_ok=0;cycle();stop_before=stopped;Booster_RequestShot(-1);cycle();assert(stopped>stop_before);
  reset(0);enabled=0;s_phase_pending=-200;s_phase_requested=1;cycle();
  assert(s_phase_reply_type==0x81 && s_phase_reply==-200);
  enabled=1;cycle();assert(Booster_DialTarget()==-2);
  feedback.angle_deg=-2;feedback.speed_dps=0;cycle();Booster_RequestShot(1);
  assert(Booster_DialTarget()==43); /* phase changes initial angle, not 45-degree step */
  s_phase_pending=500;s_phase_requested=1;cycle();assert(Booster_DialTarget()==43);assert(s_phase_reply_type==0x82);
  reset(0);Booster_RequestShot(1);feedback.angle_deg=15;feedback.speed_dps=0;
  control_dt=.007f;
  for(int i=0;i<42;i++){tick+=7;Booster_DialCtrl();}
  assert(!Booster_DialJammed());tick+=7;Booster_DialCtrl();assert(Booster_DialJammed());
  control_dt=.003f;reset(0);Booster_RequestShot(1);feedback.angle_deg=60;feedback.speed_dps=100;
  control_dt=.007f;
  for(int i=0;i<2;i++){tick+=7;Booster_DialCtrl();}
  assert(!Booster_DialJammed());tick+=7;Booster_DialCtrl();assert(Booster_DialJammed());
  control_dt=.003f;
  puts("PASS: <=45 target spans, non-stacking requests, recorded sub-degree false lock, partial arrival without target drift, normal overshoot tolerated, 10deg sustained overrun latched, overshoot is not a jam, ratio3, fixed bounds, one rollback, no forward retry, timeout, reverse and reconnect");
}
'''
with tempfile.TemporaryDirectory(prefix='dial-jam-test-') as temp:
    path=Path(temp)
    (path/'dial_config.h').write_text((root/'robots/example/dial_config.h').read_text())
    (path/'app_RM_Motor.h').write_text('/* Hardware-independent configuration stub. */\n')
    (path/'test.c').write_text(code)
    exe=path/'test'
    subprocess.run(['cc','-std=c11','-I',str(path),str(path/'test.c'),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
