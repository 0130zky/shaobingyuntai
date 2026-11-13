"""Link the real shooter translation units; replace only hardware and PID math.

Run with: python -X utf8 tests/test_shooter_integration.py
The existing DM/PID tests separately exercise the real control calculations.
"""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
STUBS = {
    'main.h': r'''
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
uint32_t HAL_GetTick(void);
extern struct TestDwt {uint32_t CYCCNT;} test_dwt;
#define DWT (&test_dwt)
''',
    'app_RM_Motor.h': r'''
#pragma once
#include "main.h"
#define M3508_ID_12H 1
#define M3508_ID_4H 2
#define M3508_ID_8H 3
#define M3508_STD_ID 0x200
#define DIAL_CAN_ID 4
#define M3508_SpeedMaxOut 16384.0f
#define M3508_AngleMaxOut 16384.0f
#define Integral_Limit 1
#define OutputFilter 2
typedef struct {
 float Ref,Measure,Last_Measure,Err,Last_Err,Pout,ITerm,Last_ITerm,Iout,Dout,
       Last_Dout,Output,Last_Output,MaxOut;
 uint32_t DWT_CNT;
} PID_t;
typedef struct {int16_t speed,current;int8_t temperature;uint32_t rx_tick;} RM_MotorMeasure_t;
typedef struct {RM_MotorMeasure_t RM_MotorMeasure;PID_t MotorCtrl_Speed;} RM_Motor_t;
void Test_PID_Init(PID_t *,float);
#define PID_Init(p,maximum,...) Test_PID_Init(p,maximum)
void PID_CalculateDt(PID_t *,float,float,float);
void Get_RM_MotorInfo(RM_MotorMeasure_t *,uint8_t data[8]);
extern int test_bus;
#define BoosterBus test_bus
void Set_RM_MotorCurrent(int *,uint16_t,int16_t,int16_t,int16_t,int16_t,void *);
''',
    'bsp_dwt.h': '#pragma once\n#include "main.h"\nfloat DWT_GetTimeline_us(void);\n',
    'bsp_tools.h': '#pragma once\n#include <math.h>\n#define absf fabsf\n',
    'FreeRTOS.h': r'''
#pragma once
extern int test_critical;
#define taskENTER_CRITICAL() (++test_critical)
#define taskEXIT_CRITICAL() (--test_critical)
''',
    'task.h': '#pragma once\n',
    'master_task.h': '#pragma once\n',
    'RC_task.h': r'''
#pragma once
#include "main.h"
extern struct TestRC {struct {uint16_t SA_CH5,SC_CH7;} ET07_DATA;} RC_Data;
uint8_t RC_LinkFresh(void);
uint8_t RC_MasterEnabled(void);
int16_t RC_FrictionWheelRpm(void);
''',
}

HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "booster_task.h"
#include "dial_port.h"
#include "RC_task.h"
#include "app_RM_Motor.h"

struct TestDwt test_dwt;
struct TestRC RC_Data;
int test_bus,test_critical;
static uint32_t tick=1000;
static uint8_t online,master;
static int updates,frames,dm_frames,zero_writes;
static int16_t currents[4];
static float target,zero;
static dial_fb_t fb;
static uint8_t feed_enabled=1;
static int16_t wheel_speed[3],wheel_current[3];
static uint8_t wheel_temp[3];
static int dial_frames;
static shooter_status_t status(void){shooter_status_t s;assert(Booster_ReadStatus(&s));return s;}
static int16_t wheel_target(uint8_t i){return status().wheel[i].target_rpm;}

uint32_t HAL_GetTick(void){return tick;}
float DWT_GetTimeline_us(void){return (float)tick*1000;}
uint8_t RC_LinkFresh(void){return online;}
uint8_t RC_MasterEnabled(void){return online ? master : !BOOSTER_RC_ONLY;}
int16_t RC_FrictionWheelRpm(void){return online && master ? 5560 : 0;}
void Test_PID_Init(PID_t *p,float maximum){memset(p,0,sizeof *p);p->MaxOut=maximum;}
void PID_CalculateDt(PID_t *p,float measure,float ref,float dt_s){
 assert(dt_s>0);
 p->Ref=ref;p->Output=fmaxf(-p->MaxOut,fminf(p->MaxOut,ref-measure));
}
void Get_RM_MotorInfo(RM_MotorMeasure_t *m,uint8_t data[8]){
 m->speed=(int16_t)((uint16_t)data[2]<<8 | data[3]);
 m->current=(int16_t)((uint16_t)data[4]<<8 | data[5]);
 m->temperature=(int8_t)data[6];m->rx_tick=tick;
}
static void feed(uint8_t i,int16_t speed,int16_t current,uint8_t temperature){
 uint8_t data[8]={0,0,(uint8_t)((uint16_t)speed>>8),(uint8_t)speed,
                     (uint8_t)((uint16_t)current>>8),(uint8_t)current,temperature,0};
 Booster_OnCanRx(M3508_STD_ID+i+1,data,8);
}
void Dial_Port_OnCanRx(const uint8_t *data){assert(data);dial_frames++;}
void Dial_Port_Init(void){fb=(dial_fb_t){0};target=DIAL_PHASE_DEFAULT_DEG;}
void Dial_Port_SetZero(float z){zero=z;zero_writes++;}
void Dial_Port_Update(void){assert(test_critical>0);updates++;}
const dial_fb_t *Dial_Port_Get(void){return &fb;}
uint8_t Dial_Port_Settled(void){return fabsf(fb.angle_deg-target)<0.01f;}
void Dial_Port_ClearIntegral(void){}
void Dial_Port_Stop(void){fb.out=0;}
void Dial_Port_SetTarget(float t,float dt_s){assert(dt_s>0);target=t;fb.out=.125f;}
void Dial_Port_Tx(void){assert(frames==updates);dm_frames++;}
void Set_RM_MotorCurrent(int *bus,uint16_t id,int16_t a,int16_t b,int16_t c,int16_t d,void *mail){
 (void)mail;assert(bus==&test_bus && id==0x200);frames++;
 currents[0]=a;currents[1]=b;currents[2]=c;currents[3]=d;
 assert(frames==updates);assert(d==0);
}
static void step(uint32_t elapsed_ms,float dt_s){
 tick+=elapsed_ms;
 if(feed_enabled)for(int i=0;i<3;i++)feed(i,wheel_speed[i],wheel_current[i],wheel_temp[i]);
 Booster_ControlStep(dt_s);assert(test_critical==0);assert(frames==dm_frames);
}
static void cycle(void){step(3,.003f);}
static void reset(void){
 RC_Data=(struct TestRC){0};feed_enabled=1;
 memset(wheel_speed,0,sizeof wheel_speed);memset(wheel_current,0,sizeof wheel_current);memset(wheel_temp,0,sizeof wheel_temp);
 RC_Data.ET07_DATA.SA_CH5=353;online=1;master=0;
 BoosterInit();
 shooter_status_t empty;assert(!Booster_ReadStatus(&empty));assert(empty.sequence==0);
 uint8_t bytes[PC_TELEM_LEN];assert(Booster_BuildTelemetry(bytes)==0);
 fb.fb_ok=1;fb.angle_deg=DIAL_PHASE_DEFAULT_DEG;fb.speed_dps=0;
 cycle();
}
static void arrive(void){fb.angle_deg=target;fb.speed_dps=0;cycle();assert(status().dial_arrived);}
static void press(void){RC_Data.ET07_DATA.SC_CH7=1696;cycle();}
static void release(void){RC_Data.ET07_DATA.SC_CH7=353;cycle();}
static void near(float a,float b){assert(fabsf(a-b)<.001f);}
static void usb(uint8_t count,uint8_t sequence,uint8_t zero_byte,uint8_t corrupt){
 uint8_t bytes[PC_CMD_LEN]={PC_CMD_HEADER,7,0xb8,0x0b,0xb8,0x0b,0xb8,0x0b,sequence,count,zero_byte,0};
 for(int i=1;i<PC_CMD_LEN-1;i++)bytes[PC_CMD_LEN-1]^=bytes[i];
 bytes[PC_CMD_LEN-1]^=corrupt;
 Booster_UsbCmdHandler(bytes,sizeof bytes);
}
int main(void){
 const float phase=DIAL_PHASE_DEFAULT_DEG;
 if (DIAL_DM_CALIB_FREE) {
   reset();master=1;usb(1,1,32,0);press();
   for(int i=0;i<10;i++)cycle();
   assert(!status().enabled && fb.out==0);
   for(int i=0;i<4;i++)assert(currents[i]==0);
   puts("PASS: calibration build cannot drive wheels or apply dial movement");return 0;
 }

 reset();master=1;cycle();arrive();
 near(wheel_target(0),-5560);near(wheel_target(1),5560);
 near(wheel_target(2),-5560);
 press();near(target,phase+45);assert(currents[0]<0 && currents[1]>0 && currents[2]<0);
 release();press();near(target,phase+45); /* busy requests do not accumulate */
 arrive();cycle();near(target,phase+45); /* holding trigger does not replay a rejected request */
 release();press();near(target,phase+90);
 release();RC_Data.ET07_DATA.SA_CH5=1696;press();near(target,phase+45); /* cancel to original start */
 arrive();
 master=0;cycle();for(int i=0;i<4;i++)assert(currents[i]==0);
 master=1;cycle();arrive();cycle();near(target,phase+45); /* held trigger after enable */
 release();press();near(target,phase); /* manual reverse remains available */
 arrive();online=0;cycle();assert(fb.out==0);assert(wheel_target(0)==0);
 online=1;cycle();arrive();cycle();near(target,phase); /* reconnect cannot fire while held */
 release();RC_Data.ET07_DATA.SA_CH5=353;press();near(target,phase+45);
 fb.fb_ok=0;cycle();assert(fb.out==0);
 fb.fb_ok=1;fb.angle_deg=phase;cycle();arrive();cycle();near(target,phase);

 reset();master=1;cycle();arrive();RC_Data.ET07_DATA.SA_CH5=1024;
 press();near(target,phase); /* continuous mode must wait for the dwell */
 for(int i=0;i<49;i++){cycle();}
 near(target,phase);
 cycle();near(target,phase+45);
 release();arrive();for(int i=0;i<60;i++)cycle();near(target,phase+45);

 reset();online=0;cycle();
 int old_zero_writes=zero_writes;usb(250,1,32,0);
 assert(zero_writes==old_zero_writes);assert(!status().usb_link_ok);
 cycle();
#if BOOSTER_RC_ONLY
 assert(!status().usb_link_ok);assert(zero_writes==old_zero_writes);
 near(wheel_target(0),0);assert(fb.out==0);
#else
 assert(status().usb_link_ok);near(zero,32*(45.0f/256));
 near(wheel_target(0),-3000);near(target,phase); /* first count is baseline */
 arrive();usb(251,2,32,0);near(target,phase);cycle();near(target,phase+45);
 usb(252,3,32,0);cycle();near(target,phase+45);arrive();cycle();near(target,phase+45);
 uint8_t last=status().usb_sequence;
 usb(253,4,32,1);cycle();assert(status().usb_sequence==last);
 tick+=FRI_CMD_TIMEOUT_MS+1;cycle();assert(!status().usb_link_ok && fb.out==0);
 usb(10,5,32,0);cycle();arrive();near(target,phase+45); /* reconnect does not replay count */
 usb(11,6,32,0);tick+=FRI_CMD_TIMEOUT_MS+1;cycle();assert(!status().usb_link_ok);
 usb(20,7,32,0);cycle();arrive();online=1;master=1;release();
 usb(21,8,32,0);cycle();near(target,phase+45); /* remote owns control while online */
#endif
 /* Keep the old telemetry length, checksum, tail, signed speeds and slot layout. */
 uint8_t packet[PC_TELEM_LEN];assert(Booster_BuildTelemetry(packet)==PC_TELEM_LEN);
 assert(packet[0]==PC_TELEM_HEADER && packet[PC_TELEM_LEN-1]==PC_TELEM_TAIL);
 uint8_t sum=0;for(int i=1;i<PC_TELEM_LEN-2;i++)sum^=packet[i];
 assert(sum==packet[PC_TELEM_LEN-2]);

 /* A raw CAN frame cannot mutate the published cycle or caller-owned copies. */
 reset();master=1;wheel_speed[0]=-100;wheel_speed[1]=200;wheel_speed[2]=-300;
 wheel_current[0]=-12;wheel_current[1]=34;wheel_current[2]=-56;
 wheel_temp[0]=200;wheel_temp[1]=31;wheel_temp[2]=32;
 shooter_status_t before=status();
 feed(0,-2222,123,44);assert(status().sequence==before.sequence);
 assert(status().wheel[0].speed_rpm==before.wheel[0].speed_rpm);
 cycle();shooter_status_t sampled=status();
 assert(sampled.wheel[0].speed_rpm==-100 && sampled.wheel[1].speed_rpm==200);
 assert(sampled.wheel[2].speed_rpm==-300 && sampled.wheel[0].temperature_c==200);
 assert(sampled.wheel[0].received_ms==sampled.sample_ms);
 assert(sampled.sequence==before.sequence+1);
 near(sampled.wheel[0].output_raw,25);near(sampled.wheel[1].output_raw,-125);
 near(sampled.wheel[2].output_raw,225);
 shooter_status_t copy=sampled;copy.wheel[0].speed_rpm=999;
 assert(status().wheel[0].speed_rpm==-100 && copy.wheel[0].speed_rpm==999);
 assert(!Booster_ReadStatus(NULL));assert(!Booster_BuildTelemetry(NULL));

 /* Fixed wire fixtures catch sign, field-order, temperature and scaling changes. */
 const uint8_t wheel_bytes[30]={
   0x9c,0xff,0x48,0xea,0x19,0x00,0xf4,0xff,0xc8,0x00,
   0xc8,0x00,0xb8,0x15,0x83,0xff,0x22,0x00,0x1f,0x00,
   0xd4,0xfe,0x48,0xea,0xe1,0x00,0xc8,0xff,0x20,0x00};
 assert(Booster_BuildTelemetry(packet)==PC_TELEM_LEN);
 assert(memcmp(packet+PC_TELEM_WHEEL_OFF,wheel_bytes,sizeof wheel_bytes)==0);
 const uint8_t dial_bytes[10]={0xff,0xff,0xff,0xff,0,0,0,8,0,0};
 assert(memcmp(packet+PC_TELEM_DIAL_OFF,dial_bytes,sizeof dial_bytes)==0);
 shooter_referee_status_t judge={16.45f,65537,65538,20,1};
 float scope[25];assert(Booster_BuildScopeTelemetry(scope,&judge)==25);
 const float expected[25]={100,200,300,5560,16.45f,1,2,0,20,1,-25,-125,-225,
                           12,34,56,-1,-1,0,.125f,0,1,0,0,0};
 for(int i=0;i<25;i++)if(i!=7)near(scope[i],expected[i]);
 near(scope[7],(float)(sampled.sample_ms&65535));
 assert(!Booster_BuildScopeTelemetry(NULL,&judge));
 assert(!Booster_BuildScopeTelemetry(scope,NULL));

 /* Time and hardware can advance while a stopped control task's status freezes. */
 feed_enabled=0;tick+=20;feed(0,-2222,123,44);fb.angle_deg=123;
 float unchanged[25];assert(Booster_BuildScopeTelemetry(unchanged,&judge)==25);
 assert(memcmp(scope,unchanged,sizeof scope)==0);
 assert(status().sequence==sampled.sequence && status().sample_ms==sampled.sample_ms);
 uint32_t received=tick;cycle();
 assert(status().wheel[0].speed_rpm==-2222 && status().wheel[0].received_ms==received);
 assert(status().wheel[1].speed_rpm==200 && status().wheel[1].received_ms==sampled.sample_ms);
 uint8_t raw[8]={0};tick+=10;
 Booster_OnCanRx(0x201,raw,7);Booster_OnCanRx(0x201,NULL,8);Booster_OnCanRx(0x555,raw,8);
 cycle();assert(status().wheel[0].received_ms==received);
 int dial_before=dial_frames;
 Booster_OnCanRx(DIAL_RX_STD_ID,raw,8);Booster_OnCanRx(DIAL_RX_STD_ID,raw,8);
 Booster_OnCanRx(DIAL_RX_STD_ID,raw,7);assert(dial_frames==dial_before+2);
 tick+=101;cycle();for(int i=0;i<3;i++)near(status().wheel[i].output_raw,0);
 /* Keep long-running board timestamps wrapping in the existing wire fields. */
 tick=0xfffffff0U;cycle();sampled=status();
 assert(sampled.sample_ms==tick);assert(Booster_BuildScopeTelemetry(scope,&judge)==25);
 near(scope[7],(float)(tick&65535));tick=2;cycle();assert(status().sample_ms==5);
 judge.frame_age_ms=100000;Booster_BuildScopeTelemetry(scope,&judge);near(scope[8],65535);

 /* Irregular scheduling keeps ramp rates tied to time, with no gap catch-up. */
 reset();master=1;cycle();arrive();
 float prior=fabsf(status().wheel[0].output_raw);
 step(5,.005f);near(fabsf(status().wheel[0].output_raw),prior+125);
 step(1,.001f);near(fabsf(status().wheel[0].output_raw),prior+150);
 RC_Data.ET07_DATA.SC_CH7=1696;
 Booster_RequestDialPhase(500);
 int16_t held[3];for(int i=0;i<3;i++)held[i]=status().wheel[i].output_raw;
 step(25,.025f);assert(status().timing_fault && !status().enabled && fb.out==0);
 int16_t phase_reply;uint8_t phase_type;
 assert(Booster_TakeDialPhaseReply(&phase_reply,&phase_type) && phase_type==0x82);
 /* 一次空档不把三轮拉回零；连续第 3 拍异常才停机。 */
 for(int i=0;i<3;i++)near(status().wheel[i].output_raw,held[i]);
 step(25,.025f);
 for(int i=0;i<3;i++)near(status().wheel[i].output_raw,held[i]);
 step(25,.025f);
 for(int i=0;i<3;i++)near(status().wheel[i].output_raw,0);
 cycle();arrive();near(target,phase); /* no delayed shot after the pause */
 release();press();near(target,phase+45);
 step(1,NAN);assert(status().timing_fault && !status().enabled && fb.out==0);
 cycle();assert(!status().timing_fault);
 step(1,0);assert(status().timing_fault && fb.out==0);
 step(1,-.001f);assert(status().timing_fault && fb.out==0);
 printf("PASS: production shooter integration, RC_ONLY=%d, DM4310\n",BOOSTER_RC_ONLY);
}
'''

with tempfile.TemporaryDirectory(prefix='shooter-integration-') as tmp:
    directory = Path(tmp)
    for name, content in STUBS.items():
        (directory / name).write_text(content, encoding='utf-8')
    for name in ('dial_config.h', 'dial_port.h'):
        (directory / name).write_text(
            (ROOT / ('robots/example' if name == 'dial_config.h' else 'Components/Devices') / name).read_text(encoding='utf-8'), encoding='utf-8')
    (directory / 'test.c').write_text(HARNESS, encoding='utf-8')
    sources = [ROOT / 'Application/booster_task.c', *sorted((ROOT / 'modules/shooter').glob('*.c'))]
    for rc_only,calibration in ((0,0),(1,0),(0,1)):
        executable = directory / ('test.exe' if os.name == 'nt' else 'test')
        subprocess.run([
            'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
            f'-DBOOSTER_RC_ONLY={rc_only}', f'-DDIAL_DM_CALIB_FREE={calibration}',
            '-I', str(directory), '-I', str(ROOT / 'Application'),
            '-I', str(ROOT / 'modules/shooter'), '-I', str(ROOT / 'Components/Devices'),
            str(directory / 'test.c'), *(str(source) for source in sources),
            '-lm', '-o', str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)
