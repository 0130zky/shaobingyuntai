"""Run real Sentry control code against simulated CAN/RC/USB/Flash interfaces."""
from pathlib import Path
import subprocess, tempfile
root=Path(__file__).resolve().parents[1]
stub=r'''
#ifndef SENTRY_TEST_STUB
#define SENTRY_TEST_STUB
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <math.h>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define __DMB() ((void)0)
#define USBD_OK 0
static uint32_t tick=1000;static int hcan1,online=1,tx_count,enable_count,save_count,tx_ok=1;static int motion_count;
uint32_t HAL_GetTick(void){return tick;}
typedef struct {float p_min,p_max,v_min,v_max,t_min,t_max,kp_max,kd_max;} dm4310_range_t;
typedef struct {uint8_t ID,ERR;int Pos_int,Vel_int,Tor_int;float Position,Velocity,Torque;uint8_t T_MOS,T_Rotor;} DM4310_MotorMeasure_t;
#define DM4310_DEFAULT_BUS hcan1
#define ERR_Disable 0
#define ERR_Enable 1
static DM4310_MotorMeasure_t sample;
void DM4310_Get_Ranged(DM4310_MotorMeasure_t *m,const uint8_t *d,const dm4310_range_t *r){(void)d;(void)r;*m=sample;}
uint8_t DM4310_SetEnable(void *b,uint32_t id,uint8_t e){(void)b;(void)id;tx_count++;enable_count+=e;return tx_ok;}
uint8_t DM4310_Send_Ranged(void *b,uint32_t id,float p,float v,float t,float kp,float kd,const dm4310_range_t *r){(void)b;(void)id;(void)p;(void)v;(void)t;(void)kp;(void)kd;(void)r;tx_count++;
 if(kp>0 && tx_ok) {motion_count++;unsigned raw=(unsigned)((p+12.5f)*65535/25);
  if(id==1)assert(raw>=38800 && raw<=45628);else if(id==2)assert(raw>=29762 && raw<=36039);
 }return tx_ok;}
static uint16_t command_raw[2];
uint8_t DM4310_Send_PositionRaw(void *b,uint32_t id,uint16_t raw,float v,float t,float kp,float kd,const dm4310_range_t *r){
 (void)b;(void)v;(void)t;(void)kp;(void)kd;(void)r;tx_count++;
 if(tx_ok){motion_count++;if(id==1){command_raw[0]=raw;}
 else {assert(id==2);command_raw[1]=raw;}}
 return tx_ok;
}
uint8_t DM4310_SendFrame(void *b,uint32_t id,const uint8_t *d){(void)b;(void)id;(void)d;tx_count++;return tx_ok;}
static struct {struct {int SB_CH6,SA_CH5,RightDirection_CH2;} ET07_DATA;} RC_Data={ {352,352,1024} };
uint8_t RC_LinkFresh(void){return online;}
uint8_t CDC_TransmitReady_FS(void){return 0;}
uint8_t CDC_Transmit_FS(uint8_t *b,uint16_t n){(void)b;(void)n;return 0;}
#endif
'''
harness=r'''
#include "test_stub.h"
#include "sentry_remote.h"
#include "sentry.c"
#include <stdio.h>
static SentryCalibration saved;
int Sentry_Load(SentryCalibration *c){if(!saved.mask)return 0;*c=saved;return 1;}
int Sentry_Save(const SentryCalibration *c){saved=*c;save_count++;return 1;}
static void remote(int s1,int left,int right){
 sentry_remote.seen=online;sentry_remote.tick=tick;sentry_remote.s1=(uint8_t)s1;sentry_remote.s2=left==1684?3:2;
 sentry_remote.channel[3]=(uint16_t)left;sentry_remote.channel[1]=(uint16_t)right;
}
static void feed(unsigned i,unsigned raw,unsigned state){
 sample=(DM4310_MotorMeasure_t){.Position=RawPosition(raw),.Velocity=state==1?axis[i].v:0,.ERR=(uint8_t)state,.Pos_int=(int)raw};
 uint8_t d[8]={(uint8_t)esc[i]};Sentry_Rx(mst[i],d,8);
}
static unsigned midpitch(void){return 29763;}
static void step(int s1,int left,int right,unsigned r0,unsigned r1,unsigned state){
 tick+=3;remote(s1,left,right);feed(0,r0,state);feed(1,r1,state);Sentry_Control(.003f);
}
static void cmd(const char *s){Sentry_UsbRx((const uint8_t *)s,(uint32_t)strlen(s));Sentry_Control(.003f);}
static void rearm(void){step(1,1024,1024,42000,midpitch(),0);assert(armed&&!fault);}
static void follow(int left,int right,unsigned *f,unsigned *p){
 step(2,left,right,*f,*p,1);*f=command_raw[0];*p=command_raw[1];
}
int main(void){
 Sentry_Init();assert(valid&&!calibration&&!armed);
 assert(cal.raw[0][0]==45628 && cal.raw[0][1]==38800 && cal.raw[1][0]==29762 && cal.raw[1][1]==36039);
 SentryCalibration old=cal;old.raw[0][0]=45659;old.end[0][0]=RawPosition(45659);assert(!CalValid(&old));
 step(2,1024,1024,42000,midpitch(),0);assert(!armed&&!enable_count);
 assert(status.fold_pitch_target==29763 && status.open_pitch_target==34370 && status.prep_pitch_target==0 && status.parallel_motion==1 && status.unfold_allowed);
 step(3,1024,1024,42000,midpitch(),0);assert(armed&&!enable_count);
 rearm();
#if SENTRY_HARDWARE_READY
 // S2 is an intentional sequence request, so only the right stick gates enable.
 rearm();step(2,1024,1684,42000,midpitch(),0);assert(fault==9&&!armed);
 rearm();for(int i=0;i<90;i++)step(2,1024,1024,42000,midpitch(),0);
 step(2,1024,1024,42000,midpitch(),1);assert(axis[0].active&&axis[1].active);
 // Parallel closing updates both motor references immediately, including from an open pose.
 step(1,1024,1024,38890,34370,0);
 step(2,1024,1024,38890,34370,1);
 assert(linkage.mode==Z_LINKED&&linkage.fold.p>38890&&linkage.pitch.p<34370);
 unsigned f=command_raw[0],p=command_raw[1];int overlap=0;
 for(int i=0;i<18000;i++){
  unsigned before_f=f,before_p=p;follow(364,1024,&f,&p);
  if(f>before_f&&p<before_p)overlap=1;
 }
 assert(!fault&&overlap&&linkage.mode==Z_FOLDED&&abs((int)p-29763)<2);
 // Parallel opening also advances both references in overlapping control ticks.
 overlap=0;
 for(int i=0;i<17000;i++){
  unsigned before_f=f,before_p=p;follow(1684,1024,&f,&p);
  if(f<before_f&&p>before_p)overlap=1;
 }
 assert(!fault&&overlap&&linkage.mode==Z_OPEN&&abs((int)p-34370)<2);
 int prior=motion_count;
 // S2 up also disables with S1 down.
 tick+=3;remote(2,1024,1024);sentry_remote.s2=1;feed(0,f,0);feed(1,p,0);
 Sentry_Control(.003f);assert(!axis[0].active&&!axis[1].active);
 // A parallel request outside old bounds starts with only its acceleration-limited step.
 const unsigned outside[4][2]={{29245,33057},{45629,32125},{42000,29761},{42000,36040}};
 for(unsigned k=0;k<4;k++) {
  step(1,1024,1024,outside[k][0],outside[k][1],0);
  tick+=3;remote(2,1024,1024);sentry_remote.s2=2;
  feed(0,outside[k][0],1);feed(1,outside[k][1],1);Sentry_Control(.003f);
  assert(!fault&&armed&&axis[0].active&&axis[1].active);
  float count_scale=65535.0f/(2*SENTRY_P_MAX);
  assert(abs((int)command_raw[0]-(int)outside[k][0])<=ceilf(SENTRY_FOLD_ACCEL*count_scale*.003f*.003f)+1);
  assert(abs((int)command_raw[1]-(int)outside[k][1])<=ceilf(SENTRY_PITCH_ACCEL*count_scale*.003f*.003f)+1);
 }
 rearm();step(2,1024,1024,42000,midpitch(),1);tick+=101;remote(2,1024,1024);prior=motion_count;Sentry_Control(.003f);assert(fault==3&&!armed&&motion_count==prior);
 rearm();step(2,1024,1024,42000,midpitch(),1);prior=motion_count;Sentry_Control(.1f);assert(fault==2&&!armed&&motion_count==prior);
 rearm();step(2,1024,1024,42000,midpitch(),1);online=0;step(2,1024,1024,42000,midpitch(),1);assert(fault==1&&!armed);online=1;
 rearm();step(2,1024,1024,42000,midpitch(),1);tx_ok=0;for(int i=0;i<6;i++)step(2,1024,1024,42000,midpitch(),1);assert(fault==6&&!armed);tx_ok=1;
 // Legacy Fold 0 status still requires post-enable feedback before position TX.
 rearm();for(int i=0;i<100;i++){tick+=3;remote(2,1024,1024);feed(0,42000,0);feed(1,midpitch(),1);Sentry_Control(.003f);}
 assert(axis[0].active&&axis[1].active&&!fault);
#else
 assert(tx_count==0&&enable_count==0);(void)follow;
#endif
 step(1,1024,1024,42000,midpitch(),0);cmd("CAL BEGIN\n");assert(calibration&&!valid);
 tick+=250;remote(1,1024,1024);
 feed(0,45628,0);cmd("CAL FOLD A\n");feed(0,38800,0);cmd("CAL FOLD B\n");feed(1,29762,0);cmd("CAL PITCH A\n");feed(1,36039,0);cmd("CAL PITCH B\n");assert(CalValid(&cal));
 feed(0,42000,0);feed(1,midpitch(),0);cmd("CAL SAVE\n");assert(valid&&save_count==1);
 unsigned seq=sequence;cmd("STATUS\nSTATUS\n");assert(sequence==seq+1);Sentry_Control(.003f);assert(sequence==seq+2);
 return 0;
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td);(p/'test_stub.h').write_text(stub)
 for name in ('app_DM4310.h','RC_task.h','FreeRTOS.h','task.h','usbd_cdc_if.h'):
  (p/name).write_text('#include "test_stub.h"\n')
 (p/'harness.c').write_text(harness)
 for ready in (0,1):
  cmd=['cc','-std=c11','-Wall','-Wextra','-Werror',f'-DSENTRY_HARDWARE_READY={ready}',
       '-I'+td,'-I'+str(root/'modules/folding_gimbal'),'-I'+str(root/'robots/sentry'),
       str(p/'harness.c'),str(root/'modules/folding_gimbal/z_linkage.c'),str(root/'modules/folding_gimbal/motion.c'),str(root/'modules/folding_gimbal/sentry_remote.c'),'-lm','-o',str(p/'test')]
  subprocess.run(cmd,check=True);subprocess.run([str(p/'test')],check=True)
  print(f'PASS Sentry ready={ready}: calibration, save gate, rearm, watchdogs, input framing, trajectory')
# Exercise actual Flash record/commit logic using a byte-addressable fake sector.
flash_stub=r'''
#ifndef FLASH_STUB
#define FLASH_STUB
#include <stdint.h>
#include <string.h>
#include <assert.h>
#define HAL_OK 0
#define FLASH_TYPEERASE_SECTORS 0
#define FLASH_SECTOR_11 11
#define FLASH_SECTOR_7 7
#define FLASH_BANK_1 1
#define FLASH_TYPEPROGRAM_FLASHWORD 1
#define FLASH_VOLTAGE_RANGE_3 3
#define FLASH_TYPEPROGRAM_WORD 0
static uint8_t sector[256] __attribute__((aligned(32)));static int fail_word=-1,words,locked=1;
#ifdef SENTRY_STM32H7
static int tear_bytes;
#endif
typedef struct {unsigned TypeErase,Sector,NbSectors,VoltageRange,Banks;} FLASH_EraseInitTypeDef;
static int HAL_FLASH_Unlock(void){locked=0;return 0;}
static void HAL_FLASH_Lock(void){locked=1;}
static int HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e,uint32_t *err){(void)e;(void)err;memset(sector,255,sizeof sector);words=0;return 0;}
static int HAL_FLASH_Program(unsigned t,uintptr_t a,uintptr_t w){
#ifdef SENTRY_STM32H7
 assert(t==FLASH_TYPEPROGRAM_FLASHWORD && !(a%32) && !(w%32));
 if(words++==fail_word){memcpy((void*)a,(const void*)w,(unsigned)tear_bytes);return 1;}
 memcpy((void*)a,(const void*)w,32);return 0;
#else
 (void)t;if(words++==fail_word)return 1;uint32_t word=(uint32_t)w;memcpy((void*)a,&word,4);return 0;
#endif
}
#endif
'''
flash_test=r'''
#include <assert.h>
#include "stm32f4xx_hal.h"
#define STORE_ADDRESS ((uintptr_t)sector)
#include "storage.c"
int main(void){
 SentryCalibration c={.end={{-1,1},{-.5,.5}},.mask=15},out;
 memset(sector,255,sizeof sector);assert(!Sentry_Load(&out));
 assert(Sentry_Save(&c)&&locked&&Sentry_Load(&out));assert(!memcmp(&c,&out,sizeof c));
 sector[12]^=1;assert(!Sentry_Load(&out));
 assert(Sentry_Save(&c));Record *r=(Record*)sector;r->identity.ids[0]++;r->hash=Hash(r);assert(!Sentry_Load(&out));
 #ifdef SENTRY_STM32H7
 for(int i=0;i<(int)(sizeof(Record)/32);i++) for(int n=0;n<32;n++) {
  fail_word=i;tear_bytes=n;assert(!Sentry_Save(&c));assert(locked && !Sentry_Load(&out));
 }
#else
 for(int i=0;i<(int)(sizeof(Record)/4);i++) {fail_word=i;assert(!Sentry_Save(&c));assert(locked && !Sentry_Load(&out));}
#endif
 fail_word=-1;assert(Sentry_Save(&c));c.mask=3;assert(!Sentry_Save(&c));assert(Sentry_Load(&out));
 return 0;
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td);(p/'stm32f4xx_hal.h').write_text(flash_stub);(p/'stm32h7xx_hal.h').write_text(flash_stub);(p/'flash.c').write_text(flash_test)
 for family in ('F407','H723'):
  defines=['-DSENTRY_STM32H7'] if family=='H723' else []
  subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',*defines,'-I'+td,
   '-I'+str(root/'modules/folding_gimbal'),'-I'+str(root/'robots/sentry'),str(p/'flash.c'),
   str(root/'modules/folding_gimbal/motion.c'),'-lm','-o',str(p/'test')],check=True)
  subprocess.run([str(p/'test')],check=True)
  print(f'PASS Flash {family}: identity, checksum, partial write at every unit, commit marker, readback')
