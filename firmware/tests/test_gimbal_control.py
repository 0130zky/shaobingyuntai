"""Compile modules/gimbal against stubs; test arming, rate limits, timeout release."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
GIMBAL = ROOT / 'modules' / 'gimbal'

MAIN_H = r'''
#pragma once
#include <stdint.h>
typedef struct {int dummy;} CAN_HandleTypeDef;
'''

DM_STUB = r'''
#pragma once
#include <stdint.h>
#include "main.h"
typedef struct {float p_min,p_max,v_min,v_max,t_min,t_max,kp_max,kd_max;} dm4310_range_t;
typedef struct {
  uint8_t ID,ERR; int Pos_int,Vel_int,Tor_int;
  float Position,Velocity,Torque; uint8_t T_MOS,T_Rotor;
} DM4310_MotorMeasure_t;
enum {ERR_Disable=0,ERR_Enable=1};
extern CAN_HandleTypeDef hcan1;
uint32_t HAL_GetTick(void);
void DM4310_Get_Ranged(DM4310_MotorMeasure_t *,const uint8_t *,const dm4310_range_t *);
uint8_t DM4310_SetEnable(CAN_HandleTypeDef *,uint32_t,uint8_t);
uint8_t DM4310_SendFrame(CAN_HandleTypeDef *,uint32_t,const uint8_t *);
uint8_t DM4310_Send_Ranged(CAN_HandleTypeDef *,uint32_t,float,float,float,float,float,
                           const dm4310_range_t *);
'''

RC_STUB = r'''
#pragma once
#include <stdint.h>
typedef struct {int16_t RightDirection_CH2,LeftTransverse_CH4;} ET07_Data_t;
typedef struct {ET07_Data_t ET07_DATA;} RC_Data_t;
extern RC_Data_t RC_Data;
uint8_t RC_LinkFresh(void);
uint8_t RC_MasterEnabled(void);
'''

DIAL_STUB = r'''
#pragma once
#define DIAL_DM_ESC_ID 0x02
#define DIAL_DM_MST_ID 0x12
'''

RTOS_STUB = r'''
#pragma once
#define taskENTER_CRITICAL()
#define taskEXIT_CRITICAL()
'''

HARNESS = r'''
#include <assert.h>
#include <stdlib.h>
#include <math.h>
#include "gimbal.h"
#include "gimbal_config.h"
#include "app_DM4310.h"
#include "RC_task.h"

CAN_HandleTypeDef hcan1;
RC_Data_t RC_Data;

static uint32_t tick = 1000;
uint32_t HAL_GetTick(void) { return tick; }

static int online = 1, master = 1;
uint8_t RC_LinkFresh(void) { return (uint8_t)online; }
uint8_t RC_MasterEnabled(void) { return (uint8_t)master; }

static float feed_pos, feed_err;
static unsigned enable_count, disable_count, status_queries;
static int reject_position;

typedef struct {float pos,vel,torq,kp,kd; unsigned n;} frame_t;
static frame_t frames[2];

static int slot(uint32_t id)
{
  if (id == GIMBAL_PITCH_ESC_ID) return 0;
  if (id == GIMBAL_YAW_ESC_ID) return 1;
  return -1;
}

uint8_t DM4310_Send_Ranged(CAN_HandleTypeDef *h, uint32_t id, float pos, float vel,
                           float torq, float kp, float kd, const dm4310_range_t *r)
{
  (void)h; (void)r;
  if (reject_position) return 0;
  const int i = slot(id);
  assert(i >= 0);
  frames[i].pos = pos; frames[i].vel = vel;
  frames[i].torq = torq; frames[i].kp = kp; frames[i].kd = kd;
  frames[i].n++;
  return 1;
}

uint8_t DM4310_SendFrame(CAN_HandleTypeDef *h,uint32_t id,const uint8_t *d) { (void)h; assert(id==0x7ff && (d[2]==0x33 || d[2]==0xcc)); if(d[2]==0xcc) status_queries++; return 1; }

uint8_t DM4310_SetEnable(CAN_HandleTypeDef *h, uint32_t id, uint8_t en)
{
  (void)h; assert(id == GIMBAL_PITCH_ESC_ID);
  if (en) enable_count++; else disable_count++;
  return 1;
}

void DM4310_Get_Ranged(DM4310_MotorMeasure_t *m, const uint8_t *d, const dm4310_range_t *r)
{
  (void)d; (void)r;
  m->Position = feed_pos;
  m->Velocity = 0.0f;
  m->Torque = 0.0f;
  m->ERR = (uint8_t)feed_err;
}

static void feed(uint32_t id, float pos, uint8_t err)
{
  static const uint8_t data[8] = {0};
  feed_pos = pos;
  feed_err = err;
  Gimbal_OnCanRx((uint16_t)id, data, 8);
}

int main(void)
{
  Gimbal_Init();
  online=1;master=0;
  feed(GIMBAL_PITCH_MST_ID,-1.759f,ERR_Disable);
  Gimbal_RequestLimit(1);Gimbal_ControlStep(.003f);
  int16_t value;uint8_t type;
  assert(Gimbal_TakeLimitReply(&value,&type) && type==0xb1);
  tick+=25;feed(GIMBAL_PITCH_MST_ID,-2.767f,ERR_Disable);
  Gimbal_RequestLimit(2);Gimbal_ControlStep(.003f);
  assert(Gimbal_TakeLimitReply(&value,&type) && type==0xb4);
  assert(abs(value+2021)<=1);
  RC_Data.ET07_DATA.RightDirection_CH2 = 1024;
  master=1; tick+=50;
  feed(GIMBAL_PITCH_MST_ID,-2.3f,ERR_Disable);
  Gimbal_RequestLimit(1); Gimbal_ControlStep(.003f);
  assert(Gimbal_TakeLimitReply(&value,&type) && type==0xbf);
  assert(enable_count==1 && frames[0].n==0);
  tick+=3; feed(GIMBAL_PITCH_MST_ID,-2.3f,ERR_Enable);
  Gimbal_ControlStep(.003f);
  assert(frames[0].n==1 && fabsf(frames[0].pos+2.3f)<1e-5f);
  /* A disturbance beyond the effort window must not erase the hold angle. */
  tick+=3; feed(GIMBAL_PITCH_MST_ID,-2.15f,ERR_Enable);
  Gimbal_ControlStep(.003f);
  const float hold_lead_rad = GIMBAL_PITCH_MAX_ERROR_DEG * 0.01745329252f;
  assert(fabsf(frames[0].pos - (-2.15f - hold_lead_rad)) < 1e-4f);
  /* 摇杆回中，速度前馈必须为 0，只留位置环。 */
  assert(fabsf(frames[0].vel) < 1e-6f);
  tick+=3; feed(GIMBAL_PITCH_MST_ID,-2.28f,ERR_Enable);
  Gimbal_ControlStep(.003f);
  assert(fabsf(frames[0].pos+2.3f)<1e-5f);

  RC_Data.ET07_DATA.RightDirection_CH2=1696;
  for(int i=0;i<1000;i++) {
    tick+=3; feed(GIMBAL_PITCH_MST_ID,-2.3f,ERR_Enable);
    Gimbal_ControlStep(.003f);
  }
  const float lead_rad = GIMBAL_PITCH_MAX_ERROR_DEG * 0.01745329252f;
  assert(fabsf(frames[0].pos - (-2.3f - lead_rad)) < 1e-4f);
  assert(frames[0].kp == GIMBAL_KP && frames[0].kd == GIMBAL_KD);
  /* 摇杆满行程：速度前馈 = 极性 × 30dps，单位是电机侧 rad/s。 */
  const float ff_rad = GIMBAL_PITCH_SIGN * GIMBAL_RATE_DPS * 0.01745329252f;
  assert(fabsf(frames[0].vel - ff_rad) < 1e-4f);
  RC_Data.ET07_DATA.RightDirection_CH2=352;
  tick+=3; feed(GIMBAL_PITCH_MST_ID,-2.3f,ERR_Enable);
  Gimbal_ControlStep(.003f);
  assert(frames[0].pos > -2.3f && frames[0].pos < -2.29f);
  assert(fabsf(frames[0].vel + ff_rad) < 1e-4f);

  RC_Data.ET07_DATA.RightDirection_CH2=1696;
  for(int i=0;i<250;i++) {
    tick+=3; feed(GIMBAL_PITCH_MST_ID,-2.72f,ERR_Enable);
    Gimbal_ControlStep(.003f);
  }
  const float margin_rad = GIMBAL_PITCH_STOP_MARGIN_DEG * 0.01745329252f;
  assert(fabsf(frames[0].pos - (-2.767f + margin_rad)) < 1e-4f);
  assert(fabsf(frames[0].vel) < 1e-6f);
  RC_Data.ET07_DATA.RightDirection_CH2=352;
  tick+=3; feed(GIMBAL_PITCH_MST_ID,-2.76f,ERR_Enable);
  Gimbal_ControlStep(.003f);
  assert(frames[0].vel > 0.0f);
  for(int i=0;i<250;i++) {
    tick+=3; feed(GIMBAL_PITCH_MST_ID,-1.80f,ERR_Enable);
    Gimbal_ControlStep(.003f);
  }
  assert(fabsf(frames[0].pos - (-1.759f - margin_rad)) < 1e-4f);
  assert(fabsf(frames[0].vel) < 1e-6f);
  RC_Data.ET07_DATA.RightDirection_CH2=1696;
  assert(frames[1].n==0);
  unsigned before_retry = frames[0].n, before_disable = disable_count;
  reject_position=1;
  tick+=3; feed(GIMBAL_PITCH_MST_ID,-2.72f,ERR_Enable);
  Gimbal_ControlStep(.003f);
  assert(frames[0].n==before_retry && disable_count==before_disable);
  reject_position=0;
  tick+=3; feed(GIMBAL_PITCH_MST_ID,-2.72f,ERR_Enable);
  Gimbal_ControlStep(.003f);
  assert(frames[0].n==before_retry+1 && Gimbal_TakePitchExit()==0);

  unsigned disabled=disable_count, queries=status_queries;
  tick+=GIMBAL_FB_POLL_AGE_MS;
  Gimbal_ControlStep(.003f);
  assert(status_queries==queries+1 && disable_count==disabled);
  assert(Gimbal_TakePitchExit()==0);
  unsigned commands=frames[0].n;
  tick+=GIMBAL_FB_TIMEOUT_MS-GIMBAL_FB_POLL_AGE_MS+1;
  Gimbal_ControlStep(.003f);
  assert(disable_count==disabled+1 && frames[0].n==commands);
  assert((Gimbal_TakePitchExit() & 255U)==3U);
  tick+=50; feed(GIMBAL_PITCH_MST_ID,-2.3f,ERR_Disable);
  Gimbal_ControlStep(.003f);
  assert(enable_count==1);
  tick+=101; Gimbal_ControlStep(.003f);
  assert(status_queries>0 && enable_count==1);
  master=0;
  tick+=3; feed(GIMBAL_PITCH_MST_ID,-2.3f,ERR_Disable);
  Gimbal_ControlStep(.003f);
  master=1; RC_Data.ET07_DATA.RightDirection_CH2=1024;
  tick+=50; feed(GIMBAL_PITCH_MST_ID,-2.3f,ERR_Disable);
  Gimbal_ControlStep(.003f);
  assert(enable_count==2);
  tick+=3; feed(GIMBAL_PITCH_MST_ID,-2.3f,ERR_Enable);
  Gimbal_ControlStep(.003f);
  reject_position=1;
  disabled=disable_count;
  for(unsigned elapsed=0; elapsed<=GIMBAL_TX_RETRY_MS; elapsed+=3U) {
    tick+=3; feed(GIMBAL_PITCH_MST_ID,-2.3f,ERR_Enable);
    Gimbal_ControlStep(.003f);
    assert(disable_count==disabled+(elapsed>=GIMBAL_TX_RETRY_MS));
  }
  assert(Gimbal_TakePitchExit()==(8U | (GIMBAL_TX_RETRY_MS<<8)));
  return 0;
}
'''

COLLISION_HARNESS = r'''
#include <assert.h>
#include <stdlib.h>
#include "gimbal.h"
#include "gimbal_config.h"
#include "app_DM4310.h"
#include "RC_task.h"

CAN_HandleTypeDef hcan1;
RC_Data_t RC_Data;
static unsigned sends, enables;
uint32_t HAL_GetTick(void) { return 1000; }
uint8_t RC_LinkFresh(void) { return 1; }
uint8_t RC_MasterEnabled(void) { return 1; }
uint8_t DM4310_Send_Ranged(CAN_HandleTypeDef *h, uint32_t id, float p, float v,
                           float t, float kp, float kd, const dm4310_range_t *r)
{ (void)h;(void)id;(void)p;(void)v;(void)t;(void)kp;(void)kd;(void)r; sends++; return 1; }
uint8_t DM4310_SendFrame(CAN_HandleTypeDef *h,uint32_t id,const uint8_t *d) { (void)h; assert(id==0x7ff && d[2]==0x33); return 1; }

uint8_t DM4310_SetEnable(CAN_HandleTypeDef *h, uint32_t id, uint8_t en)
{ (void)h;(void)id;(void)en; enables++; return 1; }
void DM4310_Get_Ranged(DM4310_MotorMeasure_t *m, const uint8_t *d, const dm4310_range_t *r)
{ (void)d;(void)r; m->Position = 0.0f; m->ERR = 1; }

int main(void)
{
  const uint8_t data[8] = {0};
  Gimbal_Init();
  Gimbal_OnCanRx((uint16_t)GIMBAL_PITCH_MST_ID, data, 8);
  Gimbal_OnCanRx((uint16_t)GIMBAL_YAW_MST_ID, data, 8);
  Gimbal_ControlStep(0.003f);
  /* 配置和拨盘 ID 撞车：一帧都不许发。 */
  assert(sends == 0 && enables == 0);
  return 0;
}
'''


def build(directory, harness, config_override=None, harness_name='harness.c'):
    directory = Path(directory)
    (directory / 'main.h').write_text(MAIN_H)
    (directory / 'app_DM4310.h').write_text(DM_STUB)
    (directory / 'RC_task.h').write_text(RC_STUB)
    (directory / 'dial_config.h').write_text(DIAL_STUB)
    (directory / 'FreeRTOS.h').write_text(RTOS_STUB)
    (directory / 'task.h').write_text('')
    source = directory / harness_name
    source.write_text(harness)

    module_source = GIMBAL / 'gimbal_control.c'
    if config_override is not None:
        # 引号 include 先找源文件自己所在目录，所以配置要和源码一起复制过去。
        patched = directory / 'cfg'
        patched.mkdir()
        text = (ROOT / 'robots/example/gimbal_config.h').read_text()
        for old, new in config_override:
            assert old in text, old
            text = text.replace(old, new)
        (patched / 'gimbal_config.h').write_text(text)
        module_source = patched / 'gimbal_control.c'
        module_source.write_text((GIMBAL / 'gimbal_control.c').read_text())

    binary = directory / 'test'
    subprocess.run([
        'cc', '-std=c11', '-Wall', '-Wextra',
        '-I', str(directory), '-I', str(GIMBAL),
        str(module_source), str(source), '-lm', '-o', str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True)


def main():
    with tempfile.TemporaryDirectory() as d:
        build(d, HARNESS)

    with tempfile.TemporaryDirectory() as d:
        build(d, COLLISION_HARNESS,
              [('#define GIMBAL_PITCH_MST_ID 0x00U', '#define GIMBAL_PITCH_MST_ID 0x12U')],
              'collision.c')
    print('PASS: Pitch calibration, measured-position takeover, disturbance recovery, reversal, tracking limit, CAN retry, exit reasons and ID collision gate')


if __name__ == '__main__':
    main()
