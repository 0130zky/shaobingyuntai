/**
 *  ███╗   ██╗███████╗██╗  ██╗████████╗  ███████╗
 *  ████╗  ██║██╔════╝╚██╗██╔╝╚══██╔══╝  ██╔════╝
 *  ██╔██╗ ██║█████╗   ╚███╔╝    ██║     █████╗
 *  ██║╚██╗██║██╔══╝   ██╔██╗    ██║     ██╔══╝
 *  ██║ ╚████║███████╗██╔╝ ██╗   ██║     ███████╗
 *  ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝   ╚═╝     ╚══════╝
 *
 *  @Author : Kaiyue Zhang
 *  @Email  : kaiyue.link@gmail.com
 *  @date   : 2026-11-08
 *  @description ：云台两轴 DM4310 的 MIT 位置控制：摇杆积分、限位、松力与失联保护。
 */
#include <math.h>
#include "gimbal.h"
#include "gimbal_config.h"
#include "app_DM4310.h"
#include "RC_task.h"
#include "dial_config.h"
#include "FreeRTOS.h"
#include "task.h"

#define GIMBAL_AXIS_NUM 2U
#define GIMBAL_RAD_TO_DEG 57.295779513f
#define GIMBAL_DEG_TO_RAD 0.01745329252f

typedef struct
{
  DM4310_MotorMeasure_t fb;   /* ISR 写，控制拍在临界区里整体拷走 */
  uint32_t rx_ms;             /* 最近一帧的时刻，ISR 写 */
  uint8_t seen;               /* 收到过真实反馈，ISR 写 */
  float target_deg;           /* 控制拍维护的目标角 */
  uint8_t active;             /* 1 = 本拍正在发位置帧 */
} gimbal_axis_t;

static gimbal_axis_t s_axis[GIMBAL_AXIS_NUM];
static uint8_t s_armed;

/* 必须和电机里配的量程一致；见 gimbal_config.h。 */
static const dm4310_range_t s_range =
{
  GIMBAL_P_MIN, GIMBAL_P_MAX,
  GIMBAL_V_MIN, GIMBAL_V_MAX,
  GIMBAL_T_MIN, GIMBAL_T_MAX,
  GIMBAL_KP_MAX, GIMBAL_KD_MAX
};

static const uint32_t s_esc_id[GIMBAL_AXIS_NUM] =
{ GIMBAL_PITCH_ESC_ID, GIMBAL_YAW_ESC_ID };
static const uint32_t s_mst_id[GIMBAL_AXIS_NUM] =
{ GIMBAL_PITCH_MST_ID, GIMBAL_YAW_MST_ID };
static const float s_sign[GIMBAL_AXIS_NUM] =
{ GIMBAL_PITCH_SIGN, GIMBAL_YAW_SIGN };
static const float s_zero[GIMBAL_AXIS_NUM] =
{ GIMBAL_PITCH_ZERO_RAD, GIMBAL_YAW_ZERO_RAD };
static const float s_min_deg[GIMBAL_AXIS_NUM] =
{ GIMBAL_PITCH_MIN, GIMBAL_YAW_MIN };
static const float s_max_deg[GIMBAL_AXIS_NUM] =
{ GIMBAL_PITCH_MAX, GIMBAL_YAW_MAX };

/**
 * @description  : 配置自检：ID 不能为 0、不能撞 M3508 电流帧/拨盘、不能互撞
 * @return        {uint8_t} 1 = 可以发帧
 */
static uint8_t ConfigValid(void)
{
#if !GIMBAL_READY
  return 0;
#else
  for (uint8_t i = 0; i < GIMBAL_AXIS_NUM * 2U; ++i)
  {
    const uint32_t id = (i < GIMBAL_AXIS_NUM) ? s_esc_id[i] : s_mst_id[i - GIMBAL_AXIS_NUM];
    if ((i < GIMBAL_AXIS_NUM && !id) || id > 0x7FFU) return 0;                  /* 标准帧范围 */
    if (id >= 0x201U && id <= 0x204U) return 0;        /* M3508 电流帧 */
    if (id == DIAL_DM_ESC_ID || id == DIAL_DM_MST_ID) return 0;
    for (uint8_t j = 0; j < i; ++j)
    {
      const uint32_t prev = (j < GIMBAL_AXIS_NUM) ? s_esc_id[j] : s_mst_id[j - GIMBAL_AXIS_NUM];
      if (id == prev) return 0;
    }
  }
  if (s_range.p_max <= s_range.p_min || s_range.v_max <= s_range.v_min ||
      s_range.t_max <= s_range.t_min || s_range.kp_max <= 0.0f ||
      s_range.kd_max <= 0.0f) return 0;
  for (uint8_t i = 0; i < GIMBAL_AXIS_NUM; ++i)
  {
    if (s_sign[i] != 1.0f && s_sign[i] != -1.0f) return 0;
    if (s_min_deg[i] >= s_max_deg[i]) return 0;
    /* 限位换算回电机位置后不能越出 MIT 帧窗口，否则目标会被窗口边界静默夹住。 */
    const float pos_a = s_zero[i] + s_sign[i] * s_min_deg[i] * GIMBAL_DEG_TO_RAD;
    const float pos_b = s_zero[i] + s_sign[i] * s_max_deg[i] * GIMBAL_DEG_TO_RAD;
    const float pos_lo = (pos_a < pos_b) ? pos_a : pos_b;
    const float pos_hi = (pos_a < pos_b) ? pos_b : pos_a;
    if (pos_lo < s_range.p_min || pos_hi > s_range.p_max) return 0;
  }
  return 1;
#endif
}

/**
 * @description  : 摇杆归一化，中位死区 0.04
 * @param         {int16_t} raw  352..1696，中位 1024
 * @return        {float} [-1,1]
 */
static float Stick(int16_t raw)
{
  float x = ((float)raw - 1024.0f) / 672.0f;
  if (x > 1.0f) x = 1.0f;
  if (x < -1.0f) x = -1.0f;
  if (fabsf(x) < 0.04f) return 0.0f;
  return x;
}

/**
 * @description  : 发一帧位置给定。target_deg 是坐标角，换算回电机输出轴。
 * @param         {float} vel_rad_s 电机输出轴速度前馈（rad/s）。摇杆匀速跟踪靠它，
 *                 不然高 KP 下要靠位置误差硬拽，一动就打满窗口。
 * @return        {uint8_t} 1 = 发送成功
 */
static uint8_t CommandAxis(uint8_t idx, float target_deg, float vel_rad_s)
{
  const float pos_rad = s_zero[idx] + s_sign[idx] * target_deg * GIMBAL_DEG_TO_RAD;
  return DM4310_Send_Ranged(&GIMBAL_BUS, s_esc_id[idx],
                            pos_rad, vel_rad_s, 0.0f,
                            GIMBAL_KP, GIMBAL_KD, &s_range);
}

void Gimbal_Init(void)
{
  for (uint8_t i = 0; i < GIMBAL_AXIS_NUM; ++i)
    s_axis[i] = (gimbal_axis_t){0};
  /* 上电后必须先看到一次 CH6 关，才允许进入位置控制。 */
  s_armed = 0;
}

void Gimbal_OnCanRx(uint16_t id, const uint8_t *data, uint8_t length)
{
  if (length != 8U || !ConfigValid()) return;
  for (uint8_t i = 0; i < GIMBAL_AXIS_NUM; ++i)
  {
    if (id != s_mst_id[i]) continue;
    DM4310_Get_Ranged(&s_axis[i].fb, data, &s_range);
    s_axis[i].rx_ms = HAL_GetTick();
    s_axis[i].seen = 1;
  }
}

/* Read-only discovery. Inspect these records through SWD; no motor writes. */
typedef struct { uint32_t esc, mst, rid, value, tick; } dm_read_record_t;
volatile dm_read_record_t g_dm_reads[96];
volatile uint32_t g_dm_read_count;
volatile uint32_t g_can_seen[2048];
static uint16_t probe_id=1;
static uint8_t probe_reg;
static uint32_t probe_tick;
static uint8_t probe_sent;
static const uint8_t probe_regs[]={7,8,10,21,22,23,7,8,10,21,22,23};
uint8_t Gimbal_DiagnosticRx(uint16_t id, const uint8_t *data, uint8_t length)
{
  if(id<2048) g_can_seen[id]++;
  if(length!=8 || !probe_sent || data[2]!=0x33 || data[3]!=probe_regs[probe_reg] ||
     ((uint16_t)data[0]|((uint16_t)data[1]<<8))!=probe_id) return 0;
  uint32_t n=g_dm_read_count;
  if(n<96) {
    g_dm_reads[n]=(dm_read_record_t){probe_id,id,data[3],
      (uint32_t)data[4]|((uint32_t)data[5]<<8)|((uint32_t)data[6]<<16)|((uint32_t)data[7]<<24),HAL_GetTick()};
    g_dm_read_count=n+1;
  }
  return 1; /* Register reply must never be decoded as position feedback. */
}
static void DiagnosticStep(void)
{
  if(probe_id>=2) return;
  uint32_t now=HAL_GetTick();
  if(now<1000U) return;
  if(probe_sent && now-probe_tick<100) return;
  if(probe_sent) {
    if(++probe_reg==sizeof(probe_regs)) {probe_reg=0;probe_id++;}
  }
  if(probe_id>=2) return; /* One bounded scan of standard IDs, no repeats. */
  uint8_t data[8]={(uint8_t)probe_id,(uint8_t)(probe_id>>8),0x33,probe_regs[probe_reg],0,0,0,0};
  probe_sent=0;
  probe_tick=now;
  if(DM4310_SendFrame(&GIMBAL_BUS,0x7ff,data)) probe_sent=1;
}

/* Manual endpoint capture while CH6 is off. */
static volatile uint8_t cal_request;
static int16_t cal_value;
static uint8_t cal_reply;
static float endpoints[2];
static uint8_t endpoint_mask;
void Gimbal_RequestLimit(uint8_t endpoint) { if(endpoint==1 || endpoint==2) cal_request=endpoint; }
uint8_t Gimbal_TakeLimitReply(int16_t *value, uint8_t *type) {
  taskENTER_CRITICAL();
  uint8_t ready=cal_reply;
  *value=cal_value; *type=cal_reply; cal_reply=0;
  taskEXIT_CRITICAL();
  return ready!=0;
}
enum {
  PITCH_EXIT_SWITCH_OFF = 1,
  PITCH_EXIT_RC_LOST,
  PITCH_EXIT_FEEDBACK_TIMEOUT,
  PITCH_EXIT_ANGLE_LIMIT,
  PITCH_EXIT_DRIVE_FAULT,
  PITCH_EXIT_DRIVE_DISABLED,
  PITCH_EXIT_STICK,
  PITCH_EXIT_TX_TIMEOUT
};

static uint16_t s_pitch_exit;
static void PitchExit(uint8_t reason, uint8_t detail)
{
  if (s_armed && !s_pitch_exit)
    s_pitch_exit = reason | ((uint16_t)detail << 8);
  s_armed = 0;
}

uint16_t Gimbal_TakePitchExit(void)
{
  taskENTER_CRITICAL();
  const uint16_t event = s_pitch_exit;
  s_pitch_exit = 0;
  taskEXIT_CRITICAL();
  return event;
}

static void PitchStep(const gimbal_axis_t *sample, float dt_s, uint32_t now,
                      uint8_t rc_online, uint8_t enabled)
{
  static uint32_t last_disable, last_enable, tx_failed_at;
  static uint8_t tx_retry, was_requested;
  gimbal_axis_t *axis = &s_axis[0];
  const float measured = GIMBAL_PITCH_SIGN *
      (sample->fb.Position - GIMBAL_PITCH_ZERO_RAD) * GIMBAL_RAD_TO_DEG;
  const float stick = Stick(RC_Data.ET07_DATA.RightDirection_CH2);
  const uint32_t age = now - sample->rx_ms;
  const uint8_t was_enabled = was_requested;
  was_requested = enabled;
  if (!rc_online) {
    if (was_enabled || axis->active)
      PitchExit(PITCH_EXIT_RC_LOST, 0);
    else
      s_armed = 0;
  } else if (!enabled) {
    if (axis->active)
      PitchExit(PITCH_EXIT_SWITCH_OFF, 0);
    else
      s_armed = 1;
  } else if (s_armed) {
    if (!sample->seen || age > GIMBAL_FB_TIMEOUT_MS)
      PitchExit(PITCH_EXIT_FEEDBACK_TIMEOUT, !sample->seen || age > 255U ? 255U : (uint8_t)age);
    else if (sample->fb.ERR > ERR_Enable)
      PitchExit(PITCH_EXIT_DRIVE_FAULT, sample->fb.ERR);
    else if (axis->active && sample->fb.ERR != ERR_Enable)
      PitchExit(PITCH_EXIT_DRIVE_DISABLED, sample->fb.ERR);
    else if (!isfinite(measured) || measured < GIMBAL_PITCH_MIN || measured > GIMBAL_PITCH_MAX)
      PitchExit(PITCH_EXIT_ANGLE_LIMIT, !isfinite(measured) ? 3U : measured < GIMBAL_PITCH_MIN ? 1U : 2U);
    else if (!axis->active && stick != 0.0f)
      PitchExit(PITCH_EXIT_STICK, 0);
    else if (tx_retry && now - tx_failed_at >= GIMBAL_TX_RETRY_MS)
      PitchExit(PITCH_EXIT_TX_TIMEOUT, GIMBAL_TX_RETRY_MS);
  }

  if (enabled && s_armed) {
    if (sample->fb.ERR == ERR_Disable) {
      if (now - last_enable >= 50U && DM4310_SetEnable(&GIMBAL_BUS, GIMBAL_PITCH_ESC_ID, 1))
        last_enable = now;
      return;
    }
    const float soft_min = GIMBAL_PITCH_MIN + GIMBAL_PITCH_STOP_MARGIN_DEG;
    const float soft_max = GIMBAL_PITCH_MAX - GIMBAL_PITCH_STOP_MARGIN_DEG;
    float target = measured;
    float rate = 0.0f;
    if (axis->active) {
      target = axis->target_deg;
      if (stick != 0.0f) {
        /* Clear pending error in the old direction before applying the stick. */
        if (stick * (target - measured) < 0.0f)
          target = measured;
        const float distance = stick > 0.0f ? soft_max - measured : measured - soft_min;
        const float scale = fminf(fmaxf(distance / GIMBAL_PITCH_SLOW_ZONE_DEG, 0.0f), 1.0f);
        rate = stick * GIMBAL_RATE_DPS * scale;
        target += rate * fminf(fmaxf(dt_s, 0.0f), GIMBAL_MAX_DT_S);
        target = fminf(fmaxf(target, soft_min), soft_max);
        if ((rate > 0.0f && target >= soft_max) || (rate < 0.0f && target <= soft_min))
          rate = 0.0f;
      }
    }
    const float lower = fmaxf(GIMBAL_PITCH_MIN, measured - GIMBAL_PITCH_MAX_ERROR_DEG);
    const float upper = fminf(GIMBAL_PITCH_MAX, measured + GIMBAL_PITCH_MAX_ERROR_DEG);
    const float command = fminf(fmaxf(target, lower), upper);
    const float vel_ff = s_sign[0] * rate * GIMBAL_DEG_TO_RAD;
    if (CommandAxis(0, command, vel_ff)) {
      /* A disturbance can limit the sent command without moving the hold angle. */
      if (stick != 0.0f || !axis->active)
        axis->target_deg = command;
      axis->active = 1;
      tx_retry = 0;
    } else if (!tx_retry) {
      tx_failed_at = now;
      tx_retry = 1;
    }
    return;
  }
  tx_retry = 0;

  if (axis->active || sample->fb.ERR == ERR_Enable || now - last_disable >= 20U) {
    if (DM4310_SetEnable(&GIMBAL_BUS, GIMBAL_PITCH_ESC_ID, 0)) {
      axis->active = 0;
      last_disable = now;
    }
  }
}

/* Request status before the feedback watchdog trips, after this cycle's
 * position/disable frame has had a chance to enter the CAN mailbox. */
static void PitchPollFeedback(const gimbal_axis_t *sample, uint32_t now)
{
  static uint32_t last_query;
  if ((sample->seen && now - sample->rx_ms < GIMBAL_FB_POLL_AGE_MS) ||
      now - last_query < GIMBAL_FB_POLL_INTERVAL_MS)
    return;

  const uint8_t query[8] = {
    (uint8_t)GIMBAL_PITCH_ESC_ID, (uint8_t)(GIMBAL_PITCH_ESC_ID >> 8), 0xcc, 0
  };
  if (DM4310_SendFrame(&GIMBAL_BUS, 0x7ff, query))
    last_query = now;
}

void Gimbal_ControlStep(float dt_s)
{
  DiagnosticStep();
  if (!ConfigValid()) return;
  uint8_t request;
  gimbal_axis_t ax;
  taskENTER_CRITICAL();
  request=cal_request; cal_request=0; ax=s_axis[0];
  taskEXIT_CRITICAL();
  const uint32_t now=HAL_GetTick();
  const uint8_t rc_online=RC_LinkFresh();
  const uint8_t enabled=rc_online && RC_MasterEnabled();
  PitchStep(&ax, dt_s, now, rc_online, enabled);
  PitchPollFeedback(&ax, now);
  if(!request) return;
  uint8_t reply=0xbf;
  int16_t value=0;
  if(rc_online && !enabled && ax.seen &&
     now-ax.rx_ms<=GIMBAL_FB_TIMEOUT_MS && ax.fb.ERR==ERR_Disable &&
     isfinite(ax.fb.Position) && fabsf(ax.fb.Velocity)<0.05f) {
    const unsigned idx=request-1;
    endpoints[idx]=ax.fb.Position;
    endpoint_mask|=(uint8_t)(1U<<idx);
    value=(int16_t)lroundf(ax.fb.Position*1000.0f); /* reply: output shaft mrad */
    reply=request==1 ? 0xb1 : 0xb2;
    if(endpoint_mask==3) {
      float span=(endpoints[1]-endpoints[0])*GIMBAL_RAD_TO_DEG;
      /* Check the measured travel against the configured mechanical span. */
      if(fabsf(fabsf(span)-(GIMBAL_PITCH_MAX-GIMBAL_PITCH_MIN))>3.0f) {
        reply=0xbe; endpoint_mask=0;
      } else {
        float sign=span>0 ? 1.0f : -1.0f;
        float zero=endpoints[0]-sign*GIMBAL_PITCH_MIN*GIMBAL_DEG_TO_RAD;
        value=(int16_t)lroundf(zero*1000.0f);
        reply=sign>0 ? 0xb3 : 0xb4;
        /* Preview result only. No enable, Flash write or motor SaveZero. */
        endpoint_mask=0;
      }
    }
  }
  taskENTER_CRITICAL(); cal_value=value; cal_reply=reply; taskEXIT_CRITICAL();
}

/* raw MIT position word, output shaft rad, rad/s, CAN age ms, drive status. */
void Gimbal_GetEncoder(float out[5])
{
  gimbal_axis_t ax;
  taskENTER_CRITICAL(); ax=s_axis[0]; taskEXIT_CRITICAL();
  uint32_t age=ax.seen ? (uint32_t)(HAL_GetTick()-ax.rx_ms) : 65535U;
  if(age>65535U) age=65535U;
  out[0]=(float)ax.fb.Pos_int;
  out[1]=ax.fb.Position;
  out[2]=ax.fb.Velocity;
  out[3]=(float)age;
  out[4]=(float)ax.fb.ERR;
}
