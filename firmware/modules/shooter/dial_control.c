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
 *  @date   : 2026-10-30
 *  @description ：拨盘动作状态机：45 度网格、卡弹回拨、锁定与越界保护。
 */
#include <math.h>
#include "shooter_internal.h"
#include "dial_port.h"
#include "FreeRTOS.h"
#include "task.h"

void Shooter_DialInit(void)
{
  //拨弹盘：整套（原始反馈 + 串级 PID + 输出）都在 port 里，这里只负责初始化它，
  //以及把默认零点交给它。增益的单位换算是 dial_config.h 里的派生宏干的活。
  Dial_Port_Init();
  Dial_Port_SetZero(DIAL_ZERO_DEFAULT_DEG);

  //卡弹处理
  BOOSTER.Dail_state.Dail_check_state = check_yse ;
  BOOSTER.Dail_state.Dail_stuck_count = 0 ;
  BOOSTER.Dail_state.Dail_Jam_Target  = 0.0f ;
  BOOSTER.Dail_state.Dial_Enable_Prev = 0 ;
  BOOSTER.Dail_state.Dial_Jam_Retry   = 0 ;
  BOOSTER.Dail_state.Dial_Phase_Deg = DIAL_PHASE_DEFAULT_DEG;
  BOOSTER.Dail_state.Dial_Move_Start  = 0.0f;
  BOOSTER.Dail_state.Dial_Move_End    = 0.0f;
  BOOSTER.Dail_state.Dial_Move_Active = 0;
  BOOSTER.Dail_state.Dial_Jam_Hold    = 0;
  BOOSTER.Dail_state.Dial_Jam_Start_Ms = 0;
  BOOSTER.Dail_state.Dial_Overrun_Ms  = 0;
}

uint8_t Booster_DialJammed(void)
{
  return (BOOSTER.Dail_state.Dail_check_state == check_no) ? 1 : 0;
}

/**
 * @description  : 1 = 当前这一格已经到位（且不在卡弹处理中）
 *                 中位连续进弹用它做节拍：到位后才允许发下一发，
 *                 所以每段都是一次独立的 45° 走位，不会连成一片。
 * @return        {uint8_t}
 */
uint8_t Booster_DialArrived(void)
{
  const dial_fb_t *fb = Dial_Port_Get();
  if (!fb->fb_ok || Booster_DialJammed() || BOOSTER.Dail_state.Dial_Move_Active) return 0;
  if (fabsf(fb->speed_dps) > DIAL_HOLD_SPD_DPS) return 0;
  return Dial_Port_Settled() &&
         fabsf(fb->angle_deg - Booster_DialTarget()) <= DIAL_DM_HOLD_EXIT_DEG;

}

/**
 * @description  : 当前应该跟随的目标角度(度)
 *                 卡弹回拨目标也限制在本次锁定区间内。
 * @return        {float}
 */
static float Dial_StrokeClamp(const Dail_state_t *st, float value)
{
  const float lo = (st->Dial_Move_Start < st->Dial_Move_End) ? st->Dial_Move_Start : st->Dial_Move_End;
  const float hi = (st->Dial_Move_Start > st->Dial_Move_End) ? st->Dial_Move_Start : st->Dial_Move_End;
  if (value < lo) value = lo;
  if (value > hi) value = hi;
  return value;
}

float Booster_DialTarget(void)
{
  const Dail_state_t *st = &BOOSTER.Dail_state;
  return Dial_StrokeClamp(st, Booster_DialJammed() ? st->Dail_Jam_Target : st->Dial_Move_End);
}

/**
 * @description  : 进弹/退弹的唯一入口（+1 进弹 / -1 退弹）
 *                 卡弹期间进弹**直接丢弃、不入队**：入队的话解卡后位置环会把
 *                 攒下的领先量一次走完，就是"卡弹时连点、解卡后一次推出好几个"。
 *                 退弹照常放行，并且立刻解卡 —— 这也是操作员手动解卡的手段。
 * @param         {int8_t} n
 * @return        {*}
 */
void Booster_RequestShot(int8_t n)
{
  if (!n) return;
  taskENTER_CRITICAL();
  Dail_state_t *st = &BOOSTER.Dail_state;
  const dial_fb_t *fb = Dial_Port_Get();
  if (!fb->fb_ok || !st->Dial_Enable_Prev) {
    taskEXIT_CRITICAL();
    return;
  }
  if (n > 0 && (Booster_DialJammed() || !Booster_DialArrived())) {
    taskEXIT_CRITICAL();
    return;
  }
  /* 反向运动未完成时，不累加更多退弹；正向途中退弹则回到原始起点。 */
  if (n < 0 && st->Dial_Move_Active &&
      (st->Dial_Move_End - st->Dial_Move_Start) * DIAL_DEG_PER_SHOT < 0.0f) {
    taskEXIT_CRITICAL();
    return;
  }
  float step = DIAL_DEG_PER_SHOT;
  if (step > DIAL_MOVE_MAX_DEG) step = DIAL_MOVE_MAX_DEG;
  if (step < -DIAL_MOVE_MAX_DEG) step = -DIAL_MOVE_MAX_DEG;
  const float previous_end = st->Dial_Move_End;
  float end = previous_end + ((n > 0) ? step : -step);
  if (n < 0 && (Booster_DialJammed() || st->Dial_Move_Active))
    end = st->Dial_Move_Start;
  /* 固定45°目标网格：不得用到位实测值重建起点，否则容差会逐格累积。
     外部3:1时，每格对应DM4310输出轴135°。 */
  st->Dial_Move_Start = previous_end;
  if (end > st->Dial_Move_Start + DIAL_MOVE_MAX_DEG)
    end = st->Dial_Move_Start + DIAL_MOVE_MAX_DEG;
  if (end < st->Dial_Move_Start - DIAL_MOVE_MAX_DEG)
    end = st->Dial_Move_Start - DIAL_MOVE_MAX_DEG;
  st->Dial_Move_End = end;
  st->Dial_Move_Active = 1;
  st->Dial_Jam_Hold = 0;
  st->Dail_check_state = check_yse;
  st->Dail_stuck_count = 0;
  st->Dial_Jam_Retry = 0;
  st->Dial_Overrun_Ms = 0;
  /* 任意非零计数差值仅执行一格，不补发、不排队。 */
  BOOSTER.Dial_Shoot = (int16_t)roundf((end - st->Dial_Phase_Deg) / DIAL_DEG_PER_SHOT);
  Dial_Port_ClearIntegral();
  if (n > 0) Shooter_ResetReadiness();
  taskEXIT_CRITICAL();
}

/**
 * @description  : 拨弹盘周期控制；每拍无条件执行，以识别失能和重新使能边沿
 * @return        {*}
 */
static volatile int16_t s_phase_pending;
static volatile uint8_t s_phase_requested;
static volatile int16_t s_phase_reply;
static volatile uint8_t s_phase_reply_type, s_phase_reply_pending;

/* USART1 RX ISR只发布请求；实际修改由拨盘任务在失能状态完成。 */
void Booster_RequestDialPhase(int16_t centidegrees)
{
  if (centidegrees < -1000 || centidegrees > 1000) return;
  s_phase_pending = centidegrees;
  s_phase_requested = 1;
}

uint8_t Booster_TakeDialPhaseReply(int16_t *phase, uint8_t *type)
{
  uint8_t pending;
  taskENTER_CRITICAL();
  pending = s_phase_reply_pending;
  *phase = s_phase_reply;
  *type = s_phase_reply_type;
  s_phase_reply_pending = 0;
  taskEXIT_CRITICAL();
  return pending;
}

void Booster_DialCtrl(void)
{
  Dail_state_t *st = &BOOSTER.Dail_state;
  const dial_fb_t *fb = Dial_Port_Get();
  const float real = fb->angle_deg;
  const float speed = fb->speed_dps;
  const uint8_t enabled = Shooter_ControlEnabled();
  if (s_phase_requested) {
    taskENTER_CRITICAL();
    const int16_t phase = s_phase_pending;
    s_phase_requested = 0;
    taskEXIT_CRITICAL();
    /* A temporary timing inhibit must not authorize changing coordinates while
       the operator still requests enable. */
    const uint8_t locked = Shooter_CommandEnabled();
    if (!locked) st->Dial_Phase_Deg = phase * 0.01f;
    s_phase_reply = (int16_t)roundf(st->Dial_Phase_Deg * 100.0f);
    s_phase_reply_type = locked ? 0x82 : 0x81;
    s_phase_reply_pending = 1;
  }
  if (!enabled || !fb->fb_ok) {
    st->Dial_Enable_Prev = 0;
    st->Dial_Move_Active = 0;
    Dial_Port_Stop();
    return;
  }
  if (!st->Dial_Enable_Prev) {
    /* 只做最多半格的初始化对齐，不恢复失联前未完成的进弹。 */
    BOOSTER.Dial_Shoot = (int16_t)roundf((real - st->Dial_Phase_Deg) / DIAL_DEG_PER_SHOT);
    st->Dial_Move_Start = real;
    st->Dial_Move_End = BOOSTER.Dial_Shoot * DIAL_DEG_PER_SHOT + st->Dial_Phase_Deg;
    st->Dial_Move_Active = 1;
    st->Dail_check_state = check_yse;
    st->Dail_stuck_count = 0;
    st->Dial_Jam_Retry = 0;
    st->Dial_Jam_Hold = 0;
    st->Dial_Overrun_Ms = 0;
    Dial_Port_ClearIntegral();
  }
  st->Dial_Enable_Prev = 1;

  /* 起点不随实测前移；目标跨度最多45°，到位容差不扩展运动区间。 */
  if (st->Dial_Move_End > st->Dial_Move_Start + DIAL_MOVE_MAX_DEG)
    st->Dial_Move_End = st->Dial_Move_Start + DIAL_MOVE_MAX_DEG;
  if (st->Dial_Move_End < st->Dial_Move_Start - DIAL_MOVE_MAX_DEG)
    st->Dial_Move_End = st->Dial_Move_Start - DIAL_MOVE_MAX_DEG;
  BOOSTER.Dial_Shoot = (int16_t)roundf((st->Dial_Move_End - st->Dial_Phase_Deg) / DIAL_DEG_PER_SHOT);

  /* 终点附近的收敛仍由位置环跟随原目标，不因速度跨过8dps而锁定。
     速度阈值只参与停稳/接收下一发判定（Booster_DialArrived），不决定故障。

     越界故障：只有"冲出本次命令区间 10° 以上并持续 20ms"才认失控。
     正常刹车 1~4° 的过冲交给位置环自己拉回；旧版 1° 即时锁存会把每次正常
     过冲都判成故障（实测表现为每发都要重新使能）。这里限制的是软件判据，
     不是机械硬限位；目标和回拨区间仍然绝不扩展。 */
  const float stroke = fabsf(st->Dial_Move_End - st->Dial_Move_Start);
  if (fabsf(real - st->Dial_Move_Start) > stroke + DIAL_OVERRUN_FAULT_DEG)
  {
    if (st->Dial_Overrun_Ms < DIAL_OVERRUN_MS)
      st->Dial_Overrun_Ms += Shooter_ControlDt() * 1000.0f;
    if (st->Dial_Overrun_Ms >= DIAL_OVERRUN_MS)
    {
      st->Dail_check_state = check_no;
      st->Dail_Jam_Target = Dial_StrokeClamp(st, real);
      st->Dial_Move_Active = 0;
      st->Dial_Jam_Hold = 1;
      st->Dial_Jam_Retry = 0; /* 越界停机，不是自动回拨 */
      st->Dail_stuck_count = 0;
      st->Dial_Overrun_Ms = 0;
      Dial_Port_Stop();
      return;
    }
  }
  else
  {
    st->Dial_Overrun_Ms = 0;
  }

  if (Booster_DialJammed()) {
    st->Dail_Jam_Target = Dial_StrokeClamp(st, st->Dail_Jam_Target);
    if ((fabsf(real - st->Dail_Jam_Target) <= DIAL_ARRIVED_DEG &&
         fabsf(speed) <= DIAL_HOLD_SPD_DPS) ||
        (uint32_t)(HAL_GetTick() - st->Dial_Jam_Start_Ms) >= DIAL_JAM_ROLLBACK_TIMEOUT_MS)
      st->Dial_Jam_Hold = 1;
    if (st->Dial_Jam_Hold) Dial_Port_Stop();
    else Dial_Port_SetTarget(st->Dail_Jam_Target, Shooter_ControlDt());
    return; /* 回拨结束后也不自动前顶，不恢复原进弹目标 */
  }

  /* 卡弹只看"该走的方向上还差多少"，不看绝对误差：
     冲过终点 5° 和差终点 5° 是两件完全不同的事 —— 前者是位置环要拉回的
     正常过冲，后者才是"该动没动"。用 abs(error) 会把过冲也当成卡弹回拨。 */
  const float error = st->Dial_Move_End - real;
  const float dir = (st->Dial_Move_End >= st->Dial_Move_Start) ? 1.0f : -1.0f;
  const float remaining = error * dir;   /* >0 = 还没走到终点 */
  uint8_t settled = fabsf(error) <= DIAL_ARRIVED_DEG && fabsf(speed) <= DIAL_HOLD_SPD_DPS;
  settled = Dial_Port_Settled() && fabsf(error) <= DIAL_DM_HOLD_EXIT_DEG &&
            fabsf(speed) <= DIAL_HOLD_SPD_DPS;
  if (settled) {
    st->Dial_Move_Active = 0;
    st->Dail_stuck_count = 0;
  } else if (remaining > DIAL_JAM_ERR_DEG && fabsf(speed) < DIAL_JAM_SPD_DPS) {
    st->Dail_stuck_count += Shooter_ControlDt() * 1000.0f;
    if (st->Dail_stuck_count >= DIAL_JAM_MS) {
      /* 回拨向本次原始起点，不跨出原45°区间，反向退弹卡住也不继续后退。 */
      float rollback = real;
      if (st->Dial_Move_End > st->Dial_Move_Start) rollback -= DIAL_ROLLBACK_DEG;
      else if (st->Dial_Move_End < st->Dial_Move_Start) rollback += DIAL_ROLLBACK_DEG;
      st->Dail_Jam_Target = Dial_StrokeClamp(st, rollback);
      st->Dail_check_state = check_no;
      st->Dail_stuck_count = 0;
      st->Dial_Jam_Retry = 1;
      st->Dial_Move_Active = 0;
      st->Dial_Jam_Hold = 0;
      st->Dial_Jam_Start_Ms = HAL_GetTick();
      st->Dial_Overrun_Ms = 0;
      Dial_Port_ClearIntegral();
    }
  } else {
    st->Dail_stuck_count = 0;
  }
  if (Booster_DialJammed()) {
    Dial_Port_SetTarget(st->Dail_Jam_Target, Shooter_ControlDt());
  } else {
    Dial_Port_SetTarget(st->Dial_Move_End, Shooter_ControlDt());
  }
}


uint8_t Shooter_DialFault(void)
{
  return BOOSTER.Dail_state.Dial_Enable_Prev && Booster_DialJammed();
}
