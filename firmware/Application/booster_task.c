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
 *  @description ：发射控制拍入口：输入仲裁、反馈采样、三轮与拨盘控制、CAN 提交和状态发布。
 */
#include "booster_task.h"
#include "shooter_internal.h"
#include "control_time.h"
#include "dial_port.h"
#include <string.h>

/* Private controller state: never exposed to CAN or telemetry callers. */
booster_t BOOSTER;
const uint8_t Fri_MotorID[FRI_WHEEL_NUM] = { M3508_ID_12H, M3508_ID_4H, M3508_ID_8H };

void BoosterInit(void)
{
  memset(&BOOSTER, 0, sizeof BOOSTER);
  Shooter_TimeInit();
  Shooter_FeedbackInit();
  Shooter_StatusInit();
  Shooter_CommandInit();
  Shooter_FrictionInit();
  Shooter_DialInit();
}

void Booster_ControlStep(float dt_s)
{
  /* 单拍的小空档不把三轮打回零：旧版用固定 3ms 一直跑，而"间隔一超限就清
     目标"会让三轮降速再重新爬坡。只有"20~30ms 的单次抖动"沿用上一拍输出，
     更大的空档（状态已经不可信）或连续第 3 拍异常仍走完整停机。
     拨盘不受影响：InhibitCommand 已停掉这一拍的动作。 */
  static uint8_t s_gap_cycles;
  static uint32_t s_prev_ms;
  static uint8_t s_started;
  Shooter_CommandStep();
  uint32_t sample_ms, sample_us;
  Shooter_CaptureFeedback(&sample_ms, &sample_us);
  const uint32_t gap_ms = s_started ? (uint32_t)(sample_ms - s_prev_ms) : 0U;
  s_prev_ms = sample_ms;
  s_started = 1;
  const uint8_t timing_ok = Shooter_TimeStep(sample_ms, dt_s);
  if (!timing_ok || DIAL_DM_CALIB_FREE) Shooter_InhibitCommand();
  Shooter_ProcessRequests();
  if (timing_ok) s_gap_cycles = 0;
  else if (s_gap_cycles < UINT8_MAX) s_gap_cycles++;
  const uint8_t hold_wheels = !timing_ok && !DIAL_DM_CALIB_FREE &&
                              gap_ms > SHOOTER_MAX_CONTROL_GAP_MS &&
                              gap_ms <= SHOOTER_MAX_CONTROL_GAP_MS + 10U &&
                              s_gap_cycles <= 2U;
  if (!hold_wheels) {
    Booster_FrictionGroupCtrl();
    Booster_UpdateWheelReadiness();
  }
  Booster_DialCtrl();
  /* Three wheel currents; unused fourth slot stays zero. */
  Set_RM_MotorCurrent(&BoosterBus, M3508_STD_ID,
                     BOOSTER.RM_Motor[M3508_ID_12H - 1].MotorCtrl_Speed.Output,
                     BOOSTER.RM_Motor[M3508_ID_4H - 1].MotorCtrl_Speed.Output,
                     BOOSTER.RM_Motor[M3508_ID_8H - 1].MotorCtrl_Speed.Output,
                     0, NULL);
  Dial_Port_Tx();
  Shooter_PublishStatus(sample_ms, sample_us);
}
