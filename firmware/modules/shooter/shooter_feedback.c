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
 *  @description ：发射机构反馈缓存：CAN 中断只写测量，控制拍统一采样。
 */
#include "shooter_internal.h"
#include "dial_port.h"
#include "bsp_dwt.h"
#include "FreeRTOS.h"
#include "task.h"

/* CAN ISR owns these measurements. It never accesses targets or PID state. */
static RM_MotorMeasure_t s_wheel_feedback[FRI_WHEEL_NUM];

void Shooter_FeedbackInit(void)
{
  for (uint8_t i = 0; i < FRI_WHEEL_NUM; ++i)
    s_wheel_feedback[i] = (RM_MotorMeasure_t){0};
}

void Booster_OnCanRx(uint16_t id, const uint8_t *data, uint8_t length)
{
  if (!data || length != 8) return;
  for (uint8_t i = 0; i < FRI_WHEEL_NUM; ++i) {
    if (id == M3508_STD_ID + Fri_MotorID[i]) {
      Get_RM_MotorInfo(&s_wheel_feedback[i], (uint8_t *)data);
      return;
    }
  }
  /* Preserve every dial frame: its unwrap logic needs adjacent positions. */
  if (id == DIAL_RX_STD_ID) Dial_Port_OnCanRx(data);
}

void Shooter_CaptureFeedback(uint32_t *sample_ms, uint32_t *sample_us)
{
  taskENTER_CRITICAL();
  *sample_ms = HAL_GetTick();
  *sample_us = (uint32_t)DWT_GetTimeline_us();
  for (uint8_t i = 0; i < FRI_WHEEL_NUM; ++i)
    BOOSTER.RM_Motor[Fri_MotorID[i] - 1].RM_MotorMeasure = s_wheel_feedback[i];
  Dial_Port_Update();
  taskEXIT_CRITICAL();
}
