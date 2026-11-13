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
 *  @description ：控制拍结束时发布只读状态快照。
 */
#include "shooter_internal.h"
#include "dial_port.h"
#include "FreeRTOS.h"
#include "task.h"

static shooter_status_t s_published;
static uint8_t s_valid;

void Shooter_StatusInit(void)
{
  s_published = (shooter_status_t){0};
  s_valid = 0;
}

void Shooter_PublishStatus(uint32_t sample_ms, uint32_t sample_us)
{
  shooter_status_t next = {0};
  next.sequence = s_published.sequence + 1U;
  next.sample_ms = sample_ms;
  next.sample_us = sample_us;
  for (uint8_t i = 0; i < FRI_WHEEL_NUM; ++i) {
    const RM_Motor_t *motor = &BOOSTER.RM_Motor[Fri_MotorID[i] - 1];
    next.wheel[i].speed_rpm = motor->RM_MotorMeasure.speed;
    next.wheel[i].target_rpm = Booster_GetWheelSetpoint(i);
    next.wheel[i].current_raw = motor->RM_MotorMeasure.current;
    next.wheel[i].temperature_c = (uint8_t)motor->RM_MotorMeasure.temperature;
    next.wheel[i].received_ms = motor->RM_MotorMeasure.rx_tick;
    next.wheel[i].output_raw = motor->MotorCtrl_Speed.Output;
  }
  const dial_fb_t *dial = Dial_Port_Get();
  next.dial_angle_deg = dial->angle_deg;
  next.dial_target_deg = Booster_DialTarget();
  next.dial_speed_dps = dial->speed_dps;
  next.dial_output_norm = dial->out;
  next.dial_current_norm = dial->current;
  next.dial_feedback_ok = dial->fb_ok;
  next.dial_jammed = Booster_DialJammed();
  next.dial_fault = Shooter_DialFault();
  next.dial_arrived = Booster_DialArrived();
  next.dial_slot = BOOSTER.Dial_Shoot;
  next.dial_jam_retry = BOOSTER.Dail_state.Dial_Jam_Retry;
  next.wheels_ready = Booster_WheelsReady();
  next.enabled = Shooter_ControlEnabled();
  next.timing_fault = Shooter_TimingFault();
  const booster_interface_t *usb = &BOOSTER.Booster_Interface;
  next.usb_link_ok = usb->link_ok;
  next.usb_sequence = usb->cmd_seq;
  for (uint8_t i = 0; i < FRI_WHEEL_NUM; ++i)
    if (usb->fri_en[i]) next.usb_enable_mask |= 1U << i;

  taskENTER_CRITICAL();
  s_published = next;
  s_valid = 1;
  taskEXIT_CRITICAL();
}

uint8_t Booster_ReadStatus(shooter_status_t *out)
{
  if (!out) return 0;
  taskENTER_CRITICAL();
  *out = s_published;
  const uint8_t valid = s_valid;
  taskEXIT_CRITICAL();
  return valid;
}

uint8_t Booster_DialFault(void)
{
  /* A byte read is enough for the indicator; use ReadStatus for multiple fields. */
  return s_valid && s_published.dial_fault;
}
