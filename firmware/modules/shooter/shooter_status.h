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
 *  @description ：发射机构对外只读状态类型与通道定义。
 */
#ifndef SHOOTER_STATUS_H
#define SHOOTER_STATUS_H

#include <stdint.h>

#define SHOOTER_WHEEL_COUNT 3
#define SHOOTER_SCOPE_CHANNELS 25

/* Values belong to one completed software control cycle. CAN measurements
 * retain their individual reception times; they are not physically simultaneous.
 */
typedef struct {
  int16_t speed_rpm;
  int16_t target_rpm;
  int16_t current_raw;
  uint8_t temperature_c;
  float output_raw;
  uint32_t received_ms;
} shooter_wheel_status_t;

typedef struct {
  uint32_t sequence;
  uint32_t sample_ms;
  uint32_t sample_us;
  shooter_wheel_status_t wheel[SHOOTER_WHEEL_COUNT];
  float dial_angle_deg, dial_target_deg, dial_speed_dps;
  float dial_output_norm, dial_current_norm;
  int16_t dial_slot;
  uint8_t dial_feedback_ok, dial_jammed, dial_fault, dial_arrived;
  uint8_t dial_jam_retry, wheels_ready, enabled;
  uint8_t usb_link_ok, usb_enable_mask, usb_sequence;
  uint8_t timing_fault;
} shooter_status_t;

/* Referee observation is independent of the motor control cycle. Its age is
 * evaluated by the caller when it copies the referee data, not at sample_ms.
 */
typedef struct {
  float bullet_speed_mps;
  uint32_t shot_count, shot_received_ms, frame_age_ms;
  uint8_t shooter_id;
} shooter_referee_status_t;

/* Copy into caller-owned storage. No internal mutable pointer is returned.
 * Returns 0 until the first cycle has completed; the output is then all zero.
 * A stopped control task leaves sequence and sample_ms frozen for diagnosis.
 */
uint8_t Booster_ReadStatus(shooter_status_t *out);
uint8_t Booster_BuildScopeTelemetry(float out[SHOOTER_SCOPE_CHANNELS],
                                  const shooter_referee_status_t *referee);

#endif
