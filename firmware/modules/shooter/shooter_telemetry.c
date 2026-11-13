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
 *  @description ：遥测编码：主帧、诊断帧和校准应答的字节打包。
 */
#include "booster_task.h"

static void PutInt16(uint8_t *out, int16_t value)
{
  const uint16_t bits = (uint16_t)value;
  out[0] = (uint8_t)bits;
  out[1] = (uint8_t)(bits >> 8);
}

uint8_t Booster_BuildTelemetry(uint8_t *out)
{
  shooter_status_t status;
  if (!out || !Booster_ReadStatus(&status)) return 0;

  out[0] = PC_TELEM_HEADER;
  /* Preserve the old USB frame's status semantics and reserved bits. */
  out[1] = (status.usb_link_ok ? 1U : 0U) | (status.usb_enable_mask << 1);
  out[2] = status.usb_sequence;
  out[3] = status.dial_jam_retry;
  for (uint8_t i = 0; i < 4; ++i) out[4 + i] = (uint8_t)(status.sample_us >> (i * 8));

  for (uint8_t i = 0; i < FRI_WHEEL_NUM; ++i) {
    const shooter_wheel_status_t *wheel = &status.wheel[i];
    uint8_t *block = out + PC_TELEM_WHEEL_OFF + i * PC_TELEM_WHEEL_LEN;
    PutInt16(block, wheel->speed_rpm);
    PutInt16(block + 2, wheel->target_rpm);
    PutInt16(block + 4, (int16_t)wheel->output_raw);
    PutInt16(block + 6, wheel->current_raw);
    PutInt16(block + 8, wheel->temperature_c);
  }
  uint8_t *dial = out + PC_TELEM_DIAL_OFF;
  PutInt16(dial, (int16_t)status.dial_angle_deg);
  PutInt16(dial + 2, (int16_t)status.dial_target_deg);
  PutInt16(dial + 4, (int16_t)status.dial_speed_dps);
  /* Existing wire units: +/-16384 represents full scale on both motor types. */
  PutInt16(dial + 6, (int16_t)(status.dial_output_norm * 16384.0f));
  PutInt16(dial + 8, (int16_t)(status.dial_current_norm * 16384.0f));
  out[PC_TELEM_SHOTS_OFF] = (uint8_t)status.dial_slot;
  out[PC_TELEM_DSTATE_OFF] = (status.dial_jammed ? 0x01 : 0) |
                           (status.usb_link_ok ? 0x02 : 0) |
                           (status.usb_link_ok && !status.dial_feedback_ok ? 0x04 : 0);
  uint8_t checksum = 0;
  for (uint8_t i = 1; i < PC_TELEM_LEN - 2; ++i) checksum ^= out[i];
  out[PC_TELEM_LEN - 2] = checksum;
  out[PC_TELEM_LEN - 1] = PC_TELEM_TAIL;
  return PC_TELEM_LEN;
}

uint8_t Booster_BuildScopeTelemetry(float out[SHOOTER_SCOPE_CHANNELS],
                                  const shooter_referee_status_t *referee)
{
  shooter_status_t status;
  if (!out || !referee || !Booster_ReadStatus(&status)) return 0;
  static const int8_t direction[FRI_WHEEL_NUM] = {-1, 1, -1};
  for (uint8_t i = 0; i < FRI_WHEEL_NUM; ++i) {
    out[i] = direction[i] * (float)status.wheel[i].speed_rpm;
    out[10 + i] = direction[i] * status.wheel[i].output_raw;
    out[13 + i] = direction[i] * (float)status.wheel[i].current_raw;
  }
  out[3] = -(float)status.wheel[FRI_IDX_12H].target_rpm;
  out[4] = referee->bullet_speed_mps;
  out[5] = (float)(referee->shot_count & 0xffffU);
  out[6] = (float)(referee->shot_received_ms & 0xffffU);
  out[7] = (float)(status.sample_ms & 0xffffU);
  out[8] = (float)(referee->frame_age_ms > 65535U ? 65535U : referee->frame_age_ms);
  out[9] = (float)referee->shooter_id;
  out[16] = status.dial_angle_deg;
  out[17] = status.dial_target_deg;
  out[18] = status.dial_speed_dps;
  out[19] = status.dial_output_norm;
  out[20] = status.dial_current_norm;
  out[21] = (float)status.dial_feedback_ok;
  out[22] = (float)status.dial_jammed;
  out[23] = (float)status.dial_slot;
  out[24] = (float)status.wheels_ready;
  return SHOOTER_SCOPE_CHANNELS;
}
