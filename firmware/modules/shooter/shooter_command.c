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
 *  @description ：发射指令仲裁：遥控与上位机输入、触发边沿和动作接受。
 */
#include "shooter_internal.h"
#include "RC_task.h"
#include "dial_port.h"
#include "FreeRTOS.h"
#include "task.h"

/* USB ISR publishes a complete command. Only the control thread applies it.
 * Intermediate continuous commands may be replaced; movements never queue.
 */
static booster_interface_t s_usb_pending;
static volatile uint8_t s_usb_received;
static uint8_t s_enabled;
static uint8_t s_requested_enabled;
static int16_t s_wheel_target[FRI_WHEEL_NUM];
static const int8_t s_direction[FRI_WHEEL_NUM] = {-1, 1, -1};
static uint8_t s_ch7_prev, s_ch7_armed;
static uint32_t s_auto_hold;
static struct {
  uint32_t now;
  uint16_t ch5;
  uint8_t ch7, rc_online;
} s_input;

void Shooter_CommandInit(void)
{
  BOOSTER.Booster_Interface = (booster_interface_t){0};
  BOOSTER.Booster_Interface.zero_deg = DIAL_ZERO_DEFAULT_DEG;
  s_usb_pending = (booster_interface_t){0};
  s_usb_received = s_enabled = s_requested_enabled = s_ch7_prev = s_ch7_armed = 0;
  s_auto_hold = 0;
  s_input.now = s_input.ch5 = s_input.ch7 = s_input.rc_online = 0;
  for (uint8_t i = 0; i < FRI_WHEEL_NUM; ++i) s_wheel_target[i] = 0;
}

void Booster_UsbCmdHandler(const uint8_t *buf, uint32_t len)
{
#if BOOSTER_RC_ONLY
  /* Enforce the selected control source at its input boundary, including zero. */
  (void)buf;
  (void)len;
#else
  if (!buf || len != PC_CMD_LEN || buf[0] != PC_CMD_HEADER) return;
  uint8_t checksum = 0;
  for (uint8_t i = 1; i < PC_CMD_LEN - 1; ++i) checksum ^= buf[i];
  if (checksum != buf[PC_CMD_LEN - 1]) return;

  booster_interface_t cmd = {0};
  for (uint8_t i = 0; i < FRI_WHEEL_NUM; ++i) {
    cmd.fri_en[i] = (buf[1] >> i) & 1U;
    cmd.fri_spd[i] = (int16_t)(buf[2 + i * 2] | ((uint16_t)buf[3 + i * 2] << 8));
  }
  cmd.cmd_seq = buf[8];
  cmd.shot_counter = buf[PC_CMD_SHOT_OFF];
  cmd.zero_deg = buf[PC_CMD_ZERO_OFF] * (DIAL_DEG_PER_SHOT / 256.0f);
  cmd.last_cmd_tick = HAL_GetTick();
  cmd.link_ok = 1;
  cmd.Fri_Mode = cmd.fri_en[0] || cmd.fri_en[1] || cmd.fri_en[2];
  /* Single producer (USB ISR); the consumer copies under a critical section. */
  s_usb_pending = cmd;
  s_usb_received = 1;
#endif
}

uint8_t Shooter_ControlEnabled(void)
{
  return s_enabled;
}

uint8_t Shooter_CommandEnabled(void)
{
  return s_requested_enabled;
}

void Shooter_InhibitCommand(void)
{
  s_enabled = s_ch7_armed = 0;
  s_auto_hold = 0;
  for (uint8_t i = 0; i < FRI_WHEEL_NUM; ++i) s_wheel_target[i] = 0;
}

int16_t Booster_GetWheelSetpoint(uint8_t idx)
{
  return idx < FRI_WHEEL_NUM ? s_wheel_target[idx] : 0;
}

static void RemoteStep(uint16_t ch5, uint8_t ch7_now, uint8_t enabled, uint32_t now)
{
  if (!enabled) s_ch7_armed = 0;
  else if (!ch7_now) s_ch7_armed = 1;
  const uint8_t fresh = enabled && s_ch7_armed;
  const uint8_t auto_mode = ch5 > 700 && ch5 < 1400;

  if (ch7_now && !s_ch7_prev && fresh && !auto_mode)
    Booster_RequestShot(ch5 > 1500 ? -1 : +1);

  if (!auto_mode || !ch7_now || !fresh || !Booster_DialArrived()) {
    s_auto_hold = 0;
  } else {
    if (!s_auto_hold) s_auto_hold = now ? now : 1;
    if ((uint32_t)(now - s_auto_hold) >= DIAL_AUTO_DWELL_MS) {
      Booster_RequestShot(+1);
      s_auto_hold = 0;
    }
  }
  s_ch7_prev = ch7_now;
}

void Shooter_CommandStep(void)
{
  booster_interface_t *itf = &BOOSTER.Booster_Interface;
  booster_interface_t incoming;
  uint8_t received, rc_online, master_enabled, ch7;
  uint16_t ch5;
  int16_t rc_rpm;
  uint32_t now;

  /* SBUS and USB IRQs cannot interrupt this input snapshot. */
  taskENTER_CRITICAL();
  now = HAL_GetTick();
  received = s_usb_received;
  incoming = s_usb_pending;
  s_usb_received = 0;
  rc_online = RC_LinkFresh();
  master_enabled = RC_MasterEnabled();
  rc_rpm = RC_FrictionWheelRpm();
  ch5 = RC_Data.ET07_DATA.SA_CH5;
  ch7 = RC_Data.ET07_DATA.SC_CH7 > 1500;
  taskEXIT_CRITICAL();

  /* Expire the old session before applying a reconnect: never replay its count. */
  if (itf->link_ok && (uint32_t)(now - itf->last_cmd_tick) > FRI_CMD_TIMEOUT_MS) {
    itf->link_ok = itf->Fri_Mode = 0;
    for (uint8_t i = 0; i < FRI_WHEEL_NUM; ++i) itf->fri_en[i] = itf->fri_spd[i] = 0;
  }
  if (received && (uint32_t)(now - incoming.last_cmd_tick) <= FRI_CMD_TIMEOUT_MS) {
    incoming.shot_applied = itf->link_ok ? itf->shot_applied : incoming.shot_counter;
    *itf = incoming;
    Dial_Port_SetZero(itf->zero_deg);
  }

  s_enabled = rc_online ? master_enabled : (!BOOSTER_RC_ONLY && itf->link_ok);
  s_requested_enabled = s_enabled;
  for (uint8_t i = 0; i < FRI_WHEEL_NUM; ++i) {
    int16_t rpm = rc_online ? rc_rpm : (s_enabled && itf->fri_en[i] ? itf->fri_spd[i] : 0);
    if (rpm > FRI_SPD_LIMIT) rpm = FRI_SPD_LIMIT;
    if (rpm < -FRI_SPD_LIMIT) rpm = -FRI_SPD_LIMIT;
    s_wheel_target[i] = s_direction[i] * rpm;
  }

  s_input.now = now;
  s_input.ch5 = ch5;
  s_input.ch7 = ch7;
  s_input.rc_online = rc_online;
}

/* Run after this cycle's feedback conversion, so a USB zero update and the
 * resulting position are observed together when deciding a movement.
 */
void Shooter_ProcessRequests(void)
{
  booster_interface_t *itf = &BOOSTER.Booster_Interface;
  RemoteStep(s_input.ch5, s_input.ch7, s_input.rc_online && s_enabled, s_input.now);
  const int8_t delta = (int8_t)(itf->shot_counter - itf->shot_applied);
  itf->shot_applied = itf->shot_counter;
  uint8_t allow = s_enabled;
  if (allow && !s_input.rc_online && delta && delta <= DIAL_MAX_SHOT_BURST && delta >= -DIAL_MAX_SHOT_BURST)
    Booster_RequestShot(delta);
}
