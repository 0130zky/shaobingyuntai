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
 *  @description ：遥测与校准任务：USB/UART 遥测发送、示波器帧与相位标定应答。
 */
#include "module_task.h"
#include "gimbal.h"
#include "booster_task.h"
#include "RC_task.h"
#include "bsp_tim.h"
#include "referee.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "tim.h"
#include <string.h>

/* 独立校准帧 A6 5A 01 int16_le(0.01deg) xor(前5字节)，不含发弹指令。 */
void FrictionScope_RxByte(uint8_t byte)
{
  static uint8_t frame[6], used;
  static uint32_t last;
  uint32_t now = HAL_GetTick();
  if (now-last > 50) used=0;
  last=now;
  if (used==0 && byte!=0xa6) return;
  if (used==1 && byte!=0x5a) { used=(byte==0xa6); return; }
  frame[used++]=byte;
  if (used==6) {
    used=0;
    if ((uint8_t)(frame[0]^frame[1]^frame[2]^frame[3]^frame[4])==frame[5]) {
      if(frame[2]==1) Booster_RequestDialPhase((int16_t)(frame[3] | ((uint16_t)frame[4]<<8)));
      if(frame[2]==3 && frame[4]==0) Gimbal_RequestLimit(frame[3]);
    }
  }
}

/* USART1 主帧仍为25 float；每20ms插入24 float诊断帧，帧尾为负无穷。
   两个时间通道均以 65536ms 回绕，保证 float 可以精确表示毫秒整数。 */
static void FrictionScope_Send(void)
{
  enum { CHANNELS = 25, PAYLOAD_BYTES = CHANNELS * sizeof(float) };
  static union {
    float channel[CHANNELS + 1];
    uint8_t bytes[PAYLOAD_BYTES + 4];
  } packet;
  referee_debug_t judge;
  if (huart1.gState != HAL_UART_STATE_READY) return;
  static uint8_t reply[6];
  int16_t phase;
  uint8_t type;
  if (Gimbal_TakeLimitReply(&phase, &type) || Booster_TakeDialPhaseReply(&phase, &type)) {
    reply[0]=0xa6; reply[1]=0x5a; reply[2]=type;
    reply[3]=(uint8_t)phase; reply[4]=(uint8_t)((uint16_t)phase>>8);
    reply[5]=reply[0]^reply[1]^reply[2]^reply[3]^reply[4];
    HAL_UART_Transmit_DMA(&huart1, reply, sizeof(reply));
    return;
  }
  static uint16_t pitch_exit_pending;
  if (!pitch_exit_pending)
    pitch_exit_pending = Gimbal_TakePitchExit();
  if (pitch_exit_pending) {
    reply[0] = 0xa6; reply[1] = 0x5a; reply[2] = 0xc2;
    reply[3] = (uint8_t)pitch_exit_pending;
    reply[4] = (uint8_t)(pitch_exit_pending >> 8);
    reply[5] = reply[0] ^ reply[1] ^ reply[2] ^ reply[3] ^ reply[4];
    if (HAL_UART_Transmit_DMA(&huart1, reply, sizeof(reply)) == HAL_OK)
      pitch_exit_pending = 0;
    return;
  }
  /* 20 Hz read-only Pitch encoder snapshot. No control commands. */
  static uint32_t pitch_sent_ms;
  static uint8_t pitch_packet[24];
  if ((uint32_t)(HAL_GetTick()-pitch_sent_ms)>=50U) {
    float values[5];
    Gimbal_GetEncoder(values);
    pitch_packet[0]=0xa6; pitch_packet[1]=0x5a; pitch_packet[2]=0xc1;
    memcpy(pitch_packet+3,values,sizeof(values));
    uint8_t checksum=0;
    for(unsigned i=0;i<23;i++) checksum^=pitch_packet[i];
    pitch_packet[23]=checksum;
    if(HAL_UART_Transmit_DMA(&huart1,pitch_packet,sizeof(pitch_packet))==HAL_OK)
      pitch_sent_ms=HAL_GetTick();
    return;
  }
  /* 单一发送者、静态DMA缓冲区，诊断不挤大每一帧主遥测。 */
  static uint32_t diag_sent_ms;
  static union { float value[FRI_DIAG_CHANNELS+1]; uint8_t bytes[(FRI_DIAG_CHANNELS+1)*4]; } diag;
  if ((uint32_t)(HAL_GetTick()-diag_sent_ms)>=20U) {
    diag_sent_ms=HAL_GetTick();
    Booster_TakeFrictionDiagnostics(diag.value);
    const uint32_t n=FRI_DIAG_CHANNELS*4U;
    diag.bytes[n]=0; diag.bytes[n+1]=0; diag.bytes[n+2]=0x80; diag.bytes[n+3]=0xff;
    HAL_UART_Transmit_DMA(&huart1, diag.bytes, sizeof(diag.bytes));
    return;
  }
  Referee_GetDebug(&judge);
  const uint32_t now = HAL_GetTick();
  uint32_t age = now - judge.last_frame_ms;
  if (!judge.valid_frame_count || age > 65535U) age = 65535U;
  const shooter_referee_status_t referee = {
    .bullet_speed_mps = judge.shot.initial_speed,
    .shot_count = judge.shot_count,
    .shot_received_ms = judge.shot_rx_ms,
    .frame_age_ms = age,
    .shooter_id = judge.shot.shooter_number,
  };
  /* Encoder copies one completed control cycle; DMA keeps its own buffer. */
  if (!Booster_BuildScopeTelemetry(packet.channel, &referee)) return;
  packet.bytes[PAYLOAD_BYTES + 0] = 0x00;
  packet.bytes[PAYLOAD_BYTES + 1] = 0x00;
  packet.bytes[PAYLOAD_BYTES + 2] = 0x80;
  packet.bytes[PAYLOAD_BYTES + 3] = 0x7f;
  HAL_UART_Transmit_DMA(&huart1, packet.bytes, sizeof(packet.bytes));
}


void Module_Init(void)
{
  /* USB is independent of IMU or an estimator task. */
  MX_USB_DEVICE_Init();
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void Module_Task(void)
{
  static uint32_t last_led;
  static uint8_t telemetry_divider;
  const uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - last_led) >= 500U) {
    last_led = now;
    if ((BOOSTER_RC_ONLY && !RC_LinkFresh()) || Booster_DialFault()) LED_Red();
    else LED_Green();
  }
  if (++telemetry_divider >= SCOPE_DIV) {
    telemetry_divider = 0;
    /* USB DMA owns this storage until the next idle transmission. */
    static uint8_t telemetry[PC_TELEM_LEN];
    if (CDC_TransmitReady_FS() &&
        Booster_BuildTelemetry(telemetry))
      CDC_Transmit_FS(telemetry, PC_TELEM_LEN);
  }
  FrictionScope_Send();
}
