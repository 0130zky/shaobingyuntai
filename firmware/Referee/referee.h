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
 *  @description ：裁判系统接口与解析状态定义。
 */
#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "main.h"
#include "protocol.h"

#include "refereeData_v1.7.h"
#include "stdio.h"
#include "string.h"
#include "tim.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024

/* 仅用于观测：计数的是校验通过的接收帧，时间是本板解析时间。 */
typedef struct {
  shoot_data_t shot;
  uint32_t shot_count;
  uint32_t shot_rx_ms;
  uint32_t valid_frame_count;
  uint32_t last_frame_ms;
} referee_debug_t;

void Referee_GetDebug(referee_debug_t *out);
extern void refereeINIT(void);
extern void referee_unpack_fifo_data(void);
extern void refereeReceiveHandler(void);

#endif
