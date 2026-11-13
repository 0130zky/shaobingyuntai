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
 *  @description ：SBUS 接收底层接口。
 */
#ifndef __BSP_WBUS_H
#define __BSP_WBUS_H

#include "main.h"

#define W_BUS_RxBuff_NUM 50u
#define RC_FRAME_NUM     25u

extern uint8_t W_BUSRxBuffer[2][RC_FRAME_NUM];

void W_BUS_Init(uint8_t *rx0_buf, uint8_t *rx1_buf, uint16_t dma_buf_num);
void W_BUS_IDLEHandler(void);

#endif
