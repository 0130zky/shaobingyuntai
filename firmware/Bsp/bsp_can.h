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
 *  @description ：CAN 底层接口与接收回调注册。
 */
#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"

#define  CAN1_FILTER_BANK  0    //CAN1起始过滤器组编号
#define  CAN2_FILTER_BANK  14   //CAN2起始过滤器组编号

typedef void (*bsp_can_rx_handler_t)(uint16_t id, const uint8_t *data, uint8_t length);
/* Initialization only: install handlers before CAN_Filter_Init enables IRQs. */
void CAN_SetRxHandlers(bsp_can_rx_handler_t can1, bsp_can_rx_handler_t can2);
void CAN_Filter_Init(void);

#endif
