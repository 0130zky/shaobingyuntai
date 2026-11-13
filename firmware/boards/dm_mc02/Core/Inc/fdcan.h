/*
 * @File   : fdcan.h
 * @Brief  : FDCAN1 初始化（经典 CAN 模式，1 Mbps）
 * @Note   : MC02 三路 CAN 引脚 —— CAN1: PD0/PD1  CAN2: PB5/PB6  CAN3: PD12/PD13
 *           本工程夹爪电机接 CAN1，按经典 CAN 1 Mbps 与 DM-J3507-2EC 通信。
 */

#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;

void MX_FDCAN1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */
