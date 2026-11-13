/*
 * @File        : main.h
 * @Brief       : DM-MC02 (STM32H723VGT6) 哨兵折叠云台工程 —— 板级引脚定义与公共声明
 * @Note        : 本文件由 CubeMX 生成后手工整理，引脚定义与达妙 MC02 官方例程保持一致
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* -------------------------------------------------------------------------- */
/* 板载可控电源使能（DM-MC02）                                                  */
/* 上电必须拉高，否则 24V / 5V 输出与部分外设不上电                              */
/* -------------------------------------------------------------------------- */
#define POWER_24V_1_Pin       GPIO_PIN_14   /* PC14 */
#define POWER_24V_1_GPIO_Port GPIOC
#define POWER_24V_2_Pin       GPIO_PIN_13   /* PC13 */
#define POWER_24V_2_GPIO_Port GPIOC
#define POWER_5V_Pin          GPIO_PIN_15   /* PC15 */
#define POWER_5V_GPIO_Port    GPIOC

/* 用户按键 PA15，低电平按下 */
#define USER_KEY_Pin          GPIO_PIN_15
#define USER_KEY_GPIO_Port    GPIOA

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
