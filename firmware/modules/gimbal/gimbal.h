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
 *  @date   : 2026-11-08
 *  @description ：云台模块对外接口：初始化、控制拍与 CAN 反馈入口。
 */
#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

uint8_t Gimbal_DiagnosticRx(uint16_t id, const uint8_t *data, uint8_t length);
void Gimbal_Init(void);
/* 由 PID_Task 每拍调用，dt_s 与发射控制拍同一个实测周期。 */
void Gimbal_ControlStep(float dt_s);
/* CAN1 接收中断入口；只更新反馈，不改控制状态。 */
void Gimbal_OnCanRx(uint16_t id, const uint8_t *data, uint8_t length);

void Gimbal_GetEncoder(float out[5]);
uint16_t Gimbal_TakePitchExit(void);
void Gimbal_RequestLimit(uint8_t endpoint);
uint8_t Gimbal_TakeLimitReply(int16_t *value, uint8_t *type);

#endif
