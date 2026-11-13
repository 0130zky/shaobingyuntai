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
 *  @description ：大疆电机驱动接口、反馈结构与 CAN ID 定义。
 */
#ifndef APP_RM_MOTOR_H
#define APP_RM_MOTOR_H
#include "main.h"
#include "controller.h"
#include "can.h"
#define BoosterBus hcan1
#define M3508_STD_ID 0x200
#define M3508_ID_12H 1
#define M3508_ID_4H 2
#define M3508_ID_8H 3
#define M3508_SpeedMaxOut 16384.0f
typedef struct
{
  uint16_t encoder;      //转子机械角度(0~8191)
  int16_t  last_encoder;

  int16_t  speed;        //转子转速
  int16_t  current;      //转矩电流
  int8_t   temperature;  //温度(2006无此项)

  int32_t  round;         //转子圈数累计
  int32_t  total_encoder;

  /* 最近一次反馈帧的 HAL_GetTick()。0 = 上电后从没收到过。
     **摩擦轮速度环必须看它**：反馈一停，speed 就冻在最后一个值上，PID 会一直
     对着一个"假速度"打电流（正转冻住 → 反方向全速跑），C620 又没有命令超时。
     详见 booster_task.c 的 Booster_FrictionCtrl()。 */
  uint32_t rx_tick;
} RM_MotorMeasure_t;
typedef struct { RM_MotorMeasure_t RM_MotorMeasure; PID_t MotorCtrl_Speed; } RM_Motor_t;
void Set_RM_MotorCurrent(CAN_HandleTypeDef *,uint32_t,int16_t,int16_t,int16_t,int16_t,uint32_t *);
void Get_RM_MotorInfo(RM_MotorMeasure_t *,uint8_t data[8]);
uint8_t RM_CanTxFailedRecently(uint32_t window_ms);
#endif
