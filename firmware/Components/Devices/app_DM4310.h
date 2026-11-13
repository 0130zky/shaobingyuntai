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
 *  @description ：DM4310 驱动接口、反馈结构与量程定义。
 */
#ifndef APP_DM4310_H
#define APP_DM4310_H
#include "main.h"
#ifdef SENTRY_STM32H7
#include "fdcan.h"
typedef FDCAN_HandleTypeDef DM4310_Bus;
#define DM4310_DEFAULT_BUS hfdcan1
#else
#include "can.h"
typedef CAN_HandleTypeDef DM4310_Bus;
#define DM4310_DEFAULT_BUS hcan1
#endif
typedef struct
{
  float p_min, p_max;   /* 输出轴 rad   */
  float v_min, v_max;   /* 输出轴 rad/s */
  float t_min, t_max;   /* 输出轴 N·m   */
  float kp_max;         /* kp 下限恒为 0 */
  float kd_max;         /* kd 下限恒为 0 */
} dm4310_range_t;
typedef enum
{
  ERR_Disable              = 0x00,  //失能
  ERR_Enable               = 0x01,  //使能
  ERR_OverVoltage          = 0x08,  //过压
  ERR_UnderVoltage         = 0x09,  //欠压
  ERR_OverCurrent          = 0x0A,  //过流
  ERR_MOS_OverTemperture   = 0x0B,  //MOS过温
  ERR_Rotor_OverTemperture = 0x0C,  //线圈过温
  ERR_Loss_Communicate     = 0x0D,  //通讯丢失
  ERR_OverLoad             = 0x0E   //过载
}DM4310_ERR_State;

/**
 * @description  : DM4310电机接收报文结构体
 * @return        {*}
 */
typedef struct
{
  uint8_t ID;
  uint8_t ERR;

  int Pos_int;
  int Vel_int;
  int Tor_int;

  float Position;
  float Velocity;
  float Torque;

  uint8_t T_MOS;
  uint8_t T_Rotor;
}DM4310_MotorMeasure_t;
float DM4310_uint_to_float(int value,float minimum,float maximum,int bits);
int DM4310_float_to_uint(float value,float minimum,float maximum,int bits);
void DM4310_Get_Ranged(DM4310_MotorMeasure_t *,const uint8_t data[8],const dm4310_range_t *);
uint8_t DM4310_SendFrame(DM4310_Bus *,uint32_t,const uint8_t data[8]);
uint8_t DM4310_SetEnable(DM4310_Bus *,uint32_t,uint8_t);
uint8_t DM4310_Send_Ranged(DM4310_Bus *,uint32_t,float,float,float,float,float,const dm4310_range_t *);
uint8_t DM4310_Send_PositionRaw(DM4310_Bus *,uint32_t,uint16_t,float,float,float,float,const dm4310_range_t *);
#endif
