/*
 * @File         :
 * @Author       : JL HUANG
 * @Date         : 2024-04-14 16:50:01
 * @LastEditTime : 2024-04-18 01:34:16
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Components\Devices\app_DM4310.h
 * @Description  :
 */

#ifndef __APP_DM4310_H
#define __APP_DM4310_H

#include "main.h"
#include "can.h"
#include "controller.h"

/* 总线选择 */
#define JointBus          hcan2

/* 电机数组标识符 */
#define Left_Front_Joint  0

/* 电机ID标识符 */
#define PITCH4310_TX_ID  0x01
#define PITCH4310_RX_ID  0x02

/* 电机参数限幅 —— **这是云台 pitch 轴的量程**，不是"DM4310 这颗电机的量程"。
   拨盘那颗 DM4310 量程和它不同（见下面的 dm4310_range_t），所以不要再拿这组宏
   去解码拨盘的帧。 */
#define P_MIN -12.56637f
#define P_MAX 12.56637f

#define V_MIN -45.0f
#define V_MAX 45.0f

#define T_MIN -54.0f
#define T_MAX 54.0f

#define KP_MIN 0.0f
#define KP_MAX 500.0f

#define KD_MIN 0.0f
#define KD_MAX 5.0f

/**
 * @description  : MIT 帧量程（按实例传入）
 *
 * MIT 帧是 **12/16 位等比映射**：pos16 = (pos - p_min) * 65535 / (p_max - p_min)。
 * 所以代码里的量程**必须**和电机里配的 PMAX/VMAX/TMAX（寄存器 0x15/0x16/0x17）一致 ——
 * 配错了不报任何错、电机照转，只是位置/速度/力矩全都按比例缩放，静态看不出来。
 *
 * 两颗 DM4310 量程不同，全局 #define 一张表已经不够用：
 *     pitch 轴：±4π / ±45 / ±54  —— 历史值，云台一直在跑，不动
 *     拨弹盘  ：±12.5 / ±30 / ±10 —— 出厂默认，见 dial_config.h 的 DM 段
 */
typedef struct
{
  float p_min, p_max;   /* 输出轴 rad   */
  float v_min, v_max;   /* 输出轴 rad/s */
  float t_min, t_max;   /* 输出轴 N·m   */
  float kp_max;         /* kp 下限恒为 0 */
  float kd_max;         /* kd 下限恒为 0 */
} dm4310_range_t;

/* pitch 轴的量程表（值就是上面那组宏）。云台继续用它，行为一个字没变 */
extern const dm4310_range_t DM4310_RangePitch;

/**
 * @description  : 电机错误码
 * @return        {*}
 */
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

typedef struct
{
  DM4310_MotorMeasure_t DM4310_MotorMeasure;
  float offsetPosition;
  PID_t             MotorCtrl_Angle;
  PID_t             MotorCtrl_Speed;
  float				RefAngle;
  float				RefSpeed;
  // PID_t                 MF9025_Ctrl;
} DM4310_Motor_t;

void DM4310_Enable(CAN_HandleTypeDef* hcan,uint32_t id);
void DM4310_Disable(CAN_HandleTypeDef* hcan,uint32_t id);

float DM4310_uint_to_float(int x_int,float x_min,float x_max,int bits);
int DM4310_float_to_uint(float x,float x_min,float x_max,int bits);

void Set_DM4310_MotorCurrent(CAN_HandleTypeDef* hcan,uint32_t id,float pos,float vel,float torq,float kp,float kd);
void Get_DM4310_MotorInfo(DM4310_MotorMeasure_t* DM4310_motor_t, uint8_t data[8]);

/* ---------------- 下面几个是给拨弹盘加的纯新增入口 ----------------
   上面那两个"老"函数内部就是拿 DM4310_RangePitch 调下面对应的 _Ranged 版本，
   所以云台 pitch 的收发逐位不变（连 HAL_CAN_Start + Delay_us 的顺序都保着）。
   拨盘用 _Ranged：量程不同，而且**不能忙等**。 */

/* 按实例量程解码一帧反馈 */
void DM4310_Get_Ranged(DM4310_MotorMeasure_t* m, const uint8_t data[8], const dm4310_range_t* rng);

/* 按实例量程发一帧 MIT 控制帧（pos 16 位 / vel、torq、kp、kd 各 12 位）。
   返回 1 = 已进发送邮箱。**不忙等、不重开 CAN**。 */
uint8_t DM4310_Send_Ranged(CAN_HandleTypeDef* hcan,uint32_t id,
                           float pos,float vel,float torq,float kp,float kd,
                           const dm4310_range_t* rng);

/* 只发帧。返回 1 = 已进发送邮箱（HAL_OK）。中断里也安全 */
uint8_t DM4310_SendFrame(CAN_HandleTypeDef* hcan,uint32_t id,const uint8_t data[8]);

/* 使能(en=1，0xFF..0xFC) / 失能(en=0，0xFF..0xFD)。**不忙等**，返回 1 = 已进邮箱 */
uint8_t DM4310_SetEnable(CAN_HandleTypeDef* hcan,uint32_t id,uint8_t en);

void DM4310_SetPosition(DM4310_Motor_t* DM4310_motor_t);

#endif
