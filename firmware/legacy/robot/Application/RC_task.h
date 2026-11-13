/*
 * @File         :
 * @Author       : JL HUANG
 * @Date         : 2023-12-24 16:16:31
 * @LastEditTime : 2024-05-19 17:28:13
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\robot_template\Application\RC_task.h
 * @Description  :
 */

#ifndef __RC_TASK_H
#define __RC_TASK_H

#include "main.h"

#include "app_et07.h"
#include "booster_task.h"
#include "gimbal_task.h"
#include "module_task.h"
#include "referee.h"

#define PITCH_MAX_ANGLE  25.5f            //pitch角度限幅
#define PITCH_MIN_ANGLE -34.0f

/**
 * 默认拨杆挡位值
*/
#define default_0 240
#define default_1 1024
#define default_2 1807

/*
用户自定义死区
*/
#define default_Deadzone 25

#define STOP_END  0 //停止完成
#define START_END 1 //初始化完成

#define LegLength_MAX 0.405f
#define LegLength_MIN 0.104f

#define Top_Mode_CW 5
#define Deploy_Mode 4
#define No_Follow_Mode 3
#define Top_Mode 2
#define Follow_Mode 1
#define Free_Mode 0
/* 整车控制最大速度，单位m/s */
#define ChassisMaxSpd 5.0f      // m/s
#define ChassisNormalSpeed 1.4f

#define Xfactor 200.0f
#define Yfactor 200.0f

typedef struct
{
  ET07_Data_t         ET07_DATA;		//8'b |  0  |  0  |  0  |  0  |  0  |  0  |  0  |  0  |
  uint8_t             Error_code;		//										   BOOST RC_ERR
  uint8_t		  	  key_static[5];
}RC_Data_t;

extern RC_Data_t RC_Data;

void RC_Task(void);

/* 接收机链路是否在线（100ms 内有新帧）。0 = 没接遥控 / 掉线。 */
uint8_t RC_LinkFresh(void);

/* CH8 旋钮 → 三路摩擦轮统一转速(rpm)。
   -100..-80 死区不转；-80..0 线性 0→5560；0..+100 线性 5560→10000。
   CH6 总使能关或链路掉线时恒为 0。 */
/* 2026-10-28最近10条5750rpm记录均值17.008m/s：
   5750*16.45/17.008≈5561rpm，先用5560校正均值；逐发散布仍须实测。 */
#define RC_FRI_RPM_MID      ( 5560)
#define RC_FRI_RPM_MAX      (10000)
#define RC_FRI_DEAD_NEG     (  -80)     /* 小于等于它就停 */
#define RC_CH8_RAW_MIN      (  353)     /* RF209S 实测端点 */
#define RC_CH8_RAW_MID      ( 1024)
#define RC_CH8_RAW_MAX      ( 1696)
int16_t RC_FrictionWheelRpm(void);

/* CH6 总使能：仅在线且 CH6 > 1500 时使能。
   显式关闭 BOOSTER_RC_ONLY 的 USB 台架版才允许无遥控控制。 */
uint8_t RC_MasterEnabled(void);

float ET07_Channel_Left_UD(float max_spd);
float ET07_Channel_Left_LR(float max_spd);
float ET07_Channel_Right_UD(float max_spd);
float TurnAngle_Channel(float max_spd);
uint8_t Mode_Channel(void);
void Transfer_Image_Control(void);

#endif
