/*
 * @File         :
 * @Author       : JL HUANG
 * @Date         : 2024-04-21 00:26:20
 * @LastEditTime : 2024-05-19 17:26:34
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\robot_template\Components\Devices\app_link.h
 * @Description  :
 */

#ifndef __APP_LINK_H
#define __APP_LINK_H

#include "main.h"
#include "can.h"

#define MasterLinkBus  hcan2

#define UP_Board_Set_ID    0x301
#define UP_Board_Get_ID    0x302

#define SPD_X_MAX 5.0f
#define SPD_X_MIN -5.0f

#define SPD_Y_MAX 5.0f
#define SPD_Y_MIN -5.0f

#define SPD_W_MAX 10.0f
#define SPD_W_MIN -10.0f

#define Leg_Length_MAX 0.3f
#define Leg_Length_MIN -0.3f

typedef struct
{
  float Spd_X;
  float Spd_Y;
  float Spd_W;
  float LegLength;
  float Pitch_Angle;
  float Yaw_Angle;

  /* 使能状态 */
  uint8_t SysStartLog;
  /* 当前模式*/
  uint8_t Mode_data;
} UP_Board_Set_t;

typedef struct
{
  uint16_t gunHeat1;
  uint16_t gunSpeed1;
  uint16_t maxHeat;
  uint8_t Color;
  float gunSpeed1_f;
  uint8_t shoot_state;
} UP_Board_Get_t;

float Link_uint_to_float(int x_int,float x_min,float x_max,int bits);
int Link_float_to_uint(float x,float x_min,float x_max,int bits);

void Set_MasterLink_Data(CAN_HandleTypeDef* hcan);
void Get_MasterLink_Info(UP_Board_Get_t* up_board_get_t,uint8_t data[8]);

#endif
