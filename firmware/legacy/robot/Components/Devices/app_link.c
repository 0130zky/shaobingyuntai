/*
 * @File         :
 * @Author       : JL HUANG
 * @Date         : 2024-04-21 00:25:52
 * @LastEditTime : 2024-05-19 17:31:30
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\robot_template\Components\Devices\app_link.c
 * @Description  :
 */

#include "app_link.h"
#include "master_task.h"
#include "RC_task.h"

/**
 * @description  :
 * @param         {int} x_int
 * @param         {float} x_min
 * @param         {float} x_max
 * @param         {int} bits
 * @return        {*}
 */
float Link_uint_to_float(int x_int,float x_min,float x_max,int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @description  :
 * @param         {float} x
 * @param         {float} x_min
 * @param         {float} x_max
 * @param         {int} bits
 * @return        {*}
 */
int Link_float_to_uint(float x,float x_min,float x_max,int bits)
{                   
  float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @description  :
 * @param         {CAN_HandleTypeDef*} hcan
 * @return        {*}
 */
void Set_MasterLink_Data(CAN_HandleTypeDef* hcan)
{
  uint16_t spd_x_tmp,spd_y_tmp,spd_w_tmp,pitch_angle_tmp,pitch_target;
  
  spd_x_tmp = Link_float_to_uint(MasterLink.UP_Board_SetData.Spd_X,SPD_X_MAX,SPD_X_MIN,8);
  spd_y_tmp = Link_float_to_uint(MasterLink.UP_Board_SetData.Spd_Y,SPD_Y_MAX,SPD_Y_MIN,8);
  spd_w_tmp = Link_float_to_uint(MasterLink.UP_Board_SetData.Spd_W,SPD_W_MAX,SPD_W_MIN,8);
  pitch_angle_tmp = Link_float_to_uint(GIMBAL.DM4310.RefAngle,PITCH_MAX_ANGLE,-20,12);
  pitch_target    = Link_float_to_uint(GIMBAL.Pitch_angle,PITCH_MAX_ANGLE,-20,12);

  uint8_t Tx_Data[8] = {0};
  CAN_TxHeaderTypeDef CAN_TxHeader;

  CAN_TxHeader.IDE   = CAN_ID_STD;    //帧类型为标准帧
  CAN_TxHeader.RTR   = CAN_RTR_DATA;  //帧格式为数据帧
  CAN_TxHeader.DLC   = 0x08;          //数据长度为8
  CAN_TxHeader.StdId = UP_Board_Set_ID;      //数据ID标识符

  Tx_Data[0] = spd_x_tmp;
  Tx_Data[1] = spd_y_tmp;
  Tx_Data[2] = spd_w_tmp;
  Tx_Data[3] = (pitch_target & 0x0FF0) >> 4;
  Tx_Data[4] = ((pitch_target & 0x0F) << 4)|(pitch_angle_tmp >> 8);
  Tx_Data[5] = pitch_angle_tmp;
  Tx_Data[6] = (MasterLink.UP_Board_SetData.SysStartLog << 7)|  //|  0  |  0  |  0  |  0  |  0  |  0  |  0  |  0  |
               (MasterLink.UP_Board_SetData.Mode_data<<4)|      //| sys |    mode_data    |Cap_E|
               (MasterLink.Hid_Hero.Supercap_Enable<<3);
  
  Tx_Data[7] =  (MasterLink.Hid_Hero.Vision_Enable<<4)|(RC_Data.Error_code);   //|  0  |  0  |  0  |  0  |  0  |  0  |  0  |  0  |
                                                                               //|      vision_mode      |         Error         |
  HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, Tx_Data, 0);

  HAL_CAN_Start(hcan);
}

void Get_MasterLink_Info(UP_Board_Get_t* up_board_get_t,uint8_t data[8])
{
  up_board_get_t -> gunHeat1    = (data[0] << 8) | (data[1]);
  up_board_get_t -> gunSpeed1   = (data[2] << 8) | (data[3]);
  up_board_get_t -> maxHeat     = (data[4] << 8) | (data[5]);
  up_board_get_t -> Color       = (data[6]);
  up_board_get_t -> gunSpeed1_f = Link_uint_to_float(up_board_get_t -> gunSpeed1, 25.0f, 0.0f, 16);
  up_board_get_t -> shoot_state = (data[7]);
}
