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
 *  @description ：大疆 C620/M3508 CAN 协议：电流帧发送与反馈解析。
 */
#include "app_RM_Motor.h"
static uint32_t s_last_failure;
static uint8_t s_failed;
/**
 * @description  : RM电机(3508,2006,6020)发送报文函数
 * @param         {CAN_HandleTypeDef*} hcan
 * @param         {uint32_t} DeviceID
 * @param         {int16_t} IQ1
 * @param         {int16_t} IQ2
 * @param         {int16_t} IQ3
 * @param         {int16_t} IQ4
 * @return        {*}
 */
void Set_RM_MotorCurrent(CAN_HandleTypeDef* hcan,uint32_t DeviceID,int16_t IQ1,int16_t IQ2,int16_t IQ3,int16_t IQ4,uint32_t *pTxMailbox)
{
  uint8_t Tx_Data[8] = {0};
  CAN_TxHeaderTypeDef CAN_TxHeader;

  CAN_TxHeader.IDE   = CAN_ID_STD;    //帧类型为标准帧
  CAN_TxHeader.RTR   = CAN_RTR_DATA;  //帧格式为数据帧
  CAN_TxHeader.DLC   = 0x08;          //数据长度为8
  CAN_TxHeader.StdId = DeviceID;      //数据ID标识符

  Tx_Data[0] = (IQ1 >> 8);
  Tx_Data[1] = IQ1;
  Tx_Data[2] = (IQ2 >> 8);
  Tx_Data[3] = IQ2;
  Tx_Data[4] = (IQ3 >> 8);
  Tx_Data[5] = IQ3;
  Tx_Data[6] = (IQ4 >> 8);
  Tx_Data[7] = IQ4;

//  if(DeviceID == GM6020_SET_STD_ID)
  uint32_t mailbox = 0;
  if (HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, Tx_Data, &mailbox) != HAL_OK) {
    s_failed = 1;
    s_last_failure = HAL_GetTick();
  }
  if (pTxMailbox) *pTxMailbox = mailbox;
//  else HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, Tx_Data, 0);

  /* CAN was started by SysInit. No busy wait in the control path. */
}

/**
 * @description  : RM电机(3508,2006,6020)拆包函数
 * @param         {RM_Motor_t*} rm_motor_t
 * @param         {uint8_t} data
 * @return        {*}
 */
void Get_RM_MotorInfo(RM_MotorMeasure_t* rm_motor_t, uint8_t data[8])
{
  rm_motor_t -> last_encoder = rm_motor_t -> encoder;             //保留上次转子角度

  rm_motor_t -> encoder     = (uint16_t)(data[0] << 8 | data[1]); //本次转子机械角度
  rm_motor_t -> speed       = (int16_t )(data[2] << 8 | data[3]); //转子转速
  rm_motor_t -> current     = (int16_t )(data[4] << 8 | data[5]); //转矩电流(real_current=current*5.f/16384.f)
  rm_motor_t -> temperature = (int8_t  )(data[6]);                //转子温度(2006无此项)
  rm_motor_t -> rx_tick     = HAL_GetTick();                      //反馈存活时间戳（速度环看门狗）
}

uint8_t RM_CanTxFailedRecently(uint32_t window_ms)
{
  return s_failed && (uint32_t)(HAL_GetTick()-s_last_failure) <= window_ms;
}
