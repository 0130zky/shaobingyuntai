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
 *  @description ：达妙 DM4310 CAN 协议：MIT 帧编解码、量程映射与使能控制。
 */
#include "app_DM4310.h"

float DM4310_uint_to_float(int x_int,float x_min,float x_max,int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int DM4310_float_to_uint(float x,float x_min,float x_max,int bits)
{
  float span = x_max - x_min;
  float offset = x_min;

  /* **超量程先夹住**。不夹的话映射结果会超过 2^bits-1，而打包时是按位段 OR 进去的，
     多出来的高位会**串到相邻字段上**（比如力矩溢出会污染 kd），发出去是一帧
     谁也看不懂的乱码。原来的代码没夹，pitch 的 `速度环输出 + 前馈力矩` 在大误差
     时是会超 ±T_MAX 的 —— 这里补上属于修 bug，量程内的值一个都不受影响。 */
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;

  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

uint8_t DM4310_SendFrame(DM4310_Bus* hcan,uint32_t id,const uint8_t data[8])
{
#ifdef SENTRY_STM32H7
  FDCAN_TxHeaderTypeDef header={0};
  if(id>0x7ffU || HAL_FDCAN_GetTxFifoFreeLevel(hcan)==0U) return 0;
  header.Identifier=id;header.IdType=FDCAN_STANDARD_ID;
  header.TxFrameType=FDCAN_DATA_FRAME;header.DataLength=FDCAN_DLC_BYTES_8;
  header.ErrorStateIndicator=FDCAN_ESI_ACTIVE;header.BitRateSwitch=FDCAN_BRS_OFF;
  header.FDFormat=FDCAN_CLASSIC_CAN;header.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
  return HAL_FDCAN_AddMessageToTxFifoQ(hcan,&header,(uint8_t *)data)==HAL_OK;
#else
  CAN_TxHeaderTypeDef CAN_TxHeader;
  uint32_t mailbox = 0;

  CAN_TxHeader.IDE   = CAN_ID_STD;    //帧类型为标准帧
  CAN_TxHeader.RTR   = CAN_RTR_DATA;  //帧格式为数据帧
  CAN_TxHeader.DLC   = 0x08;          //数据长度为8
  CAN_TxHeader.StdId = id;            //数据ID标识符

  return (HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, (uint8_t *)data, &mailbox) == HAL_OK) ? 1 : 0;
#endif
}

uint8_t DM4310_SetEnable(DM4310_Bus* hcan,uint32_t id,uint8_t en)
{
  uint8_t Tx_Data[8];
  for (uint8_t i = 0; i < 7; i++) Tx_Data[i] = 0xFF;
  Tx_Data[7] = en ? 0xFC : 0xFD;
  return DM4310_SendFrame(hcan, id, Tx_Data);
}

uint8_t DM4310_Send_Ranged(DM4310_Bus* hcan,uint32_t id,
                           float pos,float vel,float torq,float kp,float kd,
                           const dm4310_range_t* rng)
{
  uint16_t raw=(uint16_t)DM4310_float_to_uint(pos,rng->p_min,rng->p_max,16);
  return DM4310_Send_PositionRaw(hcan,id,raw,vel,torq,kp,kd,rng);
}
uint8_t DM4310_Send_PositionRaw(DM4310_Bus* hcan,uint32_t id,uint16_t raw,
                           float vel,float torq,float kp,float kd,
                           const dm4310_range_t* rng)
{
  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
  uint8_t Tx_Data[8];

  pos_tmp = raw;
  vel_tmp = DM4310_float_to_uint(vel,  rng->v_min, rng->v_max, 12);
  kp_tmp  = DM4310_float_to_uint(kp,   0.0f,     rng->kp_max, 12);
  kd_tmp  = DM4310_float_to_uint(kd,   0.0f,     rng->kd_max, 12);
  tor_tmp = DM4310_float_to_uint(torq, rng->t_min, rng->t_max, 12);

  Tx_Data[0] = (pos_tmp >> 8);
  Tx_Data[1] = pos_tmp;
  Tx_Data[2] = (vel_tmp >> 4);
  Tx_Data[3] = ((vel_tmp & 0x0F) << 4)|(kp_tmp >> 8);
  Tx_Data[4] = kp_tmp;
  Tx_Data[5] = (kd_tmp >> 4);
  Tx_Data[6] = ((kd_tmp & 0x0F) << 4)|(tor_tmp >> 8);
  Tx_Data[7] = tor_tmp;

  return DM4310_SendFrame(hcan, id, Tx_Data);
}

void DM4310_Get_Ranged(DM4310_MotorMeasure_t* m, const uint8_t data[8], const dm4310_range_t* rng)
{
  m->ID = data[0] & 0x0f;
  m->ERR = data[0] >> 4;

  m->Pos_int = (data[1] << 8) | data[2];
  m->Vel_int= (data[3] << 4) | (data[4]>>4);
  m->Tor_int = ((data[4] & 0x0F) << 8)|data[5];

  m->Position = DM4310_uint_to_float(m->Pos_int, rng->p_min, rng->p_max, 16);
  m->Velocity = DM4310_uint_to_float(m->Vel_int, rng->v_min, rng->v_max, 12);
  m->Torque   = DM4310_uint_to_float(m->Tor_int, rng->t_min, rng->t_max, 12);

  m->T_MOS = data[6];
  m->T_Rotor = data[7];
}
