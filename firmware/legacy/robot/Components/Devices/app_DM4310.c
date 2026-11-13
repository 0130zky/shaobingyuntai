/*
 * @File         :
 * @Author       : JL HUANG
 * @Date         : 2024-04-14 16:49:42
 * @LastEditTime : 2024-04-19 16:48:05
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Components\Devices\app_DM4310.c
 * @Description  :
 */

#include "app_DM4310.h"
#include "bsp_tim.h"

/* pitch 轴的量程表：值就是头文件里那组 P_MIN/V_MIN/T_MIN... 宏。
   云台的收发全部走这张表，所以加"按实例传量程"这件事对它是透明的。 */
const dm4310_range_t DM4310_RangePitch =
{
  P_MIN, P_MAX,
  V_MIN, V_MAX,
  T_MIN, T_MAX,
  KP_MAX, KD_MAX
};

/**
 * @description  : 电机使能
 * @param         {CAN_HandleTypeDef*} hcan
 * @param         {uint32_t} id
 * @return        {*}
 */
void DM4310_Enable(CAN_HandleTypeDef* hcan,uint32_t id)
{
  uint8_t Tx_Data[8] = {0};
  CAN_TxHeaderTypeDef CAN_TxHeader;

  CAN_TxHeader.IDE   = CAN_ID_STD;    //帧类型为标准帧
  CAN_TxHeader.RTR   = CAN_RTR_DATA;  //帧格式为数据帧
  CAN_TxHeader.DLC   = 0x08;          //数据长度为8
  CAN_TxHeader.StdId = id;      //数据ID标识符

  Tx_Data[0] = 0xFF;
  Tx_Data[1] = 0xFF;
  Tx_Data[2] = 0xFF;
  Tx_Data[3] = 0xFF;
  Tx_Data[4] = 0xFF;
  Tx_Data[5] = 0xFF;
  Tx_Data[6] = 0xFF;
  Tx_Data[7] = 0xFC;

  HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, Tx_Data, 0);

  HAL_CAN_Start(hcan);

  Delay_us(250);
}

/**
 * @description  : 电机失能
 * @param         {CAN_HandleTypeDef*} hcan
 * @param         {uint32_t} id
 * @return        {*}
 */
void DM4310_Disable(CAN_HandleTypeDef* hcan,uint32_t id)
{
  uint8_t Tx_Data[8] = {0};
  CAN_TxHeaderTypeDef CAN_TxHeader;

  CAN_TxHeader.IDE   = CAN_ID_STD;    //帧类型为标准帧
  CAN_TxHeader.RTR   = CAN_RTR_DATA;  //帧格式为数据帧
  CAN_TxHeader.DLC   = 0x08;          //数据长度为8
  CAN_TxHeader.StdId = id;      //数据ID标识符

  Tx_Data[0] = 0xFF;
  Tx_Data[1] = 0xFF;
  Tx_Data[2] = 0xFF;
  Tx_Data[3] = 0xFF;
  Tx_Data[4] = 0xFF;
  Tx_Data[5] = 0xFF;
  Tx_Data[6] = 0xFF;
  Tx_Data[7] = 0xFD;

  HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, Tx_Data, 0);

  HAL_CAN_Start(hcan);

  Delay_us(250);
}

/**
 * @description  :
 * @param         {int} x_int
 * @param         {float} x_min
 * @param         {float} x_max
 * @param         {int} bits   
 * @return        {*}
 */
float DM4310_uint_to_float(int x_int,float x_min,float x_max,int bits)
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

/**
 * @description  : 发一帧 8 字节到指定 StdId。**不忙等、不重开 CAN**
 *                 原来这里返回值是全程丢掉的（`HAL_CAN_AddTxMessage(hcan,&h,data,0)`）。
 *                 MIT 丢一帧不会报任何错，电机会保持上一拍收到的 pos/kp/kd 不动，
 *                 所以必须能看见失败。
 * @param         {CAN_HandleTypeDef*} hcan
 * @param         {uint32_t} id
 * @param         {const uint8_t*} data
 * @return        {uint8_t} 1 = 已进发送邮箱，0 = 失败（邮箱满/总线错）
 */
uint8_t DM4310_SendFrame(CAN_HandleTypeDef* hcan,uint32_t id,const uint8_t data[8])
{
  CAN_TxHeaderTypeDef CAN_TxHeader;
  uint32_t mailbox = 0;

  CAN_TxHeader.IDE   = CAN_ID_STD;    //帧类型为标准帧
  CAN_TxHeader.RTR   = CAN_RTR_DATA;  //帧格式为数据帧
  CAN_TxHeader.DLC   = 0x08;          //数据长度为8
  CAN_TxHeader.StdId = id;            //数据ID标识符

  return (HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, (uint8_t *)data, &mailbox) == HAL_OK) ? 1 : 0;
}

/**
 * @description  : 使能/失能，**不忙等**（老的 DM4310_Enable/Disable 保留原样给云台用）
 * @param         {uint8_t} en   1 = 使能(0xFF..0xFC)，0 = 失能(0xFF..0xFD)
 * @return        {uint8_t} 1 = 已进发送邮箱
 */
uint8_t DM4310_SetEnable(CAN_HandleTypeDef* hcan,uint32_t id,uint8_t en)
{
  uint8_t Tx_Data[8];
  for (uint8_t i = 0; i < 7; i++) Tx_Data[i] = 0xFF;
  Tx_Data[7] = en ? 0xFC : 0xFD;
  return DM4310_SendFrame(hcan, id, Tx_Data);
}

/**
 * @description  : 按实例量程发 MIT 控制帧
 * @param         {CAN_HandleTypeDef*} hcan
 * @param         {uint32_t} id
 * @param         {float} pos  输出轴 rad
 * @param         {float} vel  输出轴 rad/s
 * @param         {float} torq 输出轴 N·m（前馈）
 * @param         {float} kp   N·m/rad
 * @param         {float} kd   N·m·s/rad
 * @param         {const dm4310_range_t*} rng
 * @return        {uint8_t} 1 = 已进发送邮箱
 */
uint8_t DM4310_Send_Ranged(CAN_HandleTypeDef* hcan,uint32_t id,
                           float pos,float vel,float torq,float kp,float kd,
                           const dm4310_range_t* rng)
{
  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
  uint8_t Tx_Data[8];

  pos_tmp = DM4310_float_to_uint(pos,  rng->p_min, rng->p_max, 16);
  vel_tmp = DM4310_float_to_uint(vel,  rng->v_min, rng->v_max, 12);
  kp_tmp  = DM4310_float_to_uint(kp,   KP_MIN,     rng->kp_max, 12);
  kd_tmp  = DM4310_float_to_uint(kd,   KD_MIN,     rng->kd_max, 12);
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

/**
 * @description  : 发 MIT 控制帧（云台 pitch 用的老入口，量程固定为 pitch）
 *                 行为与重构前逐位一致：先 AddTx、再 HAL_CAN_Start、再 Delay_us(200)。
 * @return        {*}
 */
void Set_DM4310_MotorCurrent(CAN_HandleTypeDef* hcan,uint32_t id,float pos,float vel,float torq,float kp,float kd)
{
  DM4310_Send_Ranged(hcan, id, pos, vel, torq, kp, kd, &DM4310_RangePitch);

  HAL_CAN_Start(hcan);

  /* 发完一个电机包后需要等待以匹配时序 */
  Delay_us(200);
}

/**
 * @description  :
 * @param         {DM4310_MotorMeasure_t*} m
 * @param         {uint8_t} data
 * @param         {const dm4310_range_t*} rng
 * @return        {*}
 */
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

/**
 * @description  : 解一帧反馈（云台 pitch 用的老入口，量程固定为 pitch）
 * @param         {DM4310_MotorMeasure_t*} DM4310_motor_t
 * @param         {uint8_t} data
 * @return        {*}
 */
void Get_DM4310_MotorInfo(DM4310_MotorMeasure_t* DM4310_motor_t, uint8_t data[8])
{
  DM4310_Get_Ranged(DM4310_motor_t, data, &DM4310_RangePitch);
}

/**
 * @description  : 设置电机相对零点
 * @param         {DM4310_Motor_t*} DM4310_motor_t
 * @return        {*}
 */
void DM4310_SetPosition(DM4310_Motor_t* DM4310_motor_t)
{
  DM4310_motor_t->offsetPosition = DM4310_motor_t->DM4310_MotorMeasure.Position;
}

// void DM4310_ResetPosition(CAN_HandleTypeDef* hcan,uint32_t tx_id,float pos,float vel,float torq,float kp,float kd)
// {

// }
