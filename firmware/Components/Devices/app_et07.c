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
 *  @description ：ET07 遥控接收机 SBUS 帧解析与通道发布。
 */
#include "app_et07.h"
extern volatile uint32_t pack_time;

/* 调试统计：区分偶发丢帧、报文损坏与真正失控，不改控制通道。 */
volatile uint32_t g_sbus_frame_lost_count;
volatile uint32_t g_sbus_invalid_count;
volatile uint32_t g_sbus_failsafe_count;

void Get_ET07_DataDecode(ET07_Data_t* et07_data_t,uint8_t* rx_buf)
{
  /* 错帧/单帧丢失只丢弃本帧，保留上一帧时间戳，由 100ms 看门狗兜底。
     bit2=丢帧不等于失控；把它立即清零会让三轮反复停机再启动。
     bit3=接收机失控才立即撤销使能。bit0/1 是数字通道。 */
  if (rx_buf[0] != 0x0f || rx_buf[24] != 0x00)
  {
    g_sbus_invalid_count++;
    return;
  }
  if (rx_buf[23] & 0x08)
  {
    g_sbus_failsafe_count++;
    pack_time = 0;
    return;
  }
  if (rx_buf[23] & 0x04)
  {
    g_sbus_frame_lost_count++;
    return;
  }
  et07_data_t ->ET07_Start = rx_buf[0];
  
  if( et07_data_t ->ET07_Start==0x0f)
  {
  et07_data_t ->RightTransverse_CH1 = ((int16_t)rx_buf[ 1] >> 0 | ((int16_t)rx_buf[ 2] << 8 )) & 0x07FF;
  et07_data_t ->RightDirection_CH2  = ((int16_t)rx_buf[ 2] >> 3 | ((int16_t)rx_buf[ 3] << 5 )) & 0x07FF;
  et07_data_t ->LeftDirection_CH3   = ((int16_t)rx_buf[ 3] >> 6 | ((int16_t)rx_buf[ 4] << 2 ) | (int16_t)rx_buf[5] << 10 ) & 0x07FF;
  et07_data_t ->LeftTransverse_CH4  = ((int16_t)rx_buf[ 5] >> 1 | ((int16_t)rx_buf[ 6] << 7 )) & 0x07FF;
  et07_data_t ->SA_CH5              = ((int16_t)rx_buf[ 6] >> 4 | ((int16_t)rx_buf[ 7] << 4 )) & 0x07FF;
  et07_data_t ->SB_CH6              = ((int16_t)rx_buf[ 7] >> 7 | ((int16_t)rx_buf[ 8] << 1 ) | (int16_t)rx_buf[9] << 9 ) & 0x07FF;
  et07_data_t ->SC_CH7              = ((int16_t)rx_buf[ 9] >> 2 | ((int16_t)rx_buf[10] << 6 )) & 0x07FF;
  et07_data_t ->SD_CH8              = ((int16_t)rx_buf[10] >> 5 | ((int16_t)rx_buf[11] << 3 )) & 0x07FF;
  et07_data_t ->V1_CH9              = ((int16_t)rx_buf[12] >> 0 | ((int16_t)rx_buf[13] << 8 )) & 0x07FF;
  et07_data_t ->V2_CH10             = ((int16_t)rx_buf[13] >> 3 | ((int16_t)rx_buf[14] << 5 )) & 0x07FF;

  et07_data_t ->ET07_Flag = rx_buf[23];
  et07_data_t ->ET07_End = rx_buf[24];
  pack_time = HAL_GetTick();
  }
  
}

