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
 *  @description ：ET07 接收机接口与通道数据结构。
 */
#ifndef __APP_ET07_H
#define __APP_ET07_H

#include "main.h"

/**
 * @description  : 遥控通道范围：352-1024-1696
 * @return        {*}
 */
typedef struct 
{
  uint8_t ET07_Start;  //0x0f

  int16_t RightTransverse_CH1;
  int16_t RightDirection_CH2;
  int16_t LeftDirection_CH3;  //左手油门
  int16_t LeftTransverse_CH4;
  int16_t SA_CH5;
  int16_t SB_CH6;
  int16_t SC_CH7;
  int16_t SD_CH8;
  int16_t V1_CH9;
  int16_t V2_CH10;
//  int16_t CH11;
//  int16_t CH12;
//  int16_t CH13;
//  int16_t CH14;
//  int16_t CH15;
//  int16_t CH16;

  uint8_t ET07_Flag;  //0x0c接收机正常，遥控离线
  uint8_t ET07_End;
}ET07_Data_t;


extern volatile uint32_t g_sbus_frame_lost_count;
extern volatile uint32_t g_sbus_invalid_count;
extern volatile uint32_t g_sbus_failsafe_count;

void Get_ET07_DataDecode(ET07_Data_t* et07_data_t,uint8_t* rx_buf);

#endif
