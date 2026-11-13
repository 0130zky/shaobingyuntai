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
 *  @description ：拨盘电机抽象接口：反馈、目标、提交与停止。
 */
#ifndef DIAL_PORT_H
#define DIAL_PORT_H
#include "main.h"
#include "dial_config.h"
typedef struct
{
  float   angle_deg;   /* 拨盘角度(deg)，连续多圈，已减零点 */
  float   speed_dps;   /* 拨盘角速度(deg/s)。**符号必须与 angle_deg 一致**：
                          二者反号会让位置环变成正反馈、全速飞转，而角度看起来还在
                          "缩小误差"。换电机后第一件事就是手转拨盘确认同号。 */
  float   out;         /* 本拍的控制输出，归一化 [-1,1]。两种电机都是"串级环的输出"：
                          - 3508：归一化电流计数（最后并进 0x200 帧）
                          - DM4310：归一化力矩（最后乘 T_MAX 填进 MIT 帧的 torq）
                          断控时两者都是 0。遥测的 out 字段 = 它 × DIAL_TELEM_SCALE */
  float   current;     /* 归一化实测扭矩/电流 [-1,1]，±1 = 该电机满量程 */
  float   temp_c;      /* 摄氏。C620 只能报原始 0~255，3508 下就是原样透传；
                          DM 报 MOS 和线圈两个温度，这里取高的那个 */
  uint8_t fb_ok;       /* 1 = 反馈新鲜 且 无故障码 且 发送没在连续失败。断控判据用这个 */
} dial_fb_t;


#define DIAL_RX_STD_ID DIAL_DM_MST_ID
void Dial_Port_Init(void);
void Dial_Port_SetZero(float zero_deg);
void Dial_Port_OnCanRx(const uint8_t *data);
void Dial_Port_Update(void);
const dial_fb_t *Dial_Port_Get(void);
void Dial_Port_SetTarget(float dial_deg, float dt_s);
uint8_t Dial_Port_Settled(void);
void Dial_Port_ClearIntegral(void);
void Dial_Port_Stop(void);
void Dial_Port_Tx(void);
#endif
