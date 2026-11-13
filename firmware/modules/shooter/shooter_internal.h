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
 *  @description ：发射模块内部接口：私有状态、反馈缓存与控制拍类型。
 */
#ifndef SHOOTER_INTERNAL_H
#define SHOOTER_INTERNAL_H
#include "booster_task.h"
#include "app_RM_Motor.h"
#include "dial_config.h"

typedef struct
{
	uint8_t  Fri_Mode;                  //派生量：任一路使能则为1（保留给拨弹门控用）
  uint8_t  Dial_Mode;                   //保留（全工程未使用）
  uint8_t  fri_en[FRI_WHEEL_NUM];       //上位机下发的每轮使能
  int16_t  fri_spd[FRI_WHEEL_NUM];      //上位机下发的每轮转速（带符号）
  uint32_t last_cmd_tick;               //最近一次合法帧的 HAL_GetTick()
  uint8_t  cmd_seq;                     //最近一次被接受的 seq（回传时回显）
  uint8_t  link_ok;                     //1 = 指令流正常，0 = 已超时停机
  uint8_t  shot_counter;                //上位机最近的进弹计数（每点一次"进弹"+1）
  uint8_t  shot_applied;                //已经应用到 Dial_Shoot 的计数
  float    zero_deg;                    //拨弹盘零点（初始角度），上位机 byte10 实时下发
} booster_interface_t;

typedef struct
{
  uint8_t  Dail_check_state ; //卡弹处理状态: check_yse = 正常, check_no = 卡弹处理中
  float Dail_stuck_count; /* observed low-speed duration, actual milliseconds */
  float    Dail_Jam_Target ;  //进卡弹那一刻锁存的回拨目标角度(度)，卡弹期间只跟它
  uint8_t  Dial_Enable_Prev ; //上一周期的 link_ok，用来抓"使能上升沿"
  uint8_t  Dial_Jam_Retry ;   //本次是否已触发一次回拨；退弹/重新使能时清零
  float    Dial_Phase_Deg;    //固定45度网格偏移，正向为逆时针
  float    Dial_Move_Start;   //本次命令锁定的原始起点，不能随反馈前移
  float    Dial_Move_End;     //本次命令终点，距起点最多45°
  uint8_t  Dial_Move_Active;  //动作未完成时不接受新的正向进弹
  uint8_t  Dial_Jam_Hold;     //回拨完成/超时后断输出，等待人工退弹
  uint32_t Dial_Jam_Start_Ms;
  float Dial_Overrun_Ms; /* observed overrun duration, actual milliseconds */
  /* 拨盘反馈时间戳（Dial_Fb_Tick）搬进 port 了：那个时间戳只在算
     "这块电机还活着没有"时用，而那件事两个 variant 的判据不一样
     （3508 只看超时，DM 还得看故障码），所以归 port 管。 */
}Dail_state_t;

enum Dail_check_state
{
  check_yse = 0 ,
  check_no
};

typedef struct
{
  RM_Motor_t     RM_Motor[FRI_WHEEL_NUM]; //只有三个摩擦轮；拨盘见 dial_port_*.c
  booster_interface_t Booster_Interface;
  int16_t Dial_Shoot;
  Dail_state_t Dail_state ;              //卡弹处理
  /* Real_Dial_Angle 也搬进 port 了：它是 port 反馈的镜像，留在这儿就是第二个
     真相来源（谁先写谁后读说不清）。现在要角度一律 Dial_Port_Get()->angle_deg。 */
} booster_t;

extern booster_t BOOSTER;

/* 轮子索引 -> CAN 电机 ID，定义在 booster_task.c */
extern const uint8_t Fri_MotorID[FRI_WHEEL_NUM];


void Shooter_FeedbackInit(void);
void Shooter_TimeInit(void);
uint8_t Shooter_TimeStep(uint32_t now_ms, float dt_s);
float Shooter_ControlDt(void);
uint8_t Shooter_TimingFault(void);
void Shooter_InhibitCommand(void);
void Shooter_CaptureFeedback(uint32_t *sample_ms, uint32_t *sample_us);
void Shooter_StatusInit(void);
void Shooter_PublishStatus(uint32_t sample_ms, uint32_t sample_us);
uint8_t Shooter_DialFault(void);
float Booster_DialTarget(void);
uint8_t Booster_DialJammed(void);
uint8_t Booster_DialArrived(void);
uint8_t Booster_WheelsReady(void);
int16_t Booster_GetWheelSetpoint(uint8_t idx);
void Shooter_CommandInit(void);
void Shooter_CommandStep(void);
void Shooter_ProcessRequests(void);
uint8_t Shooter_ControlEnabled(void);
uint8_t Shooter_CommandEnabled(void);
void Shooter_FrictionInit(void);
void Shooter_ResetReadiness(void);
void Shooter_DialInit(void);
void Booster_DialCtrl(void);
void Booster_FrictionGroupCtrl(void);
void Booster_UpdateWheelReadiness(void);
/* Only the control thread may apply a discrete movement. No event backlog. */
void Booster_RequestShot(int8_t n);
#endif
