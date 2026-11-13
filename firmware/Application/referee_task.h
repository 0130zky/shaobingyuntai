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
 *  @description ：裁判任务接口与裁判数据结构。
 */
#ifndef __REFEREE_TASK_H
#define __REFEREE_TASK_H

#include "main.h"

typedef struct RefereeData {
  // 云台接收
  uint16_t maxHeat;   // 枪管最大热量
  uint16_t gunHeat1;  // 枪管1热量
  uint8_t target;     // 目标
  float gunSpeed1;     // 枪管1射速
} referee_data_t;

extern referee_data_t RefereeData;
extern void Referee_Task_Init(void);
extern void Referee_Task(void);
extern void RefereeDataUpdate(referee_data_t *data);

#endif
