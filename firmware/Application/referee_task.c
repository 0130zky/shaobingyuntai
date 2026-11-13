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
 *  @description ：裁判系统任务：初始化接收并解析裁判数据帧。
 */
#include "referee_task.h"
#include "referee.h"

referee_data_t RefereeData;

// 10ms 100hz执行频率
void Referee_Task_Init(void){
    refereeINIT();
}

void Referee_Task(void){
	referee_unpack_fifo_data();
}

/**
 * @brief 裁判系统数据结构体更新函数
 *
 * @param data
 */
void RefereeDataUpdate(referee_data_t *data) {
  if (shoot_data.shooter_number == 1) {
    data->gunHeat1 = power_heat_data.shooter_17mm_1_barrel_heat;
  }
  else {
    data->gunHeat1 =
      power_heat_data.shooter_17mm_2_barrel_heat;  // 如果是42mm枪管，后面再加一个判断
  }
  if (shoot_data.initial_speed < 10.0f) {  // 如过初速度小于10m/s，则发送25m/s
    data->gunSpeed1 = 25.0f;
  }
  else {
    data->gunSpeed1 = shoot_data.initial_speed;
  }

  data->maxHeat = robot_state.shooter_barrel_heat_limit;
  if (robot_state.robot_id == 3 || robot_state.robot_id == 4 || robot_state.robot_id == 5
      || robot_state.robot_id == 7) {
    data->target = 0;  // 目标为蓝色，代号为0
  }
  else {
    data->target = 1;  // 目标为红色，代号为1
  }
}
