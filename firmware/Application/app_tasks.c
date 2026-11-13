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
 *  @description ：FreeRTOS 任务创建与周期调度外壳。
 */
#include "app_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "PID_task.h"
#include "robot.h"
#include "referee_task.h"
#ifndef ROBOT_TASK_STACK_WORDS
#define ROBOT_TASK_STACK_WORDS 256
#endif
osThreadId ModuleTaskHandle,PIDTaskHandle,Referee_TaskHandle;
static void StartModuleTask(void const *argument);
static void StartPIDTask(void const *argument);
#if ROBOT_HAS_REFEREE
static void StartRefereeTask(void const *argument);
#endif
void App_TasksInit(void)
{
  osThreadDef(ModuleTask, StartModuleTask, osPriorityBelowNormal, 0, ROBOT_TASK_STACK_WORDS);
  ModuleTaskHandle = osThreadCreate(osThread(ModuleTask), NULL);
  osThreadDef(PIDTask, StartPIDTask, osPriorityHigh, 0, ROBOT_TASK_STACK_WORDS);
  PIDTaskHandle = osThreadCreate(osThread(PIDTask), NULL);
#if ROBOT_HAS_REFEREE
  osThreadDef(Referee_Task, StartRefereeTask, osPriorityBelowNormal, 0, ROBOT_TASK_STACK_WORDS);
  Referee_TaskHandle = osThreadCreate(osThread(Referee_Task), NULL);
#endif
}
static void StartModuleTask(void const *argument)
{
  (void)argument;Robot_BackgroundInit();
  for (;;) { Robot_BackgroundStep();osDelay(3); }
}
static void StartPIDTask(void const *argument)
{
  (void)argument;
  for (;;) { PID_Task();osDelay(3); }
}
#if ROBOT_HAS_REFEREE
static void StartRefereeTask(void const *argument)
{
  (void)argument;Referee_Task_Init();
  for (;;) { Referee_Task();osDelay(10); }
}

#endif
