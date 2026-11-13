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
 *  @description ：系统启动初始化：外设、设备、CAN 路由与任务启动顺序。
 */
#include "sys.h"
#include "robot.h"
#include "app_can_router.h"

/**
 * @description  : 系统初始化，在RTOS前完成
 * @return        {*}
 */
void SysInit(void)
{
  /* 板载LED初始化 */
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

  /* 板载蜂鸣器初始化 */
  BuzzerStop();
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) == HAL_OK) {
    BuzzerStart();
    HAL_Delay(120U);
    BuzzerStop();
  }

  /* PID时钟初始化 */
  DWT_Init(168);
  
  /* 在 CAN 接收和 RTOS 控制任务开始前初始化反馈与 PID。 */
  Robot_Init();

  /* CAN初始化 */
  App_CAN_InitRoutes();
  CAN_Filter_Init();

  /* sbus空闲中断双缓冲区初始化 */
  W_BUS_Init(W_BUSRxBuffer[0],W_BUSRxBuffer[1],W_BUS_RxBuff_NUM);
}
