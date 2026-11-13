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
 *  @description ：定时器与蜂鸣器底层驱动。
 */
#include "bsp_tim.h"
#include "tim.h"

void BuzzerStart(void)
{
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,9000);
}

/**
 * @description  : 
 * @return        {*}
 */
void BuzzerStop(void)
{
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
}

/**
 * @description  : 
 * @return        {*}
 */
void LED_Blue(void)
{
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,5000);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,0);
}

/**
 * @description  : 
 * @return        {*}
 */
void LED_Green(void)
{
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,5000);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,0);
}

/**
 * @description  : 
 * @return        {*}
 */
void LED_Red(void)
{
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,5000);
}

/**
 * @description  : 
 * @return        {*}
 */
void LED_White(void)
{
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,5000);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,5000);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,5000);
}
