/*
 * @File   : gpio.c
 * @Brief  : 板级 GPIO —— 可控电源使能 + 用户按键
 *
 * 注意：PC13/PC14/PC15 在 H723 上默认是 RTC 的 LSE 引脚，本板用作电源使能，
 *       因此 .ioc 中必须关闭 LSE（RCC.LSEState=RCC_LSE_OFF），否则时钟配置会失败。
 */

#include "gpio.h"

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* 先拉高再配置为输出，避免上电瞬间电源轨抖动 */
  HAL_GPIO_WritePin(GPIOC, POWER_24V_1_Pin | POWER_24V_2_Pin | POWER_5V_Pin, GPIO_PIN_SET);

  /* 可控电源使能：24V_1(PC14) / 24V_2(PC13) / 5V(PC15) */
  GPIO_InitStruct.Pin   = POWER_24V_1_Pin | POWER_24V_2_Pin | POWER_5V_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* 用户按键 PA15，外部已有上拉，按下为低 */
  GPIO_InitStruct.Pin  = USER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_KEY_GPIO_Port, &GPIO_InitStruct);
}
