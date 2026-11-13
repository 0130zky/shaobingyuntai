/*
 * @File   : fdcan.c
 * @Brief  : FDCAN1 配置 —— 经典 CAN，1 Mbps
 *
 * 位时序推导：
 *   FDCAN 内核时钟 = PLL1Q = 80 MHz（见 main.c SystemClock_Config，PLLN=40 / PLLQ=6）
 *   NominalPrescaler = 1  -> tq = 1/80MHz = 12.5 ns
 *   1 + NominalTimeSeg1 + NominalTimeSeg2 = 1 + 59 + 20 = 80 tq = 1000 ns
 *   -> 1 Mbps，采样点 = (1+59)/80 = 75%
 *
 * 与 DM-J3507-2EC 通信要求电机侧 can_br 寄存器(0x23) = 4（1M），出厂默认即为该值。
 * 若改用 CAN FD，需同时把电机侧波特率改为 >4 并在达妙调试助手中落盘。
 */

#include "fdcan.h"

FDCAN_HandleTypeDef hfdcan1;

void MX_FDCAN1_Init(void)
{
  hfdcan1.Instance                  = FDCAN1;
  hfdcan1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;   /* 经典 CAN，非 FD */
  hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission   = ENABLE;
  hfdcan1.Init.TransmitPause        = DISABLE;
  hfdcan1.Init.ProtocolException    = DISABLE;               /* 仅 FD 模式需要 */
  hfdcan1.Init.NominalPrescaler     = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 20;
  hfdcan1.Init.NominalTimeSeg1      = 59;
  hfdcan1.Init.NominalTimeSeg2      = 20;
  /* 以下 Data* 参数仅在 FD 帧下生效，保留默认值 */
  hfdcan1.Init.DataPrescaler        = 1;
  hfdcan1.Init.DataSyncJumpWidth    = 2;
  hfdcan1.Init.DataTimeSeg1         = 13;
  hfdcan1.Init.DataTimeSeg2         = 2;
  hfdcan1.Init.MessageRAMOffset     = 0;
  hfdcan1.Init.StdFiltersNbr        = 4;
  hfdcan1.Init.ExtFiltersNbr        = 4;
  hfdcan1.Init.RxFifo0ElmtsNbr      = 16;
  hfdcan1.Init.RxFifo0ElmtSize      = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr      = 0;
  hfdcan1.Init.RxFifo1ElmtSize      = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr         = 0;
  hfdcan1.Init.RxBufferSize         = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr          = 0;
  hfdcan1.Init.TxBuffersNbr         = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr  = 8;
  hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize           = FDCAN_DATA_BYTES_8;

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  if (fdcanHandle->Instance == FDCAN1)
  {
    /* FDCAN 内核时钟源 = PLL1Q = 80 MHz */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /**FDCAN1 GPIO Configuration
    PD0 ------> FDCAN1_RX
    PD1 ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcanHandle)
{
  if (fdcanHandle->Instance == FDCAN1)
  {
    __HAL_RCC_FDCAN_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  }
}
