#include "vofa_plus.h"
#include "usart.h"


void Send_Float_USART(UART_HandleTypeDef* huart, vofa_data_u* data,uint16_t len)
{
    uint16_t i,j;
    for(i = 0;i < len;i++)
    {
        for(j = 0;j < 2;j++)
        {
            HAL_UART_Transmit_DMA(huart,(const uint8_t*)(((data+i)->datau16[j])&0x00ff),8);
            HAL_UART_Transmit_DMA(huart,(const uint8_t*)((((data+i)->datau16[j])>>8)&0x00ff),8);
        }
    }

    HAL_UART_Transmit_DMA(huart, (const uint8_t*)(0x00),8);
    HAL_UART_Transmit_DMA(huart, (const uint8_t*)(0x00),8);
    HAL_UART_Transmit_DMA(huart, (const uint8_t*)(0x80),8);
    HAL_UART_Transmit_DMA(huart, (const uint8_t*)(0x7f),8);
}
