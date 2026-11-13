#ifndef _VOFA_PLUS_H
#define _VOFA_PLUS_H

#include "main.h"

typedef union{
    float datafloat;
    uint16_t  datau16[2];
}vofa_data_u;

void Send_Float_USART(UART_HandleTypeDef* huart, vofa_data_u* data,uint16_t len);

#endif
