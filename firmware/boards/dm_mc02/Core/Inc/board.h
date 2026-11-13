#ifndef SENTRY_MC02_BOARD_H
#define SENTRY_MC02_BOARD_H
#include "main.h"
extern UART_HandleTypeDef huart5;
void Board_Init(void);
void Board_RemotePoll(void);
void Board_CanStop(void);
void Board_CanRecover(void);
void Board_Buzzer(int on);
void Board_BuzzerInit(void);
void MX_FREERTOS_Init(void);
#endif
