/* Robot assembly boundary. ISR hooks must remain bounded and non-blocking. */
#ifndef ROBOT_H
#define ROBOT_H
#include <stdint.h>
void Robot_Init(void);
void Robot_ControlStep(float dt_s);
void Robot_BackgroundInit(void);
void Robot_BackgroundStep(void);
void Robot_UsbRx(uint8_t *data, uint32_t length);
void Robot_UartRxByte(uint8_t byte);
void Robot_Can1Rx(uint16_t id, const uint8_t *data, uint8_t length);
void Robot_Can2Rx(uint16_t id, const uint8_t *data, uint8_t length);
#endif
