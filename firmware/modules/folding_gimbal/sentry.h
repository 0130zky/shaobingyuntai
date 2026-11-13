#ifndef SENTRY_H
#define SENTRY_H
#include <stdint.h>
void Sentry_Init(void);
void Sentry_InvalidateFeedback(void);
void Sentry_Control(float dt);
void Sentry_Rx(uint16_t id,const uint8_t *data,uint8_t length);
void Sentry_UsbRx(const uint8_t *data,uint32_t length);
void Sentry_Background(void);
#endif
