#include "robot.h"
#include "sentry.h"
#include "usb_device.h"
void Robot_Init(void) { Sentry_Init(); }
void Robot_ControlStep(float dt) { Sentry_Control(dt); }
void Robot_BackgroundInit(void) { MX_USB_DEVICE_Init(); }
void Robot_BackgroundStep(void) { Sentry_Background(); }
void Robot_UsbRx(uint8_t *p,uint32_t n) { Sentry_UsbRx(p,n); }
void Robot_UartRxByte(uint8_t b) { (void)b; }
void Robot_Can1Rx(uint16_t id,const uint8_t *p,uint8_t n) { Sentry_Rx(id,p,n); }
void Robot_Can2Rx(uint16_t id,const uint8_t *p,uint8_t n) { (void)id;(void)p;(void)n; }
