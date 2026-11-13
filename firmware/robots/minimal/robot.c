/* Starting point: USB + LED + SBUS reception; no motor driver or motor TX.
 * Add device initialization, feedback dispatch and bounded control here.
 * CAN2 is not started by this board's BSP; configure it before using its hook.
 */
#include "robot.h"
#include "usb_device.h"
#include "bsp_tim.h"
#include "main.h"
void Robot_Init(void) {}
void Robot_ControlStep(float dt_s) { (void)dt_s; }
void Robot_BackgroundInit(void) { MX_USB_DEVICE_Init(); }
void Robot_BackgroundStep(void)
{
    static uint32_t last;
    if ((uint32_t)(HAL_GetTick()-last) >= 500U) {
        last = HAL_GetTick();
        LED_Green();
    }
}
void Robot_UsbRx(uint8_t *data, uint32_t length) { (void)data; (void)length; }
void Robot_UartRxByte(uint8_t byte) { (void)byte; }
void Robot_Can1Rx(uint16_t id, const uint8_t *data, uint8_t length)
{ (void)id; (void)data; (void)length; }
void Robot_Can2Rx(uint16_t id, const uint8_t *data, uint8_t length)
{ (void)id; (void)data; (void)length; }
