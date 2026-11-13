#include "robot.h"
#include "booster_task.h"
#include "gimbal.h"
#include "module_task.h"
#include "bsp_dwt.h"
static uint32_t control_start;
void Robot_Init(void) { BoosterInit(); Gimbal_Init(); }
void Robot_ControlStep(float dt_s)
{
    static uint32_t previous;
    static uint8_t started;
    control_start = DWT->CYCCNT;
    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    const uint32_t period_us = started ? (uint32_t)(control_start-previous)/cycles_per_us : 0U;
    previous = control_start;
    started = 1;
    (void)dt_s;
    /* Preserve the original control-entry clock and first-cycle policy. */
    dt_s = period_us ? period_us * 0.000001f : 0.003f;
    Booster_ControlStep(dt_s);
    Gimbal_ControlStep(dt_s);
    Booster_RecordPIDTiming(period_us, (uint32_t)(DWT->CYCCNT-control_start)/cycles_per_us);
}
void Robot_BackgroundInit(void) { Module_Init(); }
void Robot_BackgroundStep(void) { Module_Task(); }
void Robot_UsbRx(uint8_t *data, uint32_t length) { Booster_UsbCmdHandler(data, length); }
void Robot_UartRxByte(uint8_t byte) { FrictionScope_RxByte(byte); }
void Robot_Can1Rx(uint16_t id, const uint8_t *data, uint8_t length)
{
    if (Gimbal_DiagnosticRx(id, data, length)) return;
    Booster_OnCanRx(id, data, length);
    Gimbal_OnCanRx(id, data, length);
}
void Robot_Can2Rx(uint16_t id, const uint8_t *data, uint8_t length)
{ (void)id; (void)data; (void)length; }
