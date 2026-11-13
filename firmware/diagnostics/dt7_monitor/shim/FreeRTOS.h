#ifndef DIAG_CRITICAL_H
#define DIAG_CRITICAL_H
#include "main.h"
#define taskENTER_CRITICAL() uint32_t diag_primask=__get_PRIMASK();__disable_irq()
#define taskEXIT_CRITICAL() __set_PRIMASK(diag_primask)
#endif
