#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "board.h"
#include "sentry.h"
#include "usb_device.h"
static StaticTask_t control_tcb,background_tcb;
static StackType_t control_stack[768],background_stack[512];
static StaticTask_t idle_tcb,timer_tcb;
static StackType_t idle_stack[configMINIMAL_STACK_SIZE],timer_stack[configTIMER_TASK_STACK_DEPTH];
static void Control(void const *arg)
{
    (void)arg;uint32_t previous=DWT->CYCCNT;TickType_t wake=xTaskGetTickCount();
    for(;;) {
        uint32_t now=DWT->CYCCNT;float dt=(uint32_t)(now-previous)/(float)SystemCoreClock;
        previous=now;Board_RemotePoll();Board_CanRecover();Sentry_Control(dt);
        vTaskDelayUntil(&wake,pdMS_TO_TICKS(3));
    }
}
volatile uint32_t board_boot_beeps;
static void Background(void const *arg)
{
    (void)arg;MX_USB_DEVICE_Init();
    /* Startup indication runs below control priority; no delay in the 3ms loop. */
    for(unsigned i=0;i<2;i++) {
        Board_Buzzer(1);osDelay(100);Board_Buzzer(0);board_boot_beeps++;osDelay(100);
    }
    for(;;) {Sentry_Background();osDelay(3);}
}
void MX_FREERTOS_Init(void)
{
    osThreadStaticDef(sentryControl,Control,osPriorityHigh,0,768,control_stack,&control_tcb);
    osThreadStaticDef(sentryBackground,Background,osPriorityBelowNormal,0,512,background_stack,&background_tcb);
    if(!osThreadCreate(osThread(sentryControl),0) || !osThreadCreate(osThread(sentryBackground),0))Error_Handler();
}
void vApplicationGetIdleTaskMemory(StaticTask_t **t,StackType_t **s,uint32_t *n)
{*t=&idle_tcb;*s=idle_stack;*n=configMINIMAL_STACK_SIZE;}
void vApplicationGetTimerTaskMemory(StaticTask_t **t,StackType_t **s,uint32_t *n)
{*t=&timer_tcb;*s=timer_stack;*n=configTIMER_TASK_STACK_DEPTH;}
void vApplicationStackOverflowHook(TaskHandle_t t,char *name)
{(void)t;(void)name;Error_Handler();}
void vApplicationMallocFailedHook(void) {Error_Handler();}
