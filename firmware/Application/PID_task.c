/**
 *  ███╗   ██╗███████╗██╗  ██╗████████╗  ███████╗
 *  ████╗  ██║██╔════╝╚██╗██╔╝╚══██╔══╝  ██╔════╝
 *  ██╔██╗ ██║█████╗   ╚███╔╝    ██║     █████╗  
 *  ██║╚██╗██║██╔══╝   ██╔██╗    ██║     ██╔══╝  
 *  ██║ ╚████║███████╗██╔╝ ██╗   ██║     ███████╗
 *  ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝   ╚═╝     ╚══════╝
 *
 *  @Author : Kaiyue Zhang
 *  @Email  : kaiyue.link@gmail.com
 *  @date   : 2026-10-30
 *  @description ：高频控制任务入口：测量控制周期并驱动一次发射控制拍。
 */
#include "PID_task.h"
#include "robot.h"
#include "bsp_dwt.h"
void PID_Task(void)
{
    static uint32_t previous;
    static uint8_t started;
    uint32_t now = DWT->CYCCNT;
    float dt_s = started ? (uint32_t)(now-previous) / (float)SystemCoreClock : 0.003f;
    previous = now;
    started = 1;
    Robot_ControlStep(dt_s);
}
