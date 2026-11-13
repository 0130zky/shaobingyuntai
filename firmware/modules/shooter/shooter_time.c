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
 *  @description ：控制拍时钟：测量相邻控制入口的周期并判定时间有效性。
 */
#include "shooter_internal.h"
#include "control_time.h"
static control_time_t s_clock;
void Shooter_TimeInit(void) { s_clock = (control_time_t){0}; }
uint8_t Shooter_TimeStep(uint32_t now_ms, float dt_s) { return ControlTime_Update(&s_clock, now_ms, dt_s); }
float Shooter_ControlDt(void) { return s_clock.dt_s; }
uint8_t Shooter_TimingFault(void) { return s_clock.fault; }
