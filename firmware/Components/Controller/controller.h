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
 *  @description ：PID/PI 数学接口与控制器状态定义。
 */
#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <stdint.h>
/* Pure C math: the caller owns the clock, state and scheduling. */
typedef enum pid_Improvement_e { NONE=0, Integral_Limit=0x01, OutputFilter=0x10 } PID_Improvement_e;
typedef struct {
  float Ref,Kp,Ki,Kd,Measure,Last_Measure,Err,Last_Err,Last_ITerm;
  float Pout,Iout,Dout,ITerm,Output,Last_Output,Last_Dout;
  float MaxOut,IntegralLimit,DeadBand,Output_LPF_RC,dt;
  uint8_t Improve;
} PID_t;
void PID_Init(PID_t *pid,float maximum,float integral_limit,float kp,float ki,float kd,
              float output_filter_rc,uint8_t flags);
float PID_CalculateDt(PID_t *pid,float measure,float ref,float dt_s);
#endif
