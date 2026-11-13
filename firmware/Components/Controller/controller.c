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
 *  @description ：纯 C PID/PI 数学实现：积分限幅与输出滤波，时钟由调用方提供。
 */
#include "controller.h"
#include <math.h>
#include <string.h>
static void f_Integral_Limit(PID_t *pid);
static void f_Output_Filter(PID_t *pid);
static void f_Output_Limit(PID_t *pid);

void PID_Init(PID_t *pid,float maximum,float integral_limit,float kp,float ki,float kd,
              float output_filter_rc,uint8_t flags)
{
  memset(pid,0,sizeof *pid);
  pid->MaxOut=maximum;pid->IntegralLimit=integral_limit;
  pid->Kp=kp;pid->Ki=ki;pid->Kd=kd;
  pid->Output_LPF_RC=output_filter_rc;pid->Improve=flags;
}

float PID_CalculateDt(PID_t *pid,float measure,float ref,float dt_s)
{
  if (!isfinite(dt_s) || dt_s<=0.0f) { pid->Output=0;return 0; }
  pid->dt=dt_s;pid->Measure=measure;pid->Ref=ref;pid->Err=ref-measure;
  /* Preserve the zero-error behavior already handled explicitly by the dial. */
  if (fabsf(pid->Err)>pid->DeadBand) {
    pid->Pout=pid->Kp*pid->Err;
    pid->ITerm=pid->Ki*pid->Err*dt_s;
    pid->Dout=pid->Kd*(pid->Err-pid->Last_Err)/dt_s;
    if (pid->Improve & Integral_Limit) f_Integral_Limit(pid);
    pid->Iout+=pid->ITerm;
    pid->Output=pid->Pout+pid->Iout+pid->Dout;
    if (pid->Improve & OutputFilter) f_Output_Filter(pid);
    f_Output_Limit(pid);
  }
  pid->Last_Measure=pid->Measure;pid->Last_Output=pid->Output;
  pid->Last_Dout=pid->Dout;pid->Last_Err=pid->Err;pid->Last_ITerm=pid->ITerm;
  return pid->Output;
}

static void f_Integral_Limit(PID_t *pid)
{
    float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (fabsf(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0)
        {
            // 积分呈累积趋势
            // Integral still increasing
            pid->ITerm = 0;
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

static void f_Output_Filter(PID_t *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

static void f_Output_Limit(PID_t *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}
