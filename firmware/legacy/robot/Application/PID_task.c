/*
 * @File         :
 * @Author       : JL HUANG
 * @Date         : 2024-01-01 19:46:34
 * @LastEditTime : 2024-05-03 20:04:47
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\robot_template\Application\PID_task.c
 * @Description  :
 */

#include "PID_task.h"
#include "gimbal_task.h"
#include "booster_task.h"
#include "RC_task.h"
#include "master_task.h"

#include "controller.h"
#include "bsp_tools.h"
#include "dial_port.h"   /* 拨盘发送口：Dial_Port_RMSlot() / Dial_Port_Tx() */

#include "bsp_tim.h"
float test =0.0f;

extern float encoder_cofidence;
uint16_t can1_mail,can2_mail = 0;
/**
 * @description  :
 * @return        {*}
 */
void PID_Task(void)
{
    /* 独立读取CYCCNT，避免改动PI自身DWT时间戳。32位无符号差支持回绕。 */
    static uint32_t last_start;
    static uint8_t timing_started;
    const uint32_t start_cycles=DWT->CYCCNT;
    const uint32_t cycles_per_us=SystemCoreClock/1000000U;
    const uint32_t period_us=timing_started ? (uint32_t)(start_cycles-last_start)/cycles_per_us : 0U;
    last_start=start_cycles;
    timing_started=1;
    Booster_ControlStep();

    Booster_RecordPIDTiming(period_us, (uint32_t)(DWT->CYCCNT-start_cycles)/cycles_per_us);
    //xTaskResumeAll(); //关闭设备锁
}

/**
 * @description  : 角度环串级PID计算
 * @param         {RM_Motor_t*} rm_motor
 * @return        {*}
 */
void PID_AngleCascadeCtrl(RM_Motor_t* rm_motor, float set_angle)
{
    PID_Calculate(&rm_motor -> MotorCtrl_Angle, rm_motor -> RefAngle, set_angle);
	PID_Calculate(&rm_motor -> MotorCtrl_Speed,rm_motor -> RefSpeed, rm_motor -> MotorCtrl_Angle.Output);
}

/**
 * @description  : 角度环串级PID计算
 * @param         {RM_Motor_t*} rm_motor
 * @return        {*}
 */
void PID_AngleCascadeCtrl_DM(DM4310_Motor_t* dm_motor, float set_angle)
{
    PID_Calculate(&dm_motor -> MotorCtrl_Angle, dm_motor -> RefAngle, set_angle);
	PID_Calculate(&dm_motor -> MotorCtrl_Speed,dm_motor -> RefSpeed, dm_motor -> MotorCtrl_Angle.Output);
}

void PID_FrictionCtrl(RM_Motor_t* rm_motor,int16_t spd)
{
  PID_Calculate(&rm_motor -> MotorCtrl_Speed,rm_motor -> RM_MotorMeasure.speed,spd);
}

//过零处理
float Angle_Zero_Cross(float Angle, float Target){
	float temp = Target - Angle;
	if(temp > 180){
		Target -= 360;
	}else if(temp < -180){
		Target += 360;
	}
	return Target;
}
