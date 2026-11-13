/*
 * @File         :
 * @Author       : JL HUANG
 * @Date         : 2024-01-01 19:46:34
 * @LastEditTime : 2024-05-03 20:59:46
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\robot_template\Application\PID_task.h
 * @Description  :
 */


#ifndef __PID_TASK_H
#define __PID_TASK_H

#include "main.h"
#include "app_RM_Motor.h"
#include "app_DM4310.h"
#include "ins_task.h"

void PID_Task(void);
void PID_AngleCascadeCtrl(RM_Motor_t* rm_motor, float set_angle);
void PID_AngleCascadeCtrl_DM(DM4310_Motor_t* dm_motor, float set_angle);
void PID_FrictionCtrl(RM_Motor_t* rm_motor,int16_t spd);
float Angle_Zero_Cross(float Angle, float Target);

#endif
