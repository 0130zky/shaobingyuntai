/*
 * @File         :
 * @Author       : JL HUANG
 * @Date         : 2023-11-22 18:39:05
 * @LastEditTime : 2024-04-27 18:20:15
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\robot_template\Application\gimbal_task.h
 * @Description  :
 */

#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "main.h"
#include "app_RM_Motor.h"
#include "app_DM4310.h"
#include "module_task.h"
#include "controller.h"
#include "ins_task.h"
#include "RC_task.h"
#include "master_task.h"
#include "PID_task.h"
#include "bsp_tim.h"

typedef struct
{
  RM_Motor_t     RM_Motor[2];
  float          Pitch_angle;
  float          Yaw_angle;
  uint8_t	       Gimbal_mode;
  DM4310_Motor_t DM4310;
  float 		 DM_PitchAngle;
  float 		 DM_pitchSpeed;
  float			 torque_hat;
  winter_variance_t pitch_fli;
} gimbal_t;

typedef struct {
  // ����������
  float encoder_scale;      // ����������ϵ��     刻度
  float encoder_offset;     // ������ƫ����        抵消

  // ���Ŷȶ���ʽϵ��
  float confidence_a;       // ������ϵ��
  float confidence_b;       // һ����ϵ��
  float confidence_c;       // ������
	
  float angle_hat;			//����Ƕ�
  float encoder_hat;		//����Ƕȶ�Ӧ�ı���ֵ
} ComplementaryFilter;

extern gimbal_t GIMBAL;

void MiniGimbal_Control(float angle_image);
void Telescope_Control(float angle_telescope);
void MiniGimbal_Disable(void);
void ComplementaryFilter_Init(ComplementaryFilter* filter, float pitch_min, float pitch_max);
float ComplementaryFilter_Update(ComplementaryFilter* filter, float encoder_raw, float imu_angle, float set_angle);
void GimbalTask(void);
void GimbalInit(void);
void GimbalReset(void);
void GimbalCtrl(void);

#endif
