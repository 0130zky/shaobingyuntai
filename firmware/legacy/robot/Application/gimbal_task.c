/*
 * @File         :
 * @Author       : JL HUANG
 * @Date         : 2023-11-22 18:38:50
 * @LastEditTime : 2024-05-03 20:27:21
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\robot_template\Application\gimbal_task.c
 * @Description  : R标右边为Pitch正方向，R标上边为Roll轴正方向
 */

#include "gimbal_task.h"
#include "arm_math.h"

#define DEBUG_MODE 0
float  taque_pich = 2 ;
float tim;
gimbal_t GIMBAL;
ComplementaryFilter fil;
//winter_variance_t pitch_angle_variance;
 
/**
 * @description  :
 * @return        {*}
 */
void GimbalInit(void)
{
  PID_Init(&GIMBAL.RM_Motor[GM6020_ID_Yaw - 5].MotorCtrl_Angle,GM6020_Yaw_AngleMaxOut,GM6020_Yaw_AngleMax_I_Out,0,0.35f,0.00f,0.008f,0,0,0,0,0,Integral_Limit);
  PID_Init(&GIMBAL.RM_Motor[GM6020_ID_Yaw - 5].MotorCtrl_Speed,GM6020_Yaw_SpeedMaxOut,GM6020_Yaw_SpeedMax_I_Out,0,-6000.0f,-8.0f,0.0f,0,0,0,0,0,Integral_Limit);

 
  PID_Init(&GIMBAL.DM4310.MotorCtrl_Angle,DM4310_AngleMaxOut,DM4310_AngleMax_I_Out,0.0f,   0.55f,0.00f,0.060f,     0,0,0,0,0,  Integral_Limit|Trapezoid_Intergral);
  PID_Init(&GIMBAL.DM4310.MotorCtrl_Speed,DM4310_SpeedMaxOut,DM4310_SpeedMax_I_Out,0.0f,   4.4f,0.62f,0.0f,      0,0,0,0,0,    Integral_Limit|Trapezoid_Intergral);
	              
	     
  ComplementaryFilter_Init(&fil, 2.934f, 3.985f); 
  GIMBAL.Pitch_angle = 0.0f;
}

uint32_t mail_p;
float TEST = 0.0f ;
/**
 * @description  :
 * @return        {*}
 */
void GimbalTask(void)
{
    //vTaskSuspendAll(); //开启设备锁，防止任务打断
    if(GIMBAL.Yaw_angle >= 180)  GIMBAL.Yaw_angle -= 360;
    if(GIMBAL.Yaw_angle <= -180) GIMBAL.Yaw_angle += 360;
    //GIMBAL.RM_Motor[GM6020_ID_Pitch - 5].RefAngle = INS.Pitch;
    GIMBAL.RM_Motor[GM6020_ID_Yaw - 5].RefAngle = INS.Yaw;
    GIMBAL.RM_Motor[GM6020_ID_Yaw - 5].RefSpeed = INS.Gyro[2];
    //云台电机
	if(MasterLink.UP_Board_SetData.Mode_data != Free_Mode){
		#if !DEBUG_MODE
			//yaw轴pid计算
			PID_AngleCascadeCtrl(&GIMBAL.RM_Motor[GM6020_ID_Yaw - 5], Angle_Zero_Cross(GIMBAL.RM_Motor[GM6020_ID_Yaw - 5].RefAngle, GIMBAL.Yaw_angle));
			//循环使能4310
			DM4310_Enable(&JointBus, PITCH4310_TX_ID);
			//融合编码器和陀螺仪角度
			//GIMBAL.DM4310.RefAngle = ComplementaryFilter_Update(&fil, GIMBAL.DM4310.DM4310_MotorMeasure.Position, -INS.Pitch, GIMBAL.Pitch_angle);
			GIMBAL.DM4310.RefAngle = -INS.Pitch;
			//ComplementaryFilter_Update(&fil, GIMBAL.DM4310.DM4310_MotorMeasure.Position, -INS.Pitch, GIMBAL.Pitch_angle);
			GIMBAL.DM4310.RefSpeed = INS.Gyro[0];
			//GIMBAL.DM4310.RefSpeed = -GIMBAL.DM4310.DM4310_MotorMeasure.Velocity;
			//pitch轴pid计算
			PID_AngleCascadeCtrl_DM(&GIMBAL.DM4310, GIMBAL.Pitch_angle);
			//前馈力矩
			//GIMBAL.torque_hat = 9.5902f * arm_sin_f32((-INS.Pitch)*0.0175f + 2.4365f);
            GIMBAL.torque_hat = 11.7858f * arm_sin_f32((-INS.Pitch)*0.0175f + 2.4477f);
			//发送电流
			if(GIMBAL.Pitch_angle > GIMBAL.DM4310.RefAngle)
			//测试
			//Set_DM4310_MotorCurrent(&JointBus, PITCH4310_TX_ID, 0.0f, 0.0f, TEST , 0.0f, 0.0f);					
			Set_DM4310_MotorCurrent(&JointBus, PITCH4310_TX_ID, 0.0f, 0.0f, GIMBAL.DM4310.MotorCtrl_Speed.Output + GIMBAL.torque_hat , 0.0f, 0.0f);

			if(GIMBAL.Pitch_angle < GIMBAL.DM4310.RefAngle)
			//测试
			//Set_DM4310_MotorCurrent(&JointBus, PITCH4310_TX_ID, 0.0f, 0.0f, TEST , 0.0f, 0.0f);				
			Set_DM4310_MotorCurrent(&JointBus, PITCH4310_TX_ID, 0.0f, 0.0f, GIMBAL.DM4310.MotorCtrl_Speed.Output + GIMBAL.torque_hat , 0.0f, 0.0f);

						
			Set_RM_MotorCurrent(&GimbalBus,GM6020_SET_STD_ID,0,GIMBAL.RM_Motor[GM6020_ID_Yaw - 5].MotorCtrl_Speed.Output,0,0, &mail_p);
			if(MasterLink.Hid_Hero.MiniGimbal_Enable == 1) MiniGimbal_Control(45.0f);   //舵机云台
			if(MasterLink.Hid_Hero.MiniGimbal_Enable == 0) MiniGimbal_Control(65.0f);
			if(MasterLink.Hid_Hero.Telescope_Enable == 1) Telescope_Control(48.0f);     //倍镜
			if(MasterLink.Hid_Hero.Telescope_Enable == 0) Telescope_Control(0.0f);
		#endif
		#if DEBUG_MODE
			//MiniGimbal_Control(90.0f, 90.0f);
			GIMBAL.DM4310.RefAngle = ComplementaryFilter_Update(&fil, GIMBAL.DM4310.DM4310_MotorMeasure.Position, -INS.Pitch, GIMBAL.Pitch_angle);
			DM4310_Enable(&JointBus, PITCH4310_TX_ID);
			GIMBAL.torque_hat = (GIMBAL.Pitch_angle + 20.0f) * 0.15f;
			GIMBAL.torque_hat = 11.514f * arm_sin_f32((-INS.Pitch)*0.0175f + 2.2862f);
			Set_DM4310_MotorCurrent(&JointBus, PITCH4310_TX_ID, 0.0f, 0.0f, GIMBAL.torque_hat, 0.0f, 0.0f);
		#endif
	}
    else{
        Set_RM_MotorCurrent(&GimbalBus, GM6020_SET_STD_ID, 0, 0, 0, 0, &mail_p);
		DM4310_Disable(&JointBus, PITCH4310_TX_ID);
		MiniGimbal_Disable();
    }
//    tim = RunTimStop();
//    RunTimStart();
//    GIMBAL.Pitch_angle_calc
//    xTaskResumeAll(); //关闭设备锁
}

void MiniGimbal_Control(float angle_image) //小云台
{
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,(uint16_t)(50.0f + 1.111f * angle_image)); // 200/180
}
void Telescope_Control(float angle_telescope)  //倍镜
{
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,(uint16_t)(50.0f + 1.111f * angle_telescope));
}
void MiniGimbal_Disable(void)
{
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
}

/**
 * @brief 初始化互补滤波器结构体
 * @param filter 滤波器对象指针
 */
void ComplementaryFilter_Init(ComplementaryFilter* filter, float pitch_min, float pitch_max) {

    filter->encoder_scale  = 60 / (pitch_max - pitch_min);
    filter->encoder_offset = 40 - filter->encoder_scale * pitch_max;
    filter->confidence_a   = 0.001f;
    filter->confidence_b   = -0.02f;
    filter->confidence_c   = 0.2f;
}

/**
 * @brief 执行互补滤波计算
 * @param filter        滤波器对象指针
 * @param encoder_raw   编码器原始值
 * @param imu_angle     IMU原始角度
 * @param set_angle     目标角度
 * @return float        融合后的角度
 */
float ComplementaryFilter_Update(ComplementaryFilter* filter,
                                float encoder_raw,
                                float imu_angle,
								float set_angle)
{
    // 计算编码器推算角度
    const float encoder_angle = filter->encoder_scale * encoder_raw + filter->encoder_offset;

    // 计算编码器置信度（二次多项式）
    float confidence = filter->confidence_a * encoder_angle * encoder_angle
                      + filter->confidence_b * encoder_angle
                      + filter->confidence_c;

    // 置信度钳位在[0.0, 1.0]范围
    confidence = confidence < 0.0f ? 0.0f : (confidence > 1.0f ? 1.0f : confidence);

    // 执行互补滤波融合
    filter -> angle_hat = encoder_angle * confidence + imu_angle * (1.0f - confidence);

	//通过融合后的数据预测编码值
	filter -> encoder_hat = (set_angle - filter->encoder_offset) / filter->encoder_scale;

	return filter -> angle_hat;
	//return encoder_angle;
}

/**
 * @description  :
 * @return        {*}
 */
void GimbalReset(void)
{
}

/**
 * @description  :
 * @return        {*}
 */
void GimbalCtrl(void)
{
}
