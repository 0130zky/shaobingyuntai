/*
 * @File         :
 * @Author       : JL HUANG
 * @Date         : 2023-12-24 16:16:17
 * @LastEditTime : 2024-05-19 17:27:14
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\robot_template\Application\RC_task.c
 * @Description  :
 */

#include "RC_task.h"
#include "master_task.h"
#include "math.h"
#include "FreeRTOS.h"

RC_Data_t RC_Data;
extern osThreadId PIDTaskHandle;
extern volatile uint32_t pack_time;   /* 接收机最近一帧的时间戳（app_et07.c 更新） */

#define RC_SW_UP           		  ((uint16_t)240)
#define RC_SW_MID                 ((uint16_t)1024)
#define RC_SW_DOWN                ((uint16_t)1807)

/* 接收机在线判据 / CH6 总使能。RF209S 实测：-100≈353，+100≈1696，中心 1024，
   所以用 1500 当"开关拨到使能侧"的门限；回中/掉线都按失能处理（安全侧）。 */
uint8_t RC_LinkFresh(void)
{
    return (pack_time != 0) && ((HAL_GetTick() - pack_time) < 100);
}

uint8_t RC_MasterEnabled(void)
{
    if (!RC_LinkFresh()) return BOOSTER_RC_ONLY ? 0 : 1;       /* 合体版失联停机；USB 台架版才允许回退。 */
    return (RC_Data.ET07_DATA.SB_CH6 > 1500) ? 1 : 0;
}

/**
 * @description  : CH8 旋钮 → 三路摩擦轮统一转速(rpm)
 *                 旋钮量程 -100..+100（中位 0）。中位 5750，顺时针到底 10000；
 *                 逆时针 -80 以下到 -100 是死区，直接停转。
 *                 CH6 总使能关 / 链路掉线时返回 0。
 * @return        {int16_t} rpm，0 = 停
 */
int16_t RC_FrictionWheelRpm(void)
{
    if (!RC_LinkFresh() || !RC_MasterEnabled()) return 0;

    int32_t n = ((int32_t)RC_Data.ET07_DATA.SD_CH8 - RC_CH8_RAW_MID) * 100
                / (RC_CH8_RAW_MAX - RC_CH8_RAW_MID);
    if (n >  100) n =  100;
    if (n < -100) n = -100;

    if (n <= RC_FRI_DEAD_NEG) return 0;                       /* -100..-80：停 */
    if (n < 0)                                                 /* -80..0：0 → 中位标定转速 */
        return (int16_t)(RC_FRI_RPM_MID * (n - RC_FRI_DEAD_NEG)
                         / (0 - RC_FRI_DEAD_NEG));
    return (int16_t)(RC_FRI_RPM_MID                            /* 0..100：中位标定转速 → 10000 */
                     + (RC_FRI_RPM_MAX - RC_FRI_RPM_MID) * n / 100);
}

/**
 * @description  :
 * @return        {*}
 */
void RC_Task(void)
{
	/*****左拨轮如果旋转到底，则为图传控制*****/
	if(RC_Data.ET07_DATA.V1_CH9 >= 1024 && RC_Data.ET07_DATA.V1_CH9 <= 1820){
		//摩擦轮统一在 booster_task.c 读取 CH6/CH8，此处不改 Fri_Mode
		Transfer_Image_Control();
	}
	else
	{
		//前进逻辑，摇杆无回中
		MasterLink.UP_Board_SetData.Spd_Y = ET07_Channel_Left_UD(ChassisMaxSpd);
		//if(RC_Data.ET07_DATA.SD_CH8 == RC_SW_DOWN)MasterLink.UP_Board_SetData.Spd_Y = -MasterLink.UP_Board_SetData.Spd_Y;

		//MasterLink.UP_Board_SetData.Spd_W = TurnAngle_Channel(2.0);
		MasterLink.UP_Board_SetData.Spd_X = ET07_Channel_Left_LR(ChassisMaxSpd);

		if(MasterLink.UP_Board_SetData.Mode_data != Free_Mode){
			if(RC_Data.ET07_DATA.RightDirection_CH2 >= 1024 + default_Deadzone ||
				RC_Data.ET07_DATA.RightDirection_CH2 <= 1024 - default_Deadzone )
			{
				//摇杆通道值映射云台角度
				GIMBAL.Pitch_angle += ET07_Channel_Right_UD(0.2);
			}
			//GIMBAL.Pitch_angle = MasterLink.UP_Board_SetData.Pitch_Angle;
			if(GIMBAL.Pitch_angle >= PITCH_MAX_ANGLE)GIMBAL.Pitch_angle = PITCH_MAX_ANGLE;
			if(GIMBAL.Pitch_angle <= PITCH_MIN_ANGLE)GIMBAL.Pitch_angle = PITCH_MIN_ANGLE;
			
			if(RC_Data.ET07_DATA.RightTransverse_CH1 >= 1024 + default_Deadzone ||
				RC_Data.ET07_DATA.RightTransverse_CH1 <= 1024 - default_Deadzone)
			{
				GIMBAL.Yaw_angle -= TurnAngle_Channel(0.35);
			}
			//GIMBAL.Yaw_angle = MasterLink.UP_Board_SetData.Yaw_Angle;
		}
		//摩擦轮统一在 booster_task.c 读取 CH6/CH8，此处不改 Fri_Mode
		/* CH6 只作总使能，发弹统一由下方 CH5/CH7 映射处理。 */
		//拨杆C控制整车状态（使能）拨杆A控制部署模式
		MasterLink.UP_Board_SetData.Mode_data=Mode_Channel();
		//拨杆D控制自瞄使能
		if(RC_Data.ET07_DATA.SD_CH8 == RC_SW_DOWN && (vision1.RXData.VisionRxData.state & 0x04) != 0)
		{   
			MasterLink.Hid_Hero.Vision_Enable = 1 ;
			GIMBAL.Pitch_angle = vision1.RXData.VisionRxData.PitchAngleTarget;
			GIMBAL.Yaw_angle = vision1.RXData.VisionRxData.YawAngleTarget;
		}
		 //vision标志位置0
        else { MasterLink.Hid_Hero.Vision_Enable = 0 ; }
	}
	/*****左拨轮如果旋转到底，则为图传控制*****/

	/* 发射机构输入由 Booster_ControlStep 统一采样和处理。 */
}

/**
 * @description  :
 * @param         {float} PITCH
 * @return        {*}
 */
float ET07_Channel_Right_UD(float max_spd)
{
  float SpeedET07;
  SpeedET07=(RC_Data.ET07_DATA.RightDirection_CH2 - 1024.0f) /(1807.0f - 1024.0f) * max_spd;

  if(fabs(SpeedET07)<0.005f)
    return 0.0f;
  else
  return SpeedET07;
}
/**
 * @description  :
 * @param         {float} max_spd
 * @return        {*}
 */
float ET07_Channel_Left_UD(float max_spd)
{
  float SpeedET07;
  SpeedET07=(RC_Data.ET07_DATA.LeftDirection_CH3 - 1024.0f) /(1807.0f - 1024.0f) * max_spd;

  if(fabs(SpeedET07)<0.03f)//建立死区
    return 0.0f;
  else
  return SpeedET07;
}

/**
 * @description  :
 * @param         {float} max_spd
 * @return        {*}
 */
float ET07_Channel_Left_LR(float max_spd)
{
  float SpeedET07;
  SpeedET07=(RC_Data.ET07_DATA.LeftTransverse_CH4 - 1024.0f) /(1807.0f - 1024.0f) * max_spd;

  if(fabs(SpeedET07)<0.03f)
    return 0.0f;
  else
  return SpeedET07;
}

/**
 * @description  : 转向遥控角度
 * @return        {*}
 */
float TurnAngle_Channel(float max_spd)
{
  float TurnAngle;
  TurnAngle=(RC_Data.ET07_DATA.RightTransverse_CH1 - 1024) /(1805.0f - 1024.0f) * max_spd;
  if(fabs(TurnAngle)<0.001f)
  return 0.0f;
  else return TurnAngle;
}


uint8_t Mode_Channel(void)
{
    uint8_t mode_flag = Free_Mode;
    if(RC_Data.ET07_DATA.SA_CH5 == RC_SW_UP){
        switch(RC_Data.ET07_DATA.SC_CH7){
        case RC_SW_UP:
            mode_flag = Free_Mode;
            break;
        case RC_SW_MID:
            mode_flag = Follow_Mode;
            break;
        case RC_SW_DOWN:
            mode_flag = Top_Mode;
            break;
        }
    }
    if(RC_Data.ET07_DATA.SA_CH5 == RC_SW_DOWN){
        switch(RC_Data.ET07_DATA.SC_CH7){
        case RC_SW_UP:
            mode_flag = Free_Mode;
            break;
        case RC_SW_MID:
            mode_flag = No_Follow_Mode;
            break;
        case RC_SW_DOWN:
            mode_flag = Deploy_Mode;
            break;
        }
    }
	if(mode_flag == Top_Mode && RC_Data.ET07_DATA.SD_CH8 == RC_SW_DOWN){
		mode_flag = Top_Mode_CW;
	}
	if(mode_flag == Follow_Mode && RC_Data.ET07_DATA.SD_CH8 == RC_SW_DOWN){
		MasterLink.Hid_Hero.Supercap_Enable = 1;
	}
	else if(mode_flag == Follow_Mode && RC_Data.ET07_DATA.SD_CH8 == RC_SW_UP)
		MasterLink.Hid_Hero.Supercap_Enable = 0;
//    if(RC_Data.Error_code & 0x01)
//    {
//        return Free_Mode;
//    }
	/* 原先这里 Error_code&0x02 会 vTaskSuspend(PIDTaskHandle), 而全工程没有 vTaskResume ——
	   卡弹一次就永久死机, 必须断电重上。已去掉: 卡弹只置错误位(module_task 里亮红灯提示),
	   清电流交给 PID_Task, 免得两个任务抢同一条总线。 */
    return mode_flag;
}

void Transfer_Image_Control(void)
{

	//WSAD逻辑,WD为正方向,shift控制超电使能

	if(referee_remote_control.KEY_W == 1 && referee_remote_control.KEY_S == 0) MasterLink.UP_Board_SetData.Spd_Y = ChassisMaxSpd;
	else if(referee_remote_control.KEY_S == 1 && referee_remote_control.KEY_W == 0) MasterLink.UP_Board_SetData.Spd_Y = -ChassisMaxSpd;
	else MasterLink.UP_Board_SetData.Spd_Y = 0;
	if(referee_remote_control.KEY_D == 1 && referee_remote_control.KEY_A == 0) MasterLink.UP_Board_SetData.Spd_X = ChassisMaxSpd / 1.5;
	else if(referee_remote_control.KEY_A == 1 && referee_remote_control.KEY_D == 0) MasterLink.UP_Board_SetData.Spd_X = -ChassisMaxSpd / 1.5;
	else MasterLink.UP_Board_SetData.Spd_X = 0;
	MasterLink.Hid_Hero.Supercap_Enable = referee_remote_control.KEY_Shift;

	//小云台状态切换

	if(referee_remote_control.KEY_E == 1 && MasterLink.Hid_Hero.MiniGimbal_Enable == 0 && RC_Data.key_static[0] == 0){
		MasterLink.Hid_Hero.MiniGimbal_Enable = 1;
		RC_Data.key_static[0] = 1;
	}
	else if(referee_remote_control.KEY_E == 1 && MasterLink.Hid_Hero.MiniGimbal_Enable == 1 && RC_Data.key_static[0] == 0) {
		MasterLink.Hid_Hero.MiniGimbal_Enable = 0;
		RC_Data.key_static[0] = 1;
	}else if(referee_remote_control.KEY_E == 0 && RC_Data.key_static[0] == 1) RC_Data.key_static[0] = 0;
	//倍镜状态切换
	if(referee_remote_control.KEY_Q == 1 && MasterLink.Hid_Hero.Telescope_Enable == 0 && RC_Data.key_static[1] == 0){
		MasterLink.Hid_Hero.Telescope_Enable = 1;
		RC_Data.key_static[1] = 1;
	}
	else if(referee_remote_control.KEY_Q == 1 && MasterLink.Hid_Hero.Telescope_Enable == 1 && RC_Data.key_static[1] == 0) {
		MasterLink.Hid_Hero.Telescope_Enable  = 0;
		RC_Data.key_static[1] = 1;
	}else if(referee_remote_control.KEY_Q == 0 && RC_Data.key_static[1] == 1) RC_Data.key_static[1] = 0;

	//鼠标右键控制自瞄
	
    if(referee_remote_control.right_button_down == 1 && (vision1.RXData.VisionRxData.state & 0x04) != 0){
        GIMBAL.Pitch_angle = vision1.RXData.VisionRxData.PitchAngleTarget;
        GIMBAL.Yaw_angle = vision1.RXData.VisionRxData.YawAngleTarget;
    }
	
    //鼠标控制逻辑

     else if(referee_remote_control.KEY_Ctrl == 1){
	    GIMBAL.Pitch_angle += (fp32)referee_remote_control.mouse_y / (Yfactor * 5.0f);
		GIMBAL.Yaw_angle -= (fp32)referee_remote_control.mouse_x / (Xfactor * 5.0f);
	}
	 else{
		GIMBAL.Pitch_angle += (fp32)referee_remote_control.mouse_y / Yfactor;
		GIMBAL.Yaw_angle -= (fp32)referee_remote_control.mouse_x / Xfactor;
	 }
		
    if(GIMBAL.Pitch_angle >= PITCH_MAX_ANGLE)GIMBAL.Pitch_angle = PITCH_MAX_ANGLE;
	if(GIMBAL.Pitch_angle <= PITCH_MIN_ANGLE)GIMBAL.Pitch_angle = PITCH_MIN_ANGLE;
    //底盘模式切换逻辑

    if(referee_remote_control.KEY_Z == 1)MasterLink.UP_Board_SetData.Mode_data = Top_Mode;
    if(referee_remote_control.KEY_X == 1)MasterLink.UP_Board_SetData.Mode_data = Follow_Mode;
    if(referee_remote_control.KEY_V == 1)MasterLink.UP_Board_SetData.Mode_data = Deploy_Mode;
	/* 原先这里 Error_code&0x02 会 vTaskSuspend(PIDTaskHandle), 而全工程没有 vTaskResume ——
	   卡弹一次就永久死机, 必须断电重上。已去掉: 卡弹只置错误位(module_task 里亮红灯提示),
	   清电流交给 PID_Task, 免得两个任务抢同一条总线。 */

    /* 发弹仅由 RC_Task 的 CH5/CH7 入口处理。 */
}
