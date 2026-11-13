/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2024-01-05 17:19:33
 * @LastEditTime : 2024-04-19 00:06:11
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Components\lqr.c
 * @Description  : 
 */

#include "lqr.h"
#include "ins_task.h"
#include "app_9025.h"
#include "can.h"
#include "chassis_task.h"
#include "user_lib.h"

/**
 * @description  : 更新反馈K矩阵
 * @param         {Leg_Ctrl_t*} leg_ctrl_t
 * @param         {float} len
 * @return        {*}
 */
void SetLQR_K_Feedback(Leg_Ctrl_t* leg_ctrl_t,float len)
{
  leg_ctrl_t ->LQR_Data.K_Feedback[0][0] = K111 *len*len*len + K112 *len*len + K113 *len + K114;
  leg_ctrl_t ->LQR_Data.K_Feedback[0][1] = K121 *len*len*len + K122 *len*len + K123 *len + K124;
  leg_ctrl_t ->LQR_Data.K_Feedback[0][2] = K131 *len*len*len + K132 *len*len + K133 *len + K134;
  leg_ctrl_t ->LQR_Data.K_Feedback[0][3] = K141 *len*len*len + K142 *len*len + K143 *len + K144;
  leg_ctrl_t ->LQR_Data.K_Feedback[0][4] = K151 *len*len*len + K152 *len*len + K153 *len + K154;
  leg_ctrl_t ->LQR_Data.K_Feedback[0][5] = K161 *len*len*len + K162 *len*len + K163 *len + K164;

  leg_ctrl_t ->LQR_Data.K_Feedback[1][0] = K211 *len*len*len + K212 *len*len + K213 *len + K214;
  leg_ctrl_t ->LQR_Data.K_Feedback[1][1] = K221 *len*len*len + K222 *len*len + K223 *len + K224;
  leg_ctrl_t ->LQR_Data.K_Feedback[1][2] = K231 *len*len*len + K232 *len*len + K233 *len + K234;
  leg_ctrl_t ->LQR_Data.K_Feedback[1][3] = K241 *len*len*len + K242 *len*len + K243 *len + K244;
  leg_ctrl_t ->LQR_Data.K_Feedback[1][4] = K251 *len*len*len + K252 *len*len + K253 *len + K254;
  leg_ctrl_t ->LQR_Data.K_Feedback[1][5] = K261 *len*len*len + K262 *len*len + K263 *len + K264;
}

/**
 * @description  : 更新LQR状态向量
 * @param         {Leg_Ctrl_t*} leg_ctrl_t
 * @param         {MF9025_MotorMeasure_t*} mf9025_motor_t
 * @return        {*}
 */
void SetLQR_State(void)
{
  /********** 左腿状态反馈 **********/
  CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.theta_last = CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.theta;
  CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.phi_last = CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.phi;
 // CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.x_last = CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.x;

  CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.phi       = (-INS.Pitch - 0.0f) * 3.141593f / 180.0f;
  CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.phi_dot   = -INS.Gyro[X] * 3.141593f / 180.0f;
  // CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.phi_dot   = (CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.phi - CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.phi_last) / 0.002f;
  CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.theta     = (3.141593f / 2.0f - CHASSIS.Leg_Data.LegCtrl[0].Leg_VMC.phi0 - CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.phi);
  CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.theta_dot = 
              (CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.theta - CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.theta_last) / (0.004f);
  //CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.theta_dot = - CHASSIS.Leg_Data.LegCtrl[0].Leg_VMC.phi0_dot - CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.phi_dot;
  CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.x_dot     = CHASSIS.MF9025_Motor[MF9025_ID_Left-1].MF9025_MotorMeasure.speed_rad * WHEEL_R;
  CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.x         +=CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_State.x_dot* (0.004f);
  
  /********** 右腿状态反馈 **********/
	CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.theta_last = CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.theta;
  CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.phi_last = CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.phi;
  
  CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.phi       = (-INS.Pitch - 0.0f) * 3.141593f / 180.0f;
  CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.phi_dot   = -INS.Gyro[X] * 3.141593f / 180.0f;
  // CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.phi_dot   = (CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.phi - CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.phi_last) / 0.002f;
  CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.theta     = (3.141593f / 2.0f - CHASSIS.Leg_Data.LegCtrl[1].Leg_VMC.phi0 - CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.phi);
  CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.theta_dot = 
              (CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.theta - CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.theta_last) / (0.004f);
  //CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.theta_dot = - CHASSIS.Leg_Data.LegCtrl[1].Leg_VMC.phi0_dot - CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.phi_dot;
  
  CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.x_dot     = 0.0f-CHASSIS.MF9025_Motor[MF9025_ID_Right-1].MF9025_MotorMeasure.speed_rad * WHEEL_R;
	CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.x         += CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_State.x_dot*(0.004f);

}

/**
 * @description  : LQR计算
 * @param         {Leg_Ctrl_t*} leg_ctrl_t
 * @return        {*}
 */


//float LQR_K11 =-12.8819f;
//float LQR_K12 = -0.9713f;
//float LQR_K13 =-1.4020f;
//float LQR_K14 =-2.0458f;
//float LQR_K15 =13.4662f;
//float LQR_K16 =1.3472f;
//float LQR_K21 =  6.3617f;
//float LQR_K22 =0.3876f;
//float LQR_K23 =1.0625f;
//float LQR_K24 =1.4743f;
//float LQR_K25 =139.1380f;
//float LQR_K26 =4.6446f;



//  -13.9721   -1.6883   -4.3046   -3.9547    8.5249    1.5402
//   10.5431    1.3254    5.0876    4.2013   20.5893    2.0057


float LQR_K11 =-13.9721f;
float LQR_K12 = -1.6883f ;
float LQR_K13 = -4.3046f;
float LQR_K14 =  -3.9547f ;
float LQR_K15 =   8.5249f  ;
float LQR_K16 =     1.5402f;
float LQR_K21 =10.5431f  ;
float LQR_K22 =1.3254f ;
float LQR_K23 = 5.0876f;
float LQR_K24 =     4.2013f;
float LQR_K25 =    20.5893f;
float LQR_K26 =     2.0057f;

void LQR_Ctrl(Leg_Ctrl_t* leg_ctrl_t)
{
  leg_ctrl_t ->LQR_Data.LQR_Ctrl.T_Wheel  = (LQR_K11 * leg_ctrl_t ->LQR_Data.LQR_State.theta 
                                            + LQR_K12 * leg_ctrl_t ->LQR_Data.LQR_State.theta_dot
                                            + LQR_K13* (leg_ctrl_t ->LQR_Data.LQR_State.x)
                                            + LQR_K14 * (leg_ctrl_t ->LQR_Data.LQR_State.x_dot - CHASSIS.chassis_distance)
                                            + LQR_K15 * leg_ctrl_t ->LQR_Data.LQR_State.phi
                                            + LQR_K16 * leg_ctrl_t ->LQR_Data.LQR_State.phi_dot);
  
  leg_ctrl_t ->LQR_Data.LQR_Ctrl.Tp_Joint = (LQR_K21 * leg_ctrl_t ->LQR_Data.LQR_State.theta 
                                            + LQR_K22* leg_ctrl_t ->LQR_Data.LQR_State.theta_dot
                                            + LQR_K23 * (leg_ctrl_t ->LQR_Data.LQR_State.x)
                                            + LQR_K24 * (leg_ctrl_t ->LQR_Data.LQR_State.x_dot - CHASSIS.chassis_distance)
                                            + LQR_K25 * leg_ctrl_t ->LQR_Data.LQR_State.phi
                                            + LQR_K26 * leg_ctrl_t ->LQR_Data.LQR_State.phi_dot);
  leg_ctrl_t ->K_Debug[0] = leg_ctrl_t ->LQR_Data.K_Feedback[0][0] * leg_ctrl_t ->LQR_Data.LQR_State.theta;
  leg_ctrl_t ->K_Debug[1] = leg_ctrl_t ->LQR_Data.K_Feedback[0][1] * leg_ctrl_t ->LQR_Data.LQR_State.theta_dot;
  leg_ctrl_t ->K_Debug[2] = leg_ctrl_t ->LQR_Data.K_Feedback[0][2] * leg_ctrl_t ->LQR_Data.LQR_State.x;
  leg_ctrl_t ->K_Debug[3] = leg_ctrl_t ->LQR_Data.K_Feedback[0][3] * leg_ctrl_t ->LQR_Data.LQR_State.x_dot;
  leg_ctrl_t ->K_Debug[4] = leg_ctrl_t ->LQR_Data.K_Feedback[0][4] * leg_ctrl_t ->LQR_Data.LQR_State.phi;
  leg_ctrl_t ->K_Debug[5] = leg_ctrl_t ->LQR_Data.K_Feedback[0][5] * leg_ctrl_t ->LQR_Data.LQR_State.phi_dot;

  leg_ctrl_t ->K_Debug[6] = leg_ctrl_t ->LQR_Data.K_Feedback[1][0] * leg_ctrl_t ->LQR_Data.LQR_State.theta;
  leg_ctrl_t ->K_Debug[7] = leg_ctrl_t ->LQR_Data.K_Feedback[1][1] * leg_ctrl_t ->LQR_Data.LQR_State.theta_dot;
  leg_ctrl_t ->K_Debug[8] = leg_ctrl_t ->LQR_Data.K_Feedback[1][2] * leg_ctrl_t ->LQR_Data.LQR_State.x;
  leg_ctrl_t ->K_Debug[9] = leg_ctrl_t ->LQR_Data.K_Feedback[1][3] * leg_ctrl_t ->LQR_Data.LQR_State.x_dot;
  leg_ctrl_t ->K_Debug[10] = leg_ctrl_t ->LQR_Data.K_Feedback[1][4] * leg_ctrl_t ->LQR_Data.LQR_State.phi;
  leg_ctrl_t ->K_Debug[11] = leg_ctrl_t ->LQR_Data.K_Feedback[1][5] * leg_ctrl_t ->LQR_Data.LQR_State.phi_dot;
}

//int iq_tor;
/**
 * @description  : LQR计算结果赋值
 * @return        {*}
 */
int ww2=0;
void SetLQR_Ctrl(void)
{
  int iq[2] = {0};
  iq[0] = (int)(62.5f * (CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_Ctrl.T_Wheel / MF9025_TorqueConstant));
  iq[1] = (int)(62.5f * (CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_Ctrl.T_Wheel / MF9025_TorqueConstant));
  iq[0] = Saturate_Int(iq[0],-2000,2000);
  iq[1] = Saturate_Int(iq[1],-2000,2000);
  //iq_tor = (iq[0] + iq[1])/2;
  //Set_MF9025_MotorCurrent(&hcan1,-iq[0],-iq[1],0,0)
  iq[0]=0.0f-iq[0];
  iq[0] = iq[0] + CHASSIS.TurnCtrl.TurnPD.Output;
  iq[1] = iq[1] + CHASSIS.TurnCtrl.TurnPD.Output;
  // if(ww2==1)
  // {
  //   Set_MF9025_MotorCurrent(&hcan1,iq[0],iq[1],0,0);
  // }
  // else
  // {
  //   Set_MF9025_MotorCurrent(&hcan1,0,0,0,0);
  // }
  Set_MF9025_MotorCurrent(&hcan1,iq[0],iq[1],0,0);
  //Set_MF9025_MotorCurrent(&hcan1,0,0,0,0);
  CHASSIS.Leg_Data.LegCtrl[0].TP0 = CHASSIS.Leg_Data.LegCtrl[0].LQR_Data.LQR_Ctrl.Tp_Joint-CHASSIS.TunningCtrl.TunningPD.Output;
  CHASSIS.Leg_Data.LegCtrl[1].TP0 = CHASSIS.Leg_Data.LegCtrl[1].LQR_Data.LQR_Ctrl.Tp_Joint+CHASSIS.TunningCtrl.TunningPD.Output;
}
