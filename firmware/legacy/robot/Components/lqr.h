/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2024-01-05 17:19:33
 * @LastEditTime : 2024-03-05 17:11:41
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\balance2\Components\lqr.h
 * @Description  : 
 */

#ifndef __LQR_H
#define __LQR_H

#include "main.h"


#define WHEEL_R  0.0875f

/* 反馈矩阵拟合系数 */
#define K111 -61.2725f
#define K112 112.0577f
#define K113 -160.9419f
#define K114 -7.5692f

#define K121 29.8294f
#define K122 -38.4293f
#define K123 -15.8121f
#define K124 -1.4143f

#define K131 -20.6906f
#define K132 35.9016f
#define K133 -21.2713f
#define K134 -17.5983f

#define K141 42.7488f
#define K142 -25.8112f
#define K143 -9.1830f
#define K144 -15.1003f

#define K151 -21.5613f
#define K152 166.8896f
#define K153 -174.6207f
#define K154 74.5923f

#define K161 -25.9411f
#define K162 33.2661f
#define K163 -18.0318f
#define K164 7.1837f


#define K211 392.2652f
#define K212 -406.7644f
#define K213 136.1937f
#define K214 10.1404f

#define K221 28.5589f
#define K222 -37.0884f
#define K223 18.6474f
#define K224 1.6888f

#define K231 47.5548f
#define K232 17.8648f
#define K233 -56.9581f
#define K234 27.7686f

#define K241 -1.7344f
#define K242 39.7094f
#define K243 -46.2429f
#define K244 21.7051f

#define K251 350.5875f
#define K252 -498.9626f
#define K253 262.6577f
#define K254 147.1613f

#define K261 30.4718f
#define K262 -43.9236f
#define K263 24.3867f
#define K264 2.6871f

/**
 * @description  : 状态向量结构体
 * @return        {*}
 */
typedef struct
{
  float theta_last; //保留上一次角度
  float phi_last;
  float x_last;
  
  float theta;      //rad
  float theta_dot;
  float x;   //m
  float x_dot;
  float phi;        //rad
  float phi_dot;
}LQR_State_t;

/**
 * @description  : 控制向量结构体
 * @return        {*}
 */
typedef struct
{
  float T_Wheel;
  float Tp_Joint;
}LQR_Ctrl_t;

typedef struct
{
  float K_Feedback[2][6];
  LQR_Ctrl_t LQR_Ctrl;
  LQR_State_t LQR_State;
}LQR_Data_t;

void SetLQR_Ctrl(void);

#endif
