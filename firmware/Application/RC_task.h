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
 *  @description ：遥控输入接口与 RC_Data 数据结构。
 */
#ifndef RC_TASK_H
#define RC_TASK_H
#include "app_et07.h"
#ifndef BOOSTER_RC_ONLY
#define BOOSTER_RC_ONLY 1
#endif
typedef struct { ET07_Data_t ET07_DATA; } RC_Data_t;
extern RC_Data_t RC_Data;
#define RC_FRI_RPM_MID      ( 5560)
#define RC_FRI_RPM_MAX      (10000)
#define RC_FRI_DEAD_NEG     (  -80)     /* 小于等于它就停 */
#define RC_CH8_RAW_MIN      (  353)     /* RF209S 实测端点 */
#define RC_CH8_RAW_MID      ( 1024)
#define RC_CH8_RAW_MAX      ( 1696)
uint8_t RC_LinkFresh(void);
uint8_t RC_MasterEnabled(void);
int16_t RC_FrictionWheelRpm(void);
#endif
