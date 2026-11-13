#!/usr/bin/env python3
"""Compile production synchronization and wheel control; check bounds and stop paths."""
from pathlib import Path
import subprocess
import tempfile
import re
root=Path(__file__).resolve().parents[1]
source=(root/'modules/shooter/friction_control.c').read_text()
start=source.index('static void FrictionPid_Reset(')
end=len(source)
control=source[start:end]
macros='\n'.join(line for line in source.splitlines() if re.match(r'#define FRI_(FB_TIMEOUT_MS|REF_SLEW_UP|REF_SLEW_DN|DIV_MARGIN_RPM|DIV_MS)\b',line))
code=r'''
#include <stdint.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "friction_sync.h"
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define absf fabsf
#define FRI_WHEEL_NUM 3
#define FRI_DIAG_CHANNELS 24
static const int8_t Fri_DirSign[3]={-1,1,-1};
#define FRI_SPD_LIMIT 10000
static struct {uint32_t CYCCNT;} dwt;
#define DWT (&dwt)
typedef struct {
 float Ref,Measure,Last_Measure,Err,Last_Err,Pout,ITerm,Last_ITerm,Iout,Dout,Last_Dout,Output,Last_Output,MaxOut;
 uint32_t DWT_CNT;
} PID_t;
typedef struct {struct {int16_t speed;uint32_t rx_tick;} RM_MotorMeasure;PID_t MotorCtrl_Speed;} RM_Motor_t;
static struct {RM_Motor_t RM_Motor[3];} BOOSTER;
static const uint8_t Fri_MotorID[3]={1,2,3};
static uint32_t now=1000;
uint32_t HAL_GetTick(void){return now;}
static int16_t targets[3]={-5750,5750,-5750};
int16_t Booster_GetWheelSetpoint(uint8_t i){return targets[i];}
static float forced_output;
static float control_dt=.003f;
float Shooter_ControlDt(void){return control_dt;}
void PID_CalculateDt(PID_t *pid,float meas,float ref,float dt_s){assert(dt_s>0);pid->Ref=ref;pid->Output=forced_output ? forced_output : ref-meas;}
'''+macros+'\n'+control+r'''
int main(void){
 friction_sample_t s[3]={{-5750,-5750,999},{5750,5750,999},{-5750,-5750,999}};
 float trim[3];assert(FrictionSync_Compute(s,1000,trim)==32U);
 for(int i=0;i<3;i++)assert(trim[i]==0);
 s[0].speed=-5500;s[1].speed=5750;s[2].speed=-6000;
 FrictionSync_Compute(s,1000,trim);
 assert(fabsf(trim[0]-75)<.01 && fabsf(trim[1])<.01 && fabsf(trim[2]+75)<.01);
 s[0].speed=-5400;s[1].speed=5300;s[2].speed=-5500;
 assert(FrictionSync_Compute(s,1000,trim)==0U);
 assert(fabsf(trim[0])<.01 && fabsf(trim[1]-70)<.01 && fabsf(trim[2]+70)<.01);
 s[1].speed=5500;s[2].speed=-5300;
 assert(FrictionSync_Compute(s,1000,trim)==0U);
 assert(fabsf(trim[0])<.01 && fabsf(trim[1]+70)<.01 && fabsf(trim[2]-70)<.01);
 s[1].speed=5750;s[2].speed=-6000;
 s[0].speed=-3000;FrictionSync_Compute(s,1000,trim);
 assert(trim[0]>0 && trim[2]<0);
 assert(fabsf(trim[0]+trim[1]+trim[2])<.001);
 for(int i=0;i<3;i++)assert(fabsf(trim[i])<=100.001);
 s[1].rx_tick=990;assert(FrictionSync_Compute(s,1000,trim)==4U);
 for(int i=0;i<3;i++)assert(trim[i]==0);
 s[1].rx_tick=997;assert(FrictionSync_Compute(s,1000,trim)==8U); /* fresh but excessive skew */
 for(int i=0;i<3;i++)assert(trim[i]==0);
 s[1].rx_tick=999;s[1].target=5700;FrictionSync_Compute(s,1000,trim);
 for(int i=0;i<3;i++)assert(trim[i]==0);
 s[1].target=5750;s[0].speed=3000;FrictionSync_Compute(s,1000,trim);
 for(int i=0;i<3;i++)assert(trim[i]==0); /* reversed feedback */
 for(int i=0;i<3;i++){
  BOOSTER.RM_Motor[i].RM_MotorMeasure.rx_tick=999;
  BOOSTER.RM_Motor[i].MotorCtrl_Speed.MaxOut=16384;
 }
 BOOSTER.RM_Motor[0].RM_MotorMeasure.speed=-5500;
 BOOSTER.RM_Motor[1].RM_MotorMeasure.speed=5750;
 BOOSTER.RM_Motor[2].RM_MotorMeasure.speed=-6000;
 Booster_FrictionGroupCtrl(); /* ramping: no sync step */
 assert(BOOSTER.RM_Motor[0].MotorCtrl_Speed.Ref==-75);
 assert(BOOSTER.RM_Motor[1].MotorCtrl_Speed.Ref==75);
 assert(BOOSTER.RM_Motor[2].MotorCtrl_Speed.Ref==-75);
 for(int k=0;k<85;k++)Booster_FrictionGroupCtrl();
 assert(BOOSTER.RM_Motor[0].MotorCtrl_Speed.Ref==-5825);
 assert(BOOSTER.RM_Motor[1].MotorCtrl_Speed.Ref==5750);
 assert(BOOSTER.RM_Motor[2].MotorCtrl_Speed.Ref==-5675);
 targets[0]=-5000;targets[1]=5000;targets[2]=-5000;
 Booster_FrictionGroupCtrl();
 assert(BOOSTER.RM_Motor[0].MotorCtrl_Speed.Ref==-5600);
 assert(BOOSTER.RM_Motor[1].MotorCtrl_Speed.Ref==5600);
 assert(BOOSTER.RM_Motor[2].MotorCtrl_Speed.Ref==-5600);
 targets[0]=-5750;targets[1]=5750;targets[2]=-5750;
 for(int k=0;k<85;k++)Booster_FrictionGroupCtrl();
 BOOSTER.RM_Motor[1].RM_MotorMeasure.rx_tick=800;
 Booster_FrictionGroupCtrl();
 assert(BOOSTER.RM_Motor[1].MotorCtrl_Speed.Output==0);
 assert(BOOSTER.RM_Motor[0].MotorCtrl_Speed.Ref==-5750); /* stale peer removes coupling */
 for(int i=0;i<3;i++)targets[i]=0;
 Booster_FrictionGroupCtrl();
 for(int i=0;i<3;i++)assert(BOOSTER.RM_Motor[i].MotorCtrl_Speed.Output==0);
 float diag[24];
 Booster_TakeFrictionDiagnostics(diag); /* reset all earlier test windows */
 for(int i=0;i<3;i++){
   targets[i]=i==1 ? 5750 : -5750;
   BOOSTER.RM_Motor[i].RM_MotorMeasure.rx_tick=999;
 }
 forced_output=16384;
 Booster_FrictionGroupCtrl();Booster_RecordPIDTiming(3400,100);
 Booster_FrictionGroupCtrl();Booster_RecordPIDTiming(3200,80);
 Booster_TakeFrictionDiagnostics(diag);
 assert(diag[0]==1 && diag[1]==1000 && diag[2]==3200 && diag[3]==3400);
 assert(diag[4]==80 && diag[5]==100 && diag[22]==2 && diag[23]==2);
 assert(diag[17]==7 && diag[18]==2 && diag[19]==2 && diag[20]==2);
 assert(diag[6]==1 && diag[9]==0 && ((int)diag[10]&64));
 for(int i=0;i<3;i++)assert(diag[14+i]==150); /* normalized actual ramp reference */
 Booster_TakeFrictionDiagnostics(diag);
 assert(diag[3]==0 && diag[5]==0 && diag[18]==0 && diag[21]==0 && diag[22]==0);
 assert(diag[2]==3200 && diag[23]==2); /* latest observation retained */
 puts("PASS: signed feedback, common-mode unchanged, trim limits, fresh/aligned samples, ramp, stale feedback and zero-target stop");
}
'''
with tempfile.TemporaryDirectory() as tmp:
 p=Path(tmp);(p/'test.c').write_text(code)
 subprocess.run(['cc','-std=c11','-I',str(root/'Components/Devices'),str(p/'test.c'),'-lm','-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
