"""Link real motor drivers, DM axis and pure PID against a fake CAN peripheral."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CAN = r'''
#pragma once
#include <stdint.h>
typedef struct {int unused;} CAN_HandleTypeDef;
typedef struct {uint32_t IDE,RTR,DLC,StdId;} CAN_TxHeaderTypeDef;
extern CAN_HandleTypeDef hcan1;
enum {HAL_OK=0,HAL_ERROR=1,CAN_ID_STD=0,CAN_RTR_DATA=0};
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *,CAN_TxHeaderTypeDef *,uint8_t *,uint32_t *);
'''
TEST = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "app_RM_Motor.h"
#include "app_DM4310.h"
#include "dial_port.h"
CAN_HandleTypeDef hcan1;
static uint32_t now=1000;
static int fail,sent;
static CAN_TxHeaderTypeDef header;
static uint8_t wire[8];
uint32_t HAL_GetTick(void){return now;}
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *bus,CAN_TxHeaderTypeDef *h,uint8_t *data,uint32_t *mailbox){
 assert(bus==&hcan1);assert(h->IDE==CAN_ID_STD && h->RTR==CAN_RTR_DATA && h->DLC==8);
 sent++;header=*h;memcpy(wire,data,8);
 if(fail)return HAL_ERROR;
 *mailbox=7;return HAL_OK;
}
static void feedback(uint16_t position,uint8_t error){
 uint8_t data[8]={(uint8_t)((error<<4)|2),(uint8_t)(position>>8),(uint8_t)position,
                 0x80,0x08,0,30,31};
 Dial_Port_OnCanRx(data);Dial_Port_Update();
}
static unsigned torque_bits(void){return ((unsigned)(wire[6]&15)<<8)|wire[7];}
int main(void){
 uint32_t mailbox=0;
 Set_RM_MotorCurrent(&hcan1,0x200,-16384,100,-200,0,&mailbox);
 const uint8_t wheel[8]={0xc0,0,0,100,0xff,0x38,0,0};
 assert(header.StdId==0x200 && mailbox==7 && memcmp(wire,wheel,8)==0);
 fail=1;Set_RM_MotorCurrent(&hcan1,0x200,0,0,0,0,0);
 assert(RM_CanTxFailedRecently(20));now+=21;assert(!RM_CanTxFailedRecently(20));fail=0;

 Dial_Port_Init();feedback(32768,ERR_Enable);assert(Dial_Port_Get()->fb_ok);
 Dial_Port_SetTarget(Dial_Port_Get()->angle_deg+10,.003f);Dial_Port_Tx();
 assert(header.StdId==DIAL_DM_ESC_ID);
 if(DIAL_DM_CALIB_FREE)assert(torque_bits()==2047);
 else assert(torque_bits()>2047);
 Dial_Port_Stop();Dial_Port_Tx();assert(torque_bits()==2047);

 /* Failure protection is 15ms of actual time, even with only one failed send. */
 fail=1;Dial_Port_Tx();uint32_t since=now;
 now=since+14;feedback(32768,ERR_Enable);assert(Dial_Port_Get()->fb_ok);
 now=since+15;Dial_Port_Update();assert(!Dial_Port_Get()->fb_ok);
 fail=0;Dial_Port_Tx();Dial_Port_Update();assert(Dial_Port_Get()->fb_ok);
 feedback(32768,ERR_OverCurrent);assert(!Dial_Port_Get()->fb_ok);
 feedback(32768,ERR_Enable);now+=151;Dial_Port_Update();assert(!Dial_Port_Get()->fb_ok);

 /* Every feedback frame participates in continuous position unwrap. */
 Dial_Port_Init();feedback(65000,ERR_Enable);float before=Dial_Port_Get()->angle_deg;
 feedback(1000,ERR_Enable);float after=Dial_Port_Get()->angle_deg;
 assert(after>before && after-before<20);
 printf("PASS: real CAN encoding, wheel failure tracking, DM timed failure/unwrap, calibration=%d\n",DIAL_DM_CALIB_FREE);
}
'''

with tempfile.TemporaryDirectory(prefix='motor-transport-') as tmp:
    directory = Path(tmp)
    (directory / 'main.h').write_text(
        '#pragma once\n#include <stdint.h>\n#include <stddef.h>\nuint32_t HAL_GetTick(void);\n', encoding='utf-8')
    (directory / 'can.h').write_text(CAN, encoding='utf-8')
    (directory / 'bsp_tools.h').write_text(
        '#pragma once\n#include <math.h>\n#define absf fabsf\n', encoding='utf-8')
    (directory / 'test.c').write_text(TEST, encoding='utf-8')
    sources = ['Components/Devices/app_RM_Motor.c', 'Components/Devices/app_DM4310.c',
               'Components/Devices/dial_port_dm.c', 'Components/Controller/controller.c']
    for calibration in (0, 1):
        executable = directory / ('test.exe' if os.name == 'nt' else 'test')
        subprocess.run([
            'cc', '-std=c11', '-Wall', '-Wextra', '-Werror', f'-DDIAL_DM_CALIB_FREE={calibration}',
            '-I', str(directory), '-I', str(ROOT / 'Components/Devices'),
            '-I', str(ROOT / 'Components/Controller'), str(directory / 'test.c'),
            *(str(ROOT / source) for source in sources), '-lm', '-o', str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)
