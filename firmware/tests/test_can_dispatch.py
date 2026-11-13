"""Compile the production CAN BSP against a fake HAL; test IRQ routing gates."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CAN_STUB = r'''
#pragma once
#include <stdint.h>
typedef struct {void *Instance;} CAN_HandleTypeDef;
typedef struct {uint32_t StdId,IDE,RTR,DLC;} CAN_RxHeaderTypeDef;
typedef struct {
 uint32_t FilterActivation,FilterBank,SlaveStartFilterBank,FilterFIFOAssignment;
 uint32_t FilterIdHigh,FilterIdLow,FilterMaskIdHigh,FilterMaskIdLow,FilterMode,FilterScale;
} CAN_FilterTypeDef;
extern CAN_HandleTypeDef hcan1,hcan2;
extern int can1_instance,can2_instance;
#define CAN1 (&can1_instance)
#define CAN2 (&can2_instance)
enum {HAL_OK=0,HAL_ERROR=1,CAN_FILTER_ENABLE=1,CAN_RX_FIFO0=0,CAN_RX_FIFO1=1,
      CAN_FILTERMODE_IDMASK=0,CAN_FILTERSCALE_32BIT=1,
      CAN_IT_RX_FIFO0_MSG_PENDING=1,CAN_IT_RX_FIFO1_MSG_PENDING=2,
      CAN_ID_STD=0,CAN_ID_EXT=4,CAN_RTR_DATA=0,CAN_RTR_REMOTE=2};
int HAL_CAN_ConfigFilter(CAN_HandleTypeDef *,CAN_FilterTypeDef *);
int HAL_CAN_Start(CAN_HandleTypeDef *);
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef *,uint32_t);
int HAL_CAN_GetRxMessage(CAN_HandleTypeDef *,uint32_t,CAN_RxHeaderTypeDef *,uint8_t *);
'''
HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bsp_can.h"
#include "can.h"
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *);
int can1_instance,can2_instance;
CAN_HandleTypeDef hcan1={CAN1},hcan2={CAN2};
static CAN_RxHeaderTypeDef incoming;
static uint8_t payload[8]={1,2,3,4,5,6,7,8};
static int fail,reads,received1,received2,filters,starts,notifications;
static uint16_t last_id;
static uint8_t last_length,last_data[8];
int HAL_CAN_ConfigFilter(CAN_HandleTypeDef *bus,CAN_FilterTypeDef *filter){
 assert(filter->FilterActivation==CAN_FILTER_ENABLE);
 assert(filter->FilterBank==(bus==&hcan1 ? CAN1_FILTER_BANK : CAN2_FILTER_BANK));
 assert(filter->FilterFIFOAssignment==(bus==&hcan1 ? CAN_RX_FIFO0 : CAN_RX_FIFO1));
 filters++;return HAL_OK;
}
int HAL_CAN_Start(CAN_HandleTypeDef *bus){assert(bus==&hcan1||bus==&hcan2);starts++;return HAL_OK;}
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef *bus,uint32_t bits){
 assert(bits==(bus==&hcan1 ? CAN_IT_RX_FIFO0_MSG_PENDING : CAN_IT_RX_FIFO1_MSG_PENDING));
 notifications++;return HAL_OK;
}
int HAL_CAN_GetRxMessage(CAN_HandleTypeDef *bus,uint32_t fifo,CAN_RxHeaderTypeDef *header,uint8_t *data){
 assert(fifo==(bus==&hcan1 ? CAN_RX_FIFO0 : CAN_RX_FIFO1));reads++;
 if(fail)return HAL_ERROR;
 *header=incoming;memcpy(data,payload,8);return HAL_OK;
}
static void save(uint16_t id,const uint8_t *data,uint8_t length){
 last_id=id;last_length=length;memcpy(last_data,data,length);
}
static void sink1(uint16_t id,const uint8_t *data,uint8_t length){received1++;save(id,data,length);}
static void sink2(uint16_t id,const uint8_t *data,uint8_t length){received2++;save(id,data,length);}
int main(void){
 incoming=(CAN_RxHeaderTypeDef){0x201,CAN_ID_STD,CAN_RTR_DATA,8};
 /* An unbound bus safely consumes frames without dispatching them. */
 HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);assert(reads==1 && !received1);
 CAN_SetRxHandlers(sink1,sink2);CAN_Filter_Init();
 assert(filters==1 && starts==1 && notifications==1);
 HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
 assert(received1==1 && received2==0 && last_id==0x201 && last_length==8);
 assert(memcmp(last_data,payload,8)==0);
 incoming.StdId=0x302;HAL_CAN_RxFifo1MsgPendingCallback(&hcan2);
 assert(received1==1 && received2==1 && last_id==0x302);
 incoming.DLC=3;HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
 assert(received1==2 && last_length==3); /* device receiver decides its exact length */
 incoming.DLC=9;HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);assert(received1==2);
 incoming.DLC=8;incoming.IDE=CAN_ID_EXT;HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
 assert(received1==2);
 incoming.IDE=CAN_ID_STD;incoming.RTR=CAN_RTR_REMOTE;
 HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);assert(received1==2);
 incoming.RTR=CAN_RTR_DATA;fail=1;
 HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);assert(received1==2);fail=0;
 int before=reads;
 HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);HAL_CAN_RxFifo1MsgPendingCallback(&hcan1);
 assert(reads==before); /* no callback for the wrong bus/FIFO pairing */
 HAL_CAN_ErrorCallback(&hcan1);HAL_CAN_ErrorCallback(&hcan2);assert(notifications==3);
 puts("PASS: real CAN BSP, independent bus routes, filter setup, HAL failures and frame-type/length gates");
}
'''

with tempfile.TemporaryDirectory(prefix='can-dispatch-') as tmp:
    directory = Path(tmp)
    (directory / 'main.h').write_text('#pragma once\n#include <stdint.h>\n', encoding='utf-8')
    (directory / 'can.h').write_text(CAN_STUB, encoding='utf-8')
    (directory / 'test.c').write_text(HARNESS, encoding='utf-8')
    executable = directory / ('test.exe' if os.name == 'nt' else 'test')
    subprocess.run([
        'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
        '-I', str(directory), '-I', str(ROOT / 'Bsp'),
        str(directory / 'test.c'), str(ROOT / 'Bsp/bsp_can.c'), '-o', str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
