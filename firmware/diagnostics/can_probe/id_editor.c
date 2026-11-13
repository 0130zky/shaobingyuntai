/* Explicit SWD mailbox; NEVER starts a parameter write on boot.
 * Only authorized change: ESC_ID 1 -> 2. Feedback ID is preserved; select the physical motor at build time.
 * No enable/control/zero/other register write path. */
#include "main.h"
#include "fdcan.h"
#include <string.h>
#ifndef ID_EDITOR_FEEDBACK_ID
#define ID_EDITOR_FEEDBACK_ID 0x11U
#endif
volatile struct {
 uint32_t magic,command,seq,last_command,last_result,rx,tx,old_seen,new_seen,unexpected;
 uint32_t psr,ecr,frame_id,frame_lo,frame_hi;
 uint32_t log[32][6];
} id_editor;
static int Send(uint32_t id,const uint8_t data[8],uint32_t dlc) {
 FDCAN_TxHeaderTypeDef h={0};h.Identifier=id;h.IdType=FDCAN_STANDARD_ID;
 h.TxFrameType=FDCAN_DATA_FRAME;h.DataLength=dlc;h.FDFormat=FDCAN_CLASSIC_CAN;
 if(!HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1))return 0;
 if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&h,(uint8_t *)data)!=HAL_OK)return 0;
 id_editor.tx++;return 1;
}
void Probe_Init(void) {
 hfdcan1.Init.AutoRetransmission=DISABLE;
 HAL_FDCAN_DeInit(&hfdcan1);
 if(HAL_FDCAN_Init(&hfdcan1)!=HAL_OK)Error_Handler();
 FDCAN_FilterTypeDef f={0};f.IdType=FDCAN_STANDARD_ID;f.FilterType=FDCAN_FILTER_RANGE;
 f.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;f.FilterID2=0x7ff;
 if(HAL_FDCAN_ConfigFilter(&hfdcan1,&f)!=HAL_OK ||
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE)!=HAL_OK ||
    HAL_FDCAN_Start(&hfdcan1)!=HAL_OK)Error_Handler();
 id_editor.magic=0x49444544;
}
void Probe_Step(void) {
 for(unsigned n=0;n<16 && HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1,FDCAN_RX_FIFO0);n++) {
  FDCAN_RxHeaderTypeDef h;uint8_t d[64]={0};
  if(HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&h,d)!=HAL_OK)break;
  if(h.IdType!=FDCAN_STANDARD_ID||h.RxFrameType!=FDCAN_DATA_FRAME||(h.DataLength!=FDCAN_DLC_BYTES_8 && h.DataLength!=FDCAN_DLC_BYTES_4))continue;
  uint32_t i=id_editor.rx++%32;
  id_editor.frame_id=h.Identifier;
  memcpy((void*)&id_editor.frame_lo,d,4);memcpy((void*)&id_editor.frame_hi,d+4,4);
  id_editor.log[i][0]=id_editor.seq;id_editor.log[i][1]=id_editor.last_command;
  id_editor.log[i][2]=HAL_GetTick();id_editor.log[i][3]=h.Identifier;
  id_editor.log[i][4]=id_editor.frame_lo;id_editor.log[i][5]=id_editor.frame_hi;
  if(h.Identifier!=ID_EDITOR_FEEDBACK_ID)id_editor.unexpected++;
  if(h.Identifier==ID_EDITOR_FEEDBACK_ID && (id_editor.last_command==1 || id_editor.last_command==4)) {
   if((d[0]&15)==1)id_editor.old_seen++;
   if((d[0]&15)==2)id_editor.new_seen++;
  }
 }
 id_editor.psr=hfdcan1.Instance->PSR;id_editor.ecr=hfdcan1.Instance->ECR;
 uint32_t command=id_editor.command;if(!command)return;id_editor.command=0;
 id_editor.seq++;id_editor.last_command=command;id_editor.last_result=0;
 uint8_t q[8]={1,0,0x33,8,0,0,0,0};uint32_t id=0x7ff;
 switch(command) {
  case 1: id=1;memset(q,0xff,8);q[7]=0xfd;break;
  case 2: break;
  case 3:
   if(!id_editor.old_seen||id_editor.unexpected){id_editor.last_result=2;return;}
   q[2]=0x55;q[4]=2;break;
  case 4: id=2;memset(q,0xff,8);q[7]=0xfd;break;
  case 5: q[0]=2;break;
  case 6:
   if(!id_editor.new_seen||id_editor.unexpected){id_editor.last_result=2;return;}
   q[0]=2;q[2]=0xaa;q[3]=1;break;
  default:id_editor.last_result=3;return;
 }
 if(!Send(id,q,command==6?FDCAN_DLC_BYTES_4:FDCAN_DLC_BYTES_8))id_editor.last_result=1;
}
