/* RAM-only discovery: the sole transmitted frame is a 0x7FF / 0x33
 * register/status READ by default. Optional disable-only feedback scan requires
 * explicit operator authorization. Never sends enable, control, zero or writes. */
#include "main.h"
#include "fdcan.h"
#include <string.h>
#define FOUND_MAX 16
#ifndef PROBE_DISABLE_READ
#define PROBE_DISABLE_READ 0
#endif
static const uint8_t registers[]={7,8,9,10,21,22,23,35};
typedef struct {uint32_t esc,mst,seen,mask,value[8],feedback[2],source;} Found;
volatile struct {
 uint32_t magic,state,rate,candidate,pass,rx,tx,fail,psr,ecr,count,done,last_id;
 uint8_t last_data[8];Found motors[FOUND_MAX];
 uint32_t rates[6][4];
} can_probe;
volatile uint32_t probe_stop;
static unsigned rate_index,next_id,param_index,motor_index,query_status,previous_status,previous_id;
static uint32_t previous;
static const uint32_t prescalers[]={1,2,4,8,10,5};
void Error_Handler(void);
static void Setup(void)
{
 HAL_FDCAN_Stop(&hfdcan1);HAL_FDCAN_DeInit(&hfdcan1);
 hfdcan1.Init.NominalPrescaler=prescalers[rate_index];
 hfdcan1.Init.AutoRetransmission=DISABLE;
 if(HAL_FDCAN_Init(&hfdcan1)!=HAL_OK)Error_Handler();
 FDCAN_FilterTypeDef f={0};f.IdType=FDCAN_STANDARD_ID;
 f.FilterType=FDCAN_FILTER_RANGE;f.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;f.FilterID2=0x7ff;
 if(HAL_FDCAN_ConfigFilter(&hfdcan1,&f)!=HAL_OK ||
 HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE)!=HAL_OK ||
 HAL_FDCAN_Start(&hfdcan1)!=HAL_OK)Error_Handler();
 can_probe.rate=1000000U/prescalers[rate_index];
}
void Probe_Init(void)
{
 can_probe.magic=0x50524f42;can_probe.state=1;Setup();previous=HAL_GetTick();
}
void Probe_Step(void)
{
 for(unsigned n=0;n<16U && HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1,FDCAN_RX_FIFO0);n++) {
  FDCAN_RxHeaderTypeDef h;uint8_t b[64];
  if(HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&h,b)!=HAL_OK)break;
  can_probe.rx++;can_probe.last_id=h.Identifier;memcpy((void*)can_probe.last_data,b,8);
  if(h.IdType!=FDCAN_STANDARD_ID||h.RxFrameType!=FDCAN_DATA_FRAME||h.DataLength!=FDCAN_DLC_BYTES_8)continue;
  unsigned esc,index;
  if(b[2]==0x33 && !previous_status)esc=b[0]|((unsigned)b[1]<<8);
  else if(previous_status)esc=PROBE_DISABLE_READ && b[0]==(previous_id&255U)?(previous_id&255U):previous_id;
  else continue;
  if(esc>2046U)continue;
  for(index=0;index<can_probe.count;index++)if(can_probe.motors[index].esc==esc)break;
  if(index==can_probe.count && index<FOUND_MAX) {
   can_probe.motors[index].esc=esc;can_probe.motors[index].mst=h.Identifier;can_probe.count++;
  }
  if(index>=FOUND_MAX)continue;
  can_probe.motors[index].seen++;
  if(previous_status) {
   memcpy((void *)can_probe.motors[index].feedback,b,8);
   can_probe.motors[index].source|=PROBE_DISABLE_READ?4:2;continue;
  }
  can_probe.motors[index].source|=1;
  for(unsigned j=0;j<8;j++)if(b[3]==registers[j]) {
   uint32_t value;memcpy(&value,b+4,4);can_probe.motors[index].value[j]=value;
   can_probe.motors[index].mask|=1U<<j;
  }
 }
 can_probe.psr=hfdcan1.Instance->PSR;can_probe.ecr=hfdcan1.Instance->ECR;
 if(probe_stop || can_probe.done) {can_probe.state=3;return;}
 if(HAL_GetTick()-previous<4U)return;previous=HAL_GetTick();
 unsigned id,rid;
 if(can_probe.state==1) {
  if(next_id>=2047U) {
   can_probe.rates[rate_index][0]=can_probe.rate;
   can_probe.rates[rate_index][1]=can_probe.psr;
   can_probe.rates[rate_index][2]=can_probe.ecr;
   can_probe.rates[rate_index][3]=hfdcan1.Instance->TXBTO;
   can_probe.pass++;
   if(can_probe.count) {can_probe.state=2;param_index=motor_index=0;}
   else if(++rate_index<(PROBE_DISABLE_READ?1U:6U)) {next_id=0;Setup();}
   else {can_probe.done=1;can_probe.state=3;return;}
  }
 }
 if(can_probe.state==2) {
  if(motor_index>=can_probe.count){can_probe.done=1;can_probe.state=3;return;}
  id=can_probe.motors[motor_index].esc;rid=registers[param_index++%8];
  if(param_index>=32U){param_index=0;motor_index++;}
 } else {id=next_id;rid=8;if(PROBE_DISABLE_READ || query_status)next_id++;}
 can_probe.candidate=id;
 previous_id=id;previous_status=can_probe.state==1?(PROBE_DISABLE_READ?1:query_status):0;
 if(can_probe.state==1)query_status^=1;
 uint8_t q[8]={(uint8_t)id,(uint8_t)(id>>8),previous_status?0xcc:0x33,(uint8_t)rid,0,0,0,0};
 if(previous_status)q[3]=0;
 if(PROBE_DISABLE_READ && can_probe.state==1) {memset(q,0xff,8);q[7]=0xfd;}
 FDCAN_TxHeaderTypeDef h={0};h.Identifier=PROBE_DISABLE_READ && can_probe.state==1?id:0x7ff;h.IdType=FDCAN_STANDARD_ID;
 h.TxFrameType=FDCAN_DATA_FRAME;h.DataLength=FDCAN_DLC_BYTES_8;
 h.FDFormat=FDCAN_CLASSIC_CAN;h.BitRateSwitch=FDCAN_BRS_OFF;
 if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) && HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&h,q)==HAL_OK)can_probe.tx++;
 else can_probe.fail++;
}
