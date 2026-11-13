"""Exercise actual MC02 board routing/SBUS stream and H7 MIT encoding on host."""
from pathlib import Path
import tempfile, subprocess
r=Path(__file__).resolve().parents[1]
stub=r'''
#ifndef MC02_STUB
#define MC02_STUB
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#define HAL_OK 0
#define HAL_ERROR 1
#define FDCAN_STANDARD_ID 0
#define FDCAN_EXTENDED_ID 1
#define FDCAN_DATA_FRAME 0
#define FDCAN_REMOTE_FRAME 1
#define FDCAN_CLASSIC_CAN 0
#define FDCAN_FD_CAN 1
#define FDCAN_DLC_BYTES_8 0x80000
#define FDCAN_DLC_BYTES_7 0x70000
#define FDCAN_ESI_ACTIVE 0
#define FDCAN_BRS_OFF 0
#define FDCAN_NO_TX_EVENTS 0
#define FDCAN_RX_FIFO0 0
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST 8
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1
#define FDCAN_FILTER_RANGE 0
#define FDCAN_FILTER_TO_RXFIFO0 0
#define FDCAN_REJECT 0
#define FDCAN_REJECT_REMOTE 0
#define GPIO_PIN_2 4
#define GPIO_MODE_AF_PP 2
#define GPIO_PULLUP 1
#define GPIO_SPEED_FREQ_VERY_HIGH 3
#define GPIO_AF8_UART5 8
#define UART_WORDLENGTH_9B 9
#define UART_STOPBITS_1 1
#define UART_PARITY_EVEN 2
#define UART_MODE_RX 2
#define UART_HWCONTROL_NONE 0
#define UART_OVERSAMPLING_16 16
#define UART_ONE_BIT_SAMPLE_DISABLE 0
#define UART_PRESCALER_DIV1 1
#define UART_ADVFEATURE_NO_INIT 0
#define RCC_PERIPHCLK_UART5 1
#define RCC_USART234578CLKSOURCE_D2PCLK1 2
#define UART5_IRQn 53
#define UART5 ((void*)5)
#define GPIOD ((void*)4)
#define __HAL_RCC_UART5_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __DMB() ((void)0)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define CoreDebug_DEMCR_TRCENA_Msk 1
#define DWT_CTRL_CYCCNTENA_Msk 1
static struct {uint32_t DEMCR;} dbg;
static struct {uint32_t CYCCNT,CTRL;} dwt;
#define CoreDebug (&dbg)
#define DWT (&dwt)
#define FDCAN_PSR_BO 0x80U
#define FDCAN_CCCR_INIT 1U
typedef struct {uint32_t PSR,CCCR;} FDCAN_Registers;
static FDCAN_Registers can_registers;
typedef struct {FDCAN_Registers *Instance;} FDCAN_HandleTypeDef;
static FDCAN_HandleTypeDef hfdcan1={&can_registers};
typedef struct {uint32_t Identifier,IdType,TxFrameType,DataLength,ErrorStateIndicator,BitRateSwitch,FDFormat,TxEventFifoControl,MessageMarker;} FDCAN_TxHeaderTypeDef;
typedef struct {uint32_t Identifier,IdType,RxFrameType,FDFormat,DataLength;} FDCAN_RxHeaderTypeDef;
typedef struct {uint32_t IdType,FilterType,FilterConfig,FilterID1,FilterID2,FilterIndex;} FDCAN_FilterTypeDef;
typedef struct {uint32_t Pin,Mode,Pull,Speed,Alternate;} GPIO_InitTypeDef;
typedef struct {uint32_t PeriphClockSelection,Usart234578ClockSelection;} RCC_PeriphCLKInitTypeDef;
typedef struct {void *Instance;struct {uint32_t BaudRate,WordLength,StopBits,Parity,Mode,HwFlowCtl,OverSampling,OneBitSampling,ClockPrescaler;} Init;struct {uint32_t AdvFeatureInit;} AdvancedInit;} UART_HandleTypeDef;
static uint32_t tick=1000,freelevel=8,tx_result=HAL_OK,aborted,rx_count,invalidations;
static FDCAN_TxHeaderTypeDef tx_header;
static uint8_t tx_data[8];
static FDCAN_RxHeaderTypeDef rx_headers[16];
static unsigned read_index,write_index;
uint32_t HAL_GetTick(void){return tick;}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *h){(void)h;return freelevel;}
int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,FDCAN_TxHeaderTypeDef *t,uint8_t *p){(void)h;tx_header=*t;memcpy(tx_data,p,8);return tx_result;}
int HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *h,uint32_t m){(void)h;aborted=m;return 0;}
unsigned HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h,unsigned f){(void)h;(void)f;return write_index-read_index;}
int HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h,unsigned f,FDCAN_RxHeaderTypeDef *hdr,uint8_t *p){(void)h;(void)f;*hdr=rx_headers[read_index++];memset(p,0,64);return 0;}
int HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *h,FDCAN_FilterTypeDef *f){(void)h;assert(f->FilterID2==0x7ff);return 0;}
int HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h,unsigned a,unsigned b,unsigned c,unsigned d){(void)h;(void)a;(void)b;(void)c;(void)d;return 0;}
int HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h,unsigned n,unsigned b){(void)h;(void)n;(void)b;return 0;}
int HAL_FDCAN_Start(FDCAN_HandleTypeDef *h){(void)h;return 0;}
int HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *c){(void)c;return 0;}
void HAL_GPIO_Init(void *g,GPIO_InitTypeDef *p){(void)g;assert(p->Pin==4 && p->Alternate==8);}
int HAL_UART_Init(UART_HandleTypeDef *h){assert(h->Init.BaudRate==100000 && h->Init.WordLength==9 && h->Init.Parity==2 && h->Init.StopBits==1);return 0;}
int HAL_UARTEx_DisableFifoMode(UART_HandleTypeDef *h){(void)h;return 0;}
int HAL_UART_Receive_IT(UART_HandleTypeDef *h,uint8_t *p,unsigned n){(void)h;(void)p;assert(n==1);return 0;}
int HAL_UART_AbortReceive(UART_HandleTypeDef *h){(void)h;return 0;}
void HAL_NVIC_SetPriority(int n,int p,int s){(void)n;(void)p;(void)s;}
void HAL_NVIC_EnableIRQ(int n){(void)n;}
void Error_Handler(void){assert(0);}
void Board_BuzzerInit(void){}
void Sentry_Init(void){}
void Sentry_InvalidateFeedback(void){invalidations++;}
void Sentry_Rx(uint16_t id,const uint8_t *p,uint8_t n){(void)id;(void)p;assert(n==8);rx_count++;}
#endif
'''
harness=r'''
#include "test_stub.h"
#include "board.c"
#include "app_DM4310.c"
#include "sentry_remote.c"
static void dbus(uint8_t frame[18],int right,int left,int s1){
 memset(frame,0,18);int channels[4]={1024,right,1024,left};
 for(int i=0;i<4;i++)for(int b=0;b<11;b++)if(channels[i]&(1<<b)){
  unsigned bit=(unsigned)(i*11+b);frame[bit/8]|=(uint8_t)(1<<(bit%8));
 }
 frame[5]|=(uint8_t)((s1<<6)|(2<<4));
}
static void bytes(const uint8_t *p,unsigned n){for(unsigned i=0;i<n;i++){if(i%8==0)tick++;rx_byte=p[i];HAL_UART_RxCpltCallback(&huart5);}}
int main(void){
 Board_Init();
 uint8_t f[18];dbus(f,1600,364,2);bytes(f,9);Board_RemotePoll();assert(!sentry_remote.frames);
 bytes(f+9,9);Board_RemotePoll();assert(sentry_remote.frames==1&&!SentryRemote_Fresh(tick));
 tick+=4;bytes(f,18);Board_RemotePoll();assert(SentryRemote_Enabled(tick));
 assert(sentry_remote.channel[1]==1600 && sentry_remote.channel[3]==364);
 assert(SentryRemote_Stick(0)==-1 && SentryRemote_Stick(1)>.8f);
 for(int s2=1;s2<=3;s2++) {
  tick+=4;dbus(f,1024,1024,2);f[5]=(uint8_t)((f[5]&~0x30U)|(s2<<4));
  bytes(f,18);Board_RemotePoll();assert(sentry_remote.s2==s2);
  assert(SentryRemote_Enabled(tick)==(s2!=1));
  assert(SentryRemote_Disabled(tick)==(s2==1));
 }
 for(int s=1;s<=3;s+=2){tick+=4;dbus(f,1024,1024,s);bytes(f,18);Board_RemotePoll();assert(SentryRemote_Disabled(tick)&&!SentryRemote_Enabled(tick));}
 tick+=4;dbus(f,1024,1024,0);bytes(f,18);Board_RemotePoll();assert(!SentryRemote_Fresh(tick));
 tick+=4;dbus(f,1024,1024,2);bytes(f,18);Board_RemotePoll();assert(!SentryRemote_Fresh(tick));
 tick+=4;bytes(f,18);Board_RemotePoll();assert(SentryRemote_Enabled(tick));
 tick+=101;assert(!SentryRemote_Fresh(tick));
 bytes(f,18);tick+=101;Board_RemotePoll();assert(!SentryRemote_Fresh(tick));
 for(int i=0;i<8;i++)bytes(f,18);Board_RemotePoll();assert(!SentryRemote_Fresh(tick));
 tick+=4;bytes(f,18);Board_RemotePoll();tick+=4;bytes(f,18);Board_RemotePoll();assert(SentryRemote_Fresh(tick));
 // A panel signature requires a changing heartbeat; frozen keys expire
 // even while valid DBUS frames continue to arrive.
 tick+=4;dbus(f,1024,1024,1);f[14]=1;f[15]=0x40;
 bytes(f,18);Board_RemotePoll();assert(SentryRemote_Keys(tick)==0);
 for(int i=0;i<6;i++){tick+=14;bytes(f,18);Board_RemotePoll();}
 f[15]=0xc0;f[6]=0xfe;f[7]=0xff;f[12]=1;
 bytes(f,18);Board_RemotePoll();assert(SentryRemote_Keys(tick)==1);
 assert(sentry_remote.mouse[0]==-2&&sentry_remote.buttons[0]==1);
 for(int i=0;i<18;i++){tick+=14;bytes(f,18);Board_RemotePoll();}
 assert(SentryRemote_Fresh(tick)&&SentryRemote_Keys(tick)==0);
 f[14]=f[15]=0;tick+=4;bytes(f,18);Board_RemotePoll();assert(SentryRemote_Keys(tick)==0);
 HAL_UART_ErrorCallback(&huart5);Board_RemotePoll();assert(!SentryRemote_Fresh(tick)&&board_uart_errors==1);
 for(int i=0;i<5;i++)rx_headers[i]=(FDCAN_RxHeaderTypeDef){.Identifier=0x11,.IdType=FDCAN_STANDARD_ID,.RxFrameType=FDCAN_DATA_FRAME,.FDFormat=FDCAN_CLASSIC_CAN,.DataLength=FDCAN_DLC_BYTES_8};
 rx_headers[1].IdType=FDCAN_EXTENDED_ID;rx_headers[2].RxFrameType=FDCAN_REMOTE_FRAME;
 rx_headers[3].FDFormat=FDCAN_FD_CAN;rx_headers[4].DataLength=FDCAN_DLC_BYTES_7;write_index=5;
 HAL_FDCAN_RxFifo0Callback(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE|FDCAN_IT_RX_FIFO0_MESSAGE_LOST);
 assert(rx_count==1&&invalidations==1&&board_can_dropped==1);
 dm4310_range_t range={-12.5,12.5,-30,30,-10,10,500,5};
 assert(DM4310_Send_Ranged(&hfdcan1,2,0,0,0,20,1,&range));
 assert(tx_header.Identifier==2&&tx_header.DataLength==FDCAN_DLC_BYTES_8&&tx_header.FDFormat==FDCAN_CLASSIC_CAN&&tx_header.BitRateSwitch==FDCAN_BRS_OFF);
 assert(DM4310_Send_PositionRaw(&hfdcan1,2,31439,0,0,20,1,&range));
 assert((((unsigned)tx_data[0]<<8)|tx_data[1])==31439);
 assert(DM4310_SetEnable(&hfdcan1,1,1)&&tx_data[7]==0xfc);
 assert(DM4310_SetEnable(&hfdcan1,1,0)&&tx_data[7]==0xfd);
 freelevel=0;assert(!DM4310_SetEnable(&hfdcan1,1,1));freelevel=8;
 tx_result=HAL_ERROR;assert(!DM4310_SetEnable(&hfdcan1,1,1));
 Board_CanStop();assert(aborted==0xffU);
 // Bus-Off stays stopped while enabled, then recovers only after a fresh disable.
 can_registers.PSR=FDCAN_PSR_BO;can_registers.CCCR=FDCAN_CCCR_INIT;
 tick+=300;sentry_remote.tick=tick;sentry_remote.seen=1;sentry_remote.s1=2;sentry_remote.s2=2;
 unsigned old_invalidations=invalidations;Board_CanRecover();
 assert(can_registers.CCCR==1&&board_can_recoveries==0);
 sentry_remote.s1=1;Board_CanRecover();
 assert(can_registers.CCCR==0&&board_can_recoveries==1&&aborted==0xffU&&invalidations==old_invalidations+1);
 // Repeated errors are rate limited, and stale remote input cannot trigger recovery.
 can_registers.CCCR=1;Board_CanRecover();assert(can_registers.CCCR==1);
 tick+=300;Board_CanRecover();assert(can_registers.CCCR==1);
 sentry_remote.tick=tick;Board_CanRecover();assert(can_registers.CCCR==0&&board_can_recoveries==2);
 return 0;
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td);(p/'test_stub.h').write_text(stub)
 for name in ('main.h','fdcan.h','stm32h7xx_hal.h','FreeRTOS.h','task.h'):
  (p/name).write_text('#include "test_stub.h"\n')
 (p/'harness.c').write_text(harness)
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-DSENTRY_STM32H7','-I'+td,
  '-I'+str(r/'boards/dm_mc02/Core/Inc'),'-I'+str(r/'boards/dm_mc02/Core/Src'),
  '-I'+str(r/'modules/folding_gimbal'),'-I'+str(r/'robots/sentry'),'-I'+str(r/'Application'),'-I'+str(r/'Components/Devices'),
  str(p/'harness.c'),'-lm','-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
 print('PASS MC02: UART5 100k 8E1, DT7 channels/S1/S2, split/malformed/stale DBUS, two-frame recovery, ring overflow, UART recovery, FDCAN routing/DLC/TX failures, abort')
