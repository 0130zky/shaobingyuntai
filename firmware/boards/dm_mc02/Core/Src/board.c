/* DM-MC02: CAN1 PD0/PD1, inverted DBUS connector -> UART5 PD2.
 * UART uses byte interrupts, not DMA: all buffers may safely reside in DTCM. */
#include "board.h"
#include "fdcan.h"
#include "sentry.h"
#include "sentry_remote.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
UART_HandleTypeDef huart5;

static uint8_t rx_byte;
typedef struct { uint32_t tick; uint8_t byte; } SbusByte;
static SbusByte sbus_ring[128];
static volatile uint16_t sbus_write,sbus_read;
static volatile uint8_t sbus_error;
volatile uint32_t board_uart_errors,board_can_dropped,board_can_recoveries;

static void DbusInit(void)
{
    RCC_PeriphCLKInitTypeDef clk={0};
    clk.PeriphClockSelection=RCC_PERIPHCLK_UART5;
    clk.Usart234578ClockSelection=RCC_USART234578CLKSOURCE_D2PCLK1;
    if(HAL_RCCEx_PeriphCLKConfig(&clk)!=HAL_OK)Error_Handler();
    __HAL_RCC_UART5_CLK_ENABLE();__HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef pin={0};pin.Pin=GPIO_PIN_2;pin.Mode=GPIO_MODE_AF_PP;
    pin.Pull=GPIO_PULLUP;pin.Speed=GPIO_SPEED_FREQ_VERY_HIGH;pin.Alternate=GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD,&pin);
    huart5.Instance=UART5;huart5.Init.BaudRate=100000;
    /* 9-bit word includes parity: 8 payload bits + even parity, one stop bit. */
    huart5.Init.WordLength=UART_WORDLENGTH_9B;huart5.Init.StopBits=UART_STOPBITS_1;
    huart5.Init.Parity=UART_PARITY_EVEN;huart5.Init.Mode=UART_MODE_RX;
    huart5.Init.HwFlowCtl=UART_HWCONTROL_NONE;huart5.Init.OverSampling=UART_OVERSAMPLING_16;
    huart5.Init.OneBitSampling=UART_ONE_BIT_SAMPLE_DISABLE;
    huart5.Init.ClockPrescaler=UART_PRESCALER_DIV1;
    /* Board connector has hardware inversion. Do not invert again in UART. */
    huart5.AdvancedInit.AdvFeatureInit=UART_ADVFEATURE_NO_INIT;
    if(HAL_UART_Init(&huart5)!=HAL_OK || HAL_UARTEx_DisableFifoMode(&huart5)!=HAL_OK)Error_Handler();
    HAL_NVIC_SetPriority(UART5_IRQn,6,0);HAL_NVIC_EnableIRQ(UART5_IRQn);
    if(HAL_UART_Receive_IT(&huart5,&rx_byte,1)!=HAL_OK)Error_Handler();
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if(uart!=&huart5)return;
    uint16_t next=(sbus_write+1U)&127U;
    if(next==sbus_read)sbus_error=1;
    else {sbus_ring[sbus_write]=(SbusByte){HAL_GetTick(),rx_byte};__DMB();sbus_write=next;}
    if(HAL_UART_Receive_IT(uart,&rx_byte,1)!=HAL_OK) {sbus_error=1;board_uart_errors++;}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if(uart!=&huart5)return;
    sbus_error=1;board_uart_errors++;
    HAL_UART_AbortReceive(uart);
    (void)HAL_UART_Receive_IT(uart,&rx_byte,1);
}
void Board_RemotePoll(void)
{
    static uint8_t frame[18];static unsigned used;static uint32_t previous;
    if(sbus_error) {
        taskENTER_CRITICAL();sbus_read=sbus_write;sbus_error=0;taskEXIT_CRITICAL();
        used=0;SentryRemote_Invalidate();
    }
    for(unsigned n=0;n<128 && sbus_read!=sbus_write;n++) {
        __DMB();SbusByte b=sbus_ring[sbus_read];sbus_read=(sbus_read+1U)&127U;
        if(HAL_GetTick()-b.tick>=100U) {used=0;SentryRemote_Invalidate();continue;}
        /* DBUS has no sync byte. A >=3ms byte gap starts a new frame.
         * On corrupt windows slide one byte, and require full frame validation. */
        if(b.tick-previous>2U)used=0;
        previous=b.tick;frame[used++]=b.byte;
        if(used==18U) {
            if(SentryRemote_Decode(frame,b.tick))used=0;
            else {memmove(frame,frame+1,17);used=17;}
        }
    }
}
void Board_CanStop(void)
{
    /* Cancel queued position commands before sending disable. Avoid stale
     * motion commands being replayed when an interrupted bus recovers. */
    (void)HAL_FDCAN_AbortTxRequest(&hfdcan1,0xffU);
}
void Board_CanRecover(void)
{
    static uint32_t previous;
    uint32_t now=HAL_GetTick();
    /* Never recover into an active motion request. Invalidate stale feedback,
     * cancel queued frames, then let the control task issue disable/query frames. */
    if(!SentryRemote_Disabled(now) || now-previous<250U)return;
    if(!(hfdcan1.Instance->PSR&FDCAN_PSR_BO) ||
       !(hfdcan1.Instance->CCCR&FDCAN_CCCR_INIT))return;
    previous=now;
    Board_CanStop();Sentry_InvalidateFeedback();
    hfdcan1.Instance->CCCR&=~FDCAN_CCCR_INIT;
    board_can_recoveries++;
}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *bus,uint32_t flags)
{
    if(bus!=&hfdcan1)return;
    if(flags&FDCAN_IT_RX_FIFO0_MESSAGE_LOST) {board_can_dropped++;Sentry_InvalidateFeedback();}
    for(unsigned n=0;n<16U && HAL_FDCAN_GetRxFifoFillLevel(bus,FDCAN_RX_FIFO0);n++) {
        FDCAN_RxHeaderTypeDef header;uint8_t bytes[64];
        if(HAL_FDCAN_GetRxMessage(bus,FDCAN_RX_FIFO0,&header,bytes)!=HAL_OK)break;
        if(header.IdType==FDCAN_STANDARD_ID && header.RxFrameType==FDCAN_DATA_FRAME &&
           header.FDFormat==FDCAN_CLASSIC_CAN && header.DataLength==FDCAN_DLC_BYTES_8)
            Sentry_Rx((uint16_t)header.Identifier,bytes,8);
    }
}
void Board_Init(void)
{
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CYCCNT=0;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    Sentry_Init();
    FDCAN_FilterTypeDef filter={0};filter.IdType=FDCAN_STANDARD_ID;
    filter.FilterType=FDCAN_FILTER_RANGE;filter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1=0;filter.FilterID2=0x7ff;
    if(HAL_FDCAN_ConfigFilter(&hfdcan1,&filter)!=HAL_OK ||
       HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE)!=HAL_OK ||
       HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE|FDCAN_IT_RX_FIFO0_MESSAGE_LOST,0)!=HAL_OK ||
       HAL_FDCAN_Start(&hfdcan1)!=HAL_OK)Error_Handler();
    DbusInit();
    Board_BuzzerInit();
}
