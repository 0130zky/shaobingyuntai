#include "board.h"

/* MC02 onboard buzzer: PB15 / TIM12_CH2, confirmed in vendor-derived wiring. */
static TIM_HandleTypeDef buzzer_timer;
void Board_BuzzerInit(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_TIM12_CLK_ENABLE();
    GPIO_InitTypeDef pin={0};pin.Pin=GPIO_PIN_15;pin.Mode=GPIO_MODE_AF_PP;
    pin.Pull=GPIO_NOPULL;pin.Speed=GPIO_SPEED_FREQ_LOW;pin.Alternate=GPIO_AF2_TIM12;
    HAL_GPIO_Init(GPIOB,&pin);
    RCC_ClkInitTypeDef clocks={0};uint32_t latency;
    HAL_RCC_GetClockConfig(&clocks,&latency);
    uint32_t hz=HAL_RCC_GetPCLK1Freq();if(clocks.APB1CLKDivider!=RCC_APB1_DIV1)hz*=2;
    buzzer_timer.Instance=TIM12;buzzer_timer.Init.Prescaler=hz/1000000U-1U;
    buzzer_timer.Init.CounterMode=TIM_COUNTERMODE_UP;buzzer_timer.Init.Period=499;
    buzzer_timer.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    if(HAL_TIM_PWM_Init(&buzzer_timer)!=HAL_OK)Error_Handler();
    TIM_OC_InitTypeDef oc={0};oc.OCMode=TIM_OCMODE_PWM1;oc.Pulse=0;oc.OCPolarity=TIM_OCPOLARITY_HIGH;
    if(HAL_TIM_PWM_ConfigChannel(&buzzer_timer,&oc,TIM_CHANNEL_2)!=HAL_OK ||
       HAL_TIM_PWM_Start(&buzzer_timer,TIM_CHANNEL_2)!=HAL_OK)Error_Handler();
}
void Board_Buzzer(int on){__HAL_TIM_SET_COMPARE(&buzzer_timer,TIM_CHANNEL_2,on?250U:0U);}

