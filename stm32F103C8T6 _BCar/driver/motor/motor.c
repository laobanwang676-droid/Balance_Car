#include "motor.h"
#include "stm32f10x.h"
/*
    PWM波A : TIM1 CH4 PA11
    PWM波B : TIM1 CH1 PA8
    AN2 (用于控制左电机 正 方向) : PB12    高电平
    AN1 (用于控制左电机 正 方向) : PB13    低电平

    BN1 (用于控制右电机 正 方向) : PB14    低电平
    BN2 (用于控制右电机 正 方向) : PB15    高电平
*/
#define PWM_DUTY_MAX 90         //(0~100)
#define PWM_DUTY_MIN -90        //(-100~0)

static void Motor_IO_Init(void)
{   
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void TIM1_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;//通道初始化

    TIM_TimeBaseStruct.TIM_Period = 1000-1;//1KHZ PWM频率            
    TIM_TimeBaseStruct.TIM_Prescaler = 72-1;//1MHZ      
    TIM_TimeBaseStruct.TIM_ClockDivision = 0;//仅用于定时器的 输入捕获 和 触发输入      
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式 
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);

    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;//PWM模式1:在向上计数期间，TIMx_CNT<TIMx_CCRx时为高电平，否则为低电平
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;//输出使能
    TIM_OCInitStruct.TIM_Pulse = 500;//初始占空比为1/2;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;//pwm波高电平有效
    TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;//空闲状态为低电平
    TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;//互补输出关闭
    TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;//互补输出高电平有效
    TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//互补输出空闲状态为低电平

    TIM_OC1Init(TIM1, &TIM_OCInitStruct);//初始化通道1
    TIM_OC4Init(TIM1, &TIM_OCInitStruct);//初始化通道4

    TIM_CtrlPWMOutputs(TIM1, ENABLE);//MOE 主输出使能
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);//修改ccr寄存器值不会立即生效
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);//修改ccr寄存器值不会立即生效
    TIM_ARRPreloadConfig(TIM1, ENABLE);//修改arr寄存器值不会立即生效
    TIM_Cmd(TIM1, ENABLE);
}

void Motor_Init(void)
{
    Motor_IO_Init();
    TIM1_Init();
}

static void Motor_L_forward(void)//左电机前进
{
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
    GPIO_ResetBits(GPIOB, GPIO_Pin_13);
}

static void Motor_L_backward(void)//左电机后退
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    GPIO_SetBits(GPIOB, GPIO_Pin_13);
}

static void Motor_R_forward(void)//右电机前进
{
    GPIO_SetBits(GPIOB, GPIO_Pin_15);
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
}

static void Motor_R_backward(void)//右电机后退
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    GPIO_SetBits(GPIOB, GPIO_Pin_14);
}
void Motor_L_speed(float duty)
{
    if(duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;
    if(duty < PWM_DUTY_MIN) duty = PWM_DUTY_MIN;
    if(duty >= 0)
    {
        Motor_L_forward();
        TIM_SetCompare4(TIM1, (uint16_t)(duty / 100.0f * 999));
    }
    else
    {
        Motor_L_backward();
        TIM_SetCompare4(TIM1, (uint16_t)(-duty / 100.0f * 999));
    }
}

void Motor_R_speed(float duty)
{
    if(duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;
    if(duty < PWM_DUTY_MIN) duty = PWM_DUTY_MIN;
    if(duty >= 0)
    {
        Motor_R_forward();
        TIM_SetCompare1(TIM1, (uint16_t)(duty / 100.0f * 999));
    }
    else
    {
        Motor_R_backward();
        TIM_SetCompare1(TIM1, (uint16_t)(-duty / 100.0f * 999));
    }
}

