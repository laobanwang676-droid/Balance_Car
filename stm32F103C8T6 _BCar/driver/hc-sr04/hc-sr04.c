#include "stm32f10x.h"
#include "hc-sr04.h"
#include "delay.h"
/*
hcsr04超声波避障模块驱动代码
PA3作为发送起始信号引脚
PA2作为回响信号输入引脚
*/

static void gpio_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    // PA2 回发引脚
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;//输入下拉 
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // PA3 发射引脚
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_ResetBits(GPIOA, GPIO_Pin_2); // 初始输入引脚拉低
    GPIO_ResetBits(GPIOA, GPIO_Pin_3); // 初始输出引脚拉低
}

void HC_SR04_Init(void)
{
    gpio_init();
}

float hc_sr04_distance(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_3);
    tim_delay_us(30);
    GPIO_ResetBits(GPIOA, GPIO_Pin_3);

    uint64_t start_time1 = tim_get_us();
    while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == RESET)
    {
        if(tim_get_us() - start_time1 > 20000) 
            return 99.9f;
    }
    uint64_t start_time2 = tim_get_us();
    while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == SET)
    {
        if(tim_get_us() - start_time2 > 20000) 
            return 99.9f;
    }
    uint64_t end_time = tim_get_us();

    return ((end_time - start_time2) * 0.034f / 2.0f); // cm
}
/*
五路循迹模块驱动代码
PB1
PB0
PA7
PA6
PA5
*/

void line_gpio_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    // PB1 PB0
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;//输入下拉 
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // PA7 PA6 PA5
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;//输入下拉 
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}

uint8_t read_line_sensor(void)
{
    uint8_t pin1 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) ? 1 : 0;
    uint8_t pin0 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) ? 1 : 0;
    uint8_t pin7 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) ? 1 : 0;
    uint8_t pin6 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) ? 1 : 0;
    uint8_t pin5 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) ? 1 : 0;
    return (pin1 << 4) | (pin0 << 3) | (pin7 << 2) | (pin6 << 1) | pin5;
}
