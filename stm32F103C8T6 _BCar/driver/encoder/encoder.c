/*
编码器设置
    左电机A相： TIM2 CH2 PA1
    左电机B相： TIM2 CH1 PA0
    右电机A相： TIM4 CH2 PB7
    右电机B相： TIM4 CH1 PB6
*/
#include "stm32f10x.h"
#include "delay.h"
#include <string.h>
#include "encoder.h"

static void Encoder_IO_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void TIM_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    // 初始化TIM2和TIM4结构体
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_TimeBaseStruct.TIM_Period = 0xFFFF; // 最大计数值         
    TIM_TimeBaseStruct.TIM_Prescaler = 0;      
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;      
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);
    // 编码器接口配置
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge);
    // 输入捕获配置
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;// 配置通道1
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;// 上升沿和下降沿都捕获
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;// 配置通道2
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    // 清除标志位并启动定时器
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE); // 不需要更新中断
    TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    TIM_SetCounter(TIM2, 0); // 初始值设为0
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

static float last_time = 0;
static int32_t total_count_L = 0;
static uint16_t last_counter_L = 0;
static int32_t total_count_R = 0;
static uint16_t last_counter_R = 0;
float speed_L = 0.0f;
float speed_R = 0.0f;

// 新增：三次采样的缓存数组
#define SAMPLE_CNT 3  // 三次采样
/* 一阶低通滤波系数（α=0.5，值越大越实时，越小越平滑；范围0.1~0.5） */
#define LPF_ALPHA        0.3f
// 新增全局变量：限幅滤波和低通滤波的缓存
static float speed_L_lpf = 0.0f;        // 左电机低通后速度
static float speed_R_lpf = 0.0f;        // 右电机低通后速度

static int16_t delta_L_buf[SAMPLE_CNT] = {0};  // 左电机增量缓存
static int16_t delta_R_buf[SAMPLE_CNT] = {0};  // 右电机增量缓存
static uint8_t sample_idx = 0;                 // 采样索引

void Encoder_Init(void)
{
    Encoder_IO_Init();
    TIM_Init();
}

float Encoder_Get_L(void) // 取为正方向
{
    uint16_t current = TIM_GetCounter(TIM2);
    int16_t delta = (int16_t)(current - last_counter_L);  // 计算增量，处理溢出
    total_count_L += delta;
    last_counter_L = current;
    return total_count_L / 30.0f / 44.0f * 360.0f;// 转换为角度
}

float Encoder_Get_R(void) // 需要取反 因为电机摆放位置对称
{
    uint16_t current = TIM_GetCounter(TIM4);
    int16_t delta = (int16_t)(current - last_counter_R);  // 计算增量，处理溢出
    total_count_R += delta;
    last_counter_R = current;
    return (-total_count_R / 30.0f / 44.0f * 360.0f);// 转换为角度
}

void Encoder_Get_Speed(void)
{
    float now_time = (float)tim_get_ms();
    for(; sample_idx < SAMPLE_CNT; sample_idx++)
    {
        // 1. 读取当前计数器，计算单次增量
        uint16_t current_L = TIM_GetCounter(TIM2);
        int16_t delta_L = (int16_t)(current_L - last_counter_L);
        total_count_L += delta_L;
        last_counter_L = current_L;

        uint16_t current_R = TIM_GetCounter(TIM4);
        int16_t delta_R = (int16_t)(current_R - last_counter_R);
        total_count_R += delta_R;
        last_counter_R = current_R;

        // 2. 将单次增量存入缓存
        delta_L_buf[sample_idx] = delta_L;
        delta_R_buf[sample_idx] = delta_R;
        tim_delay_ms(1); // 延时1ms，进行下一次采样
    }
    float dt = (now_time - last_time) / SAMPLE_CNT; // 平均时间间隔
    last_time = now_time;

    sample_idx = 0; // 重置索引
    // 计算三次增量的总和
    int32_t sum_delta_L = 0;
    int32_t sum_delta_R = 0;
    for(uint8_t i=0; i<SAMPLE_CNT; i++)
    {
        sum_delta_L += delta_L_buf[i];
        sum_delta_R += delta_R_buf[i];
    }
    // 计算平均增量（三次采样平均）
    float avg_delta_L = (float)sum_delta_L / 3.0f;
    float avg_delta_R = (float)sum_delta_R / 3.0f;

    // 4. 基于平均增量计算速度（单位：度/秒）
    float raw_speed_L = avg_delta_L / 30.0f / 44.0f * 360.0f / (dt / 1000.0f );
    float raw_speed_R = -avg_delta_R / 30.0f / 44.0f * 360.0f / (dt / 1000.0f ); 

    // ========== 一阶低通滤波（平滑小毛刺） ==========
    speed_L_lpf = LPF_ALPHA * raw_speed_L + (1 - LPF_ALPHA) * speed_L_lpf;
    speed_R_lpf = LPF_ALPHA * raw_speed_R + (1 - LPF_ALPHA) * speed_R_lpf;

    // 替换原有速度值（后续用滤波后的speed_L_lpf/speed_R_lpf）
    speed_L = speed_L_lpf;
    speed_R = speed_R_lpf;    
}

