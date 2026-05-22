#include <string.h>
#include "stm32f10x.h"
#include "usart.h"
#include "delay.h"
/*
蓝牙串口：USART3
PB10 TX
PB11 RX
*/
// char rxbuf[10] = {0};
// uint8_t rxlen = 0;

volatile uint8_t flag_speed= 0; //速度控制标志位
volatile uint8_t flag_turn = 0;  //转向控制标志位
volatile uint8_t flag_stop = 0;  //停止标志位
// volatile uint8_t back;//后退
// volatile uint8_t left;//左转
// volatile uint8_t right;//右转
// volatile uint8_t stop;//停止

static void USART3_IO_Init(void)
{
    // 使能 GPIOB 和 AFIO 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    //设置为复用模式
    GPIO_InitTypeDef GPIO_InitStruct;
    // PB10 TX 
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // PB11 RX 
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void USART3_func_Init(void)
{
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    USART_InitStruct.USART_BaudRate = 9600;                  
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;    
    USART_InitStruct.USART_StopBits = USART_StopBits_1;          
    USART_InitStruct.USART_Parity = USART_Parity_No;             
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 
    USART_Init(USART3, &USART_InitStruct);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 4;  
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE ;
    NVIC_Init(&NVIC_InitStruct);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // 使能接收中断
    USART_Cmd(USART3, ENABLE);
}

void USART3_Init(void)
{
    USART3_IO_Init();
    USART3_func_Init();
}

void USART3_Send_Data( const uint8_t *data, uint16_t size)
{
    if(size == 0) 
        return;
    if(data == NULL) 
        return;
    for(uint16_t i=0; i < size; i++)
    {
        while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
        USART_SendData(USART3, data[i]);
    }
    while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}

void USART3_Receive_Data(uint8_t *data, uint16_t size)
{
    for(uint16_t i=0; i < size; i++)
    {
        while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET);
        data[i] = USART_ReceiveData(USART3);
    }
}

void USART3_IRQHandler(void)
{       
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    { 
        uint8_t rx = USART_ReceiveData(USART3);
        if(rx == 0x01) flag_speed = 1; //前进
        else if(rx == 0x02) flag_speed = 2; //后退
        else if(rx == 0x03) flag_turn = 1; //左转
        else if(rx == 0x04) flag_turn = 2; //右转
        else if(rx == 0x00) flag_stop = 1; //停止
    }
}

int fputc(int ch, FILE *f)
{
    USART_SendData(USART3, (uint8_t)ch);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    return ch;
}

