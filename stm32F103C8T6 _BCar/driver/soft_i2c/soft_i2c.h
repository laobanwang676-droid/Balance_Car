#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include "stm32f10x.h"

// ===================== 软I2C引脚定义（STM32F103C8T6，PB3=SDA，PB4=SCL） =====================
#define I2C_SDA_PIN    GPIO_Pin_3
#define I2C_SCL_PIN    GPIO_Pin_4
#define I2C_GPIO_PORT  GPIOB
#define I2C_GPIO_CLK   RCC_APB2Periph_GPIOB

// 引脚电平控制宏
#define I2C_SDA_H()  GPIO_SetBits(I2C_GPIO_PORT, I2C_SDA_PIN)   // SDA置高
#define I2C_SDA_L()  GPIO_ResetBits(I2C_GPIO_PORT, I2C_SDA_PIN) // SDA置低
#define I2C_SCL_H()  GPIO_SetBits(I2C_GPIO_PORT, I2C_SCL_PIN)   // SCL置高
#define I2C_SCL_L()  GPIO_ResetBits(I2C_GPIO_PORT, I2C_SCL_PIN) // SCL置低
// 读取SDA引脚电平（双向IO，接收数据时用）
#define I2C_READ_SDA()  GPIO_ReadInputDataBit(I2C_GPIO_PORT, I2C_SDA_PIN)

// ===================== 函数声明 =====================
void Soft_I2C_Init(void);
void Soft_I2C_Start(void);
void Soft_I2C_Stop(void);
uint8_t Soft_I2C_Send_Byte(uint8_t dat);
uint8_t Soft_I2C_Read_Byte(uint8_t ack);
int8_t Soft_I2C_SendBytes(uint8_t Addr, uint8_t *pData, uint16_t Size);
int8_t Soft_I2C_RecvBytes(uint8_t Addr, uint8_t *pData, uint16_t Size);

#endif /* __SOFT_I2C_H */
