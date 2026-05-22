#ifndef _USART_H_
#define _USART_H_

#include "stm32f10x.h"
#include "stdio.h"
// extern char *cmd;
// extern char rxbuf[10];
void USART3_Init(void);
void USART3_Send_Data( const uint8_t *data, uint16_t size);
void USART3_Receive_Data(uint8_t *data, uint16_t size);

#endif /*__TUSART_H__*/

