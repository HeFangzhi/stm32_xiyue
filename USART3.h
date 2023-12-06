#ifndef __USART3_H
#define __USART3_H


#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include "DataPara.h"
#include "SimpleDigitalFiltering.h"
#include "USART2.h"
#include "Time_2.h"

void USART3_Config(unsigned int baudrate);
void Usart3SendData_DMA(uint8_t *pdata, uint16_t Length);

extern uint8_t Usart3SendBuff[USART3SENDBUFF_SIZE];
extern uint8_t Usart3RecBuff[USART3RECBUFF_SIZE];

#endif 
