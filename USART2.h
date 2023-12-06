#ifndef __USART2_H
#define __USART2_H


#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include "DataPara.h"
#include "SimpleDigitalFiltering.h"

extern uint8_t Usart2SendBuff[USART2SENDBUFF_SIZE];
extern uint8_t Usart2RecBuff[USART2RECBUFF_SIZE];

void SendToFlight(void);
void IntegrationData(uint8_t *pdata, uint16_t Length);
void USART2_Config(uint32_t baudrate);
void Usart2SendData_DMA(uint8_t *pdata, uint16_t Length);
void SendHeightToFlight(void);
extern SensorData Sensor_Info;

#endif 
