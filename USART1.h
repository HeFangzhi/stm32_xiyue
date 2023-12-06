#ifndef __USART1_H
#define __USART1_H

#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include "DataPara.h"
#include "SimpleDigitalFiltering.h"

void USART1_Config(unsigned int baudrate);
void DMA_Config(void);
void RecUltraData(void);

extern SensorData Sensor_Info;

#endif
