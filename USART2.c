#include "USART2.h"

uint8_t Usart2SendBuff[USART2SENDBUFF_SIZE];
uint8_t Usart2RecBuff[USART2RECBUFF_SIZE];
uint8_t Usart2_Dma_SendFlag = 1;
/*
	飞控接口
	
*/


void USART2_Config(uint32_t baudrate)  
{  
    USART_InitTypeDef USART_InitStruct;  
    GPIO_InitTypeDef GPIO_InitStructure;  
    NVIC_InitTypeDef NVIC_InitStructure;  
    DMA_InitTypeDef DMA_InitStructure;
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);      
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
	// 设置GPIO为复用输出和浮空输入
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  //PA2
    GPIO_Init(GPIOA, &GPIO_InitStructure);   
	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;  //PA3
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
  
    // 使能接收和发送的中断 
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure);  
	// 设置UART2的配置
    USART_InitStruct.USART_BaudRate = baudrate;  
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;  
    USART_InitStruct.USART_StopBits = USART_StopBits_1;  
    USART_InitStruct.USART_Parity = USART_Parity_No ;  
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;      
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    USART_Init(USART2, &USART_InitStruct);    
  
    DMA_DeInit(DMA1_Channel6);    
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);  
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Usart2RecBuff;  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  
    DMA_InitStructure.DMA_BufferSize = USART2RECBUFF_SIZE;        
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;   
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;      
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;      
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;            
    DMA_Init(DMA1_Channel6,&DMA_InitStructure);  
    DMA_Cmd(DMA1_Channel6,ENABLE);  
    USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);         
  
    DMA_DeInit(DMA1_Channel7);   
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);  
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Usart2SendBuff;  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  
    DMA_InitStructure.DMA_BufferSize = USART2SENDBUFF_SIZE;  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);  
    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);  
    DMA_ITConfig(DMA1_Channel7, DMA_IT_TE, ENABLE);  
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);  
    DMA_Cmd(DMA1_Channel7, DISABLE);  
  
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure);  
  
    USART_Cmd(USART2, ENABLE);  
    USART_ClearFlag(USART2,USART_FLAG_TC);  
}  

u8 PcBuff[64];
uint16_t PcLen = 0;
unsigned char PcFinish = 0;
void USART2_IRQHandler(void)  
{   
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) 
	{   
			USART2->SR;   
			USART2->DR;  
			DMA_Cmd(DMA1_Channel6,DISABLE); 
			PcLen = USART2RECBUFF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);
			memcpy(PcBuff,Usart2RecBuff,PcLen);
			PcFinish = 1;		
			DMA_SetCurrDataCounter(DMA1_Channel6,USART2SENDBUFF_SIZE);    
			DMA_Cmd(DMA1_Channel6,ENABLE); 
	}   
}  

void DMA1_Channel7_IRQHandler(void)  
{  
  DMA_Cmd(DMA1_Channel7, DISABLE);  
  USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE); 
  DMA1_Channel7->CNDTR = USART2SENDBUFF_SIZE;  
  DMA_ClearITPendingBit(DMA1_IT_GL7);
  Usart2_Dma_SendFlag = 1;
} 

void Usart2SendData_DMA(uint8_t *pdata, uint16_t Length)  
{  
    while(Usart2_Dma_SendFlag == 0);  
    Usart2_Dma_SendFlag = 0;  
    DMA1_Channel7->CMAR = (uint32_t)pdata;  
    DMA1_Channel7->CNDTR = Length;  
    DMA_Cmd(DMA1_Channel7, ENABLE);  
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);  
}  

void IntegrationData(uint8_t *pdata, uint16_t Length){
	if(pdata[0]==0x55 && pdata[1]==0xAA && pdata[11]==0xAA && Length == 12){
		for(int i = 0;i < 9;i ++){
			Sensor_Info.RaspberryArray[i] = pdata[2+i];
		}
	}
}


void SendToFlight(void){		
	u8 dataToFlight[16];
	float_union height;
	int i;
	dataToFlight[0]=0X55;
	dataToFlight[1]=0XAA;
	for(i = 2;i < 11;i ++){
		dataToFlight[i] = Sensor_Info.RaspberryArray[i-2];
	}
	height.fvalue = Sensor_Info.US100_Zaxis;
	dataToFlight[11] = height.cv[0];
	dataToFlight[12] = height.cv[1];
	dataToFlight[13] = height.cv[2];
	dataToFlight[14] = height.cv[3];
	
	dataToFlight[15]=0XAA;
	                                      
	for(i = 0;i < 16;i++){
		Usart2SendBuff[i] = dataToFlight[i];
	}
	Usart2SendData_DMA(Usart2SendBuff,16);
}


void SendHeightToFlight(void){		
	u8 dataToFlight[16];
	float_union height;
	int i;
	dataToFlight[0]=0X55;
	dataToFlight[1]=0XAA;
	dataToFlight[2]=0X00;
	for(i = 3;i < 11;i ++){
		dataToFlight[i] = 0;
	}
	height.fvalue = Sensor_Info.US100_Zaxis;
	dataToFlight[11] = height.cv[0];
	dataToFlight[12] = height.cv[1];
	dataToFlight[13] = height.cv[2];
	dataToFlight[14] = height.cv[3];
	
	dataToFlight[15]=0XAA;
	
	for(i = 0;i < 16;i++){
		Usart2SendBuff[i] = dataToFlight[i];
	}
	Usart2SendData_DMA(Usart2SendBuff,16);
}

