#include "USART3.h"

uint8_t Usart3SendBuff[USART3SENDBUFF_SIZE];
uint8_t Usart3RecBuff[USART3RECBUFF_SIZE];
uint8_t Usart3_Dma_SendFlag = 1;

void USART3_Config(unsigned int baudrate)
{
    USART_InitTypeDef USART_InitStruct;  
    GPIO_InitTypeDef GPIO_InitStructure;  
    NVIC_InitTypeDef NVIC_InitStructure;  
    DMA_InitTypeDef DMA_InitStructure;  
    //open clk  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);      
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  
  
    /* Configure USART Tx and Rx as alternate function push-pull */  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
    GPIO_Init(GPIOB, &GPIO_InitStructure);   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  
    GPIO_Init(GPIOB, &GPIO_InitStructure);  
  
    // Enable the USARTx Interrupt   
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure);  
  
    USART_InitStruct.USART_BaudRate = baudrate;  
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;  
    USART_InitStruct.USART_StopBits = USART_StopBits_1;  
    USART_InitStruct.USART_Parity = USART_Parity_No ;  
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    USART_Init(USART3, &USART_InitStruct);        
  
    DMA_DeInit(DMA1_Channel3);  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR); 
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Usart3RecBuff;  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;       
    DMA_InitStructure.DMA_BufferSize = USART3RECBUFF_SIZE;        
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;         
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;      
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;    
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;               
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;             
    DMA_Init(DMA1_Channel3,&DMA_InitStructure);  
    DMA_Cmd(DMA1_Channel3,ENABLE); 
    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);        
  
  
    DMA_DeInit(DMA1_Channel2);   
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR); 
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Usart3SendBuff;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  
    DMA_InitStructure.DMA_BufferSize = USART3SENDBUFF_SIZE;  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);  
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);  
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TE, ENABLE);  
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);  
    DMA_Cmd(DMA1_Channel2, DISABLE);  
  
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure);  
  
    USART_Cmd(USART3, ENABLE);   
    USART_ClearFlag(USART3,USART_FLAG_TC);  
}

u8 RaspberryBuff[64];
uint16_t RaspberryLen = 0;
unsigned char RaspberryFinish = 0;
unsigned char US100_Raspberry = 0;
void USART3_IRQHandler(void)  
{ 	
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)   
	{  
		USART3->SR;   
		USART3->DR;
		DMA_Cmd(DMA1_Channel3,DISABLE);  
		RaspberryLen = USART3RECBUFF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel3);
		memcpy(RaspberryBuff,Usart3RecBuff,RaspberryLen);
		if(Mode_Select == IntegrationMode){
			RaspberryFreqflag = 0;
			IntegrationData(RaspberryBuff,RaspberryLen);
		}
		if(RaspberryBuff[0] == 0x5A && RaspberryBuff[1] == 0xA5 
			&& RaspberryBuff[2] == 0xA5 && RaspberryBuff[3] == 0x5A
				&& RaspberryBuff[4] == 0x5A && RaspberryBuff[5] == 0xA5 && RaspberryLen == 6){
			US100_Raspberry = 1;
		}
		RaspberryFinish = 1;
		DMA_SetCurrDataCounter(DMA1_Channel3,USART3RECBUFF_SIZE);   
		DMA_Cmd(DMA1_Channel3,ENABLE); 
	}   
}  

void DMA1_Channel2_IRQHandler(void)  
{  
  DMA_Cmd(DMA1_Channel2, DISABLE);
  USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);  
  DMA1_Channel2->CNDTR = USART3SENDBUFF_SIZE;  
  DMA_ClearITPendingBit(DMA1_IT_GL2);
  Usart3_Dma_SendFlag = 1; 
}  

void Usart3SendData_DMA(uint8_t *pdata, uint16_t Length)  
{  
    while(Usart3_Dma_SendFlag == 0);  
    Usart3_Dma_SendFlag = 0;  
    DMA1_Channel2->CMAR = (uint32_t)pdata;  
    DMA1_Channel2->CNDTR = Length;  
    DMA_Cmd(DMA1_Channel2, ENABLE);  
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);  
} 


