#include "USART1.h"

uint8_t Usart1SendBuff[USART1SENDBUFF_SIZE];
uint8_t Usart1RecBuff[USART1RECBUFF_SIZE];
uint8_t Usart1_Dma_SendFlag = 1;

/*
	ʹ�ô���2�Գ�����ģ���ȡ����ֵ
	
*/

void USART1_Config(unsigned int baudrate)
{
    USART_InitTypeDef USART_InitStruct;  
    GPIO_InitTypeDef GPIO_InitStructure;  
    NVIC_InitTypeDef NVIC_InitStructure;  
    DMA_InitTypeDef DMA_InitStructure;  
    // ����ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);      
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  
  
    // ����UART1 ��GPIO��ʼ��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
  
    // NVIC����  
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure);  
	
	// ����1��ʼ��		// ������9600������λ��λ��һλֹͣλ������Ҫ��żУ�飬����ҪӲ������
    USART_InitStruct.USART_BaudRate = baudrate;  
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;  
    USART_InitStruct.USART_StopBits = USART_StopBits_1;  
    USART_InitStruct.USART_Parity = USART_Parity_No ;  
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  // �������պͷ����ж�
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
    USART_Init(USART1, &USART_InitStruct);        
  
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);	// ����������·����ж�
	
    // DMAͨ��5���ã�����Ϊ����1�Ľ���DMAת��
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Usart1RecBuff;  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;     
	
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;       //	ת�˷����ɴ�����Usart1RecBuffת��
    DMA_InitStructure.DMA_BufferSize = USART1RECBUFF_SIZE;        
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;            // ������������Զ���װ
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;			 // Ӳ������ʹ��
	DMA_Init(DMA1_Channel5,&DMA_InitStructure);
    DMA_Cmd(DMA1_Channel5,ENABLE);
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);

    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Usart1SendBuff;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  	// ת�˷�����Usart1SendBuff�򴮿�ת��
    DMA_InitStructure.DMA_BufferSize = USART1SENDBUFF_SIZE;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  			// ģʽ��ѡ������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  			// ʹ��Ӳ������
	
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);  		// ʹ�ܴ�������ж�
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TE, ENABLE);  		// ʹ�ܴ�������ж�
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE); 			// ��������1�ķ���DMA 
    DMA_Cmd(DMA1_Channel4, DISABLE);
  
	// ����DMA��NVIC������
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	// 
    USART_Cmd(USART1, ENABLE);
    USART_ClearFlag(USART1,USART_FLAG_TC);	// �����־λ��������ɱ�־
}

extern Butter_Parameter US100_Parameter;
extern Butter_BufferData  US100_filter_buf[3];
uint16_t ReceiveHeight;
u8 US100buf[2];

void USART1_IRQHandler(void)  
{  
	static int HeightData[5];
	uint16_t len = 0;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{ 
		// �ȶ�SR,�ٶ�DR���ɶ�IDLE��־λ��λ
		USART1->SR;
		USART1->DR;
		
		DMA_Cmd(DMA1_Channel5,DISABLE);
		//DMA_GetCurrDataCounter ���ص�ǰ DMA ͨ�������ݴ���ʣ���ֽ���
		len = USART1RECBUFF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5);
		memcpy(US100buf,Usart1RecBuff,len);
		ReceiveHeight = ((((uint16_t)US100buf[0])<<8) + US100buf[1]);
		
		if(ReceiveHeight<=3000 && ReceiveHeight>0.0f)
		Sensor_Info.US100_Zaxis = Median_Filter(ReceiveHeight,5,HeightData)/1000;	// ��λֵ�˲�
//		Sensor_Info.US100_Zaxis = LPButterworth(ReceiveHeight,&US100_filter_buf[0],&US100_Parameter);
//		Sensor_Info.US100_Zaxis = Median_Filter(LPButterworth(ReceiveHeight,&US100_filter_buf[0],&US100_Parameter),5,HeightData)/1000;	
//		Sensor_Info.US100_Zaxis = (float)ReceiveHeight / 1000.0f;
		DMA_SetCurrDataCounter(DMA1_Channel5,USART1RECBUFF_SIZE);	// ���ô������������������DMA5�Ĵ���1����ת��
		DMA_Cmd(DMA1_Channel5,ENABLE); 
	}
}  

void DMA1_Channel4_IRQHandler(void) 	// ��������жϺʹ�������ж�
{  
	DMA_Cmd(DMA1_Channel4, DISABLE);
	USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);		// �ر�DMA4�Ĵ���1����ת��
	DMA1_Channel4->CNDTR = USART1SENDBUFF_SIZE;  // ���ô����������DMA4�Ĵ���1����ת��
	DMA_ClearITPendingBit(DMA1_IT_GL4);		// ��� DMA1 ͨ��4 ��ȫ���ж�
	Usart1_Dma_SendFlag = 1; 
}  

void Usart1SendData_DMA(uint8_t *pdata, uint16_t Length)  
{  
    while(Usart1_Dma_SendFlag == 0);	// �����ȴ�
    Usart1_Dma_SendFlag = 0;  
    DMA1_Channel4->CMAR = (uint32_t)pdata;  // �洢����ַ
    DMA1_Channel4->CNDTR = Length;  		// ���������Ĵ���
    DMA_Cmd(DMA1_Channel4, ENABLE);			// ʹ��DMA1 ��ͨ��4����Ϊ����ת��
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  // ����DMA4�Ĵ���1����ת��
} 


void RecUltraData(void){
	Usart1SendBuff[0] = 0x55;
	Usart1SendData_DMA(Usart1SendBuff,USART1SENDBUFF_SIZE);
}




