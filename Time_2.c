#include "Time_2.h"

/*TIM_Period--1000   TIM_Prescaler--71 -->中断周期为1ms*/
void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 	
		//open clk 	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
    
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_DeInit(TIM2);
    TIM_TimeBaseStructure.TIM_Period= 1000 - 1;		/* 自动重装载寄存器周期的值(计数值) */
    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
    TIM_TimeBaseStructure.TIM_Prescaler= 72 - 1 ;				/* 时钟预分频数 72M/72 */
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		/* 采样分频 */
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;   /* 向上计数模式 */
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);						/* 清除溢出中断标志 */
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
    TIM_Cmd(TIM2, ENABLE);										/* 开启时钟 */
	// 中断周期为 1ms
}



volatile u32 SendToFlightFreq = 0; // ms 计时变量
volatile u8 SendToFlightFreqflag = 0;

volatile u32 RecUltraFreq = 0;
volatile u8 RecUltraFreqflag = 0;

volatile u32 RaspberryFreq = 0;
volatile u8 RaspberryFreqflag = 0;


// 定时器二中断函数
void TIM2_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET )
	{	
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
		
		SendToFlightFreq ++;
		RecUltraFreq ++;
		RaspberryFreq ++;
		// 60hz upload data
		if(SendToFlightFreq == 16){
			SendToFlightFreq = 0;
			SendToFlightFreqflag = 1;
		}
		// 60hz rec Ultra data
		if(RecUltraFreq == 16){
			RecUltraFreq = 0;
			RecUltraFreqflag = 1;
		}
//		if(RaspberryFreq == 100){
//			RaspberryFreq = 0;
//			RaspberryFreqflag++;
//		}
	}		 	
}
