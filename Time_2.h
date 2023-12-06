#ifndef TIME_2_H
#define TIME_2_H

#include "stm32f10x.h"
 

void TIM2_Configuration(void);

extern volatile u8 SendToFlightFreqflag;

extern volatile u8 RecUltraFreqflag;

extern volatile u8 RaspberryFreqflag;

#endif	/* TIME_TEST_H */
