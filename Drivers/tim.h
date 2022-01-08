/*
 * tim.h
 *
 *  Created on: 2021-7-21
 *      Author: chenq
 */

#ifndef TIM_H_
#define TIM_H_
#include "system_init.h"

//定时器时钟源选用SCLK 设预分频23+1=24分频 120M主频       设周期为50000 则 定时10ms进一次中断
void BASIC_TIMER_Config(BTIM_SFRmap* BTIMx, InterruptIndex Peripheral,uint16_t period);

#endif /* TIM_H_ */
