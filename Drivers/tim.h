/*
 * tim.h
 *
 *  Created on: 2021-7-21
 *      Author: chenq
 */

#ifndef TIM_H_
#define TIM_H_
#include "system_init.h"

//��ʱ��ʱ��Դѡ��SCLK ��Ԥ��Ƶ23+1=24��Ƶ 120M��Ƶ       ������Ϊ50000 �� ��ʱ10ms��һ���ж�
void BASIC_TIMER_Config(BTIM_SFRmap* BTIMx, InterruptIndex Peripheral,uint16_t period);

#endif /* TIM_H_ */
