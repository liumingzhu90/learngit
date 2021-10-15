/*
 * gpio.h
 *
 *  Created on: 2021-7-7
 *      Author: chenq
 */

#ifndef GPIO_H_
#define GPIO_H_
#include "system_init.h"

void GPIO_Out_Config(void);
void GPIO_Singal_init(void);
void Set_LED1(BitAction BitsValue);
void Set_LED2(BitAction BitsValue);
void Set_LED3(BitAction BitsValue);
void Set_LED4(BitAction BitsValue);
void Set_LED5(BitAction BitsValue);
void Set_LED6(BitAction BitsValue);
void Set_LED7(BitAction BitsValue);
BitAction Read_stop_signal(void);
BitAction Read_turnleft_signal(void);
BitAction Read_turnright_signal(void);
BitAction Read_turnback_signal(void);
BitAction Read_AEB_switch();
BitAction Read_LDW_switch();
void GPIO_Power_CANFA_Control(void);
#endif /* GPIO_H_ */
