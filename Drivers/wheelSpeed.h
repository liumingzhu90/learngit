/*
 * wheelSpeed.h
 *
 *  Created on: 2021-11-30
 *      Author: Administrator
 */

#ifndef WHEELSPEED_H_
#define WHEELSPEED_H_
#include "system_init.h"

void 		WheelSpeed_Init();
uint16_t 	Calc_Bus_Encoder_Velocity();
uint16_t 	Get_Wheel_Speed();
#endif /* WHEELSPEED_H_ */
