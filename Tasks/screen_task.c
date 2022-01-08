/*
 * screen_task.c
 *
 *  Created on: 2021-6-21
 *      Author: chenq
 */

#include "screen_task.h"
#include "screen.h"

void screen_task(void *pvParameters)
{

	Screen_Init(CAN0_SFR,CAN_BAUDRATE_500K);
	//user code

	while(1){
		//user code
		//Screen_Set_Speed(20);
		//
		Screen_CAN_Send();
		vTaskDelay(20);
	}
}
