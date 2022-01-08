/*
 * aebs_task.h
 *
 *  Created on: 2021-6-24
 *      Author: shuai
 */

#ifndef AEBS_TASK_H_
#define AEBS_TASK_H_
#include "system_init.h"
#include "canhl.h"
#include "usart.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "proportional_valve.h"
#include "stereo_camera.h"
#include "uart_task.h"

typedef struct
{
	float vehicle_speed;
	float obstacle_ttc;
}Decision_Control_Information;

void AEBS_Task_Init();
void AEBS_Decision_Task(void *pvParameters);
void AEBS_Control_Task(void *pvParameters);

#endif /* AEBS_TASK_H_ */
