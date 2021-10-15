/*
 * aebs_task.c
 *
 *  Created on: 2021-6-24
 *      Author: shuai
 */
#include "aebs_task.h"

Decision_Control_Information aebsControlInfo;
SemaphoreHandle_t aebsMutex = NULL;

void AEBS_Task_Init()
{
	aebsControlInfo.vehicle_speed = 0.0;
	aebsControlInfo.obstacle_ttc = 6.3;
	//aebsMutex = xSemaphoreCreateRecursiveMutex();
}

void AEBS_Decision_Task(void *pvParameters)
{
	Camera_All_Info cameraInfoMessage;
	for( ;; )
	{
		while(xQueueReceive(CAN1_Rxqueue_Handler, ( void * )&cameraInfoMessage, portMAX_DELAY ) == pdPASS )
		{
			if( xSemaphoreTakeRecursive( aebsMutex, ( TickType_t ) 10 ) == pdTRUE )
			{
				aebsControlInfo.obstacle_ttc = cameraInfoMessage.ttc;
				xSemaphoreGiveRecursive( aebsMutex );
			}
//			UART_Puts("AEBS_Decision_Task is running. \r\n");
		}
		vTaskDelay(1);
	}
}

void AEBS_Control_Task(void *pvParameters)
{
	int count = 0;
	while(1)
	{
		if( xSemaphoreTakeRecursive( aebsMutex, ( TickType_t ) 10 ) == pdTRUE )
		{
			Valve_Braking_Force_Cal(aebsControlInfo.obstacle_ttc);
			xSemaphoreGiveRecursive( aebsMutex );
		}
		vTaskDelay(20);
	}
}

