/*
 * task_manager.h
 *
 *  Created on: 2021-6-17
 *      Author: shuai
 */

#ifndef TASK_MANAGER_H_
#define TASK_MANAGER_H_
#include "system_init.h"
/* RTOS系统运行所需头文件 */
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/* 外设驱动所需头文件 */
#include <stdio.h>
#include "usart.h"
/*任务头文件*/
#include "can_task.h"
#include "aebs_task.h"
#include "uart_task.h"
#include "data_center_services.h"

/*
typedef struct
{
	uint8_t	Millimeter_wave_radar_state;				//毫米波雷达状态，在线1，不在线0
	uint8_t	warning_state;						// 报警状态，有报警1，无报警0
	uint8_t Vehicle_can_state;					//整车can状态，在线1，不在线0
	uint8_t Camera_can_state;					//摄像头can状态，在线1，不在线0
	uint8_t Vehicle_speed_state;				//车速
	uint8_t net_state;							//4G状态，正常为1，不正常为0
	uint8_t	Ultrasonic_radar_can_state;			//超声波雷达can状态，在线1，不在线0
}_LED_STATE;
*/

//任务句柄
TaskHandle_t DataCenterTask_Handler;
TaskHandle_t LEDTask_Handler;
TaskHandle_t UART2Task_Handler;
TaskHandle_t CAN1_Txtask_Handler;
TaskHandle_t CAN1_Rxtask_Handler;
TaskHandle_t AEBS_Decision_Task_Handler;
TaskHandle_t AEBS_Control_Task_Handler;
TaskHandle_t CAN1_Analysis_Handler;
TaskHandle_t Bluetooth_task_Handler;


/*-------------------LED任务定义-------------------*/
static void LED_task(void *pvParameters);

/**
  * 描述  RTOS任务初始化
  * 输入  无。
  * 返回  无。
*/
void TaskInit(void);


#endif /* TASK_MANAGER_H_ */
