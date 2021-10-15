/*
 * task_manager.h
 *
 *  Created on: 2021-6-17
 *      Author: shuai
 */

#ifndef TASK_MANAGER_H_
#define TASK_MANAGER_H_
#include "system_init.h"
/* RTOSϵͳ��������ͷ�ļ� */
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/* ������������ͷ�ļ� */
#include <stdio.h>
#include "usart.h"
/*����ͷ�ļ�*/
#include "can_task.h"
#include "aebs_task.h"
#include "uart_task.h"
#include "data_center_services.h"

/*
typedef struct
{
	uint8_t	Millimeter_wave_radar_state;				//���ײ��״�״̬������1��������0
	uint8_t	warning_state;						// ����״̬���б���1���ޱ���0
	uint8_t Vehicle_can_state;					//����can״̬������1��������0
	uint8_t Camera_can_state;					//����ͷcan״̬������1��������0
	uint8_t Vehicle_speed_state;				//����
	uint8_t net_state;							//4G״̬������Ϊ1��������Ϊ0
	uint8_t	Ultrasonic_radar_can_state;			//�������״�can״̬������1��������0
}_LED_STATE;
*/

//������
TaskHandle_t DataCenterTask_Handler;
TaskHandle_t LEDTask_Handler;
TaskHandle_t UART2Task_Handler;
TaskHandle_t CAN1_Txtask_Handler;
TaskHandle_t CAN1_Rxtask_Handler;
TaskHandle_t AEBS_Decision_Task_Handler;
TaskHandle_t AEBS_Control_Task_Handler;
TaskHandle_t CAN1_Analysis_Handler;
TaskHandle_t Bluetooth_task_Handler;


/*-------------------LED������-------------------*/
static void LED_task(void *pvParameters);

/**
  * ����  RTOS�����ʼ��
  * ����  �ޡ�
  * ����  �ޡ�
*/
void TaskInit(void);


#endif /* TASK_MANAGER_H_ */
