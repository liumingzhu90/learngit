/*
 * uart_task.h
 *
 *  Created on: 2021-6-28
 *      Author: shuai
 */

#ifndef UART_TASK_H_
#define UART_TASK_H_
#include "system_init.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

extern SemaphoreHandle_t xMutex;
extern char Usart4_recv_buff[4096];
extern uint32_t Usart4_recv_len;

void UART_Task_Init();
/*------------------串口任务定义----------------------*/
void UART_Task(void *pvParameters);
void UART_Puts(uint8_t *strLog);

#endif /* USART_TASK_H_ */
