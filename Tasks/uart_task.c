/*
 * uart_task.c
 *
 *  Created on: 2021-6-28
 *      Author: shuai
 */
#include "uart_task.h"

SemaphoreHandle_t xMutex = NULL;
char Usart4_recv_buff[4096];
uint32_t Usart4_recv_len = 0;

void UART_Task_Init()
{
	//xMutex = xSemaphoreCreateRecursiveMutex();
	User_USART_Init(115200);
}
/*------------------串口任务定义----------------------*/
void UART_Task(void *pvParameters)
{
	while (1)
	{
		if(User_Receive_flag==1){
			UART_Puts("\r\nUART1 GET COMMAND\r\n");
			User_Rxcount=0;
			User_Receive_flag=0;
		}else{
			vTaskDelay(100);
		}
	}
}

void UART_Puts(uint8_t *strLog)
{
	if( xMutex == NULL )
		return;

	if( xSemaphoreTakeRecursive( xMutex, ( TickType_t ) 10 ) == pdTRUE )
	{
		USART_Send(USART1_SFR, strLog, strlen(strLog));
		xSemaphoreGiveRecursive( xMutex );
	}
}

void Usart4_Receive_data(uint8_t data)
{

}

