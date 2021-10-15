/*
 * task_manager.c
 *
 *  Created on: 2021-6-17
 *      Author: shuai
 */
#include "task_manager.h"
#include "usart.h"
#include "canhl.h"
#include "bluetooth.h"
#include "common.h"
#include "gpio.h"

//_LED_STATE led_state;
extern uint32_t Time14_CNT;

void GPIO_Init(void)
{
	GPIO_Write_Mode_Bits(GPIOH_SFR, GPIO_PIN_MASK_12, GPIO_MODE_IN);
	GPIO_Write_Mode_Bits(GPIOE_SFR, GPIO_PIN_MASK_6, GPIO_MODE_OUT);
	GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_0, GPIO_MODE_OUT);
	GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_1, GPIO_MODE_OUT);
	GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_2, GPIO_MODE_OUT);
	GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_3, GPIO_MODE_OUT);
	GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_4, GPIO_MODE_OUT);
	GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_7, GPIO_MODE_OUT);

	GPIO_State();
}

void Led_Control(void)
{
	//摄像头can工作状态
	Set_LED7(stCanCommSta.stCamera.status);
	//雷达
	Set_LED6(stCanCommSta.stRadar.status);
	//车can
	Set_LED5(stCanCommSta.stVehicle.status);
	//超声波
	Set_LED4(stCanCommSta.stHRadar.status);
	//车速
	Set_LED3(stCanCommSta.stSpeed.status);
	//无线
	Set_LED2(stCanCommSta.stWireless.status);
	//报警
	Set_LED1(stCanCommSta.stWarning.status);
}

/*-------------------LED任务定义-------------------*/
static void LED_task(void *pvParameters)
{
	int count = 0;
	GPIO_Write_Mode_Bits(GPIOE_SFR, GPIO_PIN_MASK_6, GPIO_MODE_OUT);
	GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_0, GPIO_MODE_OUT);
	GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_1, GPIO_MODE_OUT);
	GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_2, GPIO_MODE_OUT);
	GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_3, GPIO_MODE_OUT);
	GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_4, GPIO_MODE_OUT);
	GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_7, GPIO_MODE_OUT);
	while (1)
	{
//		IWDT_Feed_The_Dog();//喂狗系统正常工作，没喂狗时则1秒种系统复位
//		CAN_check_busoff(CAN1_SFR);
		count ++;
		switch(count){
		case 1:
			GPIO_Toggle_Output_Data_Config(GPIOE_SFR, GPIO_PIN_MASK_6);
			break;
		case 2:
			GPIO_Toggle_Output_Data_Config(GPIOF_SFR, GPIO_PIN_MASK_0);
			break;
		case 3:
			GPIO_Toggle_Output_Data_Config(GPIOF_SFR, GPIO_PIN_MASK_1);
			break;
		case 4:
			GPIO_Toggle_Output_Data_Config(GPIOF_SFR, GPIO_PIN_MASK_2);
			break;
		case 5:
			GPIO_Toggle_Output_Data_Config(GPIOF_SFR, GPIO_PIN_MASK_3);
			break;
		case 6:
			GPIO_Toggle_Output_Data_Config(GPIOF_SFR, GPIO_PIN_MASK_4);
			break;
		case 7:
			GPIO_Toggle_Output_Data_Config(GPIOF_SFR, GPIO_PIN_MASK_7);
			break;
		default:
			count = 0;
			break;
		}

		vTaskDelay(500);
	}

}

/*------------------串口任务定义----------------------*/
static void Bluetooth_task(void *pvParameters)
{
	//Bluetooth_set_cmd("AT+NAMEkongfu32zs\r\n",22);
	//vTaskDelay(100);
	//Bluetooth_set_cmd("AT+RESET\r\n",10);
	//Bluetooth_set_baud(9600);
	//Bluetooth_set_cmd("AT+RESET\r\n",10);
    int countSend = 0;
    uint8_t tempBuffer[64] = {0};
    uint8_t echoFlag = 0;
	while (1)
	{
		countSend++;
		if (countSend > 100)
		{
			countSend = 0;
//			Bluetooth_send_data("Bluetooth is running.\r\n",23);
		}
		if(echoFlag == 1)
		{
			Bluetooth_send_data(tempBuffer, 64);
			echoFlag = 0;
		}

		if(Bluetooth_Rxcount > 0 && Bluetooth_Receive_flag == 1)
		{
			for(uint8_t i = 0; i < 64; i++)
			{
				tempBuffer[i]=0;
			}
			Bluetooth_get_data(tempBuffer, 64);
			echoFlag = 1;
		}

		vTaskDelay(10);
	}

}

void Bluetooth_Send_Data(void)
{
	static uint32_t OldSendtime;
    static int countSend = 0;
    uint8_t tempBuffer[64] = {0};

    static uint8_t echoFlag = 0;
	if(SystemtimeClock - OldSendtime >= 10)
	{
		countSend++;
		if (countSend > 100)
		{
			countSend = 0;
//			Bluetooth_send_data("Bluetooth is running.\r\n",23);
		}
		if(echoFlag == 1)
		{
			Bluetooth_send_data(tempBuffer, 64);
			echoFlag = 0;
		}

		if(Bluetooth_Rxcount > 0 && Bluetooth_Receive_flag == 1)
		{
			for(uint8_t i = 0; i < 64; i++)
			{
				tempBuffer[i]=0;
			}
			Bluetooth_get_data(tempBuffer, 64);
			echoFlag = 1;
		}

	}
}
/**
  * 描述  RTOS任务初始化
  * 输入  无。
  * 返回  无。
*/
void TaskInit(void)
{
	Data_Center_Init();
	CAN1_Task_Init();
	AEBS_Task_Init();
}
/*
//	fprintf(USART1_STREAM, "System start.\r\n");
	CAN1_Rxqueue_Handler = xQueueCreate(10, sizeof(Camera_All_Info));
	if(CAN1_Rxqueue_Handler == NULL)
	{
		fprintf(USART1_STREAM, "CAN1_Rxqueue_Handler xQueueCreate fail.\r\n");
	}

	CAN1_Analysis_Queue = xQueueCreate(20, sizeof(struct can_frame));
	if(CAN1_Analysis_Queue == NULL)
	{
		fprintf(USART1_STREAM, "CAN1_Analysis_Queue xQueueCreate fail.\r\n");
	}

	//创建任务
	xTaskCreate((TaskFunction_t )Data_Center_Task,
				(const char*    )"Data_Center",
				(uint16_t       )512,
				(void*          )NULL,
				(UBaseType_t    )12,
				(TaskHandle_t*  )&DataCenterTask_Handler);

	xTaskCreate((TaskFunction_t )LED_task,
				(const char*    )"LED",
				(uint16_t       )64,
				(void*          )NULL,
				(UBaseType_t    )11,
				(TaskHandle_t*  )&LEDTask_Handler);

	xTaskCreate((TaskFunction_t )UART_Task,
				(const char*    )"UART",
				(uint16_t       )configMINIMAL_STACK_SIZE,
				(void*          )NULL,
				(UBaseType_t    )10,
				(TaskHandle_t*  )&UART2Task_Handler);

	xTaskCreate((TaskFunction_t)CAN1_Txtask,
			    (const char*    )"CAN1_Tx",
			    (uint16_t       )128,
			    (void*          )NULL,
			    (UBaseType_t    )6,
			    (TaskHandle_t*  )&CAN1_Txtask_Handler);

	xTaskCreate((TaskFunction_t)CAN1_Rxtask,
			    (const char*    )"CAN1_Rx",
			    (uint16_t       )128,
			    (void*          )NULL,
			    (UBaseType_t    )5,
			    (TaskHandle_t*  )&CAN1_Rxtask_Handler);

	xTaskCreate((TaskFunction_t)AEBS_Decision_Task,
			    (const char*    )"AEBS_Decision",
			    (uint16_t       )512,
			    (void*          )NULL,
			    (UBaseType_t    )7,
			    (TaskHandle_t*  )&AEBS_Decision_Task_Handler);

	xTaskCreate((TaskFunction_t)AEBS_Control_Task,
			    (const char*    )"AEBS_Control",
			    (uint16_t       )configMINIMAL_STACK_SIZE,
			    (void*          )NULL,
			    (UBaseType_t    )8,
			    (TaskHandle_t*  )&AEBS_Control_Task_Handler);

	xTaskCreate((TaskFunction_t)CAN1_Analysis_Task,
			    (const char*    )"CAN1_Analisis",
			    (uint16_t       )512,
			    (void*          )NULL,
			    (UBaseType_t    )9,
			    (TaskHandle_t*  )&CAN1_Analysis_Handler);

	xTaskCreate((TaskFunction_t )Bluetooth_task,
				(const char*    )"Bluetooth",
				(uint16_t       )64,
				(void*          )NULL,
				(UBaseType_t    )4,
				(TaskHandle_t*  )&Bluetooth_task_Handler);

}
*/
