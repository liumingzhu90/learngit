/*
 * can_task.c
 *
 *  Created on: 2021-6-17
 *      Author: shuai
 */
#include "can_task.h"
#include "stdio.h"
#include "canhl.h"
#include "usart.h"
#include "task_manager.h"
#include "vehicle_can.h"
#include "common.h"
#include "gpio.h"
#include "flash.h"
#include "system_init.h"

extern short speed_test;

uint8_t 	SN[21] = {"1.0.0.S1C4"};
uint8_t		SN_Display[3];

QueueHandle_t CAN1_Analysis_Queue = NULL;
static _STRUCT_VEHICLE_MESSAGE Proportional_valve_status;
static _STRUCT_VEHICLE_MESSAGE Proportional_valve_fault;
__SET_PARA parameterset;
struct can_frame  stVehicleCanData;
struct can_frame  stVehicleCanSpeedData;
struct can_frame  stVehicleCanOdomData;		// 金旅大巴里程计
struct can_frame  stVehicleCanTurnData;
struct can_frame  stVehicleCanTurnData1;
struct can_frame  stVehicleCanBreakData;
struct can_frame	stVehicleCanAbsData;
struct can_frame	stVehicleCanWarningData;
struct can_frame	stVehicleCanSWAData;
//struct can_frame	stVehicleCanAbsData;
struct can_frame	stCammeraCanData;
struct can_frame	stUraderCanData[4];		//超声波雷达
static _STRUCT_VALVE_PARA    stValveParas;//比例阀反馈数据
_STRUCT_CAN_COMM_STA 	stCanCommSta;
extern uint32_t Time14_CNT;
//extern _LED_STATE led_state;

static uint8_t ucFlagSpeedJumpErr = 0;   /* 记录车速跳变 */
static uint8_t ucFlagSpeedErr = 0;       /* 记录车速异常 */
static uint8_t ucFlagBrakePedalErr = 0;  /* 记录刹车踏板异常 */
extern void Camera_CAN_Analysis(struct can_frame *rx_frame);
void CAN1_Task_Init()
{
	Valve_Init();
	Camera_Init();
	Displayer_Init();
}

void CAN1_Txtask(void *pvParameters)
{
	struct can_frame tx_frame_valve;
	struct can_frame tx_frame_displayer;
	int erro_count = 0;
	TickType_t xLastWakeTime;

	xLastWakeTime= xTaskGetTickCount();
	while (1)
	{
		vTaskDelayUntil(&xLastWakeTime,20);
		if(Valve_Pressure_Get(&tx_frame_valve))
		{
			if(!CAN_Transmit_DATA(CAN1_SFR,tx_frame_valve)){
//				fprintf(USART2_STREAM, "CAN1 send data 0x%8x successfully.\r\n", tx_frame_valve.TargetID);
			}else{
				erro_count++;
//				fprintf(USART2_STREAM, "%dERROR: CAN1 send ID 0x%8x faild!!!!!!\r\n", erro_count, tx_frame_valve.TargetID);
			}
		}
		vTaskDelay(1);
		if(Displayer_CANTx_Get(&tx_frame_displayer))
		{
			if(!CAN_Transmit_DATA(CAN1_SFR,tx_frame_displayer)){
//				fprintf(USART2_STREAM, "CAN1 send data 0x%8x successfully.\r\n", tx_frame_displayer.TargetID);
			}else{
				erro_count++;
//				fprintf(USART2_STREAM, "%dERROR: CAN1 send ID 0x%8x faild!!!!!!\r\n", erro_count, tx_frame_displayer.TargetID);
			}
		}
	}
}

void CAN1_Rxtask(void *pvParameters)
{
	uint8_t CAN1_GET_COUNT=0;
	while (1)
	{
		while(CAN1_GET_COUNT!=CAN1_RX_COUNT)
		{
			//BaseType_t ret =xQueueSend( CAN1_Analysis_Queue, &can1_rx_frame[CAN1_GET_COUNT], 1);
			//if(ret == pdFALSE)
			{
//				UART_Puts("CAN1_Analysis_Queue xQueueSend fail.\r\n");
			}
			if(CAN1_GET_COUNT<(CAN_BUFFER_MAX-1))
				CAN1_GET_COUNT++;
			else
				CAN1_GET_COUNT=0;
		}
		vTaskDelay(2);
	}
}

void CAN1_Analysis_Task(void *pvParameters)
{
	struct can_frame rx_frame;
	for( ;; )
	{
		while(xQueueReceive(CAN1_Analysis_Queue, ( void * )&rx_frame, portMAX_DELAY ) == pdPASS )
		{
			Valve_CAN_Analysis(&rx_frame);
			Camera_CAN_Analysis(&rx_frame);
			//fprintf(USART2_STREAM, "CAN1 receive data 0x%8x\r\n",rx_frame.TargetID);
		}
		vTaskDelay(10);
	}
}

void Camera_Can_Data(struct can_frame *rx_frame)
{
	if(rx_frame->flgBoxRxEnd == 1)
	{
		rx_frame->flgBoxRxEnd = 0;
		//stCanCommSta.stCamera.oldSysTime = SystemtimeClock;
		Set_Camera_Stamp(SystemtimeClock);
		Camera_CAN_Analysis(rx_frame);
	}
}

/*
 * 摄像头数据解析
 * */
void can0_receive(struct can_frame rx_frame)
{
	uint8_t i;

	if((rx_frame.TargetID == 0x79F)|
			(rx_frame.TargetID == 0x7A0)|
			(rx_frame.TargetID == 0x7A1)|
			(rx_frame.TargetID == 0x7A2)|
			(rx_frame.TargetID == 0x7A3)|
			(rx_frame.TargetID == 0x7A4)|
			(rx_frame.TargetID == 0x7A5)|
			(rx_frame.TargetID == 0x7A6)|
			(rx_frame.TargetID == 0x7A7))
	{
		stCammeraCanData.flgBoxRxEnd = 1;
		stCammeraCanData.TargetID = rx_frame.TargetID;
		for(i = 0;i < 8;i ++)
			stCammeraCanData.data[i] = rx_frame.data[i];
		/*if(rx_frame.TargetID == 0x7A0){
			fprintf(USART1_STREAM,"%d\r\n",rx_frame.data[1]);

		}*/


		Camera_Can_Data(&stCammeraCanData);
	}
}

void Ureader_receive(struct can_frame rx_frame)
{
	uint8_t i;

	if((rx_frame.TargetID == 0x700) |
			(rx_frame.TargetID == 0x701) |
			(rx_frame.TargetID == 0x702) |
			(rx_frame.TargetID == 0x703))
	{
		stCanCommSta.stHRadar.oldSysTime = SystemtimeClock;
		if(rx_frame.TargetID == 0x700)
		{
			stUraderCanData[0].flgBoxRxEnd = 1;
			stUraderCanData[0].TargetID = rx_frame.TargetID;
			for(i = 0;i < 8;i ++)
				stUraderCanData[0].data[i] = rx_frame.data[i];
		}
		if(rx_frame.TargetID == 0x701)
		{
			stUraderCanData[1].flgBoxRxEnd = 1;
			stUraderCanData[1].TargetID = rx_frame.TargetID;
			for(i = 0;i < 8;i ++)
				stUraderCanData[1].data[i] = rx_frame.data[i];
		}
		if(rx_frame.TargetID == 0x702)
		{
			stUraderCanData[2].flgBoxRxEnd = 1;
			stUraderCanData[2].TargetID = rx_frame.TargetID;
			for(i = 0;i < 8;i ++)
				stUraderCanData[2].data[i] = rx_frame.data[i];
		}
		if(rx_frame.TargetID == 0x703)
		{
			stUraderCanData[3].flgBoxRxEnd = 1;
			stUraderCanData[3].TargetID = rx_frame.TargetID;
			for(i = 0;i < 8;i ++)
				stUraderCanData[3].data[i] = rx_frame.data[i];
		}

	}
}

void can1_receive(struct can_frame rx_frame)
{
}
/*
 * 函数名称：can2_receive
 * 函数功能：can2收数据
 * 输入：struct can_frame rx_frame
 * 输出：无
 * */
void can2_receive(struct can_frame rx_frame)
{
	uint8_t i;
	if((rx_frame.TargetID == 0x18A70017) |			//金旅转向
			(rx_frame.TargetID == 0x0CFE6CEE) |		//金旅车速
			(rx_frame.TargetID == 0x18FEC1EE) |		//金旅里程计
			(rx_frame.TargetID == 0x1C01F019) |		//BYD
			(rx_frame.TargetID == 0x1C01F020) |
			(rx_frame.TargetID == 0x1C01F021) |
			(rx_frame.TargetID == 0x1C01F022)	|
			(rx_frame.TargetID == 0x1C01F0283)	|
			(rx_frame.TargetID == 0x310)	|		//vv6转向信息
			(rx_frame.TargetID == 0x235)	|		//vv6拐角信息
			(rx_frame.TargetID == 0x240)	|		//vv6车速信息
			(rx_frame.TargetID == 0x310)	|		//vv6刹车信息
			(rx_frame.TargetID == 0x237)
			)
	{
		//stVehicleCanData.flgBoxRxEnd = 1;
		//if(stSpeedPara.lId == stTurnPara.lId)
		if(rx_frame.TargetID == stSpeedPara.lId)
		{
			stVehicleCanSpeedData.flgBoxRxEnd = 1;
			for(i = 0;i < 8;i ++)
				stVehicleCanSpeedData.data[i] = rx_frame.data[i];
		}
		if(rx_frame.TargetID == stVehicleCanOdomData.TargetID)	// 金旅大巴里程计
		{
			stVehicleCanOdomData.flgBoxRxEnd = 1;
			for(i = 0;i < 8;i ++)
				stVehicleCanOdomData.data[i] = rx_frame.data[i];
		}
		if(rx_frame.TargetID == stTurnPara.lId)
		{
			stVehicleCanTurnData.flgBoxRxEnd = 1;
			for(i = 0;i < 8;i ++)
				stVehicleCanTurnData.data[i] = rx_frame.data[i];
		}
		if(rx_frame.TargetID == stTurnPara.lId1)
		{
			stVehicleCanTurnData1.flgBoxRxEnd = 1;
			for(i = 0;i < 8;i ++)
				stVehicleCanTurnData1.data[i] = rx_frame.data[i];
		}
		if(rx_frame.TargetID == stBrakePara.lId)
		{
			stVehicleCanBreakData.flgBoxRxEnd = 1;
			for(i = 0;i < 8;i ++)
				stVehicleCanBreakData.data[i] = rx_frame.data[i];
		}
		if(rx_frame.TargetID == stWarningPara.lId)
		{
			stVehicleCanWarningData.flgBoxRxEnd = 1;
			for(i = 0;i < 8;i ++)
				stVehicleCanWarningData.data[i] = rx_frame.data[i];
		}
		if(rx_frame.TargetID == stSwaPara.lId)
		{
			stVehicleCanSWAData.flgBoxRxEnd = 1;
			for(i = 0;i < 8;i ++)
				stVehicleCanSWAData.data[i] = rx_frame.data[i];
		}
		if(rx_frame.TargetID == stAbsPara.lId)
		{
			stVehicleCanAbsData.flgBoxRxEnd = 1;
			for(i = 0;i < 8;i ++)
				stVehicleCanAbsData.data[i] = rx_frame.data[i];
		}
	}
}

void can0_can2_receive(struct can_frame rx_frame)
{
	can2_receive(rx_frame);
}
void can3_receive(struct can_frame rx_frame)
{
	uint8_t i;

	switch(rx_frame.TargetID)
	{
		case can3message0:
			Set_ValveProportionalComm_StaStamp(SystemtimeClock);
			if(Proportional_valve_status.flgBoxRxEnd == 0)
			{

				for(i = 0;i < 8;i ++)
					Proportional_valve_status.Box[i] = rx_frame.data[i];
				Proportional_valve_status.flgBoxRxEnd = 1;
			}
			break;

		case can3message1:
			Set_ValveProportionalComm_StaStamp(SystemtimeClock);
			if(Proportional_valve_fault.flgBoxRxEnd == 0)
			{

				for(i = 0;i < 8;i ++)
					Proportional_valve_fault.Box[i] = rx_frame.data[i];
				Proportional_valve_fault.flgBoxRxEnd = 1;
			}
			break;

		default:
			break;
	}
}

void Can_Set_Vechicle_Parameter(uint8_t *data)
{
	struct can_frame tx_frame;
	uint32_t ret;

	stSysPara.ucControlMode = data[0];
	stSysPara.ucVehicleWidth = data[1];
	stSysPara.ucDistaceKeep = data[2];
	stSysPara.ttc1 = data[3] * 0.1;
	stSysPara.ttc2 = data[4] * 0.1;
	stSysPara.ucBrakeSense = data[5] & 0xf;
	stSysPara.GearNum = (data[5] >> 4) & 0xf;
	stSysPara.hmw1 = data[6] * 0.1;
	stSysPara.hmw2 = data[7] * 0.1;
	ReWrite_one();
	Reaskc_Set_Commond();
	//parameterset.first = 1;
}

void Can_Get_Vechicle_Parameter(void)
{
	uint32_t wordnum;
	uint8_t i;

	for(i = 0;i < 8;i ++)
		can_recv_valve_control.data[i] = 0xff;

	can_recv_valve_control.TargetID = 0x7c8;
	can_recv_valve_control.data[0] = FLASH_ReadWord(VEHICLE_COTROL_MODE,&wordnum);
	can_recv_valve_control.data[1] = FLASH_ReadWord(VEHICLE_WITH_ADDRESS,&wordnum);
	can_recv_valve_control.data[2] = FLASH_ReadWord(VEHICLE_KEEP_DIS,&wordnum);
	can_recv_valve_control.data[3] = FLASH_ReadWord(VEHICLE_TTC1,&wordnum);
	can_recv_valve_control.data[4] = FLASH_ReadWord(VEHICLE_TTC2,&wordnum);
	can_recv_valve_control.data[5] = FLASH_ReadWord(VEHICLE_SENSITIVITY,&wordnum);
	can_recv_valve_control.data[5] = (FLASH_ReadWord(VEHICLE_GEAR_NUM,&wordnum) << 4) | can_recv_valve_control.data[5];
	can_recv_valve_control.data[6] = FLASH_ReadWord(VEHICLE_HMW1,&wordnum);
	can_recv_valve_control.data[7] = FLASH_ReadWord(VEHICLE_HMW2,&wordnum);
	CAN_Transmit_DATA(CAN4_SFR,can_recv_valve_control);
}

void Can_Set_CanRate_Parameter(uint8_t *data)
{
	uint8_t i;

	stCanPara.can0rate = data[0] & 0x03;
	stCanPara.can1rate = (data[0] >> 2) & 0x03;
	stCanPara.can2rate = (data[0] >> 4) & 0x03;
	stCanPara.can3rate = (data[0] >> 6) & 0x03;
	stCanPara.can4rate = data[1] & 0x03;
	stCanPara.can5rate = (data[0] >> 2) & 0x03;
	ReWrite_one();
	Reaskc_Set_Commond();
	//parameterset.first = 1;
}

void Can_Get_CanRate_Parameter(void)
{
	uint16_t canrata;
	uint8_t i;
	uint8_t ret[6];

	for(i = 0;i < 8;i ++)
		can_recv_valve_control.data[i] = 0xff;
	can_recv_valve_control.TargetID = 0x7cb;
	ret[0] = FLASH_ReadHalWord(CAN0_RATE_ADDRESS,&canrata);
	ret[1] = FLASH_ReadHalWord(CAN1_RATE_ADDRESS,&canrata);
	ret[2] = FLASH_ReadHalWord(CAN2_RATE_ADDRESS,&canrata);
	ret[3] = FLASH_ReadHalWord(CAN3_RATE_ADDRESS,&canrata);
	ret[4] = FLASH_ReadHalWord(CAN4_RATE_ADDRESS,&canrata);
	ret[5] = FLASH_ReadHalWord(CAN5_RATE_ADDRESS,&canrata);
	can_recv_valve_control.data[0] = (ret[0]& 0x03) |
										((ret[1] & 0x03) << 2) |
										((ret[2] & 0x03) << 4) |
										((ret[3] & 0x03) << 6);
	can_recv_valve_control.data[1] = (ret[4] & 0x03) |
										((ret[5] & 0x03) << 2);
	CAN_Transmit_DATA(CAN4_SFR,can_recv_valve_control);
}

void Can_Get_Speed_Parameter(void)
{
	uint32_t valide = 0;
	uint32_t num[9];
	uint8_t i;

	for(i = 0;i < 8;i ++)
		can_recv_valve_control.data[i] = 0xff;
	can_recv_valve_control.TargetID = 0x7ce;

	num[0] = FLASH_ReadWord(VEHICLE_SPEED_ID,&valide);//0x10000000;		//车速ID 10000000
	fprintf(BLUETOOTH_STREAM,"get speed id = %d\r\n",valide);
	can_recv_valve_control.data[0] = valide;
	can_recv_valve_control.data[1] = (valide >> 8) & 0xff;
	can_recv_valve_control.data[2] = (valide >> 16) & 0xff;
	can_recv_valve_control.data[3] = (valide >> 24) & 0xff;
	FLASH_ReadWord(VEHICLE_SPEED_MODE,&valide);			//默认为CAN通信
	can_recv_valve_control.data[4] = valide & 0x1;
	FLASH_ReadWord(VEHICLE_SPEED_BYTETYPE,&valide);			//高字节在前
	can_recv_valve_control.data[4] = can_recv_valve_control.data[4] | ((uint8_t)valide & 0x1 << 1);
	FLASH_ReadWord(VEHICLE_SPEED_BYTESTART,&valide);			//起始字节
	can_recv_valve_control.data[4] = can_recv_valve_control.data[4] | ((uint8_t)valide & 0x1 << 3);
	FLASH_ReadWord(VEHICLE_SPEED_BYTENUM,&valide);			//默认一个字节
	can_recv_valve_control.data[4] = can_recv_valve_control.data[4] | ((uint8_t)valide & 0x1 << 2);
	FLASH_ReadWord(VEHICLE_SPEED_SCALE_FACTOR_A,&valide);;				//正向比例系数默认设置1
	can_recv_valve_control.data[5] = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_SPEED_SCALE_FACTOR_B,&valide);				//负向比例系数默认设置1
	can_recv_valve_control.data[6] = (uint8_t)valide;
	CAN_Transmit_DATA(CAN4_SFR,can_recv_valve_control);
}

void Can_Set_Speed_Parameter(uint8_t *data)
{
	uint32_t ret[5];

	ret[0] = data[0];
	ret[1] = data[1];
	ret[2] = data[2];
	ret[3] = data[3];
	ret[4] = ret[0] | (ret[1] << 8) | (ret[2] << 16) | (ret[3] << 24);
	stSpeedPara.lId = ret[4];
	stSpeedPara.uSingnalType = data[4] & 0x01;
	stSpeedPara.ucByteOrder = (data[4] >> 1) & 0x01;
	stSpeedPara.ucByteLth = (data[4] >> 2) & 0x01;
	stSpeedPara.ucStartByte = (data[4] >> 3) & 0x03;
	stSpeedPara.ucCoeffA = data[5];
	stSpeedPara.ucCoeffB = data[6];
	fprintf(BLUETOOTH_STREAM,"*******speed set start********\r\nid = %d\r\nsource = %d\r\n",
			stSpeedPara.lId,stSpeedPara.uSingnalType);
	ReWrite_two();
	Reaskc_Set_Commond();

	//parameterset.second = 1;
}

void Can_Get_Turn_Parameter(void)
{
	uint32_t valide = 0;
	uint32_t num[9];
	uint8_t i;

	for(i = 0;i < 8;i ++)
		can_recv_valve_control.data[i] = 0xff;
	can_recv_valve_control.TargetID = 0x7d1;

	num[0] = FLASH_ReadWord(VEHICLE_TURN_ID,&valide);//0x10000000;		//车速ID 10000000
	can_recv_valve_control.data[0] = num[0];
	can_recv_valve_control.data[1] = (num[0] >> 8) & 0xff;
	can_recv_valve_control.data[2] = (num[0] >> 16) & 0xff;
	can_recv_valve_control.data[3] = (num[0] >> 24) & 0xff;
	FLASH_ReadWord(VEHICLE_TURN_BYPE,&valide);			//默认为CAN通信
	can_recv_valve_control.data[4] = valide & 0x1;
	FLASH_ReadWord(VEHICLE_TURN_STARTBYTE,&valide);			//高字节在前
	can_recv_valve_control.data[4] = can_recv_valve_control.data[4] | ((uint8_t)valide & 0x7 << 1);

	FLASH_ReadWord(VEHICLE_TURN_LBITNUM,&valide);			//起始字节
	can_recv_valve_control.data[5] = valide & 0x7;
	FLASH_ReadWord(VEHICLE_TURN_RBITNUM,&valide);			//起始字节
	can_recv_valve_control.data[5] =  can_recv_valve_control.data[5] | ((uint8_t)valide & 0x7 << 3);
	FLASH_ReadWord(VEHICLE_TURN_LEFFECTIVE,&valide);			//起始字节
	can_recv_valve_control.data[5] =  can_recv_valve_control.data[5] | ((uint8_t)valide & 0x1 << 6);
	FLASH_ReadWord(VEHICLE_TURN_REFFECTIVE,&valide);			//起始字节
	can_recv_valve_control.data[5] =  can_recv_valve_control.data[5] | ((uint8_t)valide & 0x7 << 7);

	FLASH_ReadWord(VEHICLE_TURN_LSTARTBIT,&valide);			//默认一个字节
	can_recv_valve_control.data[6] = (uint8_t)valide & 0x07;
	FLASH_ReadWord(VEHICLE_TURN_RSTARTBIT,&valide);;				//正向比例系数默认设置1
	can_recv_valve_control.data[7] = (uint8_t)valide & 0x07;
	CAN_Transmit_DATA(CAN4_SFR,can_recv_valve_control);
}

void Can_Get_Turn_Parameter1(void)
{
	uint32_t valide = 0;
	uint32_t num[9];
	uint8_t i;

	for(i = 0;i < 8;i ++)
		can_recv_valve_control.data[i] = 0xff;
	can_recv_valve_control.TargetID = 0x7d1;

	num[0] = FLASH_ReadWord(VEHICLE_TURN_ID1,&valide);//0x10000000;		//车速ID 10000000
	can_recv_valve_control.data[0] = num[0];
	can_recv_valve_control.data[1] = (num[0] >> 8) & 0xff;
	can_recv_valve_control.data[2] = (num[0] >> 16) & 0xff;
	can_recv_valve_control.data[3] = (num[0] >> 24) & 0xff;
	FLASH_ReadWord(VEHICLE_TURN_BYPE1,&valide);			//默认为CAN通信
	can_recv_valve_control.data[4] = valide & 0x1;
	FLASH_ReadWord(VEHICLE_TURN_STARTBYTE1,&valide);			//高字节在前
	can_recv_valve_control.data[4] = can_recv_valve_control.data[4] | ((uint8_t)valide & 0x7 << 1);

	//FLASH_ReadWord(VEHICLE_TURN_LBITNUM,&valide);			//起始字节
	//can_recv_valve_control.data[5] = valide & 0x7;
	FLASH_ReadWord(VEHICLE_TURN_RBITNUM,&valide);			//起始字节
	can_recv_valve_control.data[5] =  can_recv_valve_control.data[5] | ((uint8_t)valide & 0x7 << 3);
	//FLASH_ReadWord(VEHICLE_TURN_LEFFECTIVE,&valide);			//起始字节
	//can_recv_valve_control.data[5] =  can_recv_valve_control.data[5] | ((uint8_t)valide & 0x1 << 6);
	FLASH_ReadWord(VEHICLE_TURN_REFFECTIVE,&valide);			//起始字节
	can_recv_valve_control.data[5] =  can_recv_valve_control.data[5] | ((uint8_t)valide & 0x7 << 7);

	//FLASH_ReadWord(VEHICLE_TURN_LSTARTBIT,&valide);			//默认一个字节
	//can_recv_valve_control.data[6] = (uint8_t)valide & 0x07;
	FLASH_ReadWord(VEHICLE_TURN_RSTARTBIT,&valide);;				//正向比例系数默认设置1
	can_recv_valve_control.data[7] = (uint8_t)valide & 0x07;
	CAN_Transmit_DATA(CAN4_SFR,can_recv_valve_control);
}

void Can_Set_Turn_Parameter(uint8_t *data)
{
	uint32_t ret[5];

	ret[0] = data[0];
	ret[1] = data[1];
	ret[2] = data[2];
	ret[3] = data[3];
	ret[4] = ret[0] | (ret[1] << 8) | (ret[2] << 16) | (ret[3] << 24);
	stTurnPara.lId = ret[4];
	stTurnPara.Source = data[4] & 0x01;
	stTurnPara.ucByteNo = (data[4] >> 1) & 0x07;
	stTurnPara.ucLeftBitLth = data[5] & 0x07;
	//stTurnPara.ucRightBitLth = (data[5] >> 3) & 0x07;
	stTurnPara.ucLeftValid = (data[5] >> 6) & 0x01;
	//stTurnPara.ucRightValid = (data[5] >> 7) & 0x01;
	stTurnPara.ucLeftStartBit = data[6] & 0x07;
	//stTurnPara.ucRightStartBit = data[7] & 0x07;
	ReWrite_three();
	Reaskc_Set_Commond();
}

void Can_Set_Turn_Parameter1(uint8_t *data)
{
	uint32_t ret[5];

	ret[0] = data[0];
	ret[1] = data[1];
	ret[2] = data[2];
	ret[3] = data[3];
	ret[4] = ret[0] | (ret[1] << 8) | (ret[2] << 16) | (ret[3] << 24);
	stTurnPara.lId1 = ret[4];
	stTurnPara.Source1 = data[4] & 0x01;
	stTurnPara.ucByteNo1 = (data[4] >> 1) & 0x07;
	//stTurnPara.ucLeftBitLth = data[5] & 0x07;
	stTurnPara.ucRightBitLth = (data[5] >> 3) & 0x07;
	//stTurnPara.ucLeftValid = (data[5] >> 6) & 0x01;
	stTurnPara.ucRightValid = (data[5] >> 7) & 0x01;
	//stTurnPara.ucLeftStartBit = data[6] & 0x07;
	stTurnPara.ucRightStartBit = data[7] & 0x07;
	ReWrite_three();
	Reaskc_Set_Commond();
}

void Can_Set_Break_Parameter(uint8_t *data)
{
	uint32_t ret[5];

	ret[0] = data[0];
	ret[1] = data[1];
	ret[2] = data[2];
	ret[3] = data[3];
	ret[4] = ret[0] | (ret[1] << 8) | (ret[2] << 16) | (ret[3] << 24);
	stBrakePara.lId = ret[4];
	stBrakePara.Source = data[4] & 0x01;
	stBrakePara.Type = (data[4] >> 1) & 0x01;
	stBrakePara.ucByteNo = (data[4] >> 2) & 0x07;
	stBrakePara.ucStartBit = (data[4] >> 5) & 0x07;
	stBrakePara.ucValid = data[5] & 0x01;
	stBrakePara.ucBitLth = (data[5] >> 1) & 0x07;
	stBrakePara.ucCoeffA = data[6];
	stBrakePara.ucCoeffB = data[7];

	ReWrite_four();
	Reaskc_Set_Commond();
}

void Can_Get_Break_Parameter(void)
{
	uint32_t valide = 0;
	uint32_t num[9];
	uint8_t i;

	for(i = 0;i < 8;i ++)
		can_recv_valve_control.data[i] = 0xff;
	can_recv_valve_control.TargetID = 0x7d4;

	num[0] = FLASH_ReadWord(VEHICLE_BRAKE_ID,&valide);//0x10000000;		//车速ID 10000000
	can_recv_valve_control.data[0] = num[0];
	can_recv_valve_control.data[1] = (num[0] >> 8) & 0xff;
	can_recv_valve_control.data[2] = (num[0] >> 16) & 0xff;
	can_recv_valve_control.data[3] = (num[0] >> 24) & 0xff;
	FLASH_ReadWord(VEHICLE_BRAKE_BYPE,&valide);			//默认为CAN通信
	can_recv_valve_control.data[4] = valide & 0x1;
	FLASH_ReadWord(VEHICLE_BRAKE_NUMTYPE,&valide);			//高字节在前
	can_recv_valve_control.data[4] = can_recv_valve_control.data[4] | ((uint8_t)valide & 0x1 << 1);

	FLASH_ReadWord(VEHICLE_BRAKE_STARTBYTE,&valide);			//起始字节
	can_recv_valve_control.data[4] = can_recv_valve_control.data[4] | ((uint8_t)valide & 0x7 << 2);
	FLASH_ReadWord(VEHICLE_BRAKE_STARTBIT,&valide);			//起始字节
	can_recv_valve_control.data[4] = can_recv_valve_control.data[4] | ((uint8_t)valide & 0x7 << 5);

	FLASH_ReadWord(VEHICLE_BRAKE_LEFFECTIVE,&valide);			//起始字节
	can_recv_valve_control.data[5] =  valide & 0x1;
	FLASH_ReadWord(VEHICLE_BRAKE_BITNUM,&valide);			//起始字节
	can_recv_valve_control.data[5] =  can_recv_valve_control.data[5] | ((uint8_t)valide & 0x7 << 1);

	FLASH_ReadWord(VEHICLE_BRAKE_SCALE_FACTOR_A,&valide);			//默认一个字节
	can_recv_valve_control.data[6] = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_BRAKE_SCALE_FACTOR_B,&valide);				//正向比例系数默认设置1
	can_recv_valve_control.data[7] = (uint8_t)valide;
	CAN_Transmit_DATA(CAN4_SFR,can_recv_valve_control);
}

void Can_Set_Swa_Parameter(uint8_t *data)
{
	uint32_t ret[5];

	ret[0] = data[0];
	ret[1] = data[1];
	ret[2] = data[2];
	ret[3] = data[3];
	ret[4] = ret[0] | (ret[1] << 8) | (ret[2] << 16) | (ret[3] << 24);
	stSwaPara.lId = ret[4];
	stSwaPara.ucByteNo = data[4] & 0x07;
	stSwaPara.ucStartBit = (data[4] >> 3) & 0x07;
	stSwaPara.ucBitLth = data[5] & 0x07;

	ReWrite_five();
	Reaskc_Set_Commond();
}

void Can_Get_Swa_Parameter(void)
{
	uint32_t valide = 0;
	uint32_t num[9];
	uint8_t i;

	for(i = 0;i < 8;i ++)
		can_recv_valve_control.data[i] = 0xff;
	can_recv_valve_control.TargetID = 0x7d7;

	num[0] = FLASH_ReadWord(SWA_IID_ADDRESS,&valide);//0x10000000;		//车速ID 10000000
	can_recv_valve_control.data[0] = num[0];
	can_recv_valve_control.data[1] = (num[0] >> 8) & 0xff;
	can_recv_valve_control.data[2] = (num[0] >> 16) & 0xff;
	can_recv_valve_control.data[3] = (num[0] >> 24) & 0xff;

	FLASH_ReadWord(SWA_BYTENUM_ADDRESS,&valide);			//默认为CAN通信
	can_recv_valve_control.data[4] = valide & 0x7;
	FLASH_ReadWord(SWA_STARTBIT_ADDRESS,&valide);			//高字节在前
	can_recv_valve_control.data[4] = can_recv_valve_control.data[4] | ((uint8_t)valide & 0x7 << 3);

	FLASH_ReadWord(SWA_BITLEGH_ADDRESS,&valide);			//起始字节
	can_recv_valve_control.data[5] = valide & 0x7;
	CAN_Transmit_DATA(CAN4_SFR,can_recv_valve_control);
}

void Can_Set_Linkage_Parameter(uint8_t *data)
{
	uint32_t ret[5];

	stWarninglinkage.FCWlinkage = data[0] & 0x01;
	stWarninglinkage.HMWlinkage = (data[0] >> 1) & 0x01;
	stWarninglinkage.AEBlinkage = (data[0] >> 2) & 0x01;
	stWarninglinkage.LDWlinkage = (data[0] >> 3) & 0x01;
	stUraderSysMessage.SSSlinkage = (data[0] >> 4) & 0x01;
	stWarninglinkage.FCWlinkageSpeed = data[1];
	stWarninglinkage.HMWlinkageSpeed = data[2];
	stWarninglinkage.AEBlinkageSpeed = data[3];
	stWarninglinkage.LDWlinkageSpeed = data[4];
	stUraderSysMessage.SSSlinkageSpeed = data[5];

	ReWrite_five();
	Reaskc_Set_Commond();
}

void Can_Get_Linkage_Parameter(void)
{
	uint32_t valide = 0;
	uint32_t num[9];
	uint8_t i;

	for(i = 0;i < 8;i ++)
		can_recv_valve_control.data[i] = 0xff;
	can_recv_valve_control.TargetID = 0x7da;

	FLASH_ReadWord(FCW_LINKAGE_ADDRESS,&valide);			//默认为CAN通信
	can_recv_valve_control.data[0] = valide & 0x1;
	FLASH_ReadWord(HMW_LINKAGE_ADDRESS,&valide);			//默认为CAN通信
	can_recv_valve_control.data[0] = can_recv_valve_control.data[0] | ((valide & 0x1) << 1);
	FLASH_ReadWord(AEB_LINKAGE_ADDRESS,&valide);			//高字节在前
	can_recv_valve_control.data[0] = can_recv_valve_control.data[0] | ((valide & 0x1) << 2);
	FLASH_ReadWord(LDW_LINKAGE_ADDRESS,&valide);			//默认为CAN通信
	can_recv_valve_control.data[0] = can_recv_valve_control.data[0] | ((valide & 0x1) << 3);

	FLASH_ReadWord(FCW_LINKAGE_SPEED_ADDRESS,&valide);			//起始字节
	can_recv_valve_control.data[1] = valide;
	FLASH_ReadWord(HMW_LINKAGE_SPEED_ADDRESS,&valide);			//起始字节
	can_recv_valve_control.data[2] = valide;
	FLASH_ReadWord(AEB_LINKAGE_SPEED_ADDRESS,&valide);			//起始字节
	can_recv_valve_control.data[3] = valide;
	FLASH_ReadWord(LDW_LINKAGE_SPEED_ADDRESS,&valide);			//起始字节
	can_recv_valve_control.data[4] = valide;

	CAN_Transmit_DATA(CAN4_SFR,can_recv_valve_control);
}

void Can_Set_URader_Parameter(uint8_t *data)
{
	stUraderSysMessage.SSSlinkage = data[0] & 0x01;
	stUraderSysMessage.SSSlinkageSpeed = (data[0] >> 1) & 0x07f;
	stUraderSysMessage.speed0distance = data[1];
	stUraderSysMessage.speed5distance = data[2];
	stUraderSysMessage.speed10distance = data[3];
	stUraderSysMessage.speed15distance = data[4];
	stUraderSysMessage.speed20distance = data[5];
	stUraderSysMessage.speed25distance = data[6];
	stUraderSysMessage.firststage = data[7] & 0xf;
	stUraderSysMessage.secondstage = (data[7] >> 4) & 0xf;
	ReWrite_eight();
	Reaskc_Set_Commond();
}

void Can_Get_URader_Parameter(void)
{

	uint32_t valide = 0;
	uint32_t num[9];
	uint8_t i;

	for(i = 0;i < 8;i ++)
		can_recv_valve_control.data[i] = 0xff;
	can_recv_valve_control.TargetID = 0x7dd;

	FLASH_ReadWord(URADER_SYS_MESSAGE_LINKAGE_STATA,&valide);			//默认为CAN通信
	can_recv_valve_control.data[0] = valide & 0x1;
	FLASH_ReadWord(URADER_SYS_MESSAGE_LINKAGE_SPEED,&valide);			//默认为CAN通信
	can_recv_valve_control.data[0] = can_recv_valve_control.data[0] | ((valide & 0x7f) << 1);

	FLASH_ReadWord(URADER_SYS_MESSAGE_0SPEED_THRESHOLD,&valide);			//高字节在前
	can_recv_valve_control.data[1] = valide;
	FLASH_ReadWord(URADER_SYS_MESSAGE_5SPEED_THRESHOLD,&valide);			//默认为CAN通信
	can_recv_valve_control.data[2] = valide;

	FLASH_ReadWord(URADER_SYS_MESSAGE_10SPEED_THRESHOLD,&valide);			//起始字节
	can_recv_valve_control.data[3] = valide;
	FLASH_ReadWord(URADER_SYS_MESSAGE_15SPEED_THRESHOLD,&valide);			//起始字节
	can_recv_valve_control.data[4] = valide;
	FLASH_ReadWord(URADER_SYS_MESSAGE_20SPEED_THRESHOLD,&valide);			//起始字节
	can_recv_valve_control.data[5] = valide;
	FLASH_ReadWord(URADER_SYS_MESSAGE_25SPEED_THRESHOLD,&valide);			//起始字节
	can_recv_valve_control.data[6] = valide;

	FLASH_ReadWord(URADER_SYS_MESSAGE_FIRST_DIMENSIONAL_COEFFICIENT,&valide);			//起始字节
	can_recv_valve_control.data[7] = valide & 0xf;
	FLASH_ReadWord(URADER_SYS_MESSAGE_SECOND_DIMENSIONAL_COEFFICIENT,&valide);			//起始字节
	can_recv_valve_control.data[7] = can_recv_valve_control.data[7] | ((valide & 0xf) << 4);

	CAN_Transmit_DATA(CAN4_SFR,can_recv_valve_control);
}

void Can_Set_URader_Position_Parameter(uint8_t *data)
{
	uint8_t host,num;
	uint32_t valide = 0;

	host = data[0] & 0xf;

	if(host == 1)
	{
		num = (data[0] >> 4) & 0xf;
		Urader_Company_Message[0].Uposition[num - 1] = data[1] & 0xf;
		fprintf(BLUETOOTH_STREAM,"host = %d num = %d posiotion = %d",host,num,Urader_Company_Message[0].Uposition[num]);
		ReWrite_six();
		Reaskc_Set_Commond();
	}
	if(host == 2)
	{
		num = (data[0] >> 4) & 0xf;
		Urader_Company_Message[1].Uposition[num] = data[1] & 0xf;
		ReWrite_seven();
		Reaskc_Set_Commond();
	}
}

void Can_Get_URader_Position_Parameter(uint8_t *data)
{
	uint8_t host,num;
	uint32_t valide = 0;
	uint8_t position;
	uint8_t i;

	for(i = 0;i < 8;i ++)
		can_recv_valve_control.data[i] = 0xff;
	can_recv_valve_control.TargetID = 0x7f0;

	host = data[0] & 0xf;
	num = (data[0] >> 4) & 0xf;
	if(host == 1)
	{
		position = Bluetooth_Get_URadP_Position(1,num);
	}
	if(host == 2)
	{
		position = Bluetooth_Get_URadP_Position(2,num);
	}
	fprintf(BLUETOOTH_STREAM,"host = %d num = %d posiotion = %d",host,num,position);
	can_recv_valve_control.data[0] = host & 0xf;
	can_recv_valve_control.data[0] = can_recv_valve_control.data[0] | ((num & 0xf) << 4);
	can_recv_valve_control.data[1] = position & 0xf;
	CAN_Transmit_DATA(CAN4_SFR,can_recv_valve_control);

}

void Can_Config_Parameter(struct can_frame rx_frame)
{
	switch(rx_frame.TargetID)
	{
	//第一块
	case 0x7c5:
		Can_Set_Vechicle_Parameter(rx_frame.data);
		break;
	case 0x7c7:
		Can_Get_Vechicle_Parameter();
		break;
	case 0x7c9:
		Can_Set_CanRate_Parameter(rx_frame.data);
		break;
	case 0x7ca:
		Can_Get_CanRate_Parameter();
		break;
		//第二块
	case 0x7cc:
		Can_Set_Speed_Parameter(rx_frame.data);
		break;
	case 0x7cd:
		Can_Get_Speed_Parameter();
		break;
		//第三块
	case 0x7cf:
		Can_Set_Turn_Parameter(rx_frame.data);
		break;
	case 0x7d0:
		Can_Get_Turn_Parameter();
		break;

	case 0x7F1:
		Can_Set_Turn_Parameter1(rx_frame.data);
		break;
	case 0x7F2:
		Can_Get_Turn_Parameter1();
		break;

		//第四块
	case 0x7d2:
		Can_Set_Break_Parameter(rx_frame.data);
		break;
	case 0x7d3:
		Can_Get_Break_Parameter();
		break;
		//第五块
	case 0x7d5:
		Can_Set_Swa_Parameter(rx_frame.data);
		break;
	case 0x7d6:
		Can_Get_Swa_Parameter();
		break;
	case 0x7d8:
		Can_Set_Linkage_Parameter(rx_frame.data);
		break;
	case 0x7d9:
		Can_Get_Linkage_Parameter();
		break;
		//第八块块
	case 0x7db:
		Can_Set_URader_Parameter(rx_frame.data);
		break;
	case 0x7dc:
		Can_Get_URader_Parameter();
		break;
		//第六  第七块
	case 0x7de:
		Can_Set_URader_Position_Parameter(rx_frame.data);
		break;
	case 0x7df:
		Can_Get_URader_Position_Parameter(rx_frame.data);
		break;
	default:
		break;
	}
}

void can4_receive(struct can_frame rx_frame)
{
	if(rx_frame.TargetID == 0x18FFED81)
	{
		if((rx_frame.data[0] & 0x01) == 0)
			warning_status.AEBstatus = warning_status.AEBstatus | 0x02;
		else
			warning_status.AEBstatus = warning_status.AEBstatus & 0xFD;

		if((rx_frame.data[0] & 0x02) == 0)
			warning_status.LDWstatus = 1;
		else
			warning_status.LDWstatus = 0;
		Displayer_CANRx_Analysis(&rx_frame);
	}
	else
	{
		Can_Config_Parameter(rx_frame);
	}
}

void can5_receive(struct can_frame rx_frame)
{
}
/*
 * 比例阀通信时间
 * */
void Set_ValveProportionalComm_StaStamp(uint32_t stamp)
{
	stCanCommSta.Proportional_valve.oldSysTime = stamp;
}
/*
 * 整车CAN通信时间
 * */
void Set_ValveComm_StaStamp(uint32_t stamp)
{
	stCanCommSta.stVehicle.oldSysTime = stamp;
}

void Set_Valvespeed_Stamp(uint32_t stamp)
{
	stCanCommSta.stSpeed.oldSysTime = stamp;
}

void Set_Camera_Stamp(uint32_t stamp)
{
	stCanCommSta.stCamera.oldSysTime = stamp;
}

float fVehiclespeedKalmanFilter(float original_speed)
{
	float kg = 0;
	float x_now = 0;
	float x_pre= 0;
	static float p_last = 1;
	static float x_last = 0;
	float p_pre = 0;
	float p_now = 0;

	x_pre = x_last;									/*预测*/
    /*Get convariance */
    p_pre = p_last + VEHICLE_SPEED_NIOSE_Q;			/*Q=0.25 协方差预测*/
    kg = p_pre/(p_pre + VEHICLE_SPEED_NIOSE_R);		/*NIOSE_R=1 卡尔曼增益*/
    x_now = x_pre + kg*(original_speed - x_pre);	/*状态估计*/
    p_now = (1-kg)*p_pre;							/*协方差更新*/

    p_last = p_now;
    x_last = x_now;

    return x_now;
}

float fVehicleSpeedFilter(float fVeSpeed)
{
	float fSmoothSpeed = 0;
	static float fPreSmoothSpeed = 0;

	fSmoothSpeed = fVehiclespeedKalmanFilter(fVeSpeed);

	if((fPreSmoothSpeed - fSmoothSpeed) > VEHICLE_MAX_SPEED_ERROR)
	{
		fSmoothSpeed = fPreSmoothSpeed - VEHICLE_MAX_SPEED_ERROR;
	}
#ifdef SPEED_SOURCE_FROM_ANOLOG
	else if((fPreSmoothSpeed - fSmoothSpeed) < (-(1.2f/3.6)))
	{
		fSmoothSpeed = fPreSmoothSpeed + VEHICLE_MAX_SPEED_ERROR;
	}
#endif
	fPreSmoothSpeed = fSmoothSpeed;

	return fSmoothSpeed;
}

//==================================================================================================
// 解析CAN报文的车速信号
//--------------------------------------------------------------------------------------------------
void DecodeCanSpeed(uint8_t *ptr)
{
	short i = 0;
	float fFilteredSpeed;
	static float fPreSpeed = 0.0f;
	static uint32_t ulSpeedJumpTime = 0;
	static uint32_t ulPreTime = 0;
	uint32_t ulCurrentTime = 0;

	ulCurrentTime = SystemtimeClock;
	//stVehicleParas.fVehicleSpeed = ptr[7];

	if (ulCurrentTime - ulPreTime >= VEHICLE_CAN_OFFLINE_PERIOD)
	{
		fPreSpeed = 0; // 发现车Can掉线，将上次车速清零
	}
	ulPreTime = ulCurrentTime;

	if(stSpeedPara.ucByteLth==2)							// 两个字节长度
	{
		if(stSpeedPara.ucByteOrder==SPEED_HIGH_BYTE_FISRT)
		{
			i = *(ptr+stSpeedPara.ucStartByte);
			//i = (i<<8) + *(ptr+stSpeedPara.ucStartByte+1);
		}
		else
		{
			i = *(ptr+stSpeedPara.ucStartByte+1);
			//i = (i<<8) + *(ptr+stSpeedPara.ucStartByte);
		}

	}
	else
	{
		i = stVehicleCanData.data[stSpeedPara.ucStartByte];

	}
	stVehicleParas.fVehicleSpeed = (float)i;
	//stVehicleParas.fVehicleSpeed = ((float)i)*stSpeedPara.ucCoeffA
											// /stSpeedPara.ucCoeffB/3.6;


/*
	// 车速异常检查
	if (stVehicleParas.fVehicleSpeed * 3.6 >= VEHICLE_MAX_SPEED_VALID)
	{
		ucFlagSpeedErr = 1;
	}
	else
	{
		ucFlagSpeedErr = 0;
	}

	// 车速跳变异常检查
	if ((stVehicleParas.fVehicleSpeed - fPreSpeed) >= VEHICLE_SPEED_JUMP_THRESHOLD / 3.6)
	{
		ucFlagSpeedJumpErr = 1;
		fPreSpeed = stVehicleParas.fVehicleSpeed;
		ulSpeedJumpTime = ulCurrentTime;
	}
	else
	{
		if ((ulCurrentTime - ulSpeedJumpTime >= VEHICLE_SPEED_JUMP_HOLD_TIME)
			&& (1 == ucFlagSpeedJumpErr))
		{
			ucFlagSpeedJumpErr = 0;
			ulSpeedJumpTime = 0;

		}
		fPreSpeed = stVehicleParas.fVehicleSpeed;
	}

	//车速滤波
	fFilteredSpeed = fVehicleSpeedFilter(stVehicleParas.fVehicleSpeed);
	stVehicleParas.fVehicleSpeed = fFilteredSpeed + 0.001; // To solve precision loss problem
	*/
}
void DecodeCanTurn(uint8_t *ptr,uint8_t test)
{
	uint8_t byte_val,signal_val;

	//if(stTurnPara.Source == 0)
	{
		if(test == 0)
			byte_val = *(ptr + stTurnPara.ucByteNo);
		else
			byte_val = *(ptr + stTurnPara.ucByteNo1);
		signal_val =   (byte_val>>stTurnPara.ucLeftStartBit)
					 & (0xff>>(8-stTurnPara.ucLeftBitLth));
		if(signal_val==stTurnPara.ucLeftValid)
		{
			stVehicleParas.LeftFlagTemp = 1;
		}
		else
		{
			stVehicleParas.LeftFlagTemp = 0;
		}
		//-----------------------------------------------------------
		signal_val =   (byte_val>>stTurnPara.ucRightStartBit)
					 & (0xff>>(8-stTurnPara.ucRightBitLth));
		if(signal_val == stTurnPara.ucRightValid)
		{
			stVehicleParas.RightFlagTemp = 1;
		}
		else
		{
			stVehicleParas.RightFlagTemp = 0;
		}
		if(((ptr[2] >> 2) & 0x03) == 0x01)
			stVehicleParas.ReverseGear = 1;
		else
			stVehicleParas.ReverseGear = 0;

	}
}

void DecodeCanTurnL(uint8_t *ptr)
{
	uint8_t byte_val,signal_val;

	if(stTurnPara.Source == 0)
	{
		byte_val = *(ptr + stTurnPara.ucByteNo);
		signal_val =   (byte_val>>stTurnPara.ucLeftStartBit)
					 & (0xff>>(8-stTurnPara.ucLeftBitLth));
		if(signal_val==stTurnPara.ucLeftValid)
		{
			stVehicleParas.LeftFlagTemp = 1;
		}
		else
		{
			stVehicleParas.LeftFlagTemp = 0;
		}
	}
}

void DecodeCanTurnR(uint8_t *ptr)
{
	uint8_t byte_val,signal_val;

	if(stTurnPara.Source == 0)
	{
		signal_val =   (byte_val>>stTurnPara.ucRightStartBit)
					 & (0xff>>(8-stTurnPara.ucRightBitLth));
		if(signal_val == stTurnPara.ucRightValid)
		{
			stVehicleParas.RightFlagTemp = 1;
		}
		else
		{
			stVehicleParas.RightFlagTemp = 0;
		}
	}
}
void DecodeCanBrake(uint8_t *ptr)
{
	uint8_t byte_val,signal_val;

	if(stBrakePara.Source==0)
	{
		byte_val = *(ptr + stBrakePara.ucByteNo);
		if(stBrakePara.Type == 0)						// 数值型
		{
			signal_val =   (byte_val>>stBrakePara.ucStartBit)
					 	 & (0xff>>(8-stBrakePara.ucBitLth));
			if(signal_val == stBrakePara.ucValid)
			{
				stVehicleParas.BrakeFlag = 1;
			}
			else
			{
				stVehicleParas.BrakeFlag = 0;
			}
		}
		else													// 百分比型
		{
			if(byte_val != 0x00)
			{
				stVehicleParas.BrakeFlag = 1;
			}
			else
			{
				stVehicleParas.BrakeFlag = 0;
			}
		}
	}
}

void DecodeCanAbs(uint8_t *ptr)
{
	uint8_t byte_val = 0;
	uint8_t signal_val = 0;

	if(0 == stAbsPara.Source)
	{
		byte_val = *(ptr + stAbsPara.ucByteNo);

		signal_val =   (byte_val >> stAbsPara.ucAbsWarningStartBit)
					 & (0xff >>(8 - stAbsPara.ucAbsWarningBitLth));
		if(signal_val == stAbsPara.ucAbsWarningValid)
		{
			stVehicleParas.AbsErrorFlag = 1;
		}
		else
		{
			stVehicleParas.AbsErrorFlag = 0;
		}

	}
}

void DecodeCanSwa(uint8_t *ptr)
{
	stSWAParas.SWADegree = (uint16_t)(ptr[0] << 8) | ptr[1];
}

double readSpeedSensorValue(void)
{
	double DW_PLUSE_WIDTH=0;//捕捉到PWM周期变量
	uint32_t DW_PLUSE_WIDTH2=0;
	uint32_t temp=0;

	DW_PLUSE_WIDTH = (double)1/first_time;//CCP_Get_Capture_Result(CCP21_SFR,CCP_CHANNEL_2);
	/*
	if(DW_PLUSE_WIDTH < 5)
	{
		DW_PLUSE_WIDTH=0;
	}
	else if((DW_PLUSE_WIDTH > 5) && (DW_PLUSE_WIDTH < 100))
	{
		temp = DW_PLUSE_WIDTH;
		DW_PLUSE_WIDTH2 = temp;
	}
	else
	{
		;
	}
	*/
	return DW_PLUSE_WIDTH;
}
//uint32_t sensorvalue = 0;
//
float speedConvertSensorValue(double speedSensorValue)
{
	float speedConvertedValue = 0;

	//sensorvalue =speedSensorValue;
	if(speedSensorValue < 0.0001)
	{
		speedConvertedValue = 0;
	}
	else
	{
		speedConvertedValue = (float)((float)(stSpeedPara.ucCoeffA) / stSpeedPara.ucCoeffB *(10000000/(36*speedSensorValue)));//((float)speedSensorValue/1000)*stSpeedConfigPara.ucCoeffA
		//speedConvertedValue = (float)(stSpeedPara.ucCoeffA * sensorvalue);
		//-speedConvertedValue = speedConvertedValue / ((float)stSpeedPara.ucCoeffB);
		//speedConvertedValue = (float)((stSpeedPara.ucCoeffA) * sensorvalue/ stSpeedPara.ucCoeffB);// *(10000000/(36*speedSensorValue)));
	}

	return speedConvertedValue;

}

void DecodeHetSpeed(void)
{

	static float fPreSpeed = 0.0f;
	static uint32_t ulSpeedJumpTime = 0;
	double iFilteredSpeed = 0.0;
	float fFilteredSpeed = 0.0;
	static uint32_t ulPreSystemTime = 0;
	uint32_t ulCurrentSystemTime;

	//ulCurrentSystemTime = SystemtimeClock;

	//if ((ulCurrentSystemTime - ulPreSystemTime) >= (2 * SAM_ANA_BLINK_PERIOD))
//	{
		//ulPreSystemTime = ulCurrentSystemTime;

		iFilteredSpeed = readSpeedSensorValue();

		stVehicleParas.fVehicleSpeed = speedConvertSensorValue(iFilteredSpeed);
/*
		if (stVehicleParas.fVehicleSpeed * 3.6 >= VEHICLE_MAX_SPEED_VALID)
		{
			ucFlagSpeedErr = 1;
		}
		else
		{
			ucFlagSpeedErr = 0;
		}

		// 车速跳变异常检查
		if ((stVehicleParas.fVehicleSpeed - fPreSpeed) >= VEHICLE_SPEED_JUMP_THRESHOLD / 3.6)
		{
			ucFlagSpeedJumpErr = 1;
			fPreSpeed = stVehicleParas.fVehicleSpeed;
			ulSpeedJumpTime = ulCurrentSystemTime;
		}
		else
		{
			if ((ulCurrentSystemTime - ulSpeedJumpTime >= VEHICLE_SPEED_JUMP_HOLD_TIME)
				&& (1 == ucFlagSpeedJumpErr))
			{
				ucFlagSpeedJumpErr = 0;
				ulSpeedJumpTime = 0;

			}
			fPreSpeed = stVehicleParas.fVehicleSpeed;
		}

		//SendMsgToCamera();
		fFilteredSpeed = fVehicleSpeedFilter(stVehicleParas.fVehicleSpeed);
		stVehicleParas.fVehicleSpeed = fFilteredSpeed;
	}*/
}

void Vehicle_Parameter_Analysis(uint32_t ulSysTime)
{
	struct can_frame tx_frame;

	if(stVehicleCanSpeedData.flgBoxRxEnd == 1)
	{
		stVehicleCanSpeedData.flgBoxRxEnd = 0;
		if(stSpeedPara.uSingnalType == 0)
		{
			stCanCommSta.stVehicle.oldSysTime = ulSysTime;
			stCanCommSta.stSpeed.oldSysTime = ulSysTime;
			DecodeCanSpeed(stVehicleCanSpeedData.data);
		}
	}
	if(stVehicleCanOdomData.flgBoxRxEnd == 1)	// 金旅大巴里程计
	{
		stVehicleCanOdomData.flgBoxRxEnd 	= 0;
		// Byte4~1是累计行驶里程；Byte8~5是当前行驶里程;分辨率为5m/bit,偏移量为0;
		for(uint8_t i=0;i<4;i++){
			uint32_t temp = stVehicleCanSpeedData.data[i];
			stVehicleParas.VehicleOdom_Total 	+= (temp<<(8*i))*5;	// 单位米
			temp = stVehicleCanSpeedData.data[i+4];
			stVehicleParas.VehicleOdom_Current 	+= (temp<<(8*i))*5;	// 单位米
		}
	}
	if(stVehicleCanTurnData.flgBoxRxEnd == 1)
	{
		stVehicleCanTurnData.flgBoxRxEnd = 0;
		if(stTurnPara.lId == stTurnPara.lId1)
		{
			if(stTurnPara.Source == 0)
			{
				stCanCommSta.stVehicle.oldSysTime = ulSysTime;
				DecodeCanTurn(stVehicleCanTurnData.data,0);
			}
		}
		else
		{
			DecodeCanTurnL(stVehicleCanTurnData.data);
		}
	}
	if(stVehicleCanTurnData1.flgBoxRxEnd == 1)
	{
		stVehicleCanTurnData1.flgBoxRxEnd = 0;
		if(stTurnPara.lId == stTurnPara.lId1)
		{
			if(stTurnPara.Source1 == 0)
			{
				stCanCommSta.stVehicle.oldSysTime = ulSysTime;
				DecodeCanTurn(stVehicleCanTurnData1.data,1);
			}
		}
		else
		{
			DecodeCanTurnR(stVehicleCanTurnData1.data);
		}
	}

	if(stVehicleCanBreakData.flgBoxRxEnd == 1)
	{
		stVehicleCanBreakData.flgBoxRxEnd = 0;
		if(stBrakePara.Source == 0)
		{
			stCanCommSta.stVehicle.oldSysTime = ulSysTime;
			DecodeCanBrake(stVehicleCanBreakData.data);
		}
	}
	if(stVehicleCanAbsData.flgBoxRxEnd == 1)
	{
		stVehicleCanAbsData.flgBoxRxEnd = 0;
		if(stAbsPara.Source == 0)
		{
			stCanCommSta.stVehicle.oldSysTime = ulSysTime;
			DecodeCanAbs(stVehicleCanAbsData.data);
		}
	}

	if(stVehicleCanSWAData.flgBoxRxEnd == 1)
	{
		stVehicleCanSWAData.flgBoxRxEnd = 0;
		DecodeCanSwa(stVehicleCanSWAData.data);
	}
	if(stSpeedPara.uSingnalType == 1)
	{
		DecodeHetSpeed();
	}
	if(stTurnPara.Source == 1)
	{
		stVehicleParas.LeftFlagTemp = Read_turnleft_signal();
		stVehicleParas.RightFlagTemp = Read_turnright_signal();
	}
	if(stBrakePara.Source == 1)
	{
		stVehicleParas.BrakeFlag = Read_stop_signal();//刹车线
	}
	if(stAbsPara.Source == 1)
	{
		stVehicleParas.AbsErrorFlag = Read_AEB_switch();
	}
}

void Proprot_RxValve_Data_Analysis(uint32_t ulSysTime)
{
    uint16_t i;
    uint8_t  j;
	if(Proportional_valve_status.flgBoxRxEnd == 1)
    {
		Proportional_valve_status.flgBoxRxEnd = 0;
		//fprintf(USART1_STREAM,"file \r\n");

		stValveParas.TimeStamp = ulSysTime;

    	j = Proportional_valve_status.Box[0];
    	stValveParas.Valve_State = j;


    	j = Proportional_valve_status.Box[1];
    	stValveParas.Internal_Temp = (float)(j-40);

    	i = (uint16_t)(Proportional_valve_status.Box[3] << 8);
    	i = i | Proportional_valve_status.Box[2];
    	stValveParas.Actual_Pressure = (float)(i) / 128 * 100;

    	i = (uint16_t)(Proportional_valve_status.Box[5] << 8);
    	i = i | Proportional_valve_status.Box[4];
    	stValveParas.Target_Pressure = (float)(i) / 128 * 100;

    	j = Proportional_valve_status.Box[6];
    	stValveParas.Fault_Code = j;
/*
    	if((stValveParas.Fault_Code == 1) |
    			(stValveParas.Fault_Code == 4) |
    			(stValveParas.Fault_Code == 8) |
    			(stValveParas.Fault_Code == 9) |
    			(stValveParas.Fault_Code == 10)	|
    			(stValveParas.Fault_Code == 11))
    	{
    		GPIO_Set_Output_Data_Bits(GPIOE_SFR,GPIO_PIN_MASK_3, 0);
    		Delay(2,2);
    		GPIO_Set_Output_Data_Bits(GPIOE_SFR,GPIO_PIN_MASK_3, 1);
    	}*/
    }
	if(Proportional_valve_fault.flgBoxRxEnd == 1)
	{
		//stCanCommSta.Proportional_valve.oldSysTime = ulSysTime;
		Proportional_valve_fault.flgBoxRxEnd == 0;
	}
}

void Check_SysErr_Alarm(void)
{
	uint32_t currentTime;
	static uint32_t oldSysTime = 0;
	static uint16_t usRuningTime = 0;

	currentTime = SystemtimeClock;

	/*
	 * 整车CAN连接情况
	 * */
	if(currentTime-stCanCommSta.stVehicle.oldSysTime >= VEHICLE_CAN_OFFLINE_PERIOD)
	{
		stCanCommSta.stVehicle.status = OFFLINE;
	}
	else
	{
		stCanCommSta.stVehicle.status = ONLINE;
	}

	/*
	 * 车速can链接情况
	 * */
	if(currentTime - stCanCommSta.stSpeed.oldSysTime >= VEHICLE_CAN_OFFLINE_PERIOD)
	{
		stCanCommSta.stSpeed.status = OFFLINE;
		//stVehicleParas.fVehicleSpeed = 0;
	}
	else
	{
		stCanCommSta.stSpeed.status = ONLINE;
	}

	/*
	 * 转向CAN
	 *
	if(currentTime - stCanCommSta.stTurnSig.oldSysTime >= VEHICLE_CAN_OFFLINE_PERIOD)
	{
		stCanCommSta.stTurnSig.status = OFFLINE;
	}
	if(currentTime - stCanCommSta.stBrakeSig.oldSysTime >= VEHICLE_CAN_OFFLINE_PERIOD)
	{
		stCanCommSta.stBrakeSig.status = OFFLINE;
	}
	 */
	/*
	 * 比例阀CAN
	 * */
	if(currentTime - stCanCommSta.Proportional_valve.oldSysTime >= VALVE_CAN_OFFLINE_PERIOD)
	{
		stCanCommSta.Proportional_valve.status = OFFLINE;
	}
	else
	{
		stCanCommSta.Proportional_valve.status = ONLINE;
	}

	/*
	 * 毫米波雷达can
	 * */
	if(currentTime - stCanCommSta.stRadar.oldSysTime >= RADAR_CAN_OFFLINE_PERIOD)
	{
		stCanCommSta.stRadar.status = OFFLINE;
	}
	else
	{
		stCanCommSta.stRadar.status = ONLINE;
	}

	/*
	 *超声波雷达
	 * */
	if(currentTime - stCanCommSta.stHRadar.oldSysTime >= HRADAR_CAN_OFFLINE_PERIOD)
	{
		stCanCommSta.stHRadar.status = OFFLINE;
	}
	else
	{
		stCanCommSta.stHRadar.status = ONLINE;
	}
	//摄像机can
	if(currentTime - stCanCommSta.stCamera.oldSysTime >= CAMERA_CAN_OFFLINE_PERIOD)
	{
		stCanCommSta.stCamera.status = OFFLINE;
	}
	else
	{
		stCanCommSta.stCamera.status = ONLINE;
	}

	//4G网络连接
	if(currentTime - stCanCommSta.stWireless.oldSysTime >= VALVE_NET_OFFLINE_PERIOD)
	{
		stCanCommSta.stWireless.status = OFFLINE;
	}
	else
	{
		stCanCommSta.stWireless.status = ONLINE;
	}
	// 4G 网络连接
	stCanCommSta.stWireless.status = server_p.sevIsConnect;
}

void DisPlay_Message_Analy(uint8_t *buf)
{
	if((buf[0] & 0x01) == 0)
		warning_status.AEBstatus = warning_status.AEBstatus | 0x02;
	else
		warning_status.AEBstatus = warning_status.AEBstatus & 0xFD;

	if((buf[0] & 0x02) == 0)
		warning_status.LDWstatus = 1;
	else
		warning_status.LDWstatus = 0;

	//fprintf(USART1_STREAM,"%s",buf);
}

void Send_Display_Message(void)
{
	static uint8_t Urader_Alarm = 0;
	static uint32_t DisplayOldtime;
	struct can_frame displayer;
	uint8_t data[16] = {'\0'};
	uint8_t i = 0;

	if((SystemtimeClock - DisplayOldtime) > 100)
	{
		DisplayOldtime = SystemtimeClock;
		Displayer_CANTx_Get(&displayer);
		//if(Displayer_CANTx_Get(&displayer))
		{
			//data[0] = 0x02;
			//data[1] = 0x03;
			//for(i = 2;i < 10;i ++)
			//	data[i] = displayer.data[i - 2];
			//data[10] = 0x02;
			//data[11] = 0x03;
			//USART_Send(USART1_SFR,data,12);
			CAN_Transmit_DATA(CAN4_SFR,displayer);
			Delay(45,45);
			for(i = 0;i < 8;i ++)
				displayer.data[i] = 0xff;
			displayer.TargetID = 0x18FFED82;
			displayer.data[0] = stVehicleParas.BreakUreaderLevel;
			if(stVehicleParas.BreakUreaderDirection != 0)
			{
				if(Urader_Alarm == 3)
					displayer.data[0] = displayer.data[0] | (stVehicleParas.BreakUreaderDirection << 2);
				if(Urader_Alarm < 3)
					Urader_Alarm ++;
				else
					Urader_Alarm = 3;
			}
			else
			{
				Urader_Alarm = 0;
				displayer.data[0] = displayer.data[0];
			}

			CAN_Transmit_DATA(CAN4_SFR,displayer);

		}
	}
}



void Send_Break_Control(void)
{
	struct can_frame tx_frame_valve;

	if(stSysPara.ucControlMode == 0)		//比例阀控制
	{
		Valve_Pressure_Get(&tx_frame_valve);
		CAN_Transmit_DATA(CAN3_SFR,tx_frame_valve);
	}
	else if(stSysPara.ucControlMode == 1)	//目标减速度 VV6
	{
		Valve_Target_Deceleration_Get(&tx_frame_valve);
		CAN_Transmit_DATA(CAN2_SFR,tx_frame_valve);
	}
	else if(stSysPara.ucControlMode == 2)	//减速度请求	BYD
	{
		Valve_Deceleration_Request_Get(&tx_frame_valve);
		CAN_Transmit_DATA(CAN2_SFR,tx_frame_valve);
	}
	else
		;
}
