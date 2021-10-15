/*
 * screen.c
 *
 *  Created on: 2021-6-18
 *      Author: chenq
 */

#include "screen.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static CAN_SFRmap* Screen_CAN=NULL;
static struct screen_t screen_m;
static SemaphoreHandle_t screen_semaphore;

void Screen_Init(CAN_SFRmap* CANx,uint8_t Bdrt)
{
	Screen_CAN=CANx;
	xInit_CAN(Screen_CAN,Bdrt,CAN_MODE_NORMAL,NULL);
	screen_semaphore = xSemaphoreCreateMutex();
	if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
	{
		screen_m.LDW_Status=STATUS_DISABLE;
		screen_m.DMS_Status=STATUS_DISABLE;
		screen_m.LDW_Warning=NO_LDW_WARNING;
		screen_m.DMS_Warning=NO_DMS_WARNING;
		screen_m.Lane_Width=10;		//unit 0.25m  range 0~4m
		screen_m.Left_Line=NO_LINE;
		screen_m.Right_Line=NO_LINE;
		screen_m.Lane_curvature=0;	//unit 4  range 0~1020
		screen_m.Space_to_Left=10;	//unit 0.125  range 0~3.75
		screen_m.Space_to_Right=10;	//unit 0.125  range 0~3.75
		screen_m.Head_Angle=90;		//unit 1  range -90~90 bia -90
		//CAN_ID2
		screen_m.FCW_Status=STATUS_DISABLE;
		screen_m.AEBS_Status=STATUS_DISABLE;
		screen_m.Anti_Fault=STATUS_DISABLE;
		screen_m.FCW_Warning=STATUS_DISABLE;
		screen_m.Target=NONE_OBSTACLE;
		screen_m.Distance=255;		//unit 0.5  range 0~127.5
		screen_m.Speed=0;			//unit 0.25  range 0~63.75
		screen_m.TTC=255;			//unit 0.05  range 0~12.75
		screen_m.SLI_Speed=255;		//unit 1  range 0~255
		screen_m.Function_Active=0x00;
		screen_m.Device_Status==0x00;
		xSemaphoreGive( screen_semaphore );
	}
}

CAN_ErrorT Screen_CAN_Send()
{
	struct can_frame can_frame1,can_frame2;
	CAN_ErrorT CAN_Error;
	can_frame1.TargetID=CAN_ID1;
	can_frame1.MsgType=CAN_DATA_FRAME;
	can_frame1.RmtFrm=CAN_FRAME_FORMAT_EFF;
	can_frame1.lenth=8;
#ifdef CAN_ID2
	can_frame2.TargetID=CAN_ID2;
	can_frame2.MsgType=CAN_DATA_FRAME;
	can_frame2.RmtFrm=CAN_FRAME_FORMAT_EFF;
	can_frame2.lenth=8;
#endif
	if(Screen_CAN==NULL)
		CAN_Error=CAN_ERROR_NOINIT;
	else
	{
		if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
		{
			can_frame1.data[0]= (( screen_m.LDW_Status&0x03 ) <<6) | (( screen_m.DMS_Status&0x03 ) << 4);
			can_frame1.data[1]= (( screen_m.LDW_Warning&0x03 ) <<6) | ( screen_m.DMS_Warning&0x07 );
			can_frame1.data[2]= (( screen_m.Lane_Width&0x0f ) <<4) | (( screen_m.Left_Line&0x03 ) <<2) | ( screen_m.Right_Line&0x03 );
			can_frame1.data[3]= screen_m.Lane_curvature ;
			can_frame1.data[4]= screen_m.Space_to_Left;
			can_frame1.data[5]= screen_m.Space_to_Right;
			can_frame1.data[6]= screen_m.Head_Angle;
			can_frame1.data[7]= 0x00;
#ifdef CAN_ID2
			can_frame2.data[0]= (( screen_m.FCW_Status&0x03 ) <<6) | (( screen_m.AEBS_Status&0x03 ) << 4) | (( screen_m.Anti_Fault&0x03 ) <<2);
			can_frame2.data[1]= (( screen_m.FCW_Warning ) <<6) | (( screen_m.Target&0x07 ) << 3) ;
			can_frame2.data[2]= screen_m.Distance;
			can_frame2.data[3]= screen_m.Speed;
			can_frame2.data[4]= screen_m.TTC;
			can_frame2.data[5]= screen_m.SLI_Speed;
			can_frame2.data[6]= screen_m.Function_Active;
			can_frame2.data[7]= screen_m.Device_Status;
#endif
			xSemaphoreGive( screen_semaphore );
		}
		CAN_Error=CAN_Transmit_DATA(Screen_CAN, can_frame1);
#ifdef CAN_ID2
		CAN_Error=CAN_Transmit_DATA(Screen_CAN, can_frame2);
#endif
	}
	return CAN_Error;
}

int Screen_Set_Value(struct screen_t screen)
{
	if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
	{
		memcpy((void *)&screen_m,(void *)&screen,sizeof(struct screen_t));
		xSemaphoreGive( screen_semaphore );
		return 0;
	}
	else
		return -1;
}
int Screen_Get_Value(struct screen_t *screen)
{
	if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
	{
		memcpy((void *)screen,(void *)&screen_m,sizeof(struct screen_t));
		return 0;
	}
	else
		return -1;
}

int Screen_Set_LDW_Status(WORK_STATUS_T LDW_Status)
{
	{
		if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
			{
				screen_m.LDW_Status=LDW_Status;
				xSemaphoreGive( screen_semaphore );
				return 0;
			}
			else
				return -1;

	}
}
int Screen_Set_DMS_Status(WORK_STATUS_T DMS_Status)
{
	{
		if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
			{
				screen_m.DMS_Status=DMS_Status;
				xSemaphoreGive( screen_semaphore );
				return 0;
			}
			else
				return -1;

	}
}
int Screen_Set_LDW_Warning(LDW_WARNING_T LDW_Warning)
{
	{
		if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
			{
				screen_m.LDW_Warning=LDW_Warning;
				xSemaphoreGive( screen_semaphore );
				return 0;
			}
			else
				return -1;

	}
}
int Screen_Set_DMS_Warning(DMS_WARNING_T DMS_Warning)
{
	{
		if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
			{
				screen_m.DMS_Warning=DMS_Warning;
				xSemaphoreGive( screen_semaphore );
				return 0;
			}
			else
				return -1;

	}
}
int Screen_Set_Lane_Width(uint8_t Lane_Width)
{
	{
		if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
			{
				screen_m.Lane_Width=Lane_Width;
				xSemaphoreGive( screen_semaphore );
				return 0;
			}
			else
				return -1;

	}
}
int Screen_Set_Left_Line(Lane_Type Left_Line)
{
	{
		if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
			{
				screen_m.Left_Line=Left_Line;
				xSemaphoreGive( screen_semaphore );
				return 0;
			}
			else
				return -1;

	}
}
int Screen_Set_Right_Line(Lane_Type Right_Line)
{
	{
		if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
			{
				screen_m.Right_Line=Right_Line;
				xSemaphoreGive( screen_semaphore );
				return 0;
			}
			else
				return -1;

	}
}
int Screen_Set_Lane_curvature(uint8_t Lane_curvature)	//unit 4  range 0~1020
{
	{
		if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
			{
				screen_m.Lane_curvature=Lane_curvature;
				xSemaphoreGive( screen_semaphore );
				return 0;
			}
			else
				return -1;

	}
}
int Screen_Set_Space_to_Left(uint8_t Space_to_Left)	//unit 0.125  range 0~3.75
{
	if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
		{
			screen_m.Space_to_Left=Space_to_Left;
			xSemaphoreGive( screen_semaphore );
			return 0;
		}
		else
			return -1;

}
int Screen_Set_Space_to_Right(uint8_t Space_to_Right)	//unit 0.125  range 0~3.75
{
	if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
		{
			screen_m.Space_to_Right=Space_to_Right;
			xSemaphoreGive( screen_semaphore );
			return 0;
		}
		else
			return -1;

}
int Screen_Set_Head_Angle(uint8_t Head_Angle)		//unit 1  range -90~90
{
	if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
	{
		screen_m.Head_Angle=Head_Angle;
		xSemaphoreGive( screen_semaphore );
		return 0;
	}
	else
		return -1;
}
int Screen_Set_Distance(uint8_t Distance)
{
	if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
	{
		screen_m.Distance=Distance;
		xSemaphoreGive( screen_semaphore );
		return 0;
	}
	else
		return -1;
}
int Screen_Set_TTC(uint8_t TTC)
{
	if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
	{
		screen_m.TTC=TTC;
		xSemaphoreGive( screen_semaphore );
		return 0;
	}
	else
		return -1;
}
int Screen_Set_Speed(uint8_t Speed)
{
	if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
	{
		screen_m.Speed=Speed;
		xSemaphoreGive( screen_semaphore );
		return 0;
	}
	else
		return -1;
}
int Screen_Set_SLI_Speed(uint8_t SLI_Speed)
{
	if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
	{
		screen_m.SLI_Speed=SLI_Speed;
		xSemaphoreGive( screen_semaphore );
		return 0;
	}
	else
		return -1;
}

int Screen_Set_Active(uint8_t Function_Active)
{
	if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
	{
		screen_m.Function_Active=Function_Active;
		xSemaphoreGive( screen_semaphore );
		return 0;
	}
	else
		return -1;
}
int Screen_Set_Status(uint8_t Device_Status)
{
	if(xSemaphoreTake(screen_semaphore,( TickType_t )10 ) == pdTRUE )
	{
		screen_m.Device_Status=Device_Status;
		xSemaphoreGive( screen_semaphore );
		return 0;
	}
	else
		return -1;
}
