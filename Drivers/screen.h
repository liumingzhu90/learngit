/*
 * screen.h
 *
 *  Created on: 2021-6-18
 *      Author: chenq
 */

#ifndef SCREEN_H_
#define SCREEN_H_

#include "system_init.h"
#include "canhl.h"


typedef enum
{
    STATUS_RUNNING = 0,
    STATUS_CHECKING = 1,
    STATUS_FAULT = 2,
    STATUS_DISABLE = 3
}WORK_STATUS_T;

typedef enum
{
    NO_LDW_WARNING = 0,
    LEFT_WARNING = 1,
    RIGHT_WARNING = 2,
    MISS_WARNING = 3
}LDW_WARNING_T;

typedef enum
{
    NO_DMS_WARNING = 0,
    TIRED_WARNING = 1,
    ABNORMAL_WARNING = 2,
    PHONE_WARNING = 3,
    RESERVE_WARNING = 4
}DMS_WARNING_T;

typedef enum
{
    NONE_OBSTACLE = 0,
    LONG_COLLIED_TIME = 1,
    MID_COLLIED_TIME = 2,
    SHORT_COLLIED_TIME = 3
}FCW_WARNING_T;

typedef enum
{
	NO_LINE=0,
	VIRTUAL_LINE=0,
	DOTTED_LINE=1,
	SOLID_LINE=2
}Lane_Type;

typedef enum
{
	NO_TARGET=0,
	HUMAN=1,
	VEHICLE=2,
	OTHERS=3
}Target_Type;

#define CAN_ID1 0x18FEE0D8
#define CAN_ID2 0x18FEE1D8

#define TTC_Active_Mask		0x80
#define DTC_Active_Mask		0x40
#define SLI_Active_Mask		0x04
#define AATS_Active_Mask	0x02
#define AEBS_Active_Mask	0x01

#define Front_Radar_Status 				0x80
#define Front_Camera_Status				0x40
#define DMS_Camera_Status				0x20
#define Vehicle_CANBus_Status			0x10
#define Front_Camera_Cleaning_Request	0x08
#define DMS_Camera_Cleaning_Request		0x04
#define Ethernet_Status					0x01


struct screen_t
{
	//CAN_ID1
	WORK_STATUS_T LDW_Status;
	WORK_STATUS_T DMS_Status;
	LDW_WARNING_T LDW_Warning;
	DMS_WARNING_T DMS_Warning;
	uint8_t Lane_Width;		//unit 0.25m  range 0~4m
	Lane_Type Left_Line;
	Lane_Type Right_Line;
	uint8_t Lane_curvature;	//unit 4  range 0~1020
	uint8_t Space_to_Left;	//unit 0.125  range 0~3.75
	uint8_t Space_to_Right;	//unit 0.125  range 0~3.75
	uint8_t Head_Angle;		//unit 1  range -90~90
	//CAN_ID2
	WORK_STATUS_T FCW_Status;
	WORK_STATUS_T AEBS_Status;
	WORK_STATUS_T Anti_Fault;
	FCW_WARNING_T FCW_Warning;
	Target_Type Target;
	uint8_t Distance;		//unit 0.5  range 0~127.5
	uint8_t Speed;			//unit 0.25  range 0~63.75
	uint8_t TTC;			//unit 0.05  range 0~12.75
	uint8_t SLI_Speed;		//unit 1  range 0~255
	uint8_t Function_Active;
	uint8_t Device_Status;
};


void Screen_Init(CAN_SFRmap* CANx,uint8_t Bdrt);
int Screen_Set_Value(struct screen_t screen);
int Screen_Get_Value(struct screen_t *screen);
//int Screen_Start(int Period);//0:stop others:can_send
int Screen_Set_LDW_Status(WORK_STATUS_T LDW_Status);
int Screen_Set_DMS_Status(WORK_STATUS_T DMS_Status);
int Screen_Set_LDW_Warning(LDW_WARNING_T LDW_Warning);
int Screen_Set_DMS_Warning(DMS_WARNING_T DMS_Warning);
int Screen_Set_Lane_Width(uint8_t Lane_Width);
int Screen_Set_Left_Line(Lane_Type Left_Line);
int Screen_Set_Right_Line(Lane_Type Right_Line);
int Screen_Set_Lane_curvature(uint8_t Lane_curvature);	//unit 4  range 0~1020
int Screen_Set_Space_to_Left(uint8_t Space_to_Left);	//unit 0.125  range 0~3.75
int Screen_Set_Space_to_Right(uint8_t Space_to_Right);	//unit 0.125  range 0~3.75
int Screen_Set_Head_Angle(uint8_t Head_Angle);		//unit 1  range -90~90
int Screen_Set_Distance(uint8_t Distance);
int Screen_Set_TTC(uint8_t TTC);
int Screen_Set_Speed(uint8_t Speed);
int Screen_Set_SLI_Speed(uint8_t SLI_Speed);
int Screen_Set_Active(uint8_t Function_Active);
int Screen_Set_Status(uint8_t Device_Status);
CAN_ErrorT Screen_CAN_Send();

#endif /* SCREEN_H_ */
