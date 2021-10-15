/*
 * can_task.h
 *
 *  Created on: 2021-6-17
 *      Author: shuai
 */

#ifndef CAN_TASK_H_
#define CAN_TASK_H_
#include "system_init.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "proportional_valve.h"
#include "stereo_camera.h"
#include "displayer.h"

extern QueueHandle_t CAN1_Analysis_Queue ;

void CAN1_Task_Init();
void CAN1_Txtask(void *pvParameters);
void CAN1_Rxtask(void *pvParameters);
void CAN1_Analysis_Task(void *pvParameters);
void Vehicle_Parameter_Analysis(uint32_t ulSysTime);

#define can0message0 0x18ff100
#define can0message1 0x18ff101
#define can0message2 0x18ff102

#define can1message0 0x18ff100
#define can1message1 0x18ff101
#define can1message2 0x18ff102

#define can2message0 0x18FEF100
#define can2message1 0x18ff101
#define can2message2 0x18ff102

#define can3message0 0x08FF1081
#define can3message1 0x18FF0A81

#define can4message0 0x18ff100
#define can4message1 0x18ff101
#define can4message2 0x18ff102
//can 数据结构体
typedef struct
{
	uint8_t	flgBoxRxEnd;					// message box1 接收完成标记
	uint8_t	Box[8];						// 存储缓冲区
}_STRUCT_VEHICLE_MESSAGE;

/* 结构体--比例阀信息 */
typedef struct
{
	uint32_t 	TimeStamp;		/* time stamp: 0x00000000-0xffffffff,LSB=1[mS] */
    uint8_t   Valve_State;        /*valve state: 0 Not used,1 pressure control,2 fail safe,3 other*/
    float   Internal_Temp;      /*valve temperature:-40-150*/
    float   Actual_Pressure;    /* actual pressure, LSB=1/128[kpa]*/
    float   Target_Pressure;    /*Target Pressure, LSB=1/128[kpa]*/
    uint8_t   Fault_Code;         /*fault code:0 no fault,1 internal fault,2 low voltage,3 high voltage,4 big pressure delta,
    							5 never receive pressure control order,6 pressure control order lost,7 last period CAN bus off*/
    uint8_t    IsValveDelay;       /* valve can not output expected pressure */
}_STRUCT_VALVE_PARA   ;

// CAN通信状态
enum _ENUM_CAN_COMM_STA
{
	OFFLINE=0,
	ONLINE=1
};
//--------------------------------------------------------------------------------------------------
// 结构体--CAN通信单个信息结构
typedef struct
{
	enum	_ENUM_CAN_COMM_STA 		status;			// 在线状态
	uint32_t	oldSysTime;							// 上次更新的系统时间
}_STRUCT_CAN_PARA;

typedef struct
{
	_STRUCT_CAN_PARA	stOBD;						// OBD
	_STRUCT_CAN_PARA	stHRadar;					// 超声波雷达CAN
	_STRUCT_CAN_PARA	stRadar;					// 毫米波雷达CAN
	_STRUCT_CAN_PARA	stCamera;					// 摄像头CAN
	_STRUCT_CAN_PARA	stVehicle;					// 车辆CAN
	//----------------新增-----------------------
	_STRUCT_CAN_PARA    Proportional_valve;
	_STRUCT_CAN_PARA    stTurnSig;  /* 转向信号CAN */
	_STRUCT_CAN_PARA    stBrakeSig; /* 刹车信号CAN */
	_STRUCT_CAN_PARA    stSpeed; /* 车速CAN */
	_STRUCT_CAN_PARA    stWireless; /* 4G */
	_STRUCT_CAN_PARA    stWarning; /* 报警 */
}_STRUCT_CAN_COMM_STA;

#endif /* CAN_TASK_H_ */
