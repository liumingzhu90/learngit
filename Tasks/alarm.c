/*
 * alarm.c
 *
 *  Created on: 2021-8-6
 *      Author: wangzhenbao
 */
#include "system_init.h"
#include "vehicle_can.h"
#include <stdio.h>
#include "common.h"

extern Vehicle_Info veh_info;
extern uint32_t Time14_CNT;

void Sys_Brake_Ctrl(void)
{
	static uint32_t ulPreTimeStmp = 0;
	uint32_t ulCurTimeStmp = 0;
	uint8_t i;

	ulCurTimeStmp = SystemtimeClock;
	//if (ulCurTimeStmp - ulPreTimeStmp >= 130)
	{
		ulPreTimeStmp = ulCurTimeStmp;
		i = GPIO_Read_Input_Data_Bit(GPIOH_SFR, GPIO_PIN_MASK_12);
		if(i == 1)
		{
			//warning_status.AEBstatus = warning_status.AEBstatus | 0x01;
		}
		else
		{
			//warning_status.AEBstatus = warning_status.AEBstatus & 0xFE;
		}
		/* 输出刹车控制 */

	}
	Send_Break_Control();
}

void Sys_Send_Break(void)
{
	static uint32_t ulPreTime = 0;
	uint32_t ulCurTimeStmp = 0;
	uint8_t i;

	ulCurTimeStmp = SystemtimeClock;
	if (ulCurTimeStmp - ulPreTime >= 50)
	{
		ulPreTime = ulCurTimeStmp;
		/* 输出刹车控制 */
		Send_Break_Control();
	}
}

void Send_Buf_To_Pc(uint32_t id,uint8_t *buf)
{
	struct can_frame tx_frame;
	uint8_t i = 0;

	tx_frame.TargetID = id;
	tx_frame.lenth = 8;
	tx_frame.MsgType = CAN_DATA_FRAME;
	tx_frame.RmtFrm = CAN_FRAME_FORMAT_SFF;
	tx_frame.RefreshRate = CAMERA_INPUT_DATA_REFRESH_RATE;

	for(i = 0;i < 8;i ++)
		tx_frame.data[i] = buf[i];

	CAN_Transmit_DATA(CAN0_SFR,tx_frame);
}

void Send_Warning_Message(void)
{
	struct can_frame tx_frame;

	//for(i = 0;i < 8;i ++)
	//	buff[i] = 0xff;
	Camera_CAN_Transmition(&tx_frame,stVehicleParas.fVehicleSpeed);
	if(camera_share.CammeraEssentialData.LeftLDW == 1)
		tx_frame.data[0] = 0x01;
	else
		tx_frame.data[0] = 0x00;												//左车道偏离预警byte0 bit0

	if(camera_share.CammeraEssentialData.RightLDW == 1)
		tx_frame.data[0] = tx_frame.data[0] | (0x01 << 1);
	else
		tx_frame.data[0] = tx_frame.data[0] & 0xFD;									//左车道偏离预警byte0 bit1

	if((0 < camera_share.ObsInormation.TTC) &&
			(camera_share.ObsInormation.TTC < 6.0))
		tx_frame.data[0] = tx_frame.data[0] | (1 << 2);
	else
		tx_frame.data[0] = tx_frame.data[0] & 0xFB;

	if((0 < camera_share.ObsInormation.TTC) &&
			(camera_share.ObsInormation.TTC < stSysPara.ttc1))
		tx_frame.data[0] = tx_frame.data[0] | (1 << 3);
	else
		tx_frame.data[0] =tx_frame.data[0] & 0xF7;

	tx_frame.data[1] = tx_valve_control.data[0];
	tx_frame.data[2] = tx_valve_control.data[1];
	tx_frame.TargetID = 0x7C0;

	CAN_Transmit_DATA(CAN0_SFR,tx_frame);

}

void Send_Targt_Message(void)			//目标信息
{
	uint8_t buff[8];

	buff[0] = camera_share.ObsInormation.ObstacleType;												//障碍物类型

	buff[1] = rx_obstacle_info_b.data[2];
	buff[2] = rx_obstacle_info_b.data[3];
	buff[3] = obstacle_cipv_data.TTC;

	Send_Buf_To_Pc(0x7C1,buff);
}

void Send_Sys_Status(void)			//系统信息
{

	uint8_t buff[8];

	if(stCanCommSta.stVehicle.status == ONLINE)
		buff[0] = 0x01;
	else
		buff[0] = 0x00;

	if(stCanCommSta.stCamera.status == ONLINE)
		buff[0] = buff[0] | (0x01 << 1);
	else
		buff[0] = buff[0] & 0xFD;

	if(stCanCommSta.stRadar.status == ONLINE)
		buff[0] = buff[0] | (1 << 2);
	else
		buff[0] = buff[0] & 0xFB;

	if(stCanCommSta.stHRadar.status == ONLINE)
		buff[0] = buff[0] | (1 << 3);
	else
		buff[0] = buff[0] & 0xF7;

	if(stCanCommSta.Proportional_valve.status == ONLINE)
		buff[0] = buff[0] | (1 << 5);
	else
		buff[0] = buff[0] & 0xEF;

	Send_Buf_To_Pc(0x7C2,buff);

}

void Send_Vehicle_Status(void)			//车辆信息
{

	uint8_t buff[8];
	buff[0] = stVehicleParas.BrakeFlag;
	buff[0] = buff[0] | (stVehicleParas.LeftFlagTemp << 1);
	buff[0] = buff[0] | (stVehicleParas.RightFlagTemp << 2);

	buff[1] = stVehicleParas.fVehicleSpeed;
	if(warning_status.AEBstatus != 0)
	{
		buff[2] = 0x01;
	}
	else
	{
		buff[2] = 0x00;
	}
	//if(warning_status.LDWstatus )
	buff[2] = buff[2] | (warning_status.LDWstatus << 1);
	Send_Buf_To_Pc(0x7C3,buff);
}

void Send_Soft_Version(void)			//软件信息
{

	uint8_t buff[8];

	//buff[0] = camera_share.ObsInormation.ObstacleType;												//障碍物类型

	//buff[1] = rx_obstacle_info_b.data[2];
	//buff[2] = rx_obstacle_info_b.data[3];
	//buff[3] = obstacle_cipv_data.TTC;

	Send_Buf_To_Pc(0x7C4,buff);
}
