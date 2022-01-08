/*
 * Urader.c
 *
 *  Created on: 2021-9-1
 *      Author: wangzhenbao
 */
#include "can_task.h"
#include "stdio.h"
#include "canhl.h"
#include "usart.h"
#include "task_manager.h"
#include "vehicle_can.h"
#include "common.h"
#include "gpio.h"

uint8_t data1[16] = {0x55};

void Report_Vehicle_Base_Info(void)
{
	static uint32_t DisplayOldtime;
	struct can_frame displayer;
	uint8_t data1[16] = {0x0};
	uint8_t i = 0;
	uint8_t BreakFlag = 0;

	if((SystemtimeClock - DisplayOldtime) > 100)
	{
		DisplayOldtime = SystemtimeClock;
		data1[0] = 0x2E;
		data1[1] = 0x11;
		data1[2] = 0x05;
		data1[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
		if((stVehicleParas.BreakState & 0x01) == 0x01)
			BreakFlag = 1;
		if((stVehicleParas.BreakState & 0x02) == 0x02)
			BreakFlag = 2;
		if((stVehicleParas.BreakState & 0x04) == 0x04)
			BreakFlag = 3;
		if(stVehicleParas.BreakState == 0)
			BreakFlag = 0;
		data1[4] = (BreakFlag & 0x03) << 6;	//0无制动	1双目制动		2毫米波制动	3超声波制动
		if(warning_status.AEBstatus != 0)
			data1[4] = data1[4] | (1 << 5);
		data1[4] = data1[4] | (stVehicleParas.RightFlagTemp << 4);
		data1[4] = data1[4] | (stVehicleParas.LeftFlagTemp << 3);
		data1[4] = data1[4] | (stVehicleParas.BrakeFlag << 2);
		data1[4] = data1[4] | stVehicleParas.Car_Gear;
		data1[5] = camera_data.ErrorCode;		//相机错误代码
		if(stCanCommSta.stRadar.status == 1)
			data1[5] = 128;
		if(stCanCommSta.stOBD.status == 1)
			data1[5] = 129;
		if(stCanCommSta.stHRadar.status == 1)
			data1[5] = 130;
		if(stCanCommSta.stVehicle.status == 1)
			data1[5] = 131;						//AEBS控制器内部通信异常
		if(stCanCommSta.stWireless.status == 1)
			data1[5] = 132;
		data1[6] = 0xff;
		data1[7] = 0xff;
		for(i = 0;i < 8;i ++)
			data1[8] = data1[8] ^ data1[i];
		USART_Send(USART2_SFR,data1,9);
	}
}

void Report_Fcw_Warning_Info(void)
{
	struct can_frame displayer;
	uint8_t data1[20] = {0x0};
	uint8_t i = 0;
	static uint8_t state = 0;


	if((0 < CameraMessage.ttc) &&
			(CameraMessage.ttc < stSysPara.ttc1))
	{
		if(state == 0)
		{
			state = 1;
			data1[0] = 0x2E;
			data1[1] = 0x21;
			data1[2] = 0x10;
			data1[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
			if((rx_obstacle_info_b.data[4] & 0x3f) == 0x3f)
				data1[4] = 0x3f;
			else
				data1[4] = (uint8_t)(CameraMessage.ttc * 10);
			if(((uint8_t)(camera_data.HMW * 10) & 0x3f) == 0x3f)
				data1[5] = 0x3f;
			else
				data1[5] = (uint8_t)(camera_data.HMW * 10);
			data1[6] = (uint8_t)((uint16_t)obstacle_cipv_data.DistanceZ & 0xff);
			data1[7] = (uint8_t)((uint16_t)obstacle_cipv_data.DistanceZ >> 8 & 0xff);
			data1[8] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedZ & 0xff);
			data1[9] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedZ >> 8 & 0xff);
			data1[10] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedX & 0xff);
			data1[11] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedX >> 8 & 0xff);
			data1[12] = stVehicleParas.ThrottleOpening;
			data1[13] = stVehicleParas.BrakeOpening;
			data1[14] = stSWAParas.SWADegree & 0xff;
			data1[15] = (stSWAParas.SWADegree >> 8) & 0x7;
			data1[15] = data1[15] | (stSWAParas.Direction << 3);
			data1[15] = data1[15] | (stVehicleParas.Car_Gear << 4);
			data1[16] = camera_data.FCWLevel & 0x3;
			data1[16] = data1[16] | (obstacle_cipv_data.ObstacleType << 2);
			data1[16] = data1[16] | (1 << 6);
			data1[17] = 0xff;
			data1[18] = 0xff;
			//data1[9] = 0x00;
			for(i = 0;i < 19;i ++)
				data1[19] = data1[19] ^ data1[i];
			USART_Send(USART2_SFR,data1,20);
		}
	}
	else
	{
		if(state == 1)
		{
			state = 0;
			data1[0] = 0x2E;
			data1[1] = 0x21;
			data1[2] = 0x10;
			data1[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
			if((rx_obstacle_info_b.data[4] & 0x3f) == 0x3f)
				data1[4] = 0x3f;
			else
				data1[4] = (uint8_t)(CameraMessage.ttc * 10);
			if(((uint8_t)(camera_data.HMW * 10) & 0x3f) == 0x3f)
				data1[5] = 0x3f;
			else
				data1[5] = (uint8_t)(camera_data.HMW * 10);
			data1[6] = (uint8_t)((uint16_t)obstacle_cipv_data.DistanceZ & 0xff);
			data1[7] = (uint8_t)((uint16_t)obstacle_cipv_data.DistanceZ >> 8 & 0xff);
			data1[8] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedZ & 0xff);
			data1[9] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedZ >> 8 & 0xff);
			data1[10] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedX & 0xff);
			data1[11] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedX >> 8 & 0xff);
			data1[12] = stVehicleParas.ThrottleOpening;
			data1[13] = stVehicleParas.BrakeOpening;
			data1[14] = stSWAParas.SWADegree & 0xff;
			data1[15] = (stSWAParas.SWADegree >> 8) & 0x7;
			data1[15] = data1[15] | (stSWAParas.Direction << 3);
			data1[15] = data1[15] | (stVehicleParas.Car_Gear << 4);
			data1[16] = camera_data.FCWLevel & 0x3;
			data1[16] = data1[16] | (obstacle_cipv_data.ObstacleType << 2);
			data1[17] = 0xff;
			data1[18] = 0xff;
			//data1[9] = 0x00;
			for(i = 0;i < 19;i ++)
				data1[19] = data1[19] ^ data1[i];
			USART_Send(USART2_SFR,data1,20);
		}
	}
}

void Report_Hmw_Warning_Info(void)
{
	struct can_frame displayer;
	uint8_t data1[20] = {0x0};
	uint8_t i = 0;
	static uint8_t state = 0;

	if((0 < CameraMessage.hmw) &&
			(CameraMessage.hmw < stSysPara.hmw1))
	{
		if(state == 0)
		{
			state = 1;
			data1[0] = 0x2E;
			data1[1] = 0x23;
			data1[2] = 0x10;
			data1[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
			if((rx_obstacle_info_b.data[4] & 0x3f) == 0x3f)
				data1[4] = 0x3f;
			else
				data1[4] = (uint8_t)(CameraMessage.ttc * 10);
			if(((uint8_t)(camera_data.HMW * 10) & 0x3f) == 0x3f)
				data1[5] = 0x3f;
			else
				data1[5] = (uint8_t)(camera_data.HMW * 10);
			data1[6] = (uint8_t)((uint16_t)obstacle_cipv_data.DistanceZ & 0xff);
			data1[7] = (uint8_t)((uint16_t)obstacle_cipv_data.DistanceZ >> 8 & 0xff);
			data1[8] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedZ & 0xff);
			data1[9] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedZ >> 8 & 0xff);
			data1[10] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedX & 0xff);
			data1[11] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedX >> 8 & 0xff);
			data1[12] = stVehicleParas.ThrottleOpening;
			data1[13] = stVehicleParas.BrakeOpening;
			data1[14] = stSWAParas.SWADegree & 0xff;
			data1[15] = (stSWAParas.SWADegree >> 8) & 0x7;
			data1[15] = data1[15] | (stSWAParas.Direction << 3);
			data1[15] = data1[15] | (stVehicleParas.Car_Gear << 4);
			data1[16] = camera_data.FCWLevel & 0x3;
			data1[16] = data1[16] | (obstacle_cipv_data.ObstacleType << 2);
			data1[17] = 0xff;
			data1[18] = 0xff;
			//data1[9] = 0x00;
			for(i = 0;i < 19;i ++)
				data1[19] = data1[19] ^ data1[i];
			USART_Send(USART2_SFR,data1,20);		}
	}
	else
	{
		if(state == 1)
		{
			state = 0;
			data1[0] = 0x2E;
			data1[1] = 0x23;
			data1[2] = 0x10;
			data1[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
			if((rx_obstacle_info_b.data[4] & 0x3f) == 0x3f)
				data1[4] = 0x3f;
			else
				data1[4] = (uint8_t)(CameraMessage.ttc * 10);
			if(((uint8_t)(camera_data.HMW * 10) & 0x3f) == 0x3f)
				data1[5] = 0x3f;
			else
				data1[5] = (uint8_t)(camera_data.HMW * 10);
			data1[6] = (uint8_t)((uint16_t)obstacle_cipv_data.DistanceZ & 0xff);
			data1[7] = (uint8_t)((uint16_t)obstacle_cipv_data.DistanceZ >> 8 & 0xff);
			data1[8] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedZ & 0xff);
			data1[9] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedZ >> 8 & 0xff);
			data1[10] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedX & 0xff);
			data1[11] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedX >> 8 & 0xff);
			data1[12] = stVehicleParas.ThrottleOpening;
			data1[13] = stVehicleParas.BrakeOpening;
			data1[14] = stSWAParas.SWADegree & 0xff;
			data1[15] = (stSWAParas.SWADegree >> 8) & 0x7;
			data1[15] = data1[15] | (stSWAParas.Direction << 3);
			data1[15] = data1[15] | (stVehicleParas.Car_Gear << 4);
			data1[16] = camera_data.FCWLevel & 0x3;
			data1[16] = data1[16] | (obstacle_cipv_data.ObstacleType << 2);
			data1[17] = 0xff;
			data1[18] = 0xff;
			//data1[9] = 0x00;
			for(i = 0;i < 19;i ++)
				data1[19] = data1[19] ^ data1[i];
			USART_Send(USART2_SFR,data1,20);
		}
	}
}

void Report_LDW_Warning_Info(void)
{
	struct can_frame displayer;
	uint8_t data1[13] = {0x0};
	uint8_t i = 0;
	static uint8_t state = 0;

	if((camera_data.LeftLDW == 1) | (camera_data.RightLDW == 1))
	{
		if(state == 0)
		{
			state = 1;
			data1[0] = 0x2E;
			data1[1] = 0x22;
			data1[2] = 0x09;
			data1[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
			data1[4] = (rx_obstacle_info_tland.data[0] & 0xf) << 4;
			data1[4] = data1[4] | (rx_obstacle_info_tland.data[1] & 0xf);

			data1[5] = stVehicleParas.ThrottleOpening;
			data1[6] = stVehicleParas.BrakeOpening;
			data1[7] = stSWAParas.SWADegree & 0xff;
			data1[8] = (stSWAParas.SWADegree >> 8) & 0x7;
			data1[8] = data1[8] | (stSWAParas.Direction << 3);
			data1[8] = data1[8] | (stVehicleParas.Car_Gear << 4);

			data1[9] = camera_data.LeftLDW & 0x1;
			data1[9] = data1[9] | ((camera_data.RightLDW & 0x1) << 1);
			data1[9] = data1[9] | (1 << 2);
			data1[10] = 0xff;
			data1[11] = 0xff;
			for(i = 0;i < 12;i ++)
				data1[12] = data1[12] ^ data1[i];
			USART_Send(USART2_SFR,data1,13);
		}
	}
	else
	{
		if(state == 1)
		{
			state = 0;
			data1[0] = 0x2E;
			data1[1] = 0x22;
			data1[2] = 0x09;
			data1[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
			data1[4] = (rx_obstacle_info_tland.data[0] & 0xf) << 4;
			data1[4] = data1[4] | (rx_obstacle_info_tland.data[1] & 0xf);

			data1[5] = stVehicleParas.ThrottleOpening;
			data1[6] = stVehicleParas.BrakeOpening;
			data1[7] = stSWAParas.SWADegree & 0xff;
			data1[8] = (stSWAParas.SWADegree >> 8) & 0x7;
			data1[8] = data1[8] | (stSWAParas.Direction << 3);
			data1[8] = data1[8] | (stVehicleParas.Car_Gear << 4);

			data1[9] = camera_data.LeftLDW & 0x1;
			data1[9] = data1[9] | ((camera_data.RightLDW & 0x1) << 1);
			data1[10] = 0xff;
			data1[11] = 0xff;
			for(i = 0;i < 12;i ++)
				data1[12] = data1[12] ^ data1[i];
			USART_Send(USART2_SFR,data1,13);

		}
	}
}

//void Rport_Ultrasonic_Distance(void)
//{
//	uint8_t i,j;

//	for(i = 0;i < 2;)
//}

void Report_Breaking_Info(void)
{
	struct can_frame displayer;
	uint8_t data1[21] = {0x0};
	uint8_t i = 0;
	static uint8_t state = 0;
	if(stVehicleParas.AEBbriking == 1)
	{
		if(state == 0)
		{
			state = 1;
			data1[0] = 0x2E;
			data1[1] = 0x24;
			data1[2] = 0x11;
			data1[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
			if((rx_obstacle_info_b.data[4] & 0x3f) == 0x3f)
				data1[4] = 0x3f;
			else
				data1[4] = (uint8_t)(CameraMessage.ttc * 10);
			if((rx_obstacle_info_b.data[5] & 0x3f) == 0x3f)
				data1[5] = 0x3f;
			else
				data1[5] = (uint8_t)(CameraMessage.hmw * 10);
			data1[6] = (uint8_t)((uint16_t)obstacle_cipv_data.DistanceZ & 0xff);
			data1[7] = (uint8_t)((uint16_t)obstacle_cipv_data.DistanceZ >> 8 & 0xff);
			data1[8] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedZ & 0xff);
			data1[9] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedZ >> 8 & 0xff);
			data1[10] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedX & 0xff);
			data1[11] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedX >> 8 & 0xff);
			data1[12] = stVehicleParas.ThrottleOpening;
			data1[13] = stVehicleParas.BrakeOpening;
			data1[14] = stSWAParas.SWADegree & 0xff;
			data1[15] = (stSWAParas.SWADegree >> 8) & 0x7;
			data1[15] = data1[15] | (stSWAParas.Direction << 3);
			data1[15] = data1[15] | (stVehicleParas.Car_Gear << 4);
			data1[16] = stVehicleParas.Ultrasonicdistance;
			data1[17] = obstacle_cipv_data.ObstacleType;
			data1[17] = data1[17] | (1 << 4);
			data1[18] = 0xff;
			data1[19] = 0xff;
			for(i = 0;i < 20;i ++)
				data1[20] = data1[20] ^ data1[i];
			USART_Send(USART2_SFR,data1,21);
		}
	}
	else
	{
		if(state == 1)
		{
			state = 0;
			data1[0] = 0x2E;
			data1[1] = 0x24;
			data1[2] = 0x11;
			data1[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
			if((rx_obstacle_info_b.data[4] & 0x3f) == 0x3f)
				data1[4] = 0x3f;
			else
				data1[4] = (uint8_t)(CameraMessage.ttc * 10);
			if((rx_obstacle_info_b.data[5] & 0x3f) == 0x3f)
				data1[5] = 0x3f;
			else
				data1[5] = (uint8_t)(CameraMessage.hmw * 10);
			data1[6] = (uint8_t)((uint16_t)obstacle_cipv_data.DistanceZ & 0xff);
			data1[7] = (uint8_t)((uint16_t)obstacle_cipv_data.DistanceZ >> 8 & 0xff);
			data1[8] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedZ & 0xff);
			data1[9] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedZ >> 8 & 0xff);
			data1[10] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedX & 0xff);
			data1[11] = (uint8_t)((uint16_t)obstacle_cipv_data.RelativeSpeedX >> 8 & 0xff);
			data1[12] = stVehicleParas.ThrottleOpening;
			data1[13] = stVehicleParas.BrakeOpening;
			data1[14] = stSWAParas.SWADegree & 0xff;
			data1[15] = (stSWAParas.SWADegree >> 8) & 0x7;
			data1[15] = data1[15] | (stSWAParas.Direction << 3);
			data1[15] = data1[15] | (stVehicleParas.Car_Gear << 4);
			data1[16] = stVehicleParas.Ultrasonicdistance;
			data1[17] = obstacle_cipv_data.ObstacleType;
			data1[18] = 0xff;
			data1[19] = 0xff;
			for(i = 0;i < 20;i ++)
				data1[20] = data1[20] ^ data1[i];
			USART_Send(USART2_SFR,data1,21);
		}
	}
}

void Report_Dangerous_Driving_Info(void)
{
	struct can_frame displayer;
	uint8_t data1[13] = {0x0};
	uint8_t i = 0;
	static uint8_t state = 0;
	uint8_t BreakFlag = 0;

	if((stVehicleParas.RightFlagTemp == 1) || (stVehicleParas.LeftFlagTemp == 1) || (stVehicleParas.BrakeFlag == 1))
	{
		if(state == 0)
		{
			state = 1;
			data1[0] = 0x2E;
			data1[1] = 0x30;
			data1[2] = 0x09;
			data1[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
			if((stVehicleParas.BreakState & 0x01) == 0x01)
				BreakFlag = 1;
			if((stVehicleParas.BreakState & 0x02) == 0x02)
				BreakFlag = 2;
			if((stVehicleParas.BreakState & 0x04) == 0x04)
				BreakFlag = 3;
			if(stVehicleParas.BreakState == 0)
				BreakFlag = 0;
			data1[4] = (BreakFlag & 0x03) << 6;	//0无制动	1双目制动		2毫米波制动	3超声波制动
			if(warning_status.AEBstatus != 0)
				data1[4] = data1[4] | (1 << 5);
			data1[4] = data1[4] | (stVehicleParas.RightFlagTemp << 4);
			data1[4] = data1[4] | (stVehicleParas.LeftFlagTemp << 3);
			data1[4] = data1[4] | (stVehicleParas.BrakeFlag << 2);
			data1[4] = data1[4] | stVehicleParas.Car_Gear;
			data1[5] = camera_data.ErrorCode;		//相机错误代码
			if(stCanCommSta.stRadar.status == 1)
				data1[5] = 128;
			if(stCanCommSta.stOBD.status == 1)
				data1[5] = 129;
			if(stCanCommSta.stHRadar.status == 1)
				data1[5] = 130;
			if(stCanCommSta.stVehicle.status == 1)
				data1[5] = 131;						//AEBS控制器内部通信异常
			if(stCanCommSta.stWireless.status == 1)
				data1[5] = 132;
			data1[6] = stVehicleParas.ThrottleOpening;
			data1[7] = stVehicleParas.BrakeOpening;
			data1[8] = stSWAParas.SWADegree & 0xff;
			data1[9] = (stSWAParas.SWADegree >> 8) & 0x7;
			data1[9] = data1[9] | (stSWAParas.Direction << 3);
			data1[10] = 0xff;
			data1[11] = 0xff;
			for(i = 0;i < 12;i ++)
				data1[12] = data1[12] ^ data1[i];
			USART_Send(USART2_SFR,data1,13);
		}
	}
	else
	{
		if(state == 1)
		{
			state = 0;
		}
	}
}

void Report_Warning_Info(void)
{
	Report_Fcw_Warning_Info();
	Report_LDW_Warning_Info();
	Report_Hmw_Warning_Info();
	Report_Breaking_Info();
	Report_Dangerous_Driving_Info();
}

