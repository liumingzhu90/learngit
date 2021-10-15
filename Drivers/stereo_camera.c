/*
 * stereo_camera.c
 *
 *  Created on: 2021-6-22
 *      Author: shuai
 */
#include "stereo_camera.h"
#include "data_center_services.h"
#include "can_task.h"
#include "common.h"

QueueHandle_t CAN1_Rxqueue_Handler = NULL;

struct can_frame rx_camera_data; //0x79F
struct can_frame rx_obstacle_info; //0x7A0
struct can_frame rx_obstacle_info_a; //0x7A1
struct can_frame rx_obstacle_info_b; //0x7A2
struct can_frame rx_obstacle_info_tland; //0x7A3
struct can_frame rx_obstacle_info_lbll; //0x7A4
struct can_frame rx_obstacle_info_lbrl; //0x7A5
struct can_frame rx_obstacle_info_rbll; //0x7A6
struct can_frame rx_obstacle_info_rbrl; //0x7A7
struct can_frame tx_camera_input; //0x7B0

Camera_Essential_Data camera_data;
Obstacle_Basic_Data obstacle_Basic_data;
Obstacle_Information obstacle_cipv_data;
Camera_All_Info CameraMessage;
Camera_LDW_data camera_LDW_data;	// add lmz 20211011

uint8_t Camera_Info_Push_Flag;
uint8_t cipv_check;

void stere_camera_data_analysis()
{
	//0x7A1数据解析
	obstacle_cipv_data.ObstacleID = rx_obstacle_info_a.data[0];
	obstacle_cipv_data.TrackNumber = rx_obstacle_info_a.data[1] & 0x3F;
	obstacle_cipv_data.ObstacleWidth = (float)(rx_obstacle_info_a.data[2] + ((rx_obstacle_info_a.data[3] & 0x0F) << 8))*0.01;
	obstacle_cipv_data.ObstacleHeight = (float)(((rx_obstacle_info_a.data[3] & 0xF0) >> 4) + (rx_obstacle_info_a.data[4] << 4))*0.01;
	obstacle_cipv_data.RelativeSpeedZ = (float)(rx_obstacle_info_a.data[5] + ((rx_obstacle_info_a.data[6] & 0x0F) << 8))*0.625 - 127.9375;
	obstacle_cipv_data.DistanceY = (float)(((rx_obstacle_info_a.data[6] & 0xF0) >> 4) + (rx_obstacle_info_a.data[7] << 4))*0.01 -20.47;
	//0x7A2数据解析
	obstacle_cipv_data.DistanceX = (float)(rx_obstacle_info_b.data[0] + ((rx_obstacle_info_b.data[1] & 0x7F) << 8))*0.01 -163.83;
	obstacle_cipv_data.DistanceZ = (float)(rx_obstacle_info_b.data[2] + ((rx_obstacle_info_b.data[3] & 0x7F) << 8))*0.01;
	obstacle_cipv_data.TTC = (float)(rx_obstacle_info_b.data[4] & 0x3F)*0.1;
	obstacle_cipv_data.HMW = (float)(((rx_obstacle_info_b.data[4] & 0xC0) >> 6) + ((rx_obstacle_info_b.data[5] & 0x0F) << 2))*0.1;
	obstacle_cipv_data.RelativeSpeedX = (float)((rx_obstacle_info_b.data[6] + ((rx_obstacle_info_b.data[7] & 0x03) << 8)))*0.1 - 51.1;
	obstacle_cipv_data.CIPV = (rx_obstacle_info_b.data[7] & 0x04) >> 2;
	obstacle_cipv_data.ObstacleType = (rx_obstacle_info_b.data[7] & 0x78) >> 3;

	//因双目输出HMWGrade不可靠，自己计算。
	if(obstacle_cipv_data.HMW <= obstacle_Basic_data.HMWAlarmThreshold)
	{
		camera_data.HMWGrade = 2;
	}else if(obstacle_cipv_data.HMW <= 2.7 )
	{
		camera_data.HMWGrade = 1;
	}else{
		camera_data.HMWGrade = 3;
	}
}

void Camera_CIPV_Init()
{
	obstacle_cipv_data.ObstacleID = 0;
	obstacle_cipv_data.TrackNumber = 0;
	obstacle_cipv_data.ObstacleWidth = 40.95;
	obstacle_cipv_data.ObstacleHeight = 40.95;
	obstacle_cipv_data.RelativeSpeedZ = 128.0;
	obstacle_cipv_data.DistanceY = 20.47;
	obstacle_cipv_data.DistanceX = 163.83;
	obstacle_cipv_data.DistanceZ = 327.67;
	obstacle_cipv_data.TTC = 6.3;
	obstacle_cipv_data.HMW = 6.3;
	obstacle_cipv_data.RelativeSpeedX = 51.1;
	obstacle_cipv_data.CIPV = 0;
	obstacle_cipv_data.ObstacleType = 0;
}

void Camera_Init()
{
	rx_camera_data.TargetID = CAMERA_ESSENTIAL_DATA_ID;
	rx_camera_data.lenth = 8;
	rx_camera_data.MsgType = CAN_DATA_FRAME;
	rx_camera_data.RmtFrm = CAN_FRAME_FORMAT_SFF;
	rx_camera_data.RefreshRate = CAMERA_ESSENTIAL_REFRESH_RATE;

	rx_obstacle_info.TargetID = CAMERA_OBSTACLE_INFO_ID;
	rx_obstacle_info.lenth = 8;
	rx_obstacle_info.MsgType = CAN_DATA_FRAME;
	rx_obstacle_info.RmtFrm = CAN_FRAME_FORMAT_SFF;
	rx_obstacle_info.RefreshRate = CAMERA_OBSTACLE_INFO_REFRESH_RATE;

	rx_obstacle_info_a.TargetID = CAMERA_OBSTACLE_INFO_A_ID;
	rx_obstacle_info_a.lenth = 8;
	rx_obstacle_info_a.MsgType = CAN_DATA_FRAME;
	rx_obstacle_info_a.RmtFrm = CAN_FRAME_FORMAT_SFF;
	rx_obstacle_info_a.RefreshRate = CAMERA_OBSTACLE_INFO_A_REFRESH_RATE;

	rx_obstacle_info_b.TargetID = CAMERA_OBSTACLE_INFO_B_ID;
	rx_obstacle_info_b.lenth = 8;
	rx_obstacle_info_b.MsgType = CAN_DATA_FRAME;
	rx_obstacle_info_b.RmtFrm = CAN_FRAME_FORMAT_SFF;
	rx_obstacle_info_b.RefreshRate = CAMERA_OBSTACLE_INFO_B_REFRESH_RATE;

	tx_camera_input.TargetID = CAMERA_INPUT_DATA_ID;
	tx_camera_input.lenth = 8;
	tx_camera_input.MsgType = CAN_DATA_FRAME;
	tx_camera_input.RmtFrm = CAN_FRAME_FORMAT_SFF;
	tx_camera_input.RefreshRate = CAMERA_INPUT_DATA_REFRESH_RATE;

	for(int i=0; i<8; i++)
	{
		rx_camera_data.data[i] = 0xFF;
		rx_obstacle_info.data[i] = 0xFF;
		rx_obstacle_info_a.data[i] = 0xFF;
		rx_obstacle_info_b.data[i] = 0xFF;
		tx_camera_input.data[i] = 0xFF;
	}

	camera_data.LeftLDW = 0;
	camera_data.RightLDW = 0;
	camera_data.LDWSentivity = 0;
	camera_data.Invalid = 0;
	camera_data.OffSound = 0;
	camera_data.HMWGrade = 0;
	camera_data.HMW = 6.3;
	camera_data.ErrorCode = 0;
	camera_data.HMWEnable = 0;
	camera_data.FCWLevel = 0;
	camera_data.AmbientLuminance = 0;
	camera_data.FCWStatus = 0;

	obstacle_Basic_data.ObstacleNumber = 0;
	obstacle_Basic_data.HMWAlarmThreshold = 0.0;

	Camera_CIPV_Init();

	Camera_Info_Push_Flag = 0;
	cipv_check = 0;

	Camera_Info_Push(&camera_data, &obstacle_Basic_data, &obstacle_cipv_data);//数据中心数据初始化。
}

void Camera_CAN_Transmition(struct can_frame *tx_frame,float speed)
{
	tx_camera_input.data[1] = (uint8_t)speed; //车速60km/h
	*tx_frame = tx_camera_input;
}

void Urader_CAN_Transmition(struct can_frame *tx_frame,float speed)
{
	tx_camera_input.data[0] = ((uint16_t)(speed) >> 4) & 0xff; //低字节在前
	tx_camera_input.data[1] = (((uint16_t)(speed))) & 0xf;
	//turnleft = stVehicleParas.LeftFlagTemp;
	//turnright = stVehicleParas.RightFlagTemp;
	//tx_camera_input
	if(stVehicleParas.LeftFlagTemp == 1)
		tx_camera_input.data[3] = 0x09;
	else if(stVehicleParas.RightFlagTemp == 1)
		tx_camera_input.data[3] = 0x0a;
	else
		tx_camera_input.data[3] = 0;
	*tx_frame = tx_camera_input;
}

void Camera_CAN_Analysis(struct can_frame *rx_frame)
{
	switch(rx_frame->TargetID){
	case CAMERA_ESSENTIAL_DATA_ID:
		if(Camera_Info_Push_Flag == 0)
		{
			Camera_CIPV_Init();//当上一周期没有CIPV为1的障碍物时，补发上一周的基础数据。
			camera_data.HMWGrade = 0;
			CameraMessage.ttc = 0;
			CameraMessage.hmw = 0;
			Camera_Info_Push(&camera_data, &obstacle_Basic_data, &obstacle_cipv_data);//解析后的数据提交数据中心
		}else{
			Camera_Info_Push_Flag == 0;
		}

		rx_camera_data = *rx_frame;
		//0x79f数据解析
		camera_data.LeftLDW = (rx_camera_data.data[0] & 0x80) >> 7;
		camera_data.RightLDW = (rx_camera_data.data[0] & 0x40) >> 6;
		camera_data.LDWSentivity = (rx_camera_data.data[0] & 0x30) >> 4;
		camera_data.Invalid = (rx_camera_data.data[0] & 0x0E) >> 1;
		camera_data.OffSound = rx_camera_data.data[0] & 0x01;
		camera_data.HMWGrade = (rx_camera_data.data[1] & 0xC0) >> 6; //输出数据不可靠
		camera_data.HMW = (float)(rx_camera_data.data[1] & 0x3F)*0.1;
		camera_data.ErrorCode = (rx_camera_data.data[2] & 0xFE) >> 1;
		camera_data.HMWEnable = rx_camera_data.data[2] & 0x01;
		camera_data.FCWLevel = (rx_camera_data.data[5] & 0xC0) >> 6;
		camera_data.AmbientLuminance = (rx_camera_data.data[5] & 0x38) >> 3;
		camera_data.FCWStatus = (rx_camera_data.data[5] & 0x04) >> 2;
		break;
	case CAMERA_OBSTACLE_INFO_ID:
		rx_obstacle_info = *rx_frame;
		//0x7A0数据解析
		obstacle_Basic_data.ObstacleNumber = rx_obstacle_info.data[0];
		obstacle_Basic_data.HMWAlarmThreshold = (float)(rx_obstacle_info.data[7] & 0x1F)*0.1;
		break;
	case CAMERA_OBSTACLE_INFO_A_ID:
		rx_obstacle_info_a = *rx_frame;
		break;
	case CAMERA_OBSTACLE_INFO_B_ID:
		rx_obstacle_info_b = *rx_frame;

		cipv_check = (rx_obstacle_info_b.data[7] & 0x04) >> 2;
		//过滤CIPV为1的障碍物
		if(cipv_check == 1)
		{
			stere_camera_data_analysis();
			Camera_Info_Push(&camera_data, &obstacle_Basic_data, &obstacle_cipv_data);//解析后的数据提交数据中心。
			Camera_Info_Push_Flag = 1;//发送带有CIPV为1的数据包，并标记数据已发送。

			//turnleft = stVehicleParas.LeftFlagTemp;
			//turnright = stVehicleParas.RightFlagTemp;

			//if(turnleft | turnright)
			//	ret_pressure = 0;
			//if(stVehicleParas.BrakeFlag == 1)
			//	ret_pressure = 0;
			//if(warning_status.AEBstatus != 0)
			//	ret_pressure = 0;
			//if((stVehicleParas.LeftFlagTemp == 0) |
			//		(stVehicleParas.LeftFlagTemp == 0) |
			//		(stVehicleParas.BrakeFlag == 0) |
			//		(warning_status.AEBstatus == 0)
			//		)
			//if((obstacle_cipv_data.HMW < 2.0) && (obstacle_cipv_data.HMW > 1.5))
			//{
			//	Camera_Info_Push_Flag = 0;
			//}
			CameraMessage.ttc = obstacle_cipv_data.TTC;
			CameraMessage.hmw = obstacle_cipv_data.HMW;
			//else
			//	CameraMessage.ttc = 0;
			//BaseType_t ret = xQueueSend(CAN1_Rxqueue_Handler, ( void * )&CameraMessage, 1);
			//if(ret == pdFALSE)
			//{
			//	UART_Puts("Stereo Camera xQueueSend fail.\r\n");
			//}
		}
		else
		{
			CameraMessage.ttc = 0;
			CameraMessage.hmw = 0;
		}
		break;
	case 0x7A3:
		rx_obstacle_info_tland = *rx_frame;
		camera_LDW_data.LeftLaneStyle = rx_obstacle_info_tland.data[0] & 0x0F;
		camera_LDW_data.LeftLaneStatus = rx_obstacle_info_tland.data[0]&0x30 >> 4;
		camera_LDW_data.RightLaneStyle = rx_obstacle_info_tland.data[1] & 0x0F;
		camera_LDW_data.RightLaneStatus = rx_obstacle_info_tland.data[1]&0x30 >> 4;
		camera_LDW_data.CarDepth = rx_obstacle_info_tland.data[2]*0.01;
		camera_LDW_data.SpeedOfRushLane = (rx_obstacle_info_tland.data[3] + (rx_obstacle_info_tland.data[4]&0x03)<<8)*0.1 - 51.1;
		break;
	case 0x7A4:
		rx_obstacle_info_lbll = *rx_frame;
		break;
	case 0x7A5:
		rx_obstacle_info_lbrl = *rx_frame;
		break;
	case 0x7A6:
		rx_obstacle_info_rbll = *rx_frame;
		break;
	case 0x7A7:
		rx_obstacle_info_rbrl = *rx_frame;
		break;
	default:
		break;
	}
}
