/*
 * displayer.c
 *
 *  Created on: 2021-7-7
 *      Author: shuai
 */
#include "displayer.h"
#include "data_center_services.h"
#include "common.h"

Displayer_Show displayer_show_info;
BaseType_t isAEBTaskClose;
BaseType_t isLDWTaskClose;

struct can_frame displayer_tx_frame;

void Displayer_Init()
{
	displayer_show_info.FrontCameraStatus = 0;
	displayer_show_info.MillimeterWaveRadarStatus = 0;
	displayer_show_info.UltrasonicRadarStatus = 0;
	displayer_show_info.ValveStatus = 0;
	displayer_show_info.VehicleCANBusStatus = 0;
	displayer_show_info.VehicleSpeedStatus = 0;
	displayer_show_info.Module4GStatus = 0;
	displayer_show_info.GPSStatus = 0;
	displayer_show_info.AEBSTaskStatus = 2;
	displayer_show_info.FCWTaskStatus = 2;
	displayer_show_info.LDWTaskStatus = 2;
	displayer_show_info.SSSTaskStatus = 2;
	displayer_show_info.CMSTaskStatus = 2;
	displayer_show_info.FCWLevel = 0;
	displayer_show_info.LDW = 0;
	displayer_show_info.HMWGrade = 0;
	displayer_show_info.DigitalDisplay = 0;
	displayer_show_info.HMWTime = 6.3;
	displayer_show_info.LengthwaysRelativeSpeed = 128.0;
	displayer_show_info.TTC = 6.3;
	displayer_show_info.LengthwaysDistance = 204.0;
	displayer_show_info.ObstacleType = 0;

	isAEBTaskClose = pdFALSE;
	isLDWTaskClose = pdFALSE;

	displayer_tx_frame.TargetID = DISPLAYER_SHOW_ID;
	displayer_tx_frame.lenth = 8;
	displayer_tx_frame.MsgType = CAN_DATA_FRAME;
	displayer_tx_frame.RmtFrm = CAN_FRAME_FORMAT_EFF;
	displayer_tx_frame.RefreshRate = DISPLAYER_SHOW_REFRESH_RATE;

	for(int i=0; i<8; i++)
	{
		displayer_tx_frame.data[i] = 0xFF;
	}
}
BaseType_t Displayer_CANTx_Get(struct can_frame *tx_frame)
{
	Camera_Info_Push(&camera_data, &obstacle_Basic_data, &obstacle_cipv_data);//解析后的数据提交数据中心
	Displayer_Pull(&displayer_show_info);//更新显示数据。

	displayer_tx_frame.data[0] = 0x00;
	displayer_tx_frame.data[0] |= displayer_show_info.FrontCameraStatus;
	displayer_tx_frame.data[0] |= displayer_show_info.MillimeterWaveRadarStatus << 1;
	displayer_tx_frame.data[0] |= displayer_show_info.UltrasonicRadarStatus << 2;
	displayer_tx_frame.data[0] |= displayer_show_info.ValveStatus << 3;
	displayer_tx_frame.data[0] |= displayer_show_info.VehicleCANBusStatus << 4;
	displayer_tx_frame.data[0] |= displayer_show_info.VehicleSpeedStatus << 5;
	displayer_tx_frame.data[0] |= displayer_show_info.Module4GStatus << 6;
	displayer_tx_frame.data[0] |= displayer_show_info.GPSStatus << 7;

	displayer_tx_frame.data[1] = 0x00;
	displayer_tx_frame.data[1] |= displayer_show_info.AEBSTaskStatus;
	displayer_tx_frame.data[1] |= displayer_show_info.FCWTaskStatus << 2;
	displayer_tx_frame.data[1] |= displayer_show_info.LDWTaskStatus << 4;
	displayer_tx_frame.data[1] |= displayer_show_info.SSSTaskStatus << 6;

	displayer_tx_frame.data[2] = 0x00;
	displayer_tx_frame.data[2] |= displayer_show_info.CMSTaskStatus;
	//前向碰撞报警
	if(stWarninglinkage.FCWlinkage == 1)
	{
		if(stVehicleParas.fVehicleSpeed > stWarninglinkage.FCWlinkageSpeed)
			displayer_show_info.FCWLevel = 0;
	}
	displayer_tx_frame.data[2] |= displayer_show_info.FCWLevel << 2;
	//LDW车道偏离报警
	if(stWarninglinkage.LDWlinkage == 1)
	{
		if(stVehicleParas.fVehicleSpeed > stWarninglinkage.LDWlinkageSpeed)
			displayer_show_info.LDW = 0;
	}
	if(warning_status.LDWstatus == 1)
		displayer_show_info.LDW = 0;
	displayer_tx_frame.data[2] |= displayer_show_info.LDW << 4;

	if(stWarninglinkage.HMWlinkage == 1)
	{
		if(stVehicleParas.fVehicleSpeed > stWarninglinkage.HMWlinkageSpeed)
			displayer_show_info.HMWGrade = 0;
	}
	displayer_tx_frame.data[2] |= displayer_show_info.HMWGrade << 6;

	displayer_tx_frame.data[3] = 0x00;
	displayer_tx_frame.data[3] |= displayer_show_info.DigitalDisplay;
	displayer_tx_frame.data[3] |= (uint8_t)(displayer_show_info.HMWTime * 10.0) << 2;

	uint16_t rel_speed = (uint16_t)((displayer_show_info.LengthwaysRelativeSpeed * 4.0) + 127.9375);
	displayer_tx_frame.data[4] = (uint8_t)(rel_speed & 0x00FF);

	displayer_tx_frame.data[5] = 0x00;
	displayer_tx_frame.data[5] |= (uint8_t)((rel_speed & 0x0300) >> 8);
	displayer_tx_frame.data[5] |= (uint8_t)(displayer_show_info.TTC * 10.0) << 2;

	uint16_t rel_distance = (uint16_t)(displayer_show_info.LengthwaysDistance * 10.0);
	displayer_tx_frame.data[6] = (uint8_t)(rel_distance & 0x00FF);

	displayer_tx_frame.data[7] = 0x00;
	displayer_tx_frame.data[7] |= (uint8_t)((rel_distance & 0x0F00) >> 8);
	displayer_tx_frame.data[7] |= displayer_show_info.ObstacleType << 4;

	*tx_frame = displayer_tx_frame;

	return pdTRUE;
}
void Displayer_CANRx_Analysis(struct can_frame *rx_frame)
{
	struct can_frame tmp_frame = *rx_frame;
	if(tmp_frame.lenth < 8)
		return;
	if(tmp_frame.TargetID == DISPLAYER_FEEDBACK_ID)
	{
		isAEBTaskClose = tmp_frame.data[0] & 0x01;
		isLDWTaskClose = (tmp_frame.data[0] & 0x02) >> 1;
	}
}
