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

extern float g_AEB_CMS_outside_dec_output;
extern uint8_t g_AEB_CMS_hmw_warning;
extern uint8_t g_is_hmw_warning;
extern uint8_t g_LDW_Enable;
extern uint8_t g_AEB_ON_OFF_Displayer;
extern uint8_t g_LDW_ON_OFF_Displayer;

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
	displayer_tx_frame.data[0] |= displayer_show_info.VehicleCANBusStatus << 5;//displayer_show_info.VehicleSpeedStatus << 5;
	displayer_tx_frame.data[0] |= displayer_show_info.Module4GStatus << 6;
	displayer_tx_frame.data[0] |= displayer_show_info.GPSStatus << 7;

	displayer_tx_frame.data[1] = 0x00;
	displayer_tx_frame.data[1] |= displayer_show_info.AEBSTaskStatus;//0x00;//
	displayer_tx_frame.data[1] |= displayer_show_info.FCWTaskStatus << 2;
	displayer_tx_frame.data[1] |= displayer_show_info.LDWTaskStatus << 4;
	displayer_tx_frame.data[1] |= displayer_show_info.SSSTaskStatus << 6;

	displayer_tx_frame.data[2] = 0x00;
	displayer_tx_frame.data[2] |= displayer_show_info.CMSTaskStatus;
	//前向碰撞报警
	//if((stWarninglinkage.FCWlinkage == 1) & (stVehicleParas.fVehicleSpeed > stWarninglinkage.FCWlinkageSpeed))
	//{
	//		displayer_show_info.FCWLevel = displayer_show_info.FCWLevel;
	//}
	//else
	//	displayer_show_info.FCWLevel = 0;
	if((camera_share.ObsInormation.CIPV != 0) && (stVehicleParas.fVehicleSpeed > 0) && (displayer_show_info.FCWTaskStatus == 1))
		displayer_tx_frame.data[2] |= displayer_show_info.FCWLevel << 2;
	//2021-12-29 For test:when break,then sound as :displayer_tx_frame.data[2] |= 2 << 2;
	if(g_AEB_CMS_outside_dec_output > 0.0){
		//displayer_tx_frame.data[2] |= 2 << 2;
	}
	/*if(stVehicleParas.LeftFlagTemp == 1)
		displayer_tx_frame.data[2] |= 2 << 2;
	if(stVehicleParas.BrakeFlag == 1)
		displayer_tx_frame.data[2] |= 2 << 2;
	*/
	//
	//displayer_tx_frame.data[2] |= 2 << 2;
	//if(stVehicleParas.fVehicleSpeed > 0)
	//else
	//	displayer_tx_frame.data[2] |= displayer_show_info.FCWLevel << 2;
	//LDW车道偏离报警
	//if((stWarninglinkage.LDWlinkage == 1) & (stVehicleParas.fVehicleSpeed > stWarninglinkage.LDWlinkageSpeed))
	{
	//	if(stVehicleParas.fVehicleSpeed > stWarninglinkage.LDWlinkageSpeed)
	//		displayer_show_info.LDW = displayer_show_info.LDW;
	}
	//else
	//	displayer_show_info.LDW = 0;
	//if(warning_status.LDWstatus == 1)
	//	displayer_show_info.LDW = 0;
	if(stVehicleParas.fVehicleSpeed > 0)
	{
		if(stVehicleParas.LeftFlagTemp == 1)
				displayer_show_info.LDW = 3;
		if(stVehicleParas.RightFlagTemp == 1)
				displayer_show_info.LDW = 3;
		displayer_tx_frame.data[2] |= displayer_show_info.LDW << 4;
		//fprintf(USART1_STREAM,"TL:%d,TR:%d,LDW:%d\r\n",stVehicleParas.LeftFlagTemp,stVehicleParas.RightFlagTemp,displayer_show_info.LDW);
	}
	else
		displayer_tx_frame.data[2] |= 3 << 4;

	if(!g_LDW_Enable){
		displayer_tx_frame.data[2] |= 3 << 4;
	}
	//if((stWarninglinkage.HMWlinkage == 1) & (stVehicleParas.fVehicleSpeed > stWarninglinkage.HMWlinkageSpeed))
	{
		//if(stVehicleParas.fVehicleSpeed > stWarninglinkage.HMWlinkageSpeed)
	//		displayer_show_info.HMWGrade = displayer_show_info.HMWGrade;
	}
	//else
	//	displayer_show_info.HMWGrade = 0;
	if((stVehicleParas.fVehicleSpeed > 0) && (stVehicleParas.Car_Gear != 3)){
		if((g_AEB_CMS_hmw_warning == 1)&((displayer_show_info.HMWGrade == 2))){

			displayer_tx_frame.data[2] |= displayer_show_info.HMWGrade << 6;

		}
		else if((g_AEB_CMS_hmw_warning == 0)&(displayer_show_info.HMWGrade == 2)){
			displayer_tx_frame.data[2] |= 1 << 6;
		}
		else{
			displayer_tx_frame.data[2] |= displayer_show_info.HMWGrade << 6;
		}
	}
	if(0x00 == g_AEB_CMS_hmw_warning){
		displayer_tx_frame.data[2] |= 0 << 6;
	}
	else if(0x01 == g_AEB_CMS_hmw_warning){
		displayer_tx_frame.data[2] |= 2 << 6;
	}
	// Test for new CAN pro
	/*if(g_is_hmw_warning){
		displayer_tx_frame.data[2] |= 2 << 6;
	}
	else{
		displayer_tx_frame.data[2] |= 1 << 6;
	}*/
	//displayer_tx_frame.data[2] |= 1 << 6;
	//
	displayer_tx_frame.data[3] = 0x00;
	displayer_tx_frame.data[3] |= displayer_show_info.DigitalDisplay;
	if(camera_share.ObsInormation.CIPV == 0)
		displayer_show_info.HMWTime = 0;
	displayer_tx_frame.data[3] |= (uint8_t)(displayer_show_info.HMWTime * 10.0) << 2;

	uint16_t rel_speed = (uint16_t)((displayer_show_info.LengthwaysRelativeSpeed * 4.0) + 127.9375);
	displayer_tx_frame.data[4] = (uint8_t)(rel_speed & 0x00FF);

	displayer_tx_frame.data[5] = 0x00;
	displayer_tx_frame.data[5] |= (uint8_t)((rel_speed & 0x0300) >> 8);

	if(camera_share.ObsInormation.CIPV != 0)
		displayer_tx_frame.data[5] |= (uint8_t)(displayer_show_info.TTC * 10.0) << 2;
	else
		displayer_tx_frame.data[5] |= 0x3f << 2;

	uint16_t rel_distance = (uint16_t)(displayer_show_info.LengthwaysDistance * 10.0);
	displayer_tx_frame.data[6] = (uint8_t)(rel_distance & 0x00FF);

	displayer_tx_frame.data[7] = 0x00;
	displayer_tx_frame.data[7] |= (uint8_t)((rel_distance & 0x0F00) >> 8);

	if(camera_share.ObsInormation.CIPV != 0)
		displayer_tx_frame.data[7] |= displayer_show_info.ObstacleType << 4;
	//else
	//	displayer_tx_frame.data[7] |=

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

		g_AEB_ON_OFF_Displayer = isAEBTaskClose;
		g_LDW_ON_OFF_Displayer = isLDWTaskClose;
		//fprintf(USART1_STREAM,"AEB_ON_OFF:%d LDW_ON_OFF:%d\r\n",g_AEB_ON_OFF_Displayer,g_LDW_ON_OFF_Displayer);
	}
	return;
}
