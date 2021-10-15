/*
 * data_center_services.c
 *
 *  Created on: 2021-7-15
 *      Author: shuai
 */
#include "data_center_services.h"
#include "common.h"
Camera_Info camera_share;
SemaphoreHandle_t data_mutex = NULL;

Device_Status dev_status;

void Data_Center_Init()
{
	Camera_Info_Init();
	Device_Status_Init();
	//data_mutex = xSemaphoreCreateRecursiveMutex();
}

void Camera_Info_Init()
{
	camera_share.CammeraEssentialData.LeftLDW = 0;
	camera_share.CammeraEssentialData.RightLDW = 0;
	camera_share.CammeraEssentialData.LDWSentivity = 0;
	camera_share.CammeraEssentialData.Invalid = 0;
	camera_share.CammeraEssentialData.OffSound = 0;
	camera_share.CammeraEssentialData.HMWGrade = 0;
	camera_share.CammeraEssentialData.HMW = 6.3;
	camera_share.CammeraEssentialData.ErrorCode = 0;
	camera_share.CammeraEssentialData.HMWEnable = 0;
	camera_share.CammeraEssentialData.FCWLevel = 0;
	camera_share.CammeraEssentialData.AmbientLuminance = 0;
	camera_share.CammeraEssentialData.FCWStatus = 0;

	camera_share.ObsBasicData.ObstacleNumber = 0;
	camera_share.ObsBasicData.HMWAlarmThreshold = 3.1;

	camera_share.ObsInormation.ObstacleID = 0;
	camera_share.ObsInormation.TrackNumber = 0;
	camera_share.ObsInormation.ObstacleWidth = 40.95;
	camera_share.ObsInormation.ObstacleHeight = 40.95;
	camera_share.ObsInormation.RelativeSpeedZ = 128.0;
	camera_share.ObsInormation.DistanceY = 20.47;
	camera_share.ObsInormation.DistanceX = 163.83;
	camera_share.ObsInormation.DistanceZ = 327.67;
	camera_share.ObsInormation.TTC = 6.3;
	camera_share.ObsInormation.HMW = 6.3;
	camera_share.ObsInormation.RelativeSpeedX = 51.1;
	camera_share.ObsInormation.CIPV = 0;
	camera_share.ObsInormation.ObstacleType = 0;
}

void Device_Status_Init()
{
	dev_status.FrontCamera = 0;
	dev_status.MillimeterWaveRadar = 0;
	dev_status.UltrasonicRadar = 0;
	dev_status.Valve = 0;
	dev_status.VehicleCANBus = 0;
	dev_status.VehicleSpeed = 0;
	dev_status.Module4G = 0;
	dev_status.GPS = 0;
	dev_status.AEBSTask = 2;
	dev_status.FCWTask = 2;
	dev_status.LDWTask = 2;
	dev_status.SSSTask = 2;
	dev_status.CMSTask = 2;
}

void Data_Center_Task(void *pvParameters)
{
	while (1)
	{
		if( xSemaphoreTakeRecursive( data_mutex, ( TickType_t ) 10 ) == pdTRUE )
		{
			if(camera_share.CameraInfoFlag==1){
				camera_share.CameraInfoFlag=0;
			}else{
				Camera_Info_Init();
			}

			if(dev_status.DeviceStatusFlag==1){
				dev_status.DeviceStatusFlag=0;
			}else{
				Device_Status_Init();
			}

			xSemaphoreGiveRecursive( data_mutex );
		}
		vTaskDelay(300);
	}
}

void Camera_Info_Push(Camera_Essential_Data *camera_data, Obstacle_Basic_Data *obs_basic, Obstacle_Information *obs_info)
{
	camera_share.CammeraEssentialData = *camera_data;
	camera_share.ObsBasicData = *obs_basic;
	camera_share.ObsInormation = *obs_info;
	camera_share.CameraInfoFlag = 1;
}

void Displayer_Pull(Displayer_Show *show_info)
{

	if(stCanCommSta.stCamera.status == OFFLINE)
	{
		Camera_Info_Init();
	}
	if(warning_status.AEBstatus != 0)
		dev_status.AEBSTask = 0;
	else
		dev_status.AEBSTask = 2;
	if(warning_status.LDWstatus != 0)
	{
		dev_status.LDWTask = 0;
		camera_share.CammeraEssentialData.LeftLDW = 0;
		camera_share.CammeraEssentialData.RightLDW = 0;
	}
	else
		dev_status.LDWTask = 2;

	show_info->FrontCameraStatus = stCanCommSta.stCamera.status;//dev_status.FrontCamera;
	show_info->MillimeterWaveRadarStatus = stCanCommSta.stRadar.status;//dev_status.MillimeterWaveRadar;
	show_info->UltrasonicRadarStatus = stCanCommSta.stHRadar.status;//dev_status.UltrasonicRadar;
	show_info->ValveStatus = stCanCommSta.Proportional_valve.status;//dev_status.Valve;
	show_info->VehicleCANBusStatus = stCanCommSta.stVehicle.status;//dev_status.VehicleCANBus;
	show_info->VehicleSpeedStatus = stCanCommSta.stSpeed.status;//dev_status.VehicleSpeed;
	show_info->Module4GStatus = stCanCommSta.stWireless.status;//dev_status.Module4G;
	show_info->GPSStatus = dev_status.GPS;
	show_info->AEBSTaskStatus = dev_status.AEBSTask;
	show_info->FCWTaskStatus = dev_status.FCWTask;
	show_info->LDWTaskStatus = dev_status.LDWTask;
	show_info->SSSTaskStatus = dev_status.SSSTask;
	show_info->CMSTaskStatus = dev_status.CMSTask;
	show_info->FCWLevel = camera_share.CammeraEssentialData.FCWLevel;
	if(camera_share.CammeraEssentialData.LeftLDW)
	{
		show_info->LDW = 1;
	}else if(camera_share.CammeraEssentialData.RightLDW)
	{
		show_info->LDW = 2;
	}else{
		show_info->LDW = 3;
	}
	show_info->HMWGrade = camera_share.CammeraEssentialData.HMWGrade;
	show_info->DigitalDisplay = 1;//之后需要在配置信息中获取。
	show_info->HMWTime = camera_share.ObsInormation.HMW;
	show_info->LengthwaysRelativeSpeed = camera_share.ObsInormation.RelativeSpeedZ;
	show_info->TTC = camera_share.ObsInormation.TTC;
	show_info->LengthwaysDistance = camera_share.ObsInormation.DistanceZ;
	show_info->ObstacleType = camera_share.ObsInormation.ObstacleType;
}

