/*
 * data_center_services.h
 *
 *  Created on: 2021-7-15
 *      Author: shuai
 */

#ifndef DATA_CENTER_SERVICES_H_
#define DATA_CENTER_SERVICES_H_
#include "system_init.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stereo_camera.h"
#include "displayer.h"

typedef struct{
	Camera_Essential_Data CammeraEssentialData;
	Obstacle_Basic_Data ObsBasicData;
	Obstacle_Information ObsInormation;
	uint8_t CameraInfoFlag; //数据更新标记位。
}Camera_Info;

typedef struct
{
	uint8_t FrontCamera; //前视相机状态【0：未就绪】、【1： 就绪】
	uint8_t MillimeterWaveRadar; //毫米波雷达状态【0：未就绪】、【1： 就绪】
	uint8_t UltrasonicRadar; //超声波雷达状态【0：未就绪】、【1： 就绪】
	uint8_t Valve; //比例阀状态【0：未就绪】、【1： 就绪】
	uint8_t VehicleCANBus; //整车CAN总线状态【0：未就绪】、【1： 就绪】
	uint8_t VehicleSpeed; //车速状态【0：未就绪】、【1： 就绪】
	uint8_t Module4G; //4G模块状态【0：未就绪】、【1： 就绪】
	uint8_t GPS; //GPS模块状态【0：未就绪】、【1： 就绪】
	uint8_t AEBSTask; //AEBS任务状态【0：故障】、【1： 自检】、【2：正常】、【3：其他】
	uint8_t FCWTask; //FCW任务状态【0：故障】、【1： 自检】、【2：正常】、【3：其他】
	uint8_t LDWTask; //LDW任务状态【0：故障】、【1： 自检】、【2：正常】、【3：其他】
	uint8_t SSSTask; //SSS任务状态【0：故障】、【1： 自检】、【2：正常】、【3：其他】
	uint8_t CMSTask; //CMS任务状态【0：故障】、【1： 自检】、【2：正常】、【3：其他】

	uint8_t DeviceStatusFlag; //数据更新标记位。
}Device_Status;

extern Camera_Info camera_share;
extern SemaphoreHandle_t data_mutex;

extern Device_Status dev_status;

void Data_Center_Init();
void Camera_Info_Init();
void Device_Status_Init();

void Data_Center_Task(void *pvParameters);

void Camera_Info_Push(Camera_Essential_Data *camera_data, Obstacle_Basic_Data *obs_basic, Obstacle_Information *obs_info);

void Displayer_Pull(Displayer_Show *show_info);

#endif /* DATA_CENTER_SERVICES_H_ */
