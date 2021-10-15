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
	uint8_t CameraInfoFlag; //���ݸ��±��λ��
}Camera_Info;

typedef struct
{
	uint8_t FrontCamera; //ǰ�����״̬��0��δ����������1�� ������
	uint8_t MillimeterWaveRadar; //���ײ��״�״̬��0��δ����������1�� ������
	uint8_t UltrasonicRadar; //�������״�״̬��0��δ����������1�� ������
	uint8_t Valve; //������״̬��0��δ����������1�� ������
	uint8_t VehicleCANBus; //����CAN����״̬��0��δ����������1�� ������
	uint8_t VehicleSpeed; //����״̬��0��δ����������1�� ������
	uint8_t Module4G; //4Gģ��״̬��0��δ����������1�� ������
	uint8_t GPS; //GPSģ��״̬��0��δ����������1�� ������
	uint8_t AEBSTask; //AEBS����״̬��0�����ϡ�����1�� �Լ졿����2������������3��������
	uint8_t FCWTask; //FCW����״̬��0�����ϡ�����1�� �Լ졿����2������������3��������
	uint8_t LDWTask; //LDW����״̬��0�����ϡ�����1�� �Լ졿����2������������3��������
	uint8_t SSSTask; //SSS����״̬��0�����ϡ�����1�� �Լ졿����2������������3��������
	uint8_t CMSTask; //CMS����״̬��0�����ϡ�����1�� �Լ졿����2������������3��������

	uint8_t DeviceStatusFlag; //���ݸ��±��λ��
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
