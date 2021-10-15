/*
 * stereo_camera.h
 *
 *  Created on: 2021-6-22
 *      Author: shuai
 */

#ifndef STEREO_CAMERA_H_
#define STEREO_CAMERA_H_
#include "system_init.h"
#include "stdio.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "canhl.h"
#include "uart_task.h"

#define CAMERA_CAN_BAUDRATE CAN_BAUDRATE_500K

#define CAMERA_ESSENTIAL_DATA_ID 0x79F
#define CAMERA_ESSENTIAL_REFRESH_RATE 100

#define CAMERA_OBSTACLE_INFO_ID 0x7A0
#define CAMERA_OBSTACLE_INFO_REFRESH_RATE 100

#define CAMERA_OBSTACLE_INFO_A_ID 0x7A1
#define CAMERA_OBSTACLE_INFO_A_REFRESH_RATE 100

#define CAMERA_OBSTACLE_INFO_B_ID 0x7A2
#define CAMERA_OBSTACLE_INFO_B_REFRESH_RATE 100

#define CAMERA_INPUT_DATA_ID 0x7B0
#define CAMERA_INPUT_DATA_REFRESH_RATE 100

typedef enum{
	No_Error = 0, //û�д���
	Camera_Error = 2, //����쳣����
	Watchdog_Error = 4, //������Ź�����
	FPGA_Error = 5, //FPGA �˵��ڴ����
	No_Remap = 6, //û�� remap ����
	ISP_Error = 7, //ISP��鷵�ش���
	Camera_Unknown_Error = 8, //���δ֪����
	No_CriticalData = 11, //���ڷǱ궨ģʽ���豸����ʱ��adasCriticalData�� ·���±궨�ļ���ʧ��
	Soft_Unknown_Error = 19, //���δ֪����
	CAN_Network_Error = 31, //�޷�ͨ��ָ���� CAN Э���ȡ�����ź�
	CAN_Unknown_Error = 39 //CANδ֪����
}Camera_Fault_Code;

typedef struct{
	uint8_t LeftLDW; //����˵���� ��0����]����1���ǡ��󳵵���ƫ��
	uint8_t RightLDW; //����˵���� ��0����]����1���ǡ��ҳ�����ƫ��
	uint8_t LDWSentivity; //����ƫ��Ԥ���ȼ�,����˵���� ��0���رա�����1������������2���ߡ�����3���͡�
	uint8_t Invalid; //����˵���� ��0�� δʧЧ������1��������������2��ͼ��ģ��������3���������ӡ�����4�����±���]����5�� ͼ���쳣������6-7:������
	uint8_t OffSound; //�豸�Ƿ���������,����˵������0�� �����ơ�����1�� ���ơ�
	uint8_t HMWGrade; //����˵���� ��0����Чֵ������1����ֵ��HMW��2.7S������2�� 0��HMW����ֵ������3������ʱ��>2.7S��
	float HMW; //Ԥ���������뱾������ϰ���ĳ���ʱ�䣬��Χ: 0��6[��]Factor�� 0.1�� Offset�� 0 ��Чֵ: 0x3F
	uint8_t ErrorCode; //����˵������ 0����ʾ�޴��󡿡��� 1-9���������󡿡��� 11-19��Ӧ���������󡿡��� 31-39��CAN ͨѶ����ϡ�
	uint8_t HMWEnable; //����˵���� �� 0:�񡿡���1:�ǡ�������Ԥ�������Ƿ�����
	uint8_t FCWLevel; //����˵������0���񡿡���1��һ��Ԥ��������2������Ԥ����
	uint8_t AmbientLuminance; //����˵���� ��0�� ����������1�� ���졿����2���ƻ衿����3��ҹ������4�� �ڵ���
	uint8_t FCWStatus; //����˵���� ��0���񡿡���1�� �ǡ�����ǰ����ײԤ���������ó��ٺ�����Ԥ������ʱ�Ż���� 1��Ĭ������� 0
	/*����������ʱ�����н���*/
	//uint8_t VersionSegment; //����˵���� ��0-2�� 0:�汾�ŵ�һ�ڣ� 1: �汾�ŵڶ��ڣ�2: �汾�ŵ����ڣ�
	//uint8_t SWVersion[3]; //����汾�ţ�����˵���� ��֡һ�����ڣ�ÿ�δ���һ�ڣ����� 0.0.0
	//uint8_t HWVersion[3]; //Ӳ���汾�ţ�����˵���� ��֡һ�����ڣ�ÿ�δ���һ�ڣ����� 0.0.0
	//uint8_t CanProtocoIVersion[3]; //CANЭ��汾�ţ�����˵���� ��֡һ�����ڣ�ÿ�δ���һ�ڣ� ���� 0.0.0
}Camera_Essential_Data;

typedef struct{
//	uint8_t reserved;
	uint8_t LeftLaneStatus;	// ����˵������0��δѹ�ߡ�����1��ѹ�ߡ�����2�����ߡ�����3��������
	uint8_t LeftLaneStyle;	// ����˵������0���ޡ�����1��Ԥ�⡿����2�����ߡ�����3��ʵ�ߡ�����4��˫���ߡ�����5��˫ʵ�ߡ�����6�����ߡ�����7-15��������
//	uint8_t reserved1;
	uint8_t RightLaneStatus;// ����˵���� ��0��δѹ�ߡ�����1��ѹ�ߡ�����2�����ߡ�����3��������
	uint8_t RightLaneStyle;	// ����˵���� ��0���ޡ�����1��Ԥ�⡿����2�����ߡ�����3��ʵ�ߡ�����4��˫���ߡ�����5��˫ʵ�ߡ�����6�����ߡ�����7-15��������
	uint8_t CarDepth;		// ����˵���� ����: unsigned 8 bits����Χ: 0��2.50[��]��Factor��0.01��Offset��0����Чֵ��0xFF
	uint16_t SpeedOfRushLane;// ����˵���� ����: unsigned 10 bits����Χ: -50.0��50.0[��/��]��Factor��0.1��Offset��-51.1����Чֵ��0x3FF
//	uint8_t resered;
}Camera_LDW_data;	// 0x7A3 �����߽���

typedef struct{
	uint8_t ObstacleNumber; //
	float HMWAlarmThreshold; //����˵���� ����: unsigned 5 bits�� ��Χ:0.0��3.0[��]�� Factor�� 0.1�� Offset�� 0�� ��Чֵ: 0x1F
	/*����������ʱ�����н���*/
	//uint8_t TimeID; //����˵���� ��λ���룬 ��׼ʱ����� 8 λ
	//float YawAngle; //����˵���� ����: unsigned 14 bits�� ��Χ: -0.8��0.8[����]�� Factor�� 0.0001�� Offset�� -0.8191����Чֵ: 0x3FFF
	//float PithAngle; //����˵���� ����: unsigned 12 bits�� ��Χ: -0.2��0.2[����]�� Factor�� 0.0001�� Offset�� -0.2047����Чֵ: 0x7FF
	//float RotationAngle; //����˵���� ����: unsigned 12 bits�� ��Χ: -0.2��0.2[����]�� Factor�� 0.0001�� Offset�� -0.2047����Чֵ: 0x7FF
}Obstacle_Basic_Data;

typedef struct{
	uint8_t ObstacleID; //�ϰ���ID,����˵���� ��0��255��
	uint8_t TrackNumber; //�ϰ�����������֡������Χ: 1��63�� �ڶ��μ�⵽�ϰ����� 1 ��ʼ�� ���ֵΪ 63��ÿ֡���� 1�������� 63 �󱣳�Ϊ 63
	float ObstacleWidth; //�ϰ����ȣ�����˵������Χ: 0.0��40.0[��]�� Factor�� 0.01�� Offset�� 0�� ��Чֵ�� 0xFFF
	float ObstacleHeight; //�ϰ���߶ȣ�����˵������Χ: 0.0��40.0[��]�� Factor�� 0.01�� Offset�� 0�� ��Чֵ�� 0xFFF
	float RelativeSpeedZ; //��������ٶȣ�����˵������Χ: -127.93��127.93[��/��]�� Factor�� 0.0625�� Offset��-127.9375�� ��Чֵ�� 0xFFF
	float DistanceY; //������Ծ��룬����˵���� ����: ��Χ: -20.0��20.0[��]�� Factor�� 0.01�� Offset�� -20.47����Чֵ�� 0xFFF
	float DistanceX; //������Ծ��룬����˵���� ��Χ: -150.0��150.0[��]�� Factor�� 0.01�� Offset�� -163.83����Чֵ�� 0x7FFF
	float DistanceZ; //������Ծ��룬����˵���� ��Χ: 0.0��300.0[��]�� Factor�� 0.01�� Offset�� 0�� ��Чֵ�� 0x7FFF
	float TTC; //�������ϰ�������ײʱ�䣬����˵���� ��Χ: 0.0��6.0[��]�� Factor�� 0.1�� Offset�� 0�� ��Чֵ��0x3F
	float HMW; //����ʱ�䣬����˵����  ��Χ: 0.0��6.0[��]�� Factor�� 0.1�� Offset�� 0�� ��Чֵ��0x3F��˵���� CIPV Ϊ 1 ���ϰ������������ݡ�
	float RelativeSpeedX; //��������ٶȣ�����˵���� ����: ��Χ: -50.0��50.0[��/��]�� Factor�� 0.1�� Offset�� -51.1����Чֵ�� 0x3FF
	uint8_t CIPV; //Ԥ���������뱾������ϰ���ı�ʶ������˵���� ��0���񡿡���1����Ԥ���������뱾��������ϰ��
	uint8_t ObstacleType; //�ϰ������ͣ�����˵������0����Чֵ������1�� ����������2���ˡ�����3�� ����������4�������ߡ�����5-11��������
}Obstacle_Information;

typedef struct{
	//Camera_Essential_Data CammeraEssentialData;
	//Obstacle_Basic_Data ObsBasicData;
	//Obstacle_Information ObsInormation;
	float hmw;
	float ttc;
}Camera_All_Info;

extern QueueHandle_t CAN1_Rxqueue_Handler;

void Camera_CIPV_Init();
void Camera_Init();
void Camera_CAN_Transmition(struct can_frame *tx_frame,float speed);
void Urader_CAN_Transmition(struct can_frame *tx_frame,float speed);
void Camera_CAN_Analysis(struct can_frame *rx_frame);

#endif /* STEREO_CAMERA_H_ */
