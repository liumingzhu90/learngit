/*
 * displayer.h
 *
 *  Created on: 2021-7-7
 *      Author: shuai
 */

#ifndef DISPLAYER_H_
#define DISPLAYER_H_
#include "system_init.h"
#include "canhl.h"

#define DISPLAYER_CAN_BAUDRATE CAN_BAUDRATE_250K

#define DISPLAYER_SHOW_ID 0x18FFED80
#define DISPLAYER_SHOW_REFRESH_RATE 100

#define DISPLAYER_FEEDBACK_ID 0x18FFED81
#define DISPLAYER_FEEDBACK_REFRESH_RATE 100

typedef struct
{
	uint8_t FrontCameraStatus; //ǰ�����״̬��0��δ����������1�� ������
	uint8_t MillimeterWaveRadarStatus; //���ײ��״�״̬��0��δ����������1�� ������
	uint8_t UltrasonicRadarStatus; //�������״�״̬��0��δ����������1�� ������
	uint8_t ValveStatus; //������״̬��0��δ����������1�� ������
	uint8_t VehicleCANBusStatus; //����CAN����״̬��0��δ����������1�� ������
	uint8_t VehicleSpeedStatus; //����״̬��0��δ����������1�� ������
	uint8_t Module4GStatus; //4Gģ��״̬��0��δ����������1�� ������
	uint8_t GPSStatus; //GPSģ��״̬��0��δ����������1�� ������
	uint8_t AEBSTaskStatus; //AEBS����״̬��0�����ϡ�����1�� �Լ졿����2������������3��������
	uint8_t FCWTaskStatus; //FCW����״̬��0�����ϡ�����1�� �Լ졿����2������������3��������
	uint8_t LDWTaskStatus; //LDW����״̬��0�����ϡ�����1�� �Լ졿����2������������3��������
	uint8_t SSSTaskStatus; //SSS����״̬��0�����ϡ�����1�� �Լ졿����2������������3��������
	uint8_t CMSTaskStatus; //CMS����״̬��0�����ϡ�����1�� �Լ졿����2������������3��������
	uint8_t FCWLevel; //ǰ����ײԤ���ȼ���0���񡿡���1��һ��Ԥ��������2������Ԥ��������3��������
	uint8_t LDW; //����ƫ��Ԥ����0��δʶ�𳵵��ߡ�����1���󳵵�ƫ��Ԥ��������2���ҳ���ƫ��Ԥ��������3����ƫ��Ԥ����
	uint8_t HMWGrade; //������Ԥ���ȼ���0����Чֵ������1����ֵ��HMW��2.7S]����2�� 0��HMW����ֵ������3��HMW>2.7S��
	uint8_t DigitalDisplay; //������ʾ���ݡ�0���񡿡���1��TTC������2��DTC������3��HMW��
	float HMWTime; //����ʱ�䣬��Χ: 0.0��6.0[��]�� Factor�� 0.1�� Offset�� 0�� ��Чֵ��0x3F
	float LengthwaysRelativeSpeed; //��������ٶȣ���Χ: -127.93��127.93[��/��]�� Factor�� 0.25�� Offset��-127.9375�� ��Чֵ�� 0xFFF
	float TTC; //�������ϰ�������ײʱ�䣬��Χ: 0.0��6.0[��]�� Factor�� 0.1�� Offset�� 0�� ��Чֵ��0x3F
	float LengthwaysDistance; //DTC������Ծ��룬��Χ: 0.0��300.0[��]�� Factor�� 0.1�� Offset�� 0�� ��Чֵ�� 0x7FF
	uint8_t ObstacleType; //�ϰ������͡�0����Чֵ������1������������2���ˡ�����3������������4�������ߡ�����5-11��������
}Displayer_Show;

extern Displayer_Show displayer_show_info;
extern BaseType_t isAEBTaskClose;
extern BaseType_t isLDWTaskClose;

void Displayer_Init();
BaseType_t Displayer_CANTx_Get(struct can_frame *tx_frame);
void Displayer_CANRx_Analysis(struct can_frame *rx_frame);

#endif /* DISPLAYER_H_ */
