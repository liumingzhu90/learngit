/*
 * vehicles_can.h
 *
 *  Created on: 2021-7-24
 *      Author: shuai
 */

#ifndef VEHICLE_CAN_H_
#define VEHICLE_CAN_H_
#include "system_init.h"
#include "canhl.h"

typedef enum{
	ZKHY,
	ZhongXing,
	ZhongTong,
	ZhongChe,
	WanXiang,
	SunLong,
	LiuQi,
	KingLong,
	JinLv,
	JinLv2,
	FuTian,
	DongFeng
}Vehicle_Type;

/* Vehicle Configuration *************************************/
#define VEHICLE_TYPE JinLv2

typedef struct{
	float vehicle_speed; //����
	uint8_t turn_signal_light; //ת���źŵ� 0��ת��1��ת��2��ת��3˫��
	uint8_t brake_light; //ɲ�����ź�
	float roll_rate; //��������
	float pitch_rate; //��������
	float yaw_rate; //������ڽ�
	float steering_wheel_angle; //�����̽Ƕȣ���ʱ��Ϊ��
	float lateral_acceleration; //������ٶ�
	float longitudinal_acceleration; //������ٶ�
	uint8_t current_gear; //��ǰ��λ��Ϣ,0:N,1:D,2:R,3:P
	float brake_pedal_degree; //ɲ��̤�忪�϶�
	float accelerator_pedal_degree; //����̤�忪�϶�
	uint8_t wiper_status; //�����״̬

	uint8_t vehicle_info_flag; //������Ϣ���±��λ
}Vehicle_Info;

void Vehicle_CAN_Init();
void Vehicle_Can_Analysis(struct can_frame *rx_frame);

#endif /* VEHICLE_CAN_H_ */
