/*
 * vehicle_can.c
 *
 *  Created on: 2021-7-24
 *      Author: shuai
 */
#include "vehicle_can.h"
#include "vehicle_can_zkhy.h"
#include "vehicle_can_zhongxing.h"
#include "vehicle_can_zhongtong.h"
#include "vehicle_can_zhongche.h"
#include "vehicle_can_wanxiang.h"
#include "vehicle_can_sunlong.h"
#include "vehicle_can_liuqi.h"
#include "vehicle_can_kinglong.h"
#include "vehicle_can_jinlv.h"
#include "vehicle_can_futian.h"
#include "vehicle_can_dongfeng.h"
#include "stereo_camera.h"
extern void Camera_CAN_Transmition(struct can_frame *tx_frame,float speed);
Vehicle_Info veh_info;

inline void Info_Init()
{
	veh_info.vehicle_speed = 0.0; //����
	veh_info.turn_signal_light = 0; //ת���źŵ�
	veh_info.brake_light = 0; //ɲ�����ź�
	veh_info.roll_rate = 0.0; //��������
	veh_info.pitch_rate = 0.0; //��������
	veh_info.yaw_rate = 0.0; //������ڽ�
	veh_info.steering_wheel_angle = 0.0; //�����̽Ƕȣ���ʱ��Ϊ��
	veh_info.lateral_acceleration = 0.0; //������ٶ�
	veh_info.longitudinal_acceleration = 0.0; //������ٶ�
	veh_info.current_gear = 0; //��ǰ��λ��Ϣ��0�յ���1ǰ������2��������3ͣ����
	veh_info.brake_pedal_degree = 0.0; //ɲ��̤�忪�϶�
	veh_info.accelerator_pedal_degree = 0.0; //����̤�忪�϶�
	veh_info.wiper_status = 0; //�����״̬

	veh_info.vehicle_info_flag = 0; //������Ϣ���±��λ
}

void Vehicle_CAN_Init(void)
{
	Info_Init();
}
void Vehicle_Can_Analysis(struct can_frame *rx_frame)
{
	struct can_frame tx_frame;
	if(rx_frame->flgBoxRxEnd == 1)
	{
		rx_frame->flgBoxRxEnd = 0;
		switch(VEHICLE_TYPE)
		{
		case ZKHY:
			Vehicle_Analysis_ZKHY(rx_frame, &veh_info);
			break;
		case ZhongXing:
			Vehicle_Analysis_Zhongxing(rx_frame, &veh_info);
			break;
		case ZhongTong:
			Vehicle_Analysis_Zhongtong(rx_frame, &veh_info);
			break;
		case ZhongChe:
			Vehicle_Analysis_Zhongche(rx_frame, &veh_info);
			break;
		case WanXiang:
			Vehicle_Analysis_Wanxiang(rx_frame, &veh_info);
			break;
		case SunLong:
			Vehicle_Analysis_Sunlong(rx_frame, &veh_info);
			break;
		case LiuQi:
			Vehicle_Analysis_Liuqi(rx_frame, &veh_info);
			break;
		case KingLong:
			Vehicle_Analysis_Kinglong(rx_frame, &veh_info);
			break;
		case JinLv:
			Vehicle_Analysis_Jinlv(rx_frame, &veh_info);
			break;
		case JinLv2:
			Vehicle_Analysis_Jinlv2(rx_frame, &veh_info);
			break;
		case FuTian:
			break;
		case DongFeng:
			Vehicle_Analysis_Dongfeng(rx_frame, &veh_info);
			break;
		default:
			break;
		}
		Camera_CAN_Transmition(&tx_frame,veh_info.vehicle_speed);
	}
}
