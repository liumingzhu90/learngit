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
	veh_info.vehicle_speed = 0.0; //车速
	veh_info.turn_signal_light = 0; //转向信号灯
	veh_info.brake_light = 0; //刹车灯信号
	veh_info.roll_rate = 0.0; //车身翻滚角
	veh_info.pitch_rate = 0.0; //车身俯仰角
	veh_info.yaw_rate = 0.0; //车身航横摆角
	veh_info.steering_wheel_angle = 0.0; //方向盘角度，逆时针为正
	veh_info.lateral_acceleration = 0.0; //横向加速度
	veh_info.longitudinal_acceleration = 0.0; //纵向加速度
	veh_info.current_gear = 0; //当前档位信息，0空档，1前进档，2倒车档，3停车档
	veh_info.brake_pedal_degree = 0.0; //刹车踏板开合度
	veh_info.accelerator_pedal_degree = 0.0; //油门踏板开合度
	veh_info.wiper_status = 0; //雨刮器状态

	veh_info.vehicle_info_flag = 0; //车辆信息更新标记位
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
