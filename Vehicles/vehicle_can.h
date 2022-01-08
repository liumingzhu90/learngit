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
	float vehicle_speed; //车速
	uint8_t turn_signal_light; //转向信号灯 0无转向，1右转向，2左转向，3双闪
	uint8_t brake_light; //刹车灯信号
	float roll_rate; //车身翻滚角
	float pitch_rate; //车身俯仰角
	float yaw_rate; //车身航横摆角
	float steering_wheel_angle; //方向盘角度，逆时针为正
	float lateral_acceleration; //横向加速度
	float longitudinal_acceleration; //纵向加速度
	uint8_t current_gear; //当前档位信息,0:N,1:D,2:R,3:P
	float brake_pedal_degree; //刹车踏板开合度
	float accelerator_pedal_degree; //油门踏板开合度
	uint8_t wiper_status; //雨刮器状态

	uint8_t vehicle_info_flag; //车辆信息更新标记位
}Vehicle_Info;

void Vehicle_CAN_Init();
void Vehicle_Can_Analysis(struct can_frame *rx_frame);

#endif /* VEHICLE_CAN_H_ */
