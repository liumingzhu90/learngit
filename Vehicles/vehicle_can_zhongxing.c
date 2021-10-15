/*
 * vehicle_can_zhongxing.c
 *
 *  Created on: 2021-7-24
 *      Author: shuai
 */
#include "vehicle_can_zhongxing.h"
#include "common.h"

void Vehicle_Analysis_Zhongxing(struct can_frame *rx_frame, Vehicle_Info *veh_info)
{
	uint8_t left_turn, right_turn;
	switch(rx_frame->TargetID){
	case ZHONGXING_VEHICLE_SPEED_ID:
		Set_ValveComm_StaStamp(SystemtimeClock);
		Set_Valvespeed_Stamp(SystemtimeClock);

		veh_info->vehicle_speed = (float)rx_frame->data[7];
		break;
	case ZHONGXING_VEHICLE_SWITCH_ID:
		Set_ValveComm_StaStamp(SystemtimeClock);
		left_turn = ((rx_frame->data[0] & 0x40) == 0x40);
		right_turn = ((rx_frame->data[0] & 0x10) == 0x10);
		veh_info->turn_signal_light = (left_turn << 1) + right_turn;
		break;
	case ZHONGXING_VEHICLE_ANGLE_ID:
		Set_ValveComm_StaStamp(SystemtimeClock);
		veh_info->steering_wheel_angle = ((float)(((uint16_t)rx_frame->data[1])<<8 | rx_frame->data[0]))/1024 - 31.374f;
		veh_info->yaw_rate = ((float)(((uint16_t)rx_frame->data[4])<<8 | rx_frame->data[3]))/8192 - 3.92f;
		veh_info->lateral_acceleration  = ((float)(((uint16_t)rx_frame->data[6])<<8 | rx_frame->data[5]))/2048 - 15.687f;
		veh_info->longitudinal_acceleration = (float)(rx_frame->data[7])*0.1f - 12.5f;
		break;
	default:
		break;
	}
}

