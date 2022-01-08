/*
 * vehicle_can_liuqi.c
 *
 *  Created on: 2021-7-26
 *      Author: shuai
 */
#include "vehicle_can_liuqi.h"
#include "common.h"

void Vehicle_Analysis_Liuqi(struct can_frame *rx_frame, Vehicle_Info *veh_info)
{
	uint8_t left_turn,right_turn;
	switch(rx_frame->TargetID){
	case LIUQI_VEHICLE_SPEED_ID:
		Set_ValveComm_StaStamp(SystemtimeClock);
		Set_Valvespeed_Stamp(SystemtimeClock);
		veh_info->vehicle_speed = (float)rx_frame->data[1];
		break;
	case LIUQI_VEHICLE_SWITCH_ID:
		Set_ValveComm_StaStamp(SystemtimeClock);
		left_turn = (((rx_frame->data[1] >> 6) & 0x03) == 1);
		right_turn = (((rx_frame->data[1] >> 4) & 0x03) == 1);
		veh_info->turn_signal_light = (left_turn << 1) + right_turn;
		veh_info->brake_light = ((rx_frame->data[0] & 0x03) == 1);
		break;
	case LIUQI_VEHICLE_ANGLE_ID:
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

