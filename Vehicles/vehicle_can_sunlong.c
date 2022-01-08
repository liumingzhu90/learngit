/*
 * vehicle_can_sunlong.c
 *
 *  Created on: 2021-7-24
 *      Author: shuai
 */
#include "vehicle_can_sunlong.h"
#include "common.h"

void Vehicle_Analysis_Sunlong(struct can_frame *rx_frame, Vehicle_Info *veh_info)
{
	static uint8_t counter = 0;
	static uint8_t lr_light_status = 0;
	uint8_t left_turn,right_turn;
	switch(rx_frame->TargetID){
	case SUNLONG_VEHICLE_MOTOR_ID:
		Set_ValveComm_StaStamp(SystemtimeClock);
		veh_info->accelerator_pedal_degree = (float)rx_frame->data[6];
		break;
	case SUNLONG_VEHICLE_SPEED_ID:
		Set_ValveComm_StaStamp(SystemtimeClock);
		Set_Valvespeed_Stamp(SystemtimeClock);
		veh_info->vehicle_speed = (float)(rx_frame->data[6]);//取车速的整数部分。
		break;
	case SUNLONG_VEHICLE_SWITCH_ID:
		Set_ValveComm_StaStamp(SystemtimeClock);
		left_turn = rx_frame->data[0] & 0x01;
		right_turn = (rx_frame->data[0] >> 1) & 0x02;
		veh_info->turn_signal_light = (left_turn << 1) + right_turn;
		veh_info->current_gear = (rx_frame->data[0] >> 2) & 0x03;
		break;
	case SUNLONG_VEHICLE_ANGLE_ID:
		Set_ValveComm_StaStamp(SystemtimeClock);
		veh_info->steering_wheel_angle = ((float)(((uint16_t)rx_frame->data[1])<<8 | rx_frame->data[0]))/1024 - 31.374f;
		veh_info->yaw_rate = ((float)(((uint16_t)rx_frame->data[4])<<8 | rx_frame->data[3]))/8192 - 3.92f;
		veh_info->lateral_acceleration  = ((float)(((uint16_t)rx_frame->data[6])<<8 | rx_frame->data[5]))/2048 - 15.687f;
		veh_info->longitudinal_acceleration = (float)(rx_frame->data[7])*0.1f - 12.5f;
		break;
	case SUNLONG_VEHICLE_EBC1_ID:
		Set_ValveComm_StaStamp(SystemtimeClock);
		veh_info->brake_pedal_degree = (float)rx_frame->data[1];
	default:
		break;
	}
}

