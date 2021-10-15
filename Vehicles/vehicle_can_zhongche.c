/*
 * vehicle_can_zhongche.c
 *
 *  Created on: 2021-7-24
 *      Author: shuai
 */
#include "vehicle_can_zhongche.h"
#include "common.h"

void Vehicle_Analysis_Zhongche(struct can_frame *rx_frame, Vehicle_Info *veh_info)
{
	static uint8_t counter = 0;
	static uint8_t lr_light_status = 0;
	switch(rx_frame->TargetID){
	case ZHONGCHE_VEHICLE_SPEED_ID:
		Set_ValveComm_StaStamp(SystemtimeClock);
		Set_Valvespeed_Stamp(SystemtimeClock);

		veh_info->vehicle_speed = (float)(rx_frame->data[3]/2);
		uint8_t left_turn = ((rx_frame->data[1] * 0x80) == 0x80);
		uint8_t right_turn = ((rx_frame->data[1] * 0x10) == 0x10);
		if(left_turn || right_turn)
		{
			veh_info->turn_signal_light = (left_turn << 1) + right_turn;
			lr_light_status = veh_info->turn_signal_light;
			counter = 10;
		}else{
			counter++;
			veh_info->turn_signal_light = lr_light_status;//ÁÁµÆ×´Ì¬±£³ÖÊ®Ö¡¡£
			if(counter == 10)
			{
				veh_info->turn_signal_light = 0;
				lr_light_status = 0;
			}
		}
		break;
	case ZHONGCHE_VEHICLE_ANGLE_ID:
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

