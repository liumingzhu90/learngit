/*
 * vehicle_can_wanxiang.c
 *
 *  Created on: 2021-7-26
 *      Author: shuai
 */
#include "vehicle_can_wanxiang.h"
#include "common.h"

void Vehicle_Analysis_Wanxiang(struct can_frame *rx_frame, Vehicle_Info *veh_info)
{
	uint8_t left_turn, right_turn, gear;
	switch(rx_frame->TargetID){
	case WANXIANG_VEHICLE_SPEED_ID:
		Set_ValveComm_StaStamp(SystemtimeClock);
		Set_Valvespeed_Stamp(SystemtimeClock);
		veh_info->vehicle_speed = (float)((rx_frame->data[3] << 8) + rx_frame->data[2])*0.1;
		gear = rx_frame->data[4] & 0x0F;
		if(gear == 0)
		{
			veh_info->current_gear = 0;
		}else if(gear == 13){
			veh_info->current_gear = 2;
		}else if(gear == 15){
			veh_info->current_gear = 3;
		}else{
			veh_info->current_gear = 0;
		}
		break;
	case WANXIANG_VEHICLE_SWITCH_ID:
		Set_ValveComm_StaStamp(SystemtimeClock);
		left_turn = ((rx_frame->data[4] & 0x03) == 1);
		right_turn = (((rx_frame->data[4] >> 2) & 0x03) == 1);
		veh_info->turn_signal_light = (left_turn << 1) + right_turn;
		break;
	default:
		break;
	}
}

