/*
 * vehicle_can_zkhy.c
 *
 *  Created on: 2021-7-23
 *      Author: shuai
 */
#include "vehicle_can_zkhy.h"
#include "common.h"

void Vehicle_Analysis_ZKHY(struct can_frame *rx_frame, Vehicle_Info *veh_info)
{
	switch(rx_frame->TargetID){
	case ZKHY_VEHICLE_ID:
		Set_ValveComm_StaStamp(SystemtimeClock);
		Set_Valvespeed_Stamp(SystemtimeClock);
		veh_info->turn_signal_light = rx_frame->data[0] & 0x03;
		veh_info->wiper_status =  (rx_frame->data[0] >> 2) & 0x03;
		veh_info->brake_light = (rx_frame->data[0] >> 4) & 0x03;
		veh_info->vehicle_speed = (float)rx_frame->data[1];
		break;
	default:
		break;
	}
}
