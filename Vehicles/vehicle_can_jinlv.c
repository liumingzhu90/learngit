/*
 * vehicle_can_jinlv.c
 *
 *  Created on: 2021-7-26
 *      Author: shuai
 */
#include "vehicle_can_jinlv.h"
#include "common.h"

void Vehicle_Analysis_Jinlv(struct can_frame *rx_frame, Vehicle_Info *veh_info)
{
	uint8_t n_gear,d_gear,r_gear,p_gear;
	switch(rx_frame->TargetID){
	case JINLV_VEHICLE_SPEED_ID:
		Set_ValveComm_StaStamp(SystemtimeClock);
		Set_Valvespeed_Stamp(SystemtimeClock);
		veh_info->vehicle_speed = (float)rx_frame->data[3]; //取车速整数部分。
		veh_info->brake_light = (rx_frame->data[0] >> 7) & 0x01;
		veh_info->accelerator_pedal_degree = rx_frame->data[1] * 0.4;
		veh_info->brake_pedal_degree = rx_frame->data[4] * 0.4;
		d_gear = (rx_frame->data[0] >> 2) & 0x01;
		r_gear = (rx_frame->data[0] >> 3) & 0x01;
		n_gear = (rx_frame->data[0] >> 4) & 0x01;
		p_gear = (rx_frame->data[0] >> 5) & 0x01;

		if(p_gear == 1){
			veh_info->current_gear = 3;
		}else if(r_gear == 1){
			veh_info->current_gear = 2;
		}else if(d_gear == 1){
			veh_info->current_gear = 1;
		}else{
			veh_info->current_gear = 0;
		}

		break;
	default:
		break;
	}
}

uint8_t daodang = 0;
uint8_t rturn = 0;
uint8_t lturn = 0;
uint8_t br = 0;
uint8_t	shousha = 0;
void Vehicle_Analysis_Jinlv2(struct can_frame *rx_frame, Vehicle_Info *veh_info)
{
	//uint8_t rturn = 0;
	//uint8_t lturn = 0;

	switch(rx_frame->TargetID){
	case JINLV_VEHICLE_SPEED_ID2:
		Set_ValveComm_StaStamp(SystemtimeClock);
		stVehicleParas.LeftFlagTemp = rx_frame->data[4] & 0x03;
		lturn = stVehicleParas.LeftFlagTemp;
		stVehicleParas.RightFlagTemp = (rx_frame->data[4] >> 2) & 0x03;
		rturn = stVehicleParas.RightFlagTemp;
		if((lturn == 0) && (rturn == 0))
			veh_info->turn_signal_light = 0;
		if((lturn == 1) && (rturn == 1))
			veh_info->turn_signal_light = 3;
		if((lturn == 0) && (rturn == 1))
			veh_info->turn_signal_light = 1;
		if((lturn == 1) && (rturn == 0))
			veh_info->turn_signal_light = 2;
		//veh_info->brake_light =
		stVehicleParas.BrakeFlag = (rx_frame->data[4] >> 4) & 0x03;
		stVehicleParas.kongdangFlag = (rx_frame->data[2] >> 2) & 0x03;
		stVehicleParas.shoudBrakeFlag = (rx_frame->data[4] >> 6) & 0x03;
		break;
	case JINLV_VEHICLE_SPEED_ID3:
		Set_ValveComm_StaStamp(SystemtimeClock);
		Set_Valvespeed_Stamp(SystemtimeClock);
		stVehicleParas.fVehicleSpeed = (float)rx_frame->data[7];
		break;
	default:
		break;
	}
}
