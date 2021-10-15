/*
 * vehicle_can_sunlong.h
 *
 *  Created on: 2021-7-24
 *      Author: shuai
 */

#ifndef VEHICLE_CAN_SUNLONG_H_
#define VEHICLE_CAN_SUNLONG_H_

#include "system_init.h"
#include "canhl.h"
#include "vehicle_can.h"

#define SUNLONG_VEHICLE_SPEED_ID 0x1804C1D0 //speed,voltage,electricity READ
#define SUNLONG_VEHICLE_SWITCH_ID 0x1803C1D0 //SWITCH READ
#define SUNLONG_VEHICLE_MOTOR_ID 0x1802C1D0 //MOTOR READ
#define SUNLONG_VEHICLE_AEB1_ID 0x1801D0C1 //AEB1 Tx
#define SUNLONG_VEHICLE_ANGLE_ID 0x18F0090B //VDC2, angle
#define SUNLONG_VEHICLE_EBC1_ID 0x18F0010B //EBC1, brake pedal %

void Vehicle_Analysis_Sunlong(struct can_frame *rx_frame, Vehicle_Info *veh_info);

#endif /* VEHICLE_CAN_SUNLONG_H_ */
