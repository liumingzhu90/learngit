/*
 * vehicle_can_kinglong.h
 *
 *  Created on: 2021-7-24
 *      Author: shuai
 */

#ifndef VEHICLE_CAN_KINGLONG_H_
#define VEHICLE_CAN_KINGLONG_H_

#include "system_init.h"
#include "canhl.h"
#include "vehicle_can.h"

#define KINGLONG_VEHICLE_SPEED_ID 0x18FEF117
#define KINGLONG_VEHICLE_SWITCH_ID 0x0CFDCC27
#define KINGLONG_VEHICLE_ANGLE_ID 0x18F0090B

void Vehicle_Analysis_Kinglong(struct can_frame *rx_frame, Vehicle_Info *veh_info);

#endif /* VEHICLE_CAN_KINGLONG_H_ */
