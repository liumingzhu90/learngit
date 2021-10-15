/*
 * vehicle_can_dongfeng.h
 *
 *  Created on: 2021-7-24
 *      Author: shuai
 */

#ifndef VEHICLE_CAN_DONGFENG_H_
#define VEHICLE_CAN_DONGFENG_H_

#include "system_init.h"
#include "canhl.h"
#include "vehicle_can.h"

#define DONGFENG_VEHICLE_SPEED_ID 0x18FEF100
#define DONGFENG_VEHICLE_ANGLE_ID 0x18F0090B
#define DONGFENG_VEHICLE_SWITCH_ID 0x0CFDCC32

void Vehicle_Analysis_Dongfeng(struct can_frame *rx_frame, Vehicle_Info *veh_info);

#endif /* VEHICLE_CAN_DONGFENG_H_ */
