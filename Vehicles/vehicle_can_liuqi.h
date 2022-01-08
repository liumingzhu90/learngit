/*
 * vehicle_can_liuqi.h
 *
 *  Created on: 2021-7-26
 *      Author: shuai
 */

#ifndef VEHICLE_CAN_LIUQI_H_
#define VEHICLE_CAN_LIUQI_H_

#include "system_init.h"
#include "canhl.h"
#include "vehicle_can.h"

#define LIUQI_VEHICLE_SPEED_ID 0x18FEF117
#define LIUQI_VEHICLE_ANGLE_ID 0x18F0090B
#define LIUQI_VEHICLE_SWITCH_ID 0x18FF0B21

void Vehicle_Analysis_Liuqi(struct can_frame *rx_frame, Vehicle_Info *veh_info);

#endif /* VEHICLE_CAN_LIUQI_H_ */
