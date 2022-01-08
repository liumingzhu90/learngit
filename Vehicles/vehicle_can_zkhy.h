/*
 * vehicle_can_zkhy.h
 *
 *  Created on: 2021-7-23
 *      Author: shuai
 */

#ifndef VEHICLE_CAN_ZKHY_H_
#define VEHICLE_CAN_ZKHY_H_
#include "system_init.h"
#include "canhl.h"
#include "vehicle_can.h"

#define ZKHY_VEHICLE_ID 0x7B0
#define ZKHY_VEHICLE_REFRESH_RATE 100

void Vehicle_Analysis_ZKHY(struct can_frame *rx_frame, Vehicle_Info *veh_info);

#endif /* VEHICLE_CAN_ZKHY_H_ */
