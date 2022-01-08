/*
 * vehicle_can_wanxiang.h
 *
 *  Created on: 2021-7-26
 *      Author: shuai
 */

#ifndef VEHICLE_CAN_WANXIANG_H_
#define VEHICLE_CAN_WANXIANG_H_

#include "system_init.h"
#include "canhl.h"
#include "vehicle_can.h"

#define WANXIANG_VEHICLE_SPEED_ID 0x18FA2FD0
#define WANXIANG_VEHICLE_SWITCH_ID 0x18FAC917

void Vehicle_Analysis_Wanxiang(struct can_frame *rx_frame, Vehicle_Info *veh_info);

#endif /* VEHICLE_CAN_WANXIANG_H_ */
