/*
 * vehicle_can_zhongtong.h
 *
 *  Created on: 2021-7-24
 *      Author: shuai
 */

#ifndef VEHICLE_CAN_ZHONGTONG_H_
#define VEHICLE_CAN_ZHONGTONG_H_

#include "system_init.h"
#include "canhl.h"
#include "vehicle_can.h"

#define ZHONGTONG_VEHICLE_SPEED_ID	0x18FEF100
#define ZHONGTONG_VEHICLE_SWITCH_ID	0x0CFDCC17
#define ZHONGTONG_VEHICLE_ANGLE_ID	0x18F0090B

void Vehicle_Analysis_Zhongtong(struct can_frame *rx_frame, Vehicle_Info *veh_info);

#endif /* VEHICLE_CAN_ZHONGTONG_H_ */
