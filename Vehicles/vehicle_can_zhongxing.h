/*
 * vehicle_can_zhongxing.h
 *
 *  Created on: 2021-7-24
 *      Author: shuai
 */

#ifndef VEHICLE_CAN_ZHONGXING_H_
#define VEHICLE_CAN_ZHONGXING_H_

#include "system_init.h"
#include "canhl.h"
#include "vehicle_can.h"

#define ZHONGXING_VEHICLE_SPEED_ID	0x0CF11AD0
#define ZHONGXING_VEHICLE_SWITCH_ID	0x18F15317
#define ZHONGXING_VEHICLE_ANGLE_ID	0x18F0090B

void Vehicle_Analysis_Zhongxing(struct can_frame *rx_frame, Vehicle_Info *veh_info);

#endif /* VEHICLE_CAN_ZHONGXING_H_ */
