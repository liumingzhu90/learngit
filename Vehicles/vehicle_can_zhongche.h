/*
 * vehicle_can_zhongche.h
 *
 *  Created on: 2021-7-24
 *      Author: shuai
 */

#ifndef VEHICLE_CAN_ZHONGCHE_H_
#define VEHICLE_CAN_ZHONGCHE_H_

#include "system_init.h"
#include "canhl.h"
#include "vehicle_can.h"

#define ZHONGCHE_VEHICLE_SPEED_ID	0x0C19A7A1
#define ZHONGCHE_VEHICLE_ANGLE_ID	0x18F0090B

void Vehicle_Analysis_Zhongche(struct can_frame *rx_frame, Vehicle_Info *veh_info);

#endif /* VEHICLE_CAN_ZHONGCHE_H_ */
