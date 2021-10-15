/*
 * vehicle_can_jinlv.h
 *
 *  Created on: 2021-7-26
 *      Author: shuai
 */

#ifndef VEHICLE_CAN_JINLV_H_
#define VEHICLE_CAN_JINLV_H_

#include "system_init.h"
#include "canhl.h"
#include "vehicle_can.h"

#define JINLV_VEHICLE_SPEED_ID 0x0CF101A7

#define JINLV_VEHICLE_SPEED_ID2 0x18A70017
/*
 * BYTE3	bit2-bit3	倒挡			00-OFF		01-ON		11为无效	10为故障
 * BYTE4   bit5-bit6	近光灯信号	00-近光灯OFF 01-近光灯ON  11为无效  10为故障
 * BYTE4	bit7-bit8	远光灯信号	00-远光灯OFF	01-远光灯ON	11为无效  10为故障
 * BYTE5	bit0-bit1	左转向		00-左转向OFF	01-左转向ON	11为无效 10为故障
 * BYTE5	bit2-bit3	右转向		00-右转向OFF 01-右转向ON	11为无效	10为故障
 * BYTE5	bit4-bit5	制动状态		00-制动OFF	01-制动ON	11为无效  10为故障
 * BYTE5	bit6-bit7				00-OFF		01-ON		11为无效	10为故障
 */
#define JINLV_VEHICLE_SPEED_ID3 0x0CFE6CEE
 /*
  * BYTE7 - BYTE8 两个字节
  * */

void Vehicle_Analysis_Jinlv(struct can_frame *rx_frame, Vehicle_Info *veh_info);
void Vehicle_Analysis_Jinlv2(struct can_frame *rx_frame, Vehicle_Info *veh_info);

#endif /* VEHICLE_CAN_JINLV_H_ */
