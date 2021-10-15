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
 * BYTE3	bit2-bit3	����			00-OFF		01-ON		11Ϊ��Ч	10Ϊ����
 * BYTE4   bit5-bit6	������ź�	00-�����OFF 01-�����ON  11Ϊ��Ч  10Ϊ����
 * BYTE4	bit7-bit8	Զ����ź�	00-Զ���OFF	01-Զ���ON	11Ϊ��Ч  10Ϊ����
 * BYTE5	bit0-bit1	��ת��		00-��ת��OFF	01-��ת��ON	11Ϊ��Ч 10Ϊ����
 * BYTE5	bit2-bit3	��ת��		00-��ת��OFF 01-��ת��ON	11Ϊ��Ч	10Ϊ����
 * BYTE5	bit4-bit5	�ƶ�״̬		00-�ƶ�OFF	01-�ƶ�ON	11Ϊ��Ч  10Ϊ����
 * BYTE5	bit6-bit7				00-OFF		01-ON		11Ϊ��Ч	10Ϊ����
 */
#define JINLV_VEHICLE_SPEED_ID3 0x0CFE6CEE
 /*
  * BYTE7 - BYTE8 �����ֽ�
  * */

void Vehicle_Analysis_Jinlv(struct can_frame *rx_frame, Vehicle_Info *veh_info);
void Vehicle_Analysis_Jinlv2(struct can_frame *rx_frame, Vehicle_Info *veh_info);

#endif /* VEHICLE_CAN_JINLV_H_ */
