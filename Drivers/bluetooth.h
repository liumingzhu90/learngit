/*
 * bluetooth.h
 *
 *  Created on: 2021-6-22
 *      Author: chenq
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_
#include "system_init.h"
#define Bluetooth_Rxbuffer_MAX 256
extern uint8_t Bluetooth_Receive_flag;
extern uint8_t Bluetooth_Rxbuffer[Bluetooth_Rxbuffer_MAX];
extern uint8_t Bluetooth_Rxcount;

void Bluetooth_init();//д╛хо9600
int Bluetooth_get_role(uint8_t *ismaster);
int Bluetooth_set_cmd(uint8_t *cmd,uint32_t length);
int Bluetooth_cfg_cmd(uint8_t *cmd,uint32_t length,uint8_t * respon);
int Bluetooth_send_data(uint8_t *tx_data,uint32_t length);
int Bluetooth_get_data(uint8_t *rx_data,uint32_t length);
int Bluetooth_set_baud(uint32_t baud);
void Clear_Bluetooth_Buffer(void);
#endif /* BLUETOOTH_H_ */
