/*
 * get_parameter.h
 *
 *  Created on: 2021-8-13
 *      Author: wangzhenbao
 */

#ifndef GET_PARAMETER_H_
#define GET_PARAMETER_H_

void Bluetooth_Get_Commond(uint8_t *data);
uint32_t Bluetooth_Get_Commond_Annalyse(uint8_t *data);
void Get_Can_Rata_Config(uint8_t cannum,uint16_t *canrata);

#endif /* GET_PARAMETER_H_ */
