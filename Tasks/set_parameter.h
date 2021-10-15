/*
 * set_parameter.h
 *
 *  Created on: 2021-8-13
 *      Author: wangzhenbao
 */

#ifndef SET_PARAMETER_H_
#define SET_PARAMETER_H_

void Bluetooth_Set_Commond(uint8_t *data);
uint32_t Bluetooth_Set_Commond_Annalyse(uint8_t *data);
uint32_t Bluetooth_Set_Vehicle_With(uint8_t *data,uint32_t flag1,uint32_t flag2);
uint32_t Bluetooth_Set_Vehicle_Sensitivity(uint8_t *data,uint32_t flag1,uint32_t flag2);
uint32_t Bluetooth_Set_TTC(uint8_t *data,uint32_t flag1,uint32_t flag2);
uint32_t Bluetooth_Set_Keep_Dis(uint8_t *data,uint32_t flag1);
void Init_Flash_Parameter(void);
void Init_Config_Parameter(void);
void Init_Sys_Parameter(void);

#endif /* SET_PARAMETER_H_ */
