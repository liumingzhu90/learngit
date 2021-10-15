/*
 * EC200U.h
 *
 *  Created on: 2021-7-10
 *      Author: chenq
 */

#ifndef EC200U_H_
#define EC200U_H_
#include "system_init.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#define EC200U_Rxbuffer_MAX 256
extern uint8_t EC200U_Receive_flag;
extern uint8_t EC200U_Rxbuffer[EC200U_Rxbuffer_MAX];
extern uint8_t EC200U_Rxcount;

#define EC200U_POWERON(x)  GPIO_Set_Output_Data_Bits(GPIOD_SFR,GPIO_PIN_MASK_1, x)
#define EC200U_POWERKEY(x) GPIO_Set_Output_Data_Bits(GPIOB_SFR,GPIO_PIN_MASK_8, x)
#define EC200U_RESET(x)    GPIO_Set_Output_Data_Bits(GPIOB_SFR,GPIO_PIN_MASK_7, x)
#define EC200U_WAKEUP(x)   GPIO_Set_Output_Data_Bits(GPIOB_SFR,GPIO_PIN_MASK_6, x)
#define EC200U_DISABLE(x)  GPIO_Set_Output_Data_Bits(GPIOB_SFR,GPIO_PIN_MASK_9, x)

#define IPADDR "192.168.2.251"
#define PORT  1000
typedef enum
{
    EC200U_WORKMODE_NET = 0,  /*TCP/IP����͸��*/
    EC200U_WORKMODE_HTTP,     /*HTTP����͸��*/
    EC200U_WORKMODE_MQTT,     /*MQTT����͸��*/
    EC200U_WORKMODE_ALIYUN,   /*������͸��*/
    EC200U_WORKMODE_BAIDUYUN, /*�ٶ���͸��*/
} work_mode_eu;

typedef struct
{
    uint32_t timeout;
    char *read_cmd;
    char *write_cmd;
} atcmd_st;


void EC200U_INIT();
int EC200U_Start(work_mode_eu mode);
void EC200U_SendData(uint8_t* Databuf, uint32_t length);
int EC200U_Send_StrData(char *bufferdata,uint32_t length);
int EC200U_Get_StrData(char *bufferdata,uint32_t *length);
void EC20Send_HexData(char *bufferdata,uint32_t length);
#endif /* EC200U_H_ */
