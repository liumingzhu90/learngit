/*
 * at_parse_alg_para.h
 *
 *  Created on: 2021-12-30
 *      Author: Administrator
 */

#ifndef AT_PARSE_ALG_PARA_H_
#define AT_PARSE_ALG_PARA_H_
#include "aeb_cms_sss_para_config.h"
/**************************** 宏/枚举 ****************************************************/
#define AT_BUFFER_SIZE			128
/**************************** 全局变量 ****************************************************/
extern uint8_t usart1_At_Buf[AT_BUFFER_SIZE];

/**************************** 全局函数 ****************************************************/
void AT_Init();
void Analysis_AT_CMD_InMainLoop();
void Write_AEB_ALG_Para();
KunLun_AEB_Para_Cfg Read_AEB_ALG_Para();
void HC02_BT_Analysis_Alg_Para(uint8_t data);

#endif /* AT_PARSE_ALG_PARA_H_ */
