/*
 * usart_upgrade.h
 *
 *  Created on: 2021-11-17
 *      Author: Administrator
 */

#ifndef USART_UPGRADE_H_
#define USART_UPGRADE_H_
#include "upgrade_common.h"
#include "w25qxx.h"
/*******************************宏定义**********************************************/
/* -----------------------全局函数声明------------------------------- */
void 	Usart_Upgrade_Start();
void 	Usart_Upgrade_End();

void 	Usart1_UserCmd_Analysis();
void 	Usart1_UserUpgrade_Interactoin_In_MainLoop();
uint32_t Get_CRC(uint32_t bin_size);
#endif /* USART_UPGRADE_H_ */
