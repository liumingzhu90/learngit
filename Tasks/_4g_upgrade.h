/*
 * _4g_upgrade.h
 *
 *  Created on: 2021-12-3
 *      Author: Administrator
 * 功能说明：包括OTA升级|数据上传|获取GPS定位信息|蓝牙功能
 */

#ifndef _4G_UPGRADE_H_
#define _4G_UPGRADE_H_
#include "upgrade_common.h"
#include "EC200U.h"

/****************************宏、枚举定义**************************************/

/********************************结构体定义**************************************/

/****************************全局变量**************************************/
extern Quectel4GStatus	_4G_status;
//extern uint8_t			internal_stage;
/****************************全局函数**************************************/
// 连接平台服务器
uint8_t Connect_Platform_Server(Quectel4GStatus ota_status,FunctionalState ota_on);

// OTA升级
char 	getche(void);
void 	AT_4G_Module_Init();
void 	OTA_Start(Upgrade_Mode upgrade_mode,uint32_t size);
void 	OTA_End(FunctionalState flag);
uint8_t OTA_Upgrade_Interaction_In_MainLoop();		// 在main循环体交互函数体
uint8_t OTA_Upgrade_Write_In_MainLoop();			// 在main循环体写升级包函数体
void 	OTA_Upgrade_In_USART0IT(uint8_t data);		// 在串口0中断函数体
void 	OTA_In_SysClick_IT(uint32_t sysTimeClock);	// 在滴答定时器函数体
void 	OTA_UserCmd_Analysis_In_USART1IT(uint8_t *User_Rxbuffer);

#endif /* _4G_UPGRADE_H_ */
