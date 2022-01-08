/*
 * _4g_upgrade.h
 *
 *  Created on: 2021-12-3
 *      Author: Administrator
 * ����˵��������OTA����|�����ϴ�|��ȡGPS��λ��Ϣ|��������
 */

#ifndef _4G_UPGRADE_H_
#define _4G_UPGRADE_H_
#include "upgrade_common.h"
#include "EC200U.h"

/****************************�ꡢö�ٶ���**************************************/

/********************************�ṹ�嶨��**************************************/

/****************************ȫ�ֱ���**************************************/
extern Quectel4GStatus	_4G_status;
//extern uint8_t			internal_stage;
/****************************ȫ�ֺ���**************************************/
// ����ƽ̨������
uint8_t Connect_Platform_Server(Quectel4GStatus ota_status,FunctionalState ota_on);

// OTA����
char 	getche(void);
void 	AT_4G_Module_Init();
void 	OTA_Start(Upgrade_Mode upgrade_mode,uint32_t size);
void 	OTA_End(FunctionalState flag);
uint8_t OTA_Upgrade_Interaction_In_MainLoop();		// ��mainѭ���彻��������
uint8_t OTA_Upgrade_Write_In_MainLoop();			// ��mainѭ����д������������
void 	OTA_Upgrade_In_USART0IT(uint8_t data);		// �ڴ���0�жϺ�����
void 	OTA_In_SysClick_IT(uint32_t sysTimeClock);	// �ڵδ�ʱ��������
void 	OTA_UserCmd_Analysis_In_USART1IT(uint8_t *User_Rxbuffer);

#endif /* _4G_UPGRADE_H_ */
