/*
 * _4g_bt.c
 *
 *  Created on: 2021-12-29
 *      Author: Administrator
 */
#include "_4g_bt.h"
#include "w25qxx.h"
#include "upgrade_common.h"
uint8_t _4g_BT_In_MainLoop()
{

}


KunLun_AEB_Para_Cfg Get_AEB_Para_Cfg(uint8_t id)
{
	KunLun_AEB_Para_Cfg para = {0};
	// ��w25q16��ȡ������
	uint8_t data[ALG_PARA_SIZE] = {0};
	Get_AEB_Alg_Parameter(data,id);
	if(strlen(data) > 10){
		// ��������
	}

	return para;
}

uint8_t Set_AEB_Para_Cfg(KunLun_AEB_Para_Cfg para)
{
	uint8_t data[ALG_PARA_SIZE] = {0};
	// ƴ���ַ���

	Set_AEB_Alg_Parameter(data,para.id);
}
