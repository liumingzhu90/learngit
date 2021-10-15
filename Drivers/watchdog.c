/*
 * watchdog.c
 *
 *  Created on: 2021-7-22
 *      Author: chenq
 */

#include "watchdog.h"

/**
  * ���� �������Ź��趨ʱ�����á�
  * ����  Overflow��ȡֵΪ0~0xFFF�� �������Ź����ֵ
  *     Prescaler����INTLF��Ԥ��Ƶֵ
  * ����  �ޡ�
  */
void IWDT_Config(uint32_t Overflow,uint32_t Prescaler)
{

#ifdef SYSCLK_FREQ_HSI
	OSC_SCK_Source_Config(SCLK_SOURCE_INTHF);                       //ѡ��INTHF��Ϊϵͳʱ�ӽ�Ƶ
#else
	OSC_SCK_Source_Config(SCLK_SOURCE_EXTHF);                       //ѡ��EXTHF��Ϊϵͳʱ�ӽ�Ƶ
#endif
	BKP_Write_And_Read_Enable(TRUE);                                //�������дʹ��
	PM_Independent_Watchdog_Reset_Config(PERIPHERAL_OUTRST_STATUS); //ʹ�ܶ������Ź��˳���λ״̬
	PM_Internal_Low_Frequency_Enable(TRUE);                         //�ڲ���Ƶ����ʹ��
	IWDT_Overflow_Config (Overflow);                                //�������Ź����ֵ��ȡֵΪ500��//1���ӿ��Ź����
	IWDT_Prescaler_Config(Prescaler);                               //�������Ź���INTLF��1��64Ԥ��Ƶֵ
	IWDT_Enable(TRUE);                                              //ʹ�ܶ������Ź�

	OSC_SCK_Source_Config(SCLK_SOURCE_PLL);                         //ѡ��pll��Ϊϵͳʱ��
}
