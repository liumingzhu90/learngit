/*
 * watchdog.c
 *
 *  Created on: 2021-7-22
 *      Author: chenq
 */

#include "watchdog.h"

/**
  * 描述 独立看门狗设定时间配置。
  * 输入  Overflow：取值为0~0xFFF。 独立看门狗溢出值
  *     Prescaler：对INTLF的预分频值
  * 返回  无。
  */
void IWDT_Config(uint32_t Overflow,uint32_t Prescaler)
{

#ifdef SYSCLK_FREQ_HSI
	OSC_SCK_Source_Config(SCLK_SOURCE_INTHF);                       //选择INTHF作为系统时钟降频
#else
	OSC_SCK_Source_Config(SCLK_SOURCE_EXTHF);                       //选择EXTHF作为系统时钟降频
#endif
	BKP_Write_And_Read_Enable(TRUE);                                //备份域读写使能
	PM_Independent_Watchdog_Reset_Config(PERIPHERAL_OUTRST_STATUS); //使能独立看门狗退出复位状态
	PM_Internal_Low_Frequency_Enable(TRUE);                         //内部低频晶振使能
	IWDT_Overflow_Config (Overflow);                                //独立看门狗溢出值，取值为500。//1秒钟看门狗溢出
	IWDT_Prescaler_Config(Prescaler);                               //独立看门狗对INTLF的1：64预分频值
	IWDT_Enable(TRUE);                                              //使能独立看门狗

	OSC_SCK_Source_Config(SCLK_SOURCE_PLL);                         //选择pll作为系统时钟
}
