/*
 * tim.c
 *
 *  Created on: 2021-7-21
 *      Author: chenq
 */
#include "tim.h"

/*/*描述基本定时器初始化配置 其他通用定时器和高级定时器同理
* 输入  BTIMx:取值为T14_SFR/T15_SFR
*    Peripheral：取值为INT_T14/INT_T15
*/
void BASIC_TIMER_Config(BTIM_SFRmap* BTIMx, InterruptIndex Peripheral,uint16_t period)
{
	//定时器时钟源选用SCLK  设周期为50000 设预分频23+1=24分频 120M主频 定时10ms进一次中断

	TIM_Reset(BTIMx);												//定时器外设复位，使能外设时钟
	BTIM_Updata_Immediately_Config(BTIMx,TRUE);						//立即更新控制
	BTIM_Updata_Enable(BTIMx,TRUE);									//配置更新使能
	BTIM_Work_Mode_Config(BTIMx,BTIM_TIMER_MODE);					//定时模式选择
	BTIM_Set_Counter(BTIMx,0);										//定时器计数值
	BTIM_Set_Period(BTIMx,period);									//定时器周期值50000
	BTIM_Set_Prescaler(BTIMx,23);								    //定时器预分频值23+1=24
	BTIM_Counter_Mode_Config(BTIMx,BTIM_COUNT_UP_OF);				//向上计数,上溢产生中断标志
	BTIM_Clock_Config(BTIMx,BTIM_SCLK);								//选用SCLK时钟
	INT_Interrupt_Priority_Config(Peripheral,4,0);					//抢占优先级4,子优先级0
	BTIM_Overflow_INT_Enable(BTIMx,TRUE);							//计数溢出中断使能
	INT_Interrupt_Enable(Peripheral,TRUE);						    //外设中断使能
	INT_Clear_Interrupt_Flag(Peripheral);							//清中断标志
	BTIM_Cmd(BTIMx,TRUE);											//定时器启动控制使能
	INT_Stack_Align_Config(INT_STACK_SINGLE_ALIGN);					//中断自动堆栈使用单字对齐
	INT_All_Enable (TRUE);											//全局可屏蔽中断使能,该中断使能控制不包含复位/NMI/硬件错误中断

}
