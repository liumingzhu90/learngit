/*
 * tim.c
 *
 *  Created on: 2021-7-21
 *      Author: chenq
 */
#include "tim.h"

/*/*����������ʱ����ʼ������ ����ͨ�ö�ʱ���͸߼���ʱ��ͬ��
* ����  BTIMx:ȡֵΪT14_SFR/T15_SFR
*    Peripheral��ȡֵΪINT_T14/INT_T15
*/
void BASIC_TIMER_Config(BTIM_SFRmap* BTIMx, InterruptIndex Peripheral,uint16_t period)
{
	//��ʱ��ʱ��Դѡ��SCLK  ������Ϊ50000 ��Ԥ��Ƶ23+1=24��Ƶ 120M��Ƶ ��ʱ10ms��һ���ж�

	TIM_Reset(BTIMx);												//��ʱ�����踴λ��ʹ������ʱ��
	BTIM_Updata_Immediately_Config(BTIMx,TRUE);						//�������¿���
	BTIM_Updata_Enable(BTIMx,TRUE);									//���ø���ʹ��
	BTIM_Work_Mode_Config(BTIMx,BTIM_TIMER_MODE);					//��ʱģʽѡ��
	BTIM_Set_Counter(BTIMx,0);										//��ʱ������ֵ
	BTIM_Set_Period(BTIMx,period);									//��ʱ������ֵ50000
	BTIM_Set_Prescaler(BTIMx,23);								    //��ʱ��Ԥ��Ƶֵ23+1=24
	BTIM_Counter_Mode_Config(BTIMx,BTIM_COUNT_UP_OF);				//���ϼ���,��������жϱ�־
	BTIM_Clock_Config(BTIMx,BTIM_SCLK);								//ѡ��SCLKʱ��
	INT_Interrupt_Priority_Config(Peripheral,4,0);					//��ռ���ȼ�4,�����ȼ�0
	BTIM_Overflow_INT_Enable(BTIMx,TRUE);							//��������ж�ʹ��
	INT_Interrupt_Enable(Peripheral,TRUE);						    //�����ж�ʹ��
	INT_Clear_Interrupt_Flag(Peripheral);							//���жϱ�־
	BTIM_Cmd(BTIMx,TRUE);											//��ʱ����������ʹ��
	INT_Stack_Align_Config(INT_STACK_SINGLE_ALIGN);					//�ж��Զ���ջʹ�õ��ֶ���
	INT_All_Enable (TRUE);											//ȫ�ֿ������ж�ʹ��,���ж�ʹ�ܿ��Ʋ�������λ/NMI/Ӳ�������ж�

}
