/*
 * gpio.c
 *
 *  Created on: 2021-7-7
 *      Author: chenq
 */
#include "gpio.h"

void GPIO_Out_Config(void)
{
	GPIO_Write_Mode_Bits(GPIOE_SFR, GPIO_PIN_MASK_6, GPIO_MODE_OUT);//LED1
	//GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_0|GPIO_PIN_MASK_1|GPIO_PIN_MASK_2, GPIO_MODE_OUT);//LED2 3 4
	//GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_3|GPIO_PIN_MASK_4|GPIO_PIN_MASK_7, GPIO_MODE_OUT);//LED5 6 7
	GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_0|GPIO_PIN_MASK_1, GPIO_MODE_OUT);//LED2 3
	GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_4|GPIO_PIN_MASK_7, GPIO_MODE_OUT);//LED6 7
	GPIO_Write_Mode_Bits(GPIOC_SFR, GPIO_PIN_MASK_7|GPIO_PIN_MASK_8, GPIO_MODE_OUT);//LED4 5
	GPIO_Write_Mode_Bits(GPIOB_SFR, GPIO_PIN_MASK_15, GPIO_MODE_OUT);//输出控制供电拐角
	GPIO_Set_Output_Data_Bits(GPIOB_SFR,GPIO_PIN_MASK_15, 1);
	GPIO_Write_Mode_Bits(GPIOH_SFR , GPIO_PIN_MASK_14 | GPIO_PIN_MASK_15, GPIO_MODE_OUT );//stoplight+  stoplight-
}

void GPIO_Power_CANFA_Control(void)
{
	GPIO_Write_Mode_Bits(GPIOE_SFR, GPIO_PIN_MASK_3, GPIO_MODE_OUT);
	GPIO_Set_Output_Data_Bits(GPIOE_SFR,GPIO_PIN_MASK_3, 1);
	GPIO_Set_Output_Data_Bits(GPIOH_SFR,GPIO_PIN_MASK_14, 0);//正控刹车灯亮 1 0灭
}

void GPIO_Singal_init(void)
{
	GPIO_Write_Mode_Bits(GPIOG_SFR , GPIO_PIN_MASK_6 | GPIO_PIN_MASK_7, GPIO_MODE_IN );  //stop         turnleft
	GPIO_Write_Mode_Bits(GPIOH_SFR , GPIO_PIN_MASK_5 | GPIO_PIN_MASK_6, GPIO_MODE_IN );  //right        back
	GPIO_Write_Mode_Bits(GPIOH_SFR , GPIO_PIN_MASK_8 | GPIO_PIN_MASK_9, GPIO_MODE_IN );  //IN1          IN2
	GPIO_Write_Mode_Bits(GPIOH_SFR , GPIO_PIN_MASK_12 | GPIO_PIN_MASK_13, GPIO_MODE_IN );//AEB switch   LDW switch
	GPIO_Write_Mode_Bits(GPIOC_SFR,GPIO_PIN_MASK_4,GPIO_MODE_IN);
	//GPIO_Pin_RMP_Config(GPIOE_SFR,GPIO_PIN_MASK_4,GPIO_RMP_AF3_T21);
	//GPIO_Write_Mode_Bits(GPIOC_SFR , GPIO_PIN_MASK_4 , GPIO_MODE_IN );//speedin
}
void Set_LED1(BitAction BitsValue)
{
	GPIO_Set_Output_Data_Bits(GPIOE_SFR,GPIO_PIN_MASK_6, BitsValue);
}
void Set_LED2(BitAction BitsValue)
{
	GPIO_Set_Output_Data_Bits(GPIOF_SFR,GPIO_PIN_MASK_0, BitsValue);
}
void Set_LED3(BitAction BitsValue)
{
	GPIO_Set_Output_Data_Bits(GPIOF_SFR,GPIO_PIN_MASK_1, BitsValue);
}
void Set_LED4(BitAction BitsValue)
{
	GPIO_Set_Output_Data_Bits(GPIOC_SFR,GPIO_PIN_MASK_7, BitsValue);
}
void Set_LED5(BitAction BitsValue)
{
	GPIO_Set_Output_Data_Bits(GPIOC_SFR,GPIO_PIN_MASK_8, BitsValue);
}
void Set_LED6(BitAction BitsValue)
{
	GPIO_Set_Output_Data_Bits(GPIOF_SFR,GPIO_PIN_MASK_4, BitsValue);
}
void Set_LED7(BitAction BitsValue)
{
	GPIO_Set_Output_Data_Bits(GPIOF_SFR,GPIO_PIN_MASK_7, BitsValue);
}
BitAction Read_stop_signal(void)
{
	return GPIO_Read_Input_Data_Bit (GPIOG_SFR, GPIO_PIN_MASK_6);
}

BitAction Read_turnleft_signal(void)
{
	return GPIO_Read_Input_Data_Bit (GPIOG_SFR, GPIO_PIN_MASK_7);
}

BitAction Read_turnright_signal(void)
{
	return GPIO_Read_Input_Data_Bit (GPIOH_SFR, GPIO_PIN_MASK_5);
}

BitAction Read_turnback_signal(void)
{
	return GPIO_Read_Input_Data_Bit (GPIOH_SFR, GPIO_PIN_MASK_6);
}

BitAction Read_AEB_switch(void)
{
	return GPIO_Read_Input_Data_Bit (GPIOH_SFR, GPIO_PIN_MASK_12);
}

BitAction Read_LDW_switch(void)
{
	return GPIO_Read_Input_Data_Bit (GPIOH_SFR, GPIO_PIN_MASK_13);
}
