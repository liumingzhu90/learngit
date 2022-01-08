/*
 * bluetooth.c
 *
 *  Created on: 2021-6-22
 *      Author: chenqi
 */

#include "bluetooth.h"
#include "kf32a_basic_usart.h"
#include "kf32a_basic_rst.h"
#include "kf32a_basic_pclk.h"
#include "usart.h"
#include "stdio.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#define KEY PB3

USART_SFRmap * Bluetooth_USART=USART4_SFR;
uint8_t Bluetooth_Rxbuffer[Bluetooth_Rxbuffer_MAX]={0};
uint8_t Bluetooth_Rxcount=0;
uint8_t Bluetooth_Receive_flag=0;

void HC_KEY(BitAction BitsValue)
{
//	GPIO_Set_Output_Data_Bits(GPIOB_SFR,GPIO_PIN_MASK_3,BitsValue);//PB3
}

void Bluetooth_init()
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	USART_InitTypeDef	USART_InitStructure;

//	RST_CTL1_Peripheral_Reset_Enable(RST_CTL1_USART4RST,FALSE);//注意修改RST_CTL1_USART1RST为对应的值
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIODCLKEN,TRUE);
//	PCLK_CTL1_Peripheral_Clock_Enable(PCLK_CTL1_USART4CLKEN,TRUE);

	GPIO_Pin_RMP_Config(GPIOD_SFR,GPIO_Pin_Num_3,GPIO_RMP_AF6_USART4);
	GPIO_Pin_RMP_Config(GPIOD_SFR,GPIO_Pin_Num_4,GPIO_RMP_AF6_USART4);

	GPIO_Struct_Init(&GPIO_InitStructure);
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
		GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
		GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
		GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
		GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin =	GPIO_PIN_MASK_3|GPIO_PIN_MASK_4;
	GPIO_Configuration(GPIOD_SFR,&GPIO_InitStructure);
//	//KEY PB3
//	GPIO_Struct_Init(&GPIO_InitStructure);
//	GPIO_InitStructure.m_Mode = GPIO_MODE_OUT;
//	GPIO_InitStructure.m_Pin =	GPIO_PIN_MASK_3;
//	GPIO_InitStructure.m_PullUp = GPIO_PULLUP;
//	GPIO_Configuration(GPIOB_SFR,&GPIO_InitStructure);
//	//STATE  PB2
//	GPIO_Struct_Init(&GPIO_InitStructure);
//	GPIO_InitStructure.m_Mode = GPIO_MODE_IN;
//	GPIO_InitStructure.m_Pin =	GPIO_PIN_MASK_2;
//	GPIO_InitStructure.m_PullDown = GPIO_PULLDOWN;
//	GPIO_Configuration(GPIOB_SFR,&GPIO_InitStructure);

//	USART_Struct_Init(&USART_InitStructure);
	/*
	BaudRateInteger = (1000000/baud);
	if((BaudRateInteger*baud)>985000)
	{
		BaudRateNumerator = 0;
	}
	else
	{
		decimal = (float)1000000/(BaudRateInteger*baud)-1;
		BaudRateNumerator = (uint16_t)(decimal*1000);
		if(BaudRateNumerator < 64)
			BaudRateNumerator = 0;
		else
			BaudRateNumerator += 32;
			BaudRateNumerator >>= 6;
		if(BaudRateNumerator > 0x0f) BaudRateNumerator=0x0f;
		BaudRateDenominator = 0x0f;
	}
	USART_InitStructure.m_BaudRateInteger = BaudRateInteger;
	USART_InitStructure.m_BaudRateDenominator = BaudRateDenominator;
	USART_InitStructure.m_BaudRateNumerator = BaudRateNumerator;
	*/
    /* 波特率 =Fck/(16*z（1+x/y)) 外设时钟内部高频16M*/
    //4800    z:208    x:0    y:0
    //9600    z:104    x:0    y:0
    //19200   z:52     x:0    y:0
    //115200  z:8      x:1    y:13
    //波特率9600
    USART_InitStructure.m_BaudRateInteger=104;         //USART波特率整数部分z，取值为0~65535
    USART_InitStructure.m_BaudRateNumerator=0;         //USART波特率小数分子部分x，取值为0~0xF
    USART_InitStructure.m_BaudRateDenominator=0;       //USART波特率小数分母部分y，取值为0~0xF
	USART_InitStructure.m_BaudRateBRCKS = USART_CLK_HFCLK;
	USART_InitStructure.m_Mode = USART_MODE_FULLDUPLEXASY;
	USART_InitStructure.m_WordLength = USART_WORDLENGTH_8B;
	USART_InitStructure.m_StopBits = USART_STOPBITS_1;
	USART_InitStructure.m_TransferDir = USART_DIRECTION_FULL_DUPLEX;

	USART_Reset(USART4_SFR);
	USART_Configuration(USART4_SFR,&USART_InitStructure);
    USART_Passageway_Select_Config(USART4_SFR,USART_U7816R_PASSAGEWAY_TX0);//UASRT4选择TX0通道
	USART_Clear_Transmit_BUFR_INT_Flag(USART4_SFR);                //USART4发送BUF清零
	USART_Cmd(USART4_SFR,ENABLE);
	USART_Transmit_Data_Enable(USART4_SFR,ENABLE);
	USART_Receive_Data_Enable(USART4_SFR,ENABLE);
	USART_ReceiveInt_config(USART4_SFR,INT_USART4);
	HC_KEY(Bit_RESET);
}
//此函数用于设置HC05，适用于仅返回OK应答的	AT指令
//cmd:AT指令字符串，例如 "AT+RESET" / "AT+UART=9600,0,0" / "AT+ROLE=0" 等
//返回值：0设置成功，其他，设置失败
int Bluetooth_set_cmd(uint8_t *cmd,uint32_t length)
{
	uint8_t retry=0x0f;
	uint8_t temp,t;
	Bluetooth_Receive_flag=0;
	Bluetooth_Rxcount=0;
	temp=0xFF;
	while(retry--)
	{
		HC_KEY(Bit_SET);
		vTaskDelay(100);
		USART_Send(Bluetooth_USART,cmd,length);
		//USART_Send(Bluetooth_USART,"\r\n",2);
		HC_KEY(Bit_RESET);

		for(t=0;t<20;t++)
		{
			if(Bluetooth_Receive_flag==1)break;
			vTaskDelay(100);
		}
		if(Bluetooth_Receive_flag)
		{
			Bluetooth_Receive_flag=0;
			Bluetooth_Rxcount=0;
			if(Bluetooth_Rxbuffer[0]=='O'&&Bluetooth_Rxbuffer[1]=='K')
			{
				temp=0;
				break;
			}
		}
	}
	return temp;
}
//发送其他调试命令，不需要输入回车符
int Bluetooth_cfg_cmd(uint8_t *cmd,uint32_t length,uint8_t * respon)
{
	uint8_t retry=0x0f;
	uint8_t temp,t;
	Bluetooth_Receive_flag=0;
	Bluetooth_Rxcount=0;
	temp=0xFF;
	HC_KEY(Bit_SET);
	vTaskDelay(10);
	USART_Send(Bluetooth_USART,cmd,length);
	USART_Send(Bluetooth_USART,"\r\n",2);
	HC_KEY(Bit_RESET);
	for(t=0;t<20;t++)
	{
		if(Bluetooth_Receive_flag)break;
		vTaskDelay(100);
	}
	if(Bluetooth_Receive_flag)
	{
		temp=0;
		memcpy(respon,Bluetooth_Rxbuffer,Bluetooth_Rxcount);
		Bluetooth_Receive_flag=0;
		Bluetooth_Rxcount=0;
	}
	return temp;
}

int Bluetooth_send_data(uint8_t *tx_data,uint32_t length)
{
	USART_Send(Bluetooth_USART,tx_data,length);
}
int Bluetooth_get_data(uint8_t *rx_data,uint32_t length)
{
	int read_length=0;
	if(length>Bluetooth_Rxcount)
		read_length=Bluetooth_Rxcount;
	else
		read_length=length;
	if(Bluetooth_Rxcount==0)
		return 0;
	else
	{
		memcpy(rx_data,Bluetooth_Rxbuffer,read_length);
		Bluetooth_Rxcount=0;
		Bluetooth_Receive_flag=0;
		return read_length;
	}

}
//返回 0：从机  1:主机  0xff:获取失败
int Bluetooth_get_role(uint8_t *ismaster)
{
	uint8_t cmd[]="AT+ROLE?\r\n";
	uint8_t retry=0x0f;
	uint8_t temp,t;
	Bluetooth_Receive_flag=0;
	Bluetooth_Rxcount=0;
	temp=0xFF;
	HC_KEY(Bit_SET);
	vTaskDelay(10);
	USART_Send(Bluetooth_USART,cmd,10);
	HC_KEY(Bit_RESET);
	for(t=0;t<20;t++)
	{
		if(Bluetooth_Receive_flag)break;
		vTaskDelay(10);
	}
	if(Bluetooth_Receive_flag)
	{
		if(Bluetooth_Rxbuffer[0]=='+')
		{
			temp=Bluetooth_Rxbuffer[6]-'0';
		}
		Bluetooth_Receive_flag=0;
		Bluetooth_Rxcount=0;

	}
	return temp;
}
int Bluetooth_set_baud(uint32_t baud)
{
	uint8_t cmd[25],t,temp;
	USART_InitTypeDef	USART_InitStructure;
	uint16_t BaudRateInteger=0,BaudRateNumerator=0,BaudRateDenominator=0;
	float decimal;
	sprintf(cmd,"AT+UART%d,0,0\r\n",baud);
	HC_KEY(Bit_SET);
	vTaskDelay(10);
	USART_Send(Bluetooth_USART,cmd,20);
	HC_KEY(Bit_RESET);

	for(t=0;t<20;t++)
	{
		if(Bluetooth_Receive_flag==1)break;
		vTaskDelay(5);
	}
	if(Bluetooth_Receive_flag)
	{
		Bluetooth_Receive_flag=0;
		Bluetooth_Rxcount=0;
	}


	USART_Struct_Init(&USART_InitStructure);
	BaudRateInteger = (1000000/baud);
	if((BaudRateInteger*baud)>985000)
	{
		BaudRateNumerator = 0;
	}
	else
	{
		decimal = (float)1000000/(BaudRateInteger*baud)-1;
		BaudRateNumerator = (uint16_t)(decimal*1000);
		if(BaudRateNumerator < 64)
			BaudRateNumerator = 0;
		else
			BaudRateNumerator += 32;
			BaudRateNumerator >>= 6;
		if(BaudRateNumerator > 0x0f) BaudRateNumerator=0x0f;
		BaudRateDenominator = 0x0f;
	}
	USART_InitStructure.m_BaudRateInteger = BaudRateInteger;
	USART_InitStructure.m_BaudRateDenominator = BaudRateDenominator;
	USART_InitStructure.m_BaudRateNumerator = BaudRateNumerator;
	USART_InitStructure.m_BaudRateBRCKS = USART_CLK_HFCLK;
	USART_InitStructure.m_Mode = USART_MODE_FULLDUPLEXASY;
	USART_InitStructure.m_WordLength = USART_WORDLENGTH_8B;
	USART_InitStructure.m_StopBits = USART_STOPBITS_1;
	USART_InitStructure.m_TransferDir = USART_DIRECTION_FULL_DUPLEX;

	USART_Reset(USART4_SFR);
	USART_Configuration(USART4_SFR,&USART_InitStructure);
    USART_Passageway_Select_Config(USART4_SFR,USART_U7816R_PASSAGEWAY_TX0);//UASRT4选择TX0通道
	USART_Clear_Transmit_BUFR_INT_Flag(USART4_SFR);                //USART4发送BUF清零
	USART_Cmd(USART4_SFR,ENABLE);
	USART_Transmit_Data_Enable(USART4_SFR,ENABLE);
	USART_Receive_Data_Enable(USART4_SFR,ENABLE);
	USART_ReceiveInt_config(USART4_SFR,INT_USART4);
	return 0;
}

void Clear_Bluetooth_Buffer(void)
{
 uint8_t i;
    for(i=0;i<Bluetooth_Rxbuffer_MAX;i++)
    {
    	Bluetooth_Rxbuffer[i]=0;
    }
    Bluetooth_Rxcount=0;
    Bluetooth_Receive_flag=0;
}


