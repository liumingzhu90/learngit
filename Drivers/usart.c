/**
  ******************************************************************************
  * �ļ���  Usart.c
  * ��  ��  ChipON_AE/FAE_Group
  * ��  ��  2019-10-19
  * ��  ��  ���ļ��ṩ�˴�������������ú���������
  *          + ���ڷ���
  *          + �����첽����
  *          + ����ͬ������
  *          + ���ڽ����ж�ʹ��
  ******************************************************************************/

#include "kf32a_basic_usart.h"
#include "kf32a_basic_rst.h"
#include "kf32a_basic_pclk.h"
#include "usart.h"
#include "flash.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "common.h"
//#include <ConfigParameter.h>
#include <get_parameter.h>
#include <set_parameter.h>

uint8_t User_Rxbuffer[1024];
uint8_t User_Rxbuf[1024]={'1',0xff};
uint8_t User_RxString[] = "USTART\tSETPARAMETER\tCAN0RATE\t500K\tUSTOP";
uint8_t User_Rx2String[30][15];
uint32_t User_Rxcount=0;
uint32_t User_Rxc=0;
uint8_t User_Receive_flag=0;

uint8_t Uart2_Rxbuffer[1024];
uint32_t Uart2_Rxcount=0;
//uint8_t sencond_ret;
///uint8_t bak_second_ret[1];
//uint32_t ret;
/**
  * ����  ���ڳ�ʼ��,Ĭ��ʹ���������ʱ��16M��
  * ����  ���ڲ�����,��ǰ����С�ڵ���1M
  * ����  �ޡ�
*/
void User_USART_Init(uint32_t baud)
{
	GPIO_Write_Mode_Bits(GPIOA_SFR ,GPIO_PIN_MASK_15, GPIO_MODE_RMP);          //��ӳ��IO�ڹ���ģʽ
	GPIO_Write_Mode_Bits(GPIOE_SFR ,GPIO_PIN_MASK_0, GPIO_MODE_RMP);          //��ӳ��IO�ڹ���ģʽ
	GPIO_Pin_RMP_Config (GPIOA_SFR ,GPIO_Pin_Num_15, GPIO_RMP_AF5_USART1);	  //��ӳ��ΪUSART1
	GPIO_Pin_RMP_Config (GPIOE_SFR ,GPIO_Pin_Num_0, GPIO_RMP_AF5_USART1);     //��ӳ��ΪUSART1
	GPIO_Pin_Lock_Config (GPIOA_SFR ,GPIO_PIN_MASK_15, TRUE);                  //��������
	GPIO_Pin_Lock_Config (GPIOE_SFR ,GPIO_PIN_MASK_0, TRUE);                  //��������
}

void User_USART4_Init(uint32_t baud)
{
	GPIO_Write_Mode_Bits(GPIOD_SFR ,GPIO_PIN_MASK_3, GPIO_MODE_RMP);          //��ӳ��IO�ڹ���ģʽ
	GPIO_Write_Mode_Bits(GPIOD_SFR ,GPIO_PIN_MASK_4, GPIO_MODE_RMP);          //��ӳ��IO�ڹ���ģʽ
	GPIO_Pin_RMP_Config (GPIOD_SFR ,GPIO_Pin_Num_3, GPIO_RMP_AF6_USART4);	  //��ӳ��ΪUSART4
	GPIO_Pin_RMP_Config (GPIOD_SFR ,GPIO_Pin_Num_4, GPIO_RMP_AF6_USART4);     //��ӳ��ΪUSART4
	GPIO_Pin_Lock_Config (GPIOD_SFR ,GPIO_PIN_MASK_3, TRUE);                  //��������
	GPIO_Pin_Lock_Config (GPIOD_SFR ,GPIO_PIN_MASK_4, TRUE);                  //��������
}

void User_USART0_Init(uint32_t baud)
{
	GPIO_Write_Mode_Bits(GPIOA_SFR ,GPIO_PIN_MASK_0, GPIO_MODE_RMP);          //��ӳ��IO�ڹ���ģʽ
	GPIO_Write_Mode_Bits(GPIOA_SFR ,GPIO_PIN_MASK_1, GPIO_MODE_RMP);          //��ӳ��IO�ڹ���ģʽ
	GPIO_Pin_RMP_Config (GPIOA_SFR ,GPIO_Pin_Num_0, GPIO_RMP_AF5_USART0);	  //��ӳ��ΪUSART1
	GPIO_Pin_RMP_Config (GPIOA_SFR ,GPIO_Pin_Num_1, GPIO_RMP_AF5_USART0);     //��ӳ��ΪUSART1
	GPIO_Pin_Lock_Config (GPIOA_SFR ,GPIO_PIN_MASK_0, TRUE);                  //��������
	GPIO_Pin_Lock_Config (GPIOA_SFR ,GPIO_PIN_MASK_1, TRUE);                  //��������
}

void User_USART2_Init(uint32_t baud)
{
	GPIO_Write_Mode_Bits(GPIOA_SFR ,GPIO_PIN_MASK_8, GPIO_MODE_RMP);          //��ӳ��IO�ڹ���ģʽ
	GPIO_Write_Mode_Bits(GPIOA_SFR ,GPIO_PIN_MASK_9, GPIO_MODE_RMP);          //��ӳ��IO�ڹ���ģʽ
	GPIO_Pin_RMP_Config (GPIOA_SFR ,GPIO_Pin_Num_8, GPIO_RMP_AF5_USART2);	  //��ӳ��ΪUSART1
	GPIO_Pin_RMP_Config (GPIOA_SFR ,GPIO_Pin_Num_9, GPIO_RMP_AF5_USART2);     //��ӳ��ΪUSART1
	GPIO_Pin_Lock_Config (GPIOA_SFR ,GPIO_PIN_MASK_8, TRUE);                  //��������
	GPIO_Pin_Lock_Config (GPIOA_SFR ,GPIO_PIN_MASK_9, TRUE);                  //��������
}


/**
  * ����   ���ڷ���
  * ����   USARTx:   ָ��USART�ڴ�ṹ��ָ�룬ȡֵΪUSART0_SFR~USART8_SFR
  *      Databuf��   ָ�������ݵ�ָ��
  *      length��      ���͵ĳ���
  * ����   ��
  */
#ifdef KM_MASTER

void xGPIO_USART0()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*����USART��������Ϊ����ģʽ��������Ӧ�˿�ʱ��*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_0|GPIO_PIN_MASK_1;
	GPIO_Configuration(GPIOA_SFR,&GPIO_InitStructure);
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOACLKEN,TRUE);
	/*��������ӳ�书��ΪUSARTģʽ*/
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_0,GPIO_RMP_AF5_USART0);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_1,GPIO_RMP_AF5_USART0);
}

void xGPIO_USART1()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*����USART��������Ϊ����ģʽ��������Ӧ�˿�ʱ��*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_0;
	GPIO_Configuration(GPIOE_SFR,&GPIO_InitStructure);
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_15;
	GPIO_Configuration(GPIOA_SFR,&GPIO_InitStructure);
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOACLKEN,TRUE);
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOECLKEN,TRUE);
	/*��������ӳ�书��ΪUSARTģʽ*/
	GPIO_Pin_RMP_Config(GPIOE_SFR,GPIO_Pin_Num_0,GPIO_RMP_AF5_USART1);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_15,GPIO_RMP_AF5_USART1);
}

void xGPIO_USART2()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*����USART��������Ϊ����ģʽ��������Ӧ�˿�ʱ��*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_8|GPIO_PIN_MASK_9;
	GPIO_Configuration(GPIOA_SFR,&GPIO_InitStructure);
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOACLKEN,TRUE);
	/*��������ӳ�书��ΪUSARTģʽ*/
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_8,GPIO_RMP_AF5_USART2);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_9,GPIO_RMP_AF5_USART2);
}

void xGPIO_USART3()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*����USART��������Ϊ����ģʽ��������Ӧ�˿�ʱ��*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_2|GPIO_PIN_MASK_3;
	GPIO_Configuration(GPIOB_SFR,&GPIO_InitStructure);
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOBCLKEN,TRUE);
	/*��������ӳ�书��ΪUSARTģʽ*/
	GPIO_Pin_RMP_Config(GPIOB_SFR,GPIO_Pin_Num_2,GPIO_RMP_AF6_USART3);
	GPIO_Pin_RMP_Config(GPIOB_SFR,GPIO_Pin_Num_3,GPIO_RMP_AF6_USART3);
}

void xGPIO_USART4()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*����USART��������Ϊ����ģʽ��������Ӧ�˿�ʱ��*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_3|GPIO_PIN_MASK_4;
	GPIO_Configuration(GPIOD_SFR,&GPIO_InitStructure);
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIODCLKEN,TRUE);
	/*��������ӳ�书��ΪUSARTģʽ*/
	GPIO_Pin_RMP_Config(GPIOD_SFR,GPIO_Pin_Num_3,GPIO_RMP_AF6_USART4);
	GPIO_Pin_RMP_Config(GPIOD_SFR,GPIO_Pin_Num_4,GPIO_RMP_AF6_USART4);
}

void xGPIO_USART5()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*����USART��������Ϊ����ģʽ��������Ӧ�˿�ʱ��*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_1;
	GPIO_Configuration(GPIOC_SFR,&GPIO_InitStructure);
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_5;
	GPIO_Configuration(GPIOG_SFR,&GPIO_InitStructure);
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOCCLKEN,TRUE);
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOGCLKEN,TRUE);
	/*��������ӳ�书��ΪUSARTģʽ*/
	GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_1,GPIO_RMP_AF6_USART5);
	GPIO_Pin_RMP_Config(GPIOG_SFR,GPIO_Pin_Num_5,GPIO_RMP_AF6_USART5);
}

void xGPIO_USART6()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*����USART��������Ϊ����ģʽ��������Ӧ�˿�ʱ��*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_4|GPIO_PIN_MASK_5;
	GPIO_Configuration(GPIOB_SFR,&GPIO_InitStructure);
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOBCLKEN,TRUE);
	/*��������ӳ�书��ΪUSARTģʽ*/
	GPIO_Pin_RMP_Config(GPIOB_SFR,GPIO_Pin_Num_4,GPIO_RMP_AF6_USART6);
	GPIO_Pin_RMP_Config(GPIOB_SFR,GPIO_Pin_Num_5,GPIO_RMP_AF6_USART6);
}

void xGPIO_USART7()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*����USART��������Ϊ����ģʽ��������Ӧ�˿�ʱ��*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_11|GPIO_PIN_MASK_12;
	GPIO_Configuration(GPIOB_SFR,&GPIO_InitStructure);
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOBCLKEN,TRUE);
	/*��������ӳ�书��ΪUSARTģʽ*/
	GPIO_Pin_RMP_Config(GPIOB_SFR,GPIO_Pin_Num_11,GPIO_RMP_AF6_USART7);
	GPIO_Pin_RMP_Config(GPIOB_SFR,GPIO_Pin_Num_12,GPIO_RMP_AF6_USART7);
}

#endif


void USART_Send(USART_SFRmap* USARTx, uint8_t* Databuf, uint32_t length)
{
	uint32_t i;
	for(i=0;i<length;i++)
	{
		//���ڷ���
		USART_SendData(USARTx,Databuf[i]);
		//������ɱ�־
		while(!USART_Get_Transmitter_Empty_Flag(USARTx));
	}
}

/**
  * ����  �����첽ȫ˫������(Ĭ��8bit�շ�ʹ��  ȫ˫�� 9600)
  * ����   ָ��USART�ڴ�ṹ��ָ�룬ȡֵΪUSART0_SFR~USART8_SFR
  * ����   ��
  */
void USART_Async_config(USART_SFRmap *USARTx,uint32_t baud)
{
	USART_InitTypeDef USART_InitStructure;

	USART_Struct_Init(&USART_InitStructure);
    USART_InitStructure.m_Mode=USART_MODE_FULLDUPLEXASY;                        //ȫ˫��
    USART_InitStructure.m_TransferDir=USART_DIRECTION_FULL_DUPLEX;              //���䷽��
    USART_InitStructure.m_WordLength=USART_WORDLENGTH_8B;                       //8λ����
    USART_InitStructure.m_StopBits=USART_STOPBITS_1;                            //1λֹͣλ
    USART_InitStructure.m_BaudRateBRCKS=USART_CLK_HFCLK;                        //�ڲ���Ƶʱ����Ϊ USART�����ʷ�����ʱ��

    /* ������ =Fck/(16*z��1+x/y)) ����ʱ���ڲ���Ƶ16M*/
    //4800    z:208    x:0    y:0
    //9600    z:104    x:0    y:0
    //19200   z:52     x:0    y:0
    //115200  z:8      x:1    y:13
    //������115200
    switch(baud)
    {
    case 4800:
        USART_InitStructure.m_BaudRateInteger=208;         //USART��������������z��ȡֵΪ0~65535
        USART_InitStructure.m_BaudRateNumerator=0;         //USART������С�����Ӳ���x��ȡֵΪ0~0xF
        USART_InitStructure.m_BaudRateDenominator=0;       //USART������С����ĸ����y��ȡֵΪ0~0xF
        break;
    case 9600:
        USART_InitStructure.m_BaudRateInteger=104;         //USART��������������z��ȡֵΪ0~65535
        USART_InitStructure.m_BaudRateNumerator=0;         //USART������С�����Ӳ���x��ȡֵΪ0~0xF
        USART_InitStructure.m_BaudRateDenominator=0;       //USART������С����ĸ����y��ȡֵΪ0~0xF
        break;
    case 19200:
        USART_InitStructure.m_BaudRateInteger=52;         //USART��������������z��ȡֵΪ0~65535
        USART_InitStructure.m_BaudRateNumerator=0;         //USART������С�����Ӳ���x��ȡֵΪ0~0xF
        USART_InitStructure.m_BaudRateDenominator=0;       //USART������С����ĸ����y��ȡֵΪ0~0xF
        break;
    case 115200:
    	USART_InitStructure.m_BaudRateInteger=8;         //USART��������������z��ȡֵΪ0~65535
    	USART_InitStructure.m_BaudRateNumerator=1;         //USART������С�����Ӳ���x��ȡֵΪ0~0xF
    	USART_InitStructure.m_BaudRateDenominator=13;       //USART������С����ĸ����y��ȡֵΪ0~0xF
    	break;
    default:
    	USART_InitStructure.m_BaudRateInteger=8;         //USART��������������z��ȡֵΪ0~65535
    	USART_InitStructure.m_BaudRateNumerator=1;         //USART������С�����Ӳ���x��ȡֵΪ0~0xF
    	USART_InitStructure.m_BaudRateDenominator=13;       //USART������С����ĸ����y��ȡֵΪ0~0xF
    	break;
    }
	USART_Reset(USARTx);                                       //USARTx��λ
	USART_Configuration(USARTx,&USART_InitStructure);          //USARTx����
    USART_Passageway_Select_Config(USARTx,USART_U7816R_PASSAGEWAY_TX0);//UASRTxѡ��TX0ͨ��
	USART_Clear_Transmit_BUFR_INT_Flag(USARTx);                //USARTx����BUF����
	USART_RESHD_Enable (USARTx, TRUE);						   //ʹ��RESHDλ
	USART_Cmd(USARTx,TRUE);                                    //USARTxʹ��
}

void USART_Async_config_tx1(USART_SFRmap *USARTx,uint32_t baud)
{
	USART_InitTypeDef USART_InitStructure;

	USART_Struct_Init(&USART_InitStructure);
    USART_InitStructure.m_Mode=USART_MODE_FULLDUPLEXASY;                        //ȫ˫��
    USART_InitStructure.m_TransferDir=USART_DIRECTION_FULL_DUPLEX;              //���䷽��
    USART_InitStructure.m_WordLength=USART_WORDLENGTH_8B;                       //8λ����
    USART_InitStructure.m_StopBits=USART_STOPBITS_1;                            //1λֹͣλ
    USART_InitStructure.m_BaudRateBRCKS=USART_CLK_HFCLK;                        //�ڲ���Ƶʱ����Ϊ USART�����ʷ�����ʱ��

    /* ������ =Fck/(16*z��1+x/y)) ����ʱ���ڲ���Ƶ16M*/
    //4800    z:208    x:0    y:0
    //9600    z:104    x:0    y:0
    //19200   z:52     x:0    y:0
    //115200  z:8      x:1    y:13
    //������115200
    switch(baud)
    {
    case 4800:
        USART_InitStructure.m_BaudRateInteger=208;         //USART��������������z��ȡֵΪ0~65535
        USART_InitStructure.m_BaudRateNumerator=0;         //USART������С�����Ӳ���x��ȡֵΪ0~0xF
        USART_InitStructure.m_BaudRateDenominator=0;       //USART������С����ĸ����y��ȡֵΪ0~0xF
        break;
    case 9600:
        USART_InitStructure.m_BaudRateInteger=104;         //USART��������������z��ȡֵΪ0~65535
        USART_InitStructure.m_BaudRateNumerator=0;         //USART������С�����Ӳ���x��ȡֵΪ0~0xF
        USART_InitStructure.m_BaudRateDenominator=0;       //USART������С����ĸ����y��ȡֵΪ0~0xF
        break;
    case 19200:
        USART_InitStructure.m_BaudRateInteger=52;         //USART��������������z��ȡֵΪ0~65535
        USART_InitStructure.m_BaudRateNumerator=0;         //USART������С�����Ӳ���x��ȡֵΪ0~0xF
        USART_InitStructure.m_BaudRateDenominator=0;       //USART������С����ĸ����y��ȡֵΪ0~0xF
        break;
    case 115200:
    	USART_InitStructure.m_BaudRateInteger=8;         //USART��������������z��ȡֵΪ0~65535
    	USART_InitStructure.m_BaudRateNumerator=1;         //USART������С�����Ӳ���x��ȡֵΪ0~0xF
    	USART_InitStructure.m_BaudRateDenominator=13;       //USART������С����ĸ����y��ȡֵΪ0~0xF
    	break;
    default:
    	USART_InitStructure.m_BaudRateInteger=8;         //USART��������������z��ȡֵΪ0~65535
    	USART_InitStructure.m_BaudRateNumerator=1;         //USART������С�����Ӳ���x��ȡֵΪ0~0xF
    	USART_InitStructure.m_BaudRateDenominator=13;       //USART������С����ĸ����y��ȡֵΪ0~0xF
    	break;
    }
	USART_Reset(USARTx);                                       //USARTx��λ
	USART_Configuration(USARTx,&USART_InitStructure);          //USARTx����
    USART_Passageway_Select_Config(USARTx,USART_U7816R_PASSAGEWAY_TX1);//UASRTxѡ��TX0ͨ��
	USART_Clear_Transmit_BUFR_INT_Flag(USARTx);                //USARTx����BUF����
	USART_RESHD_Enable (USARTx, TRUE);						   //ʹ��RESHDλ
	USART_Cmd(USARTx,TRUE);                                    //USARTxʹ��
}

/**
  * ����   ���ڰ�˫��ͬ������(Ĭ����ģʽ��9bit���ͣ�9600������)
  * ����   ָ��USART�ڴ�ṹ��ָ�룬ȡֵΪUSART0_SFR~USART8_SFR
  * ����   ��
  */
void USART_Sync_config(USART_SFRmap* USARTx)
{
	USART_InitTypeDef USART_InitStructure;

	USART_Struct_Init(&USART_InitStructure);
    USART_InitStructure.m_Mode=USART_MODE_HALFDUPLEXSYN;                        //��˫��
    USART_InitStructure.m_HalfDuplexClkSource=USART_MASTER_CLOCKSOURCE_INTER;   //��ģʽ
    USART_InitStructure.m_TransferDir=USART_DIRECTION_TRANSMIT;                 //���䷽��"����"
    USART_InitStructure.m_WordLength=USART_WORDLENGTH_9B;                       //9λ����
    USART_InitStructure.m_Parity=USART_PARITY_ODD;                              //��У��
    USART_InitStructure.m_BaudRateBRCKS=USART_CLK_HFCLK;                        //�ڲ���Ƶʱ����Ϊ USART�����ʷ�����ʱ��

    /* ������ =Fck/(16*z��1+x/y)) ����ʱ���ڲ���Ƶ16M*/
    //4800    z:208    x:0    y:0
    //9600    z:104    x:0    y:0
    //19200   z:52     x:0    y:0
    //115200  z:8      x:1    y:13
    //������9600
    USART_InitStructure.m_BaudRateInteger=104;         //USART��������������z��ȡֵΪ0~65535
    USART_InitStructure.m_BaudRateNumerator=0;         //USART������С�����Ӳ���x��ȡֵΪ0~0xF
    USART_InitStructure.m_BaudRateDenominator=0;       //USART������С����ĸ����y��ȡֵΪ0~0xF

	USART_Reset(USARTx);                                       //USARTx��λ
	USART_Configuration(USARTx,&USART_InitStructure);          //USARTx����
	USART_Passageway_Select_Config(USARTx,USART_U7816R_PASSAGEWAY_TX0);//UASRTxѡ��TX0ͨ��
	USART_Clear_Transmit_BUFR_INT_Flag(USARTx);                //USARTx����BUF����
	USART_RESHD_Enable (USARTx, TRUE);						   //ʹ��RESHDλ
	USART_Cmd(USARTx,TRUE);                                    //USARTxʹ��
}


/**
  * ����   ���ڽ����ж�����
  * ����   USARTx:ָ��USART�ڴ�ṹ��ָ�룬ȡֵΪUSART0_SFR~USART8_SFR
  *      Peripheral:������ں��ж�������ţ�ȡֵ��ΧΪ��
  *                 ö������InterruptIndex�е������ж��������
  * ����   ��
  */
void USART_ReceiveInt_config(USART_SFRmap *USARTx,InterruptIndex Peripheral)
{

	USART_RDR_INT_Enable(USARTx,TRUE);
	INT_Interrupt_Enable(Peripheral,TRUE);
	USART_ReceiveData(USARTx);//����ձ�־λ
	INT_All_Enable(TRUE);
}

/**
  * ����   ���ڷ����ͻ��з�
  * ����   ָ��USART�ڴ�ṹ��ָ�룬ȡֵΪUSART0_SFR~USART8_SFR
  * ����   ��
  */
void USART_line_feed(USART_SFRmap *USARTx)
{
	USART_SendData(USARTx,0x0D);
	while(!USART_Get_Transmitter_Empty_Flag(USARTx));
	USART_SendData(USARTx,0x0A);
	while(!USART_Get_Transmitter_Empty_Flag(USARTx));
}

uint8_t Bluetooth_Commod_Type_Annalyse(uint8_t *data)
{
	uint8_t ret;

	if((data[7] == 'S') &&
			(data[8] == 'E') &&
			(data[9] == 'T'))
		ret = 1;
	else if((data[7] == 'G') &&
			(data[8] == 'E') &&
			(data[9] == 'T'))
		ret = 2;
	else
		ret = 0;
	return ret;
}

//uint32_t vechicle;

void Bluetooth_Data_Annalyse(uint8_t *data)
{

	uint8_t ret;
	ret = Bluetooth_Commod_Type_Annalyse(data);
	if(ret == 1)
	{
		Bluetooth_Set_Commond(data);
	}
	else if(ret == 2)
	{
		Bluetooth_Get_Commond(data);
	}
	else
	{}

}

void GPS_Data_Analysis(uint8_t *data)
{
	stCanCommSta.stWireless.oldSysTime = SystemtimeClock;
	fprintf(BLUETOOTH_STREAM,"%s",data);
}
