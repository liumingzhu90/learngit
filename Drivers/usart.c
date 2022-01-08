/**
  ******************************************************************************
  * 文件名  Usart.c
  * 作  者  ChipON_AE/FAE_Group
  * 日  期  2019-10-19
  * 描  述  该文件提供了串口例程相关配置函数，包括
  *          + 串口发送
  *          + 串口异步配置
  *          + 串口同步配置
  *          + 串口接收中断使能
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
  * 描述  串口初始化,默认使用外设高速时钟16M。
  * 输入  串口波特率,当前波特小于等于1M
  * 返回  无。
*/
void User_USART_Init(uint32_t baud)
{
	GPIO_Write_Mode_Bits(GPIOA_SFR ,GPIO_PIN_MASK_15, GPIO_MODE_RMP);          //重映射IO口功能模式
	GPIO_Write_Mode_Bits(GPIOE_SFR ,GPIO_PIN_MASK_0, GPIO_MODE_RMP);          //重映射IO口功能模式
	GPIO_Pin_RMP_Config (GPIOA_SFR ,GPIO_Pin_Num_15, GPIO_RMP_AF5_USART1);	  //重映射为USART1
	GPIO_Pin_RMP_Config (GPIOE_SFR ,GPIO_Pin_Num_0, GPIO_RMP_AF5_USART1);     //重映射为USART1
	GPIO_Pin_Lock_Config (GPIOA_SFR ,GPIO_PIN_MASK_15, TRUE);                  //配置锁存
	GPIO_Pin_Lock_Config (GPIOE_SFR ,GPIO_PIN_MASK_0, TRUE);                  //配置锁存
}

void User_USART4_Init(uint32_t baud)
{
	GPIO_Write_Mode_Bits(GPIOD_SFR ,GPIO_PIN_MASK_3, GPIO_MODE_RMP);          //重映射IO口功能模式
	GPIO_Write_Mode_Bits(GPIOD_SFR ,GPIO_PIN_MASK_4, GPIO_MODE_RMP);          //重映射IO口功能模式
	GPIO_Pin_RMP_Config (GPIOD_SFR ,GPIO_Pin_Num_3, GPIO_RMP_AF6_USART4);	  //重映射为USART4
	GPIO_Pin_RMP_Config (GPIOD_SFR ,GPIO_Pin_Num_4, GPIO_RMP_AF6_USART4);     //重映射为USART4
	GPIO_Pin_Lock_Config (GPIOD_SFR ,GPIO_PIN_MASK_3, TRUE);                  //配置锁存
	GPIO_Pin_Lock_Config (GPIOD_SFR ,GPIO_PIN_MASK_4, TRUE);                  //配置锁存
}

void User_USART0_Init(uint32_t baud)
{
	GPIO_Write_Mode_Bits(GPIOA_SFR ,GPIO_PIN_MASK_0, GPIO_MODE_RMP);          //重映射IO口功能模式
	GPIO_Write_Mode_Bits(GPIOA_SFR ,GPIO_PIN_MASK_1, GPIO_MODE_RMP);          //重映射IO口功能模式
	GPIO_Pin_RMP_Config (GPIOA_SFR ,GPIO_Pin_Num_0, GPIO_RMP_AF5_USART0);	  //重映射为USART1
	GPIO_Pin_RMP_Config (GPIOA_SFR ,GPIO_Pin_Num_1, GPIO_RMP_AF5_USART0);     //重映射为USART1
	GPIO_Pin_Lock_Config (GPIOA_SFR ,GPIO_PIN_MASK_0, TRUE);                  //配置锁存
	GPIO_Pin_Lock_Config (GPIOA_SFR ,GPIO_PIN_MASK_1, TRUE);                  //配置锁存
}

void User_USART2_Init(uint32_t baud)
{
	GPIO_Write_Mode_Bits(GPIOA_SFR ,GPIO_PIN_MASK_8, GPIO_MODE_RMP);          //重映射IO口功能模式
	GPIO_Write_Mode_Bits(GPIOA_SFR ,GPIO_PIN_MASK_9, GPIO_MODE_RMP);          //重映射IO口功能模式
	GPIO_Pin_RMP_Config (GPIOA_SFR ,GPIO_Pin_Num_8, GPIO_RMP_AF5_USART2);	  //重映射为USART1
	GPIO_Pin_RMP_Config (GPIOA_SFR ,GPIO_Pin_Num_9, GPIO_RMP_AF5_USART2);     //重映射为USART1
	GPIO_Pin_Lock_Config (GPIOA_SFR ,GPIO_PIN_MASK_8, TRUE);                  //配置锁存
	GPIO_Pin_Lock_Config (GPIOA_SFR ,GPIO_PIN_MASK_9, TRUE);                  //配置锁存
}


/**
  * 描述   串口发送
  * 输入   USARTx:   指向USART内存结构的指针，取值为USART0_SFR~USART8_SFR
  *      Databuf：   指向发送数据的指针
  *      length：      发送的长度
  * 返回   无
  */
#ifdef KM_MASTER

void xGPIO_USART0()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*配置USART引脚类型为复用模式，开启对应端口时钟*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_0|GPIO_PIN_MASK_1;
	GPIO_Configuration(GPIOA_SFR,&GPIO_InitStructure);
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOACLKEN,TRUE);
	/*配置引脚映射功能为USART模式*/
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_0,GPIO_RMP_AF5_USART0);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_1,GPIO_RMP_AF5_USART0);
}

void xGPIO_USART1()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*配置USART引脚类型为复用模式，开启对应端口时钟*/
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
	/*配置引脚映射功能为USART模式*/
	GPIO_Pin_RMP_Config(GPIOE_SFR,GPIO_Pin_Num_0,GPIO_RMP_AF5_USART1);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_15,GPIO_RMP_AF5_USART1);
}

void xGPIO_USART2()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*配置USART引脚类型为复用模式，开启对应端口时钟*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_8|GPIO_PIN_MASK_9;
	GPIO_Configuration(GPIOA_SFR,&GPIO_InitStructure);
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOACLKEN,TRUE);
	/*配置引脚映射功能为USART模式*/
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_8,GPIO_RMP_AF5_USART2);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_9,GPIO_RMP_AF5_USART2);
}

void xGPIO_USART3()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*配置USART引脚类型为复用模式，开启对应端口时钟*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_2|GPIO_PIN_MASK_3;
	GPIO_Configuration(GPIOB_SFR,&GPIO_InitStructure);
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOBCLKEN,TRUE);
	/*配置引脚映射功能为USART模式*/
	GPIO_Pin_RMP_Config(GPIOB_SFR,GPIO_Pin_Num_2,GPIO_RMP_AF6_USART3);
	GPIO_Pin_RMP_Config(GPIOB_SFR,GPIO_Pin_Num_3,GPIO_RMP_AF6_USART3);
}

void xGPIO_USART4()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*配置USART引脚类型为复用模式，开启对应端口时钟*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_3|GPIO_PIN_MASK_4;
	GPIO_Configuration(GPIOD_SFR,&GPIO_InitStructure);
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIODCLKEN,TRUE);
	/*配置引脚映射功能为USART模式*/
	GPIO_Pin_RMP_Config(GPIOD_SFR,GPIO_Pin_Num_3,GPIO_RMP_AF6_USART4);
	GPIO_Pin_RMP_Config(GPIOD_SFR,GPIO_Pin_Num_4,GPIO_RMP_AF6_USART4);
}

void xGPIO_USART5()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*配置USART引脚类型为复用模式，开启对应端口时钟*/
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
	/*配置引脚映射功能为USART模式*/
	GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_1,GPIO_RMP_AF6_USART5);
	GPIO_Pin_RMP_Config(GPIOG_SFR,GPIO_Pin_Num_5,GPIO_RMP_AF6_USART5);
}

void xGPIO_USART6()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*配置USART引脚类型为复用模式，开启对应端口时钟*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_4|GPIO_PIN_MASK_5;
	GPIO_Configuration(GPIOB_SFR,&GPIO_InitStructure);
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOBCLKEN,TRUE);
	/*配置引脚映射功能为USART模式*/
	GPIO_Pin_RMP_Config(GPIOB_SFR,GPIO_Pin_Num_4,GPIO_RMP_AF6_USART6);
	GPIO_Pin_RMP_Config(GPIOB_SFR,GPIO_Pin_Num_5,GPIO_RMP_AF6_USART6);
}

void xGPIO_USART7()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*配置USART引脚类型为复用模式，开启对应端口时钟*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_11|GPIO_PIN_MASK_12;
	GPIO_Configuration(GPIOB_SFR,&GPIO_InitStructure);
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOBCLKEN,TRUE);
	/*配置引脚映射功能为USART模式*/
	GPIO_Pin_RMP_Config(GPIOB_SFR,GPIO_Pin_Num_11,GPIO_RMP_AF6_USART7);
	GPIO_Pin_RMP_Config(GPIOB_SFR,GPIO_Pin_Num_12,GPIO_RMP_AF6_USART7);
}

#endif


void USART_Send(USART_SFRmap* USARTx, uint8_t* Databuf, uint32_t length)
{
	uint32_t i;
	for(i=0;i<length;i++)
	{
		//串口发送
		USART_SendData(USARTx,Databuf[i]);
		//发送完成标志
		while(!USART_Get_Transmitter_Empty_Flag(USARTx));
	}
}

/**
  * 描述  串口异步全双工配置(默认8bit收发使能  全双工 9600)
  * 输入   指向USART内存结构的指针，取值为USART0_SFR~USART8_SFR
  * 返回   无
  */
void USART_Async_config(USART_SFRmap *USARTx,uint32_t baud)
{
	USART_InitTypeDef USART_InitStructure;

	USART_Struct_Init(&USART_InitStructure);
    USART_InitStructure.m_Mode=USART_MODE_FULLDUPLEXASY;                        //全双工
    USART_InitStructure.m_TransferDir=USART_DIRECTION_FULL_DUPLEX;              //传输方向
    USART_InitStructure.m_WordLength=USART_WORDLENGTH_8B;                       //8位数据
    USART_InitStructure.m_StopBits=USART_STOPBITS_1;                            //1位停止位
    USART_InitStructure.m_BaudRateBRCKS=USART_CLK_HFCLK;                        //内部高频时钟作为 USART波特率发生器时钟

    /* 波特率 =Fck/(16*z（1+x/y)) 外设时钟内部高频16M*/
    //4800    z:208    x:0    y:0
    //9600    z:104    x:0    y:0
    //19200   z:52     x:0    y:0
    //115200  z:8      x:1    y:13
    //波特率115200
    switch(baud)
    {
    case 4800:
        USART_InitStructure.m_BaudRateInteger=208;         //USART波特率整数部分z，取值为0~65535
        USART_InitStructure.m_BaudRateNumerator=0;         //USART波特率小数分子部分x，取值为0~0xF
        USART_InitStructure.m_BaudRateDenominator=0;       //USART波特率小数分母部分y，取值为0~0xF
        break;
    case 9600:
        USART_InitStructure.m_BaudRateInteger=104;         //USART波特率整数部分z，取值为0~65535
        USART_InitStructure.m_BaudRateNumerator=0;         //USART波特率小数分子部分x，取值为0~0xF
        USART_InitStructure.m_BaudRateDenominator=0;       //USART波特率小数分母部分y，取值为0~0xF
        break;
    case 19200:
        USART_InitStructure.m_BaudRateInteger=52;         //USART波特率整数部分z，取值为0~65535
        USART_InitStructure.m_BaudRateNumerator=0;         //USART波特率小数分子部分x，取值为0~0xF
        USART_InitStructure.m_BaudRateDenominator=0;       //USART波特率小数分母部分y，取值为0~0xF
        break;
    case 115200:
    	USART_InitStructure.m_BaudRateInteger=8;         //USART波特率整数部分z，取值为0~65535
    	USART_InitStructure.m_BaudRateNumerator=1;         //USART波特率小数分子部分x，取值为0~0xF
    	USART_InitStructure.m_BaudRateDenominator=13;       //USART波特率小数分母部分y，取值为0~0xF
    	break;
    default:
    	USART_InitStructure.m_BaudRateInteger=8;         //USART波特率整数部分z，取值为0~65535
    	USART_InitStructure.m_BaudRateNumerator=1;         //USART波特率小数分子部分x，取值为0~0xF
    	USART_InitStructure.m_BaudRateDenominator=13;       //USART波特率小数分母部分y，取值为0~0xF
    	break;
    }
	USART_Reset(USARTx);                                       //USARTx复位
	USART_Configuration(USARTx,&USART_InitStructure);          //USARTx配置
    USART_Passageway_Select_Config(USARTx,USART_U7816R_PASSAGEWAY_TX0);//UASRTx选择TX0通道
	USART_Clear_Transmit_BUFR_INT_Flag(USARTx);                //USARTx发送BUF清零
	USART_RESHD_Enable (USARTx, TRUE);						   //使能RESHD位
	USART_Cmd(USARTx,TRUE);                                    //USARTx使能
}

void USART_Async_config_tx1(USART_SFRmap *USARTx,uint32_t baud)
{
	USART_InitTypeDef USART_InitStructure;

	USART_Struct_Init(&USART_InitStructure);
    USART_InitStructure.m_Mode=USART_MODE_FULLDUPLEXASY;                        //全双工
    USART_InitStructure.m_TransferDir=USART_DIRECTION_FULL_DUPLEX;              //传输方向
    USART_InitStructure.m_WordLength=USART_WORDLENGTH_8B;                       //8位数据
    USART_InitStructure.m_StopBits=USART_STOPBITS_1;                            //1位停止位
    USART_InitStructure.m_BaudRateBRCKS=USART_CLK_HFCLK;                        //内部高频时钟作为 USART波特率发生器时钟

    /* 波特率 =Fck/(16*z（1+x/y)) 外设时钟内部高频16M*/
    //4800    z:208    x:0    y:0
    //9600    z:104    x:0    y:0
    //19200   z:52     x:0    y:0
    //115200  z:8      x:1    y:13
    //波特率115200
    switch(baud)
    {
    case 4800:
        USART_InitStructure.m_BaudRateInteger=208;         //USART波特率整数部分z，取值为0~65535
        USART_InitStructure.m_BaudRateNumerator=0;         //USART波特率小数分子部分x，取值为0~0xF
        USART_InitStructure.m_BaudRateDenominator=0;       //USART波特率小数分母部分y，取值为0~0xF
        break;
    case 9600:
        USART_InitStructure.m_BaudRateInteger=104;         //USART波特率整数部分z，取值为0~65535
        USART_InitStructure.m_BaudRateNumerator=0;         //USART波特率小数分子部分x，取值为0~0xF
        USART_InitStructure.m_BaudRateDenominator=0;       //USART波特率小数分母部分y，取值为0~0xF
        break;
    case 19200:
        USART_InitStructure.m_BaudRateInteger=52;         //USART波特率整数部分z，取值为0~65535
        USART_InitStructure.m_BaudRateNumerator=0;         //USART波特率小数分子部分x，取值为0~0xF
        USART_InitStructure.m_BaudRateDenominator=0;       //USART波特率小数分母部分y，取值为0~0xF
        break;
    case 115200:
    	USART_InitStructure.m_BaudRateInteger=8;         //USART波特率整数部分z，取值为0~65535
    	USART_InitStructure.m_BaudRateNumerator=1;         //USART波特率小数分子部分x，取值为0~0xF
    	USART_InitStructure.m_BaudRateDenominator=13;       //USART波特率小数分母部分y，取值为0~0xF
    	break;
    default:
    	USART_InitStructure.m_BaudRateInteger=8;         //USART波特率整数部分z，取值为0~65535
    	USART_InitStructure.m_BaudRateNumerator=1;         //USART波特率小数分子部分x，取值为0~0xF
    	USART_InitStructure.m_BaudRateDenominator=13;       //USART波特率小数分母部分y，取值为0~0xF
    	break;
    }
	USART_Reset(USARTx);                                       //USARTx复位
	USART_Configuration(USARTx,&USART_InitStructure);          //USARTx配置
    USART_Passageway_Select_Config(USARTx,USART_U7816R_PASSAGEWAY_TX1);//UASRTx选择TX0通道
	USART_Clear_Transmit_BUFR_INT_Flag(USARTx);                //USARTx发送BUF清零
	USART_RESHD_Enable (USARTx, TRUE);						   //使能RESHD位
	USART_Cmd(USARTx,TRUE);                                    //USARTx使能
}

/**
  * 描述   串口半双工同步配置(默认主模式，9bit发送，9600波特率)
  * 输入   指向USART内存结构的指针，取值为USART0_SFR~USART8_SFR
  * 返回   无
  */
void USART_Sync_config(USART_SFRmap* USARTx)
{
	USART_InitTypeDef USART_InitStructure;

	USART_Struct_Init(&USART_InitStructure);
    USART_InitStructure.m_Mode=USART_MODE_HALFDUPLEXSYN;                        //半双工
    USART_InitStructure.m_HalfDuplexClkSource=USART_MASTER_CLOCKSOURCE_INTER;   //主模式
    USART_InitStructure.m_TransferDir=USART_DIRECTION_TRANSMIT;                 //传输方向"发送"
    USART_InitStructure.m_WordLength=USART_WORDLENGTH_9B;                       //9位数据
    USART_InitStructure.m_Parity=USART_PARITY_ODD;                              //奇校验
    USART_InitStructure.m_BaudRateBRCKS=USART_CLK_HFCLK;                        //内部高频时钟作为 USART波特率发生器时钟

    /* 波特率 =Fck/(16*z（1+x/y)) 外设时钟内部高频16M*/
    //4800    z:208    x:0    y:0
    //9600    z:104    x:0    y:0
    //19200   z:52     x:0    y:0
    //115200  z:8      x:1    y:13
    //波特率9600
    USART_InitStructure.m_BaudRateInteger=104;         //USART波特率整数部分z，取值为0~65535
    USART_InitStructure.m_BaudRateNumerator=0;         //USART波特率小数分子部分x，取值为0~0xF
    USART_InitStructure.m_BaudRateDenominator=0;       //USART波特率小数分母部分y，取值为0~0xF

	USART_Reset(USARTx);                                       //USARTx复位
	USART_Configuration(USARTx,&USART_InitStructure);          //USARTx配置
	USART_Passageway_Select_Config(USARTx,USART_U7816R_PASSAGEWAY_TX0);//UASRTx选择TX0通道
	USART_Clear_Transmit_BUFR_INT_Flag(USARTx);                //USARTx发送BUF清零
	USART_RESHD_Enable (USARTx, TRUE);						   //使能RESHD位
	USART_Cmd(USARTx,TRUE);                                    //USARTx使能
}


/**
  * 描述   串口接收中断配置
  * 输入   USARTx:指向USART内存结构的指针，取值为USART0_SFR~USART8_SFR
  *      Peripheral:外设或内核中断向量编号，取值范围为：
  *                 枚举类型InterruptIndex中的外设中断向量编号
  * 返回   无
  */
void USART_ReceiveInt_config(USART_SFRmap *USARTx,InterruptIndex Peripheral)
{

	USART_RDR_INT_Enable(USARTx,TRUE);
	INT_Interrupt_Enable(Peripheral,TRUE);
	USART_ReceiveData(USARTx);//清接收标志位
	INT_All_Enable(TRUE);
}

/**
  * 描述   串口发闪送换行符
  * 输入   指向USART内存结构的指针，取值为USART0_SFR~USART8_SFR
  * 返回   无
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
