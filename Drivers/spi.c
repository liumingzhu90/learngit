/******************** (C) COPYRIGHT  源地工作室 ********************************
 * 文件名  ：spi.c
 * 描述    ：SPI模块的初始化代码，配置成主机模式
 * 作者    ：lmz
 * 版本更新: 2021-08-30
 * 硬件连接  :PC12-/CS   PC9--CLK   PC10--MISO  PC11--MOSI
 * 调试方式：KungFu KF32DP2
**********************************************************************************/

#include "spi.h"
#include "system_init.h"
#include "spi.h"

/**
  * 描述   GPIO_SPI2()引脚重映射
  * 输入   无
  * 返回   无
  * PC12=SS,PC9=SCK,PC10=SDI,PC11=SDO
  */
void GPIO_SPI2()
{
	/*SPI2_IO配置*/
	GPIO_Write_Mode_Bits(GPIOC_SFR,GPIO_PIN_MASK_9  \
			                      |GPIO_PIN_MASK_10  \
			                      |GPIO_PIN_MASK_11 ,GPIO_MODE_RMP);	//重映射IO口功能模式

	GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_9,GPIO_RMP_AF7_SPI2);	//重映射为SPI2
	GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_10,GPIO_RMP_AF7_SPI2);	//重映射为SPI2
	GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_11,GPIO_RMP_AF7_SPI2);	//重映射为SPI2

	GPIO_Write_Mode_Bits(GPIOC_SFR,GPIO_PIN_MASK_12,GPIO_MODE_OUT);		// cs
	// SET read and write SPEED
	GPIO_Speed_Config(GPIOC_SFR,GPIO_PIN_MASK_12,GPIO_LOW_SPEED);		// set speed 10MHz
	GPIO_Speed_Config(GPIOC_SFR,GPIO_Pin_Num_9,GPIO_HIGH_SPEED);		// set speed 50MHz
	GPIO_Speed_Config(GPIOC_SFR,GPIO_Pin_Num_10,GPIO_HIGH_SPEED);		// set speed 50MHz
	GPIO_Speed_Config(GPIOC_SFR,GPIO_Pin_Num_11,GPIO_HIGH_SPEED);		// set speed 50MHz
}
/**
  * 描述   SPI2_Init() SPI2 init
  * 输入   无
  * 返回   无
  */
void SPI2_Init(void)
{
	/* init gpio */
	SPI_InitTypeDef newStruct_SPI;
	SPI_SFRmap* SPIx = SPI_COM;

	/*SPI配置*/
//#if SPI_MASTER
	newStruct_SPI.m_Mode 		= SPI_MODE_MASTER_CLKDIV4;    	//主模式主时钟4分频 30M
//#else
//	newStruct_SPI.m_Mode		= SPI_MODE_SLAVE;              	//从模式
//#endif
 	newStruct_SPI.m_Clock 		= SPI_CLK_SCLK;					//SPI主频时钟
	newStruct_SPI.m_FirstBit 	= SPI_FIRSTBIT_MSB;				//MSB
	newStruct_SPI.m_CKP 		= SPI_CKP_HIGH;              	//SCK空闲为高 //串行同步时钟的空闲状态为低电平
	newStruct_SPI.m_CKE			= SPI_CKE_1EDGE;            	//第一个时钟开始发送数据 //设置SPI在SCLK的前一边沿采样，后一边沿输出。
	newStruct_SPI.m_DataSize	= SPI_DATASIZE_8BITS;       	//8bit
	// 0x12B		20us	50MHz	读时最大频率
	// 0x176		25us	40MHz
	// 0x257		40us	25MHz
	// 0x2ED		50us	20MHz
	// 0x5DB		100us	10MHz	默认
	// 0xBB7		200us	5MHz
	newStruct_SPI.m_BaudRate 	= 0x12B;                    	//Fck_spi=Fck/2(m_BaudRate+1)=10us 20us
    SPI_Reset(SPIx);											//复位模块
    SPI_Configuration(SPIx, &newStruct_SPI);					//写入结构体配置
	SPI_Cmd(SPIx,TRUE);											//使能
}

/**
*函数名称:SPI2_ReadWriteByte
*功能概要:SPI12读写一个字节
*参数名称:TxData:要写入的字节
*函数返回:读取到的字节
*/
//=============================================================================
uint8_t SPI2_ReadWriteByte(uint8_t TxData)
{
    /*等待发送buffer空闲 */
    while (SPI_Get_Transmit_Buf_Flag(SPI_COM) == SET);
    /*发送 byte */
	SPI_I2S_SendData8(SPI_COM, TxData);						//MOSI
	/*等待发送完毕*/
	while (SPI_Get_BUSY_Flag(SPI_COM)==SET);
	/*等待接收完毕*/
//    while (SPI_Get_Receive_Buf_Flag(SPI_COM) == RESET);
    /*返回接收数据*/
	return SPI_I2S_ReceiveData(SPI_COM);					//MISO
}






