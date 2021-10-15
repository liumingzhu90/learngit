/*
 * spi_flash.c
 *
 *  Created on: 2021-8-9
 *      Author: wangzhenbao
 */

#include "system_init.h"
#include "spi_flash.h"

#define W25Q16_RD	(0x03)
#define W25Q16_WREN	(0x06)
#define W25Q16_WRDI	(0x04)
#define W25Q16_RDID	(0x9F)
#define W25Q16_READ_ID	(0x90)
#define W25Q16_SE	(0xD8)
#define W25Q16_PP	(0x02)
#define W25Q16_RDSR	(0x05)
#define W25Q16_CLSR	(0x82)

#define W25Q16_PAGE_SIZE    256                          //一页的大小，256字节
#define W25Q16_SECTOR_SIZE (4*1024)                      //扇区大小，字节
#define W25Q16_BLOCK_SIZE  (16*W25Q16_SECTOR_SIZE)
#define W25Q16_SIZE        (32*W25Q16_BLOCK_SIZE)
#define MOSI_H					 GPIO_Set_Output_Data_Bits(GPIOC_SFR,GPIO_PIN_MASK_11,1)//(GpioDataRegs.GPASET.bit.GPIO16 = 1)
#define MOSI_L					 GPIO_Set_Output_Data_Bits(GPIOC_SFR,GPIO_PIN_MASK_11,0)//(GpioDataRegs.GPACLEAR.bit.GPIO16 = 1)
#define SPICLK_H				 GPIO_Set_Output_Data_Bits(GPIOC_SFR,GPIO_PIN_MASK_9,1)//(GpioDataRegs.GPASET.bit.GPIO17 = 1)
#define SPICLK_L				 GPIO_Set_Output_Data_Bits(GPIOC_SFR,GPIO_PIN_MASK_9,0)//(GpioDataRegs.GPACLEAR.bit.GPIO17 = 1)
#define FLASH_CS_SET			GPIO_Set_Output_Data_Bits(GPIOC_SFR,GPIO_PIN_MASK_12,1)//(GpioDataRegs.AIOSET.bit.AIO12  = 1)
#define FLASH_CS_RESET		    GPIO_Set_Output_Data_Bits(GPIOC_SFR,GPIO_PIN_MASK_12,0)//(GpioDataRegs.AIOCLEAR.bit.AIO12  = 1)
#define READ_MISO GPIO_Read_Input_Data_Bit(GPIOC_SFR,GPIO_PIN_MASK_10);//GpioDataRegs.GPADAT.bit.GPIO6
//#define READ_MISO GPIO_Read_Input_Data_Bit(GPIOD_SFR,GPIO_PIN_MASK_7)
const uint8_t byDUMMY = 0xff;

uint8_t DeviceID=0xff;

/**
  * 描述  GPIOx 输出初始化配置。
  * 输入 : GPIOx: 指向GPIO内存结构的指针，取值为GPIOA_SFR~GPIOH_SFR。
  *       GpioPin: 端口引脚掩码，取值为GPIO_PIN_MASK_0~GPIO_PIN_MASK_15中的一个或多个组合。
  * 返回  无。
  */
void GPIOInit_Output_Config(GPIO_SFRmap* GPIOx,uint16_t GpioPin)
{
	/*初始化复位GPIOx外设，使能GPIOx外设时钟*/
		GPIO_Reset(GPIOx);

	/* 配置 Pxy作为输出模式参数 */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_Struct_Init(&GPIO_InitStructure);
	GPIO_InitStructure.m_Pin = GpioPin;
	GPIO_InitStructure.m_Speed = GPIO_LOW_SPEED;          //初始化 GPIO输出速度
	GPIO_InitStructure.m_Mode = GPIO_MODE_OUT;            //初始化 GPIO方向为输出
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;            //初始化 GPIO是否上拉
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;          //初始化 GPIO是否下拉
	GPIO_Configuration(GPIOx,&GPIO_InitStructure);

	//GPIO_Set_Output_Data_Bits(GPIOx,GpioPin,Bit_SET);	 //先设置为高电平

}

/**
  * 描述  GPIOx 输入初始化配置。
  * 输入 : GPIOx: 指向GPIO内存结构的指针，取值为GPIOA_SFR~GPIOH_SFR。
  *       GpioPin: 端口引脚掩码，取值为GPIO_PIN_MASK_0~GPIO_PIN_MASK_15中的一个或多个组合。
  * 返回  无。
  */
void GPIOInit_Input_Config(GPIO_SFRmap* GPIOx,uint16_t GpioPin)
{
	/*初始化复位GPIOx外设，使能GPIOx外设时钟*/
		GPIO_Reset(GPIOx);

	/* 配置 Pxy作为输入模式 */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_Struct_Init(&GPIO_InitStructure);
	GPIO_InitStructure.m_Pin = GpioPin;
	GPIO_InitStructure.m_Speed = GPIO_LOW_SPEED;                   //初始化 GPIO输出速度
	GPIO_InitStructure.m_Mode = GPIO_MODE_IN;                      //初始化 GPIO方向为输入
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;                     //初始化 GPIO是否上拉 不上拉
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;                   //初始化 GPIO是否下拉 不下拉
	GPIO_Configuration(GPIOx,&GPIO_InitStructure);
}

void GPIO_SPI2(void)
{
	/*SPI1_IO配置*/
	//PB0=SS,PB1=SCK,PB2=SDI,PB3=SDO

	GPIO_Write_Mode_Bits(GPIOC_SFR,GPIO_PIN_MASK_9  \
			                      |GPIO_PIN_MASK_10  \
			                      |GPIO_PIN_MASK_11,GPIO_MODE_RMP);//重映射IO口功能模式
			                      //|GPIO_PIN_MASK_12,GPIO_MODE_RMP);//重映射IO口功能模式
	//GPIO_Write_Mode_Bits(GPIOC_SFR,GPIO_PIN_MASK_12,GPIO_MODE_OUT);
	GPIOInit_Output_Config(GPIOC_SFR,GPIO_PIN_MASK_12);
	GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_9,GPIO_RMP_AF7_SPI2);//重映射为SPI1
	GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_10,GPIO_RMP_AF7_SPI2);//重映射为SPI1
	GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_11,GPIO_RMP_AF7_SPI2);//重映射为SPI1
    //GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_12,GPIO_RMP_AF7_SPI2);//重映射为SPI1

    SPI_Init_Configuration(SPI2_SFR);
    INT_Interrupt_Enable(INT_SPI2,TRUE);//SPI1中断使能
	INT_All_Enable(TRUE);//总中断使能
    /*
	//GPIOInit_Output_Config(GPIOC_SFR,GPIO_PIN_MASK_12);
	//GPIOInit_Output_Config(GPIOC_SFR,GPIO_PIN_MASK_9);
	//GPIO_Set_Output_Data_Bits(GPIOC_SFR,GPIO_PIN_MASK_9,Bit_SET);
	//GPIOInit_Output_Config(GPIOC_SFR,GPIO_PIN_MASK_11);
	//GPIO_Set_Output_Data_Bits(GPIOC_SFR,GPIO_PIN_MASK_11,Bit_SET);
	//GPIOInit_Output_Config(GPIOC_SFR,GPIO_PIN_MASK_12);
	GPIOInit_Input_Config(GPIOC_SFR,GPIO_PIN_MASK_10);
	GPIOInit_Output_Config(GPIOC_SFR,GPIO_PIN_MASK_9|GPIO_PIN_MASK_11|GPIO_PIN_MASK_12);
	GPIO_Set_Output_Data_Bits(GPIOC_SFR,GPIO_PIN_MASK_9|GPIO_PIN_MASK_11|GPIO_PIN_MASK_12,Bit_SET);
*/
}

/**
  * 描述    SPI配置
  * 输入   SPIx：指向SPI内存结构的指针，取值为SPI0_SFR~SPI3_SFR
  * 返回   无
  */
void SPI_Init_Configuration(SPI_SFRmap* SPIx)
{
	SPI_InitTypeDef newStruct_SPI;
	/*SPI配置*/
#if SPI_MASTER
	newStruct_SPI.m_Mode = SPI_MODE_MASTER_CLKDIV4;                   //主模式主时钟4分频 18M
#else
	newStruct_SPI.m_Mode = SPI_MODE_SLAVE;                            //从模式
#endif
 	newStruct_SPI.m_Clock = SPI_CLK_SCLK;							  //SPI主频时钟
	newStruct_SPI.m_FirstBit = SPI_FIRSTBIT_MSB;					  //MSB
	newStruct_SPI.m_CKP = SPI_CKP_LOW;                                //SCK空闲为高
	newStruct_SPI.m_CKE = SPI_CKE_1EDGE;                              //第一个时钟开始发送数据
	newStruct_SPI.m_DataSize = SPI_DATASIZE_8BITS;                    //8bit
	newStruct_SPI.m_BaudRate = 0x59;                                  //Fck_spi=Fck/2(m_BaudRate+1)=10us
    SPI_Reset(SPIx);											      //复位模块
    SPI_Configuration(SPIx, &newStruct_SPI);					      //写入结构体配置
	SPI_Cmd(SPIx,TRUE);											      //使能
}

uint32_t P_MID[2];
uint32_t P_DID;

void W25Q16_readID(void)
{
	uint32_t  i = 0;

	FLASH_CS_RESET;
	SPI_I2S_SendData8(SPI2_SFR,0x90);
	while(SPI_Get_Transmit_Buf_Flag(SPI2_SFR) == SET)
		;
	SPI_I2S_SendData8(SPI2_SFR,0x00);
	while(SPI_Get_Transmit_Buf_Flag(SPI2_SFR) == SET)
		;
	while(SPI_Get_Receive_Buf_Flag(SPI2_SFR) == SET)
	{
		P_MID[i] = SPI_I2S_ReceiveData(SPI2_SFR);
		i ++;
	}
	//while(SPI_Get_Receive_Buf_Flag(SPI2_SFR) == SET)
	//	P_DID = SPI_I2S_ReceiveData(SPI2_SFR);
	FLASH_CS_SET;
}

uint8_t SPI2_Write_and_Read_Byte_Commond(uint8_t TxData)
{
	uint8_t ret;
	FLASH_CS_RESET;
	SPI2_Write_and_Read_Byte(TxData);
	FLASH_CS_SET;

	return ret;
}

uint8_t SPI2_Write_and_Read_Byte(uint8_t TxData)
{
 while(SPI_Get_Transmit_Buf_Flag(SPI2_SFR) == SET);
 SPI_I2S_SendData8(SPI2_SFR,TxData);
 while (SPI_Get_Receive_Buf_Flag(SPI2_SFR) == RESET);
 return SPI_I2S_ReceiveData(SPI2_SFR);
}

void W23Q16_SendCommond(uint8_t commond)		//发送命令
{
	//SPI_I2S_SendData8(SPI_COM,Command);
}

void SPI_Init(void)
{
	FLASH_CS_SET;
	SPICLK_L;
	MOSI_H;
}
static void SPI_Set_MOSI(uint8_t val)
{
	if(val)
		MOSI_H;
	else
		MOSI_L;
}

void DSP28x_usDelay(uint32_t delay)
{
	while(delay --)
		;
}
/*******************************************************************************
* Function Name  : SPIReadWriteByte
* Description    : SPI读写函数
* Input          : val 		传输数据
* Output         : RevData  接收数据
* Return         : None
*******************************************************************************/
static uint8_t SPIReadWriteByte(uint8_t val)
{
	uint16_t i;
	uint8_t  RevData = 0;
	//Uint16 wReadDO;
	for(i=0; i<8; i++)
	{
		SPICLK_L;
		SPI_Set_MOSI(val & 0x80);
		DSP28x_usDelay(1);
		val <<= 1;
		SPICLK_H;
		RevData <<= 1;
		//wReadDO = GpioDataRegs.GPADAT.bit.GPIO6;
		RevData |=READ_MISO;
		DSP28x_usDelay(1);
	}
	return RevData;
}
/*******************************************************************************
* Function Name  : W25Q16_ReadWriteDate
* Description    : SPIFLASH 读写函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t W25Q16_ReadWriteDate(uint8_t val)
{
	return SPIReadWriteByte(val);
}
/*******************************************************************************
* Function Name  : W25q64_WriteEnable
* Description    : 写使能
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void W25q16_WriteEnable(void)
{
	FLASH_CS_RESET;
	W25Q16_ReadWriteDate(0x06);
	FLASH_CS_SET;
}
/*******************************************************************************
* Function Name  : W25Q16_WaitForWriteEnd
* Description    : 等待FLASH内部时序操作完成
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void W25Q16_WaitForWriteEnd(void)
{
	uint8_t status = 0;
    FLASH_CS_RESET;
    W25Q16_ReadWriteDate(0x05);
    do
    {
    	status = W25Q16_ReadWriteDate(byDUMMY);
    }
    while((status & 0x01) == 1);
    FLASH_CS_SET;
}
/*******************************************************************************
* Function Name  : W25q16_WriteStatusReg1
* Description    : 写状态寄存器1
* Input          : status:状态值
* Output         : None
* Return         : None
*******************************************************************************/
void W25q16_WriteStatusReg1(uint16_t status)
{
	W25q16_WriteEnable();
	FLASH_CS_RESET;
	W25Q16_ReadWriteDate(0x01);
	W25Q16_ReadWriteDate(status & 0xFF);
	W25Q16_ReadWriteDate((status>>8) & 0xFF);
	FLASH_CS_SET;
	W25Q16_WaitForWriteEnd();
}
/*******************************************************************************
* Function Name  : W25q16_ReadStatus
* Description    : 读状态寄存器
* Input          : None
* Output         : None
* Return         : 状态寄存器值
*******************************************************************************/
uint16_t W25q16_ReadStatus(void)
{
	uint16_t status=0;
	FLASH_CS_RESET;
	W25Q16_ReadWriteDate(0x05);
	status = W25Q16_ReadWriteDate(byDUMMY);
	FLASH_CS_SET;
	FLASH_CS_RESET;
	W25Q16_ReadWriteDate(0x35);
	status |= W25Q16_ReadWriteDate(byDUMMY)<<8;
	FLASH_CS_SET;
	return status;
}
/*******************************************************************************
* Function Name  : W25q16_ReadDeviceID
* Description    : 读芯片ID
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t W25q16_ReadDeviceID(void)
{

	FLASH_CS_RESET;
	W25Q16_ReadWriteDate(0xAB);
	W25Q16_ReadWriteDate(byDUMMY);
	W25Q16_ReadWriteDate(byDUMMY);
	W25Q16_ReadWriteDate(byDUMMY);
	DeviceID = W25Q16_ReadWriteDate(0X00);
	FLASH_CS_SET;
	return DeviceID;
}
/*******************************************************************************
* Function Name  : W25q16_ReadJEDE_ID
* Description    : 读芯片ID
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t W25q16_ReadJEDE_ID(void)
{
	//uint16_t DeviceID=0;
	FLASH_CS_RESET;
	W25Q16_ReadWriteDate(0x9F);
	W25Q16_ReadWriteDate(byDUMMY);
//	W25Q16_ReadWriteDate(byDUMMY);
//	W25Q16_ReadWriteDate(0x00);
	DeviceID = W25Q16_ReadWriteDate(byDUMMY)<<8;
	DeviceID |= W25Q16_ReadWriteDate(byDUMMY);
	FLASH_CS_SET;
	return DeviceID;
}
/*******************************************************************************
* Function Name  : W25q16_Read
* Description    : 读数据
* Input          : buf:数据缓冲区
                   len:长度
                   addr:起始地址
* Output         : None
* Return         : None
*******************************************************************************/
void W25q16_Read(uint8_t *buf, uint32_t len, uint32_t addr)
{
	uint32_t i;
	FLASH_CS_RESET;
	W25Q16_ReadWriteDate(0x03);
	W25Q16_ReadWriteDate((addr & 0xFF0000)>>16);
	W25Q16_ReadWriteDate((addr & 0x00FF00)>>8);
	W25Q16_ReadWriteDate(addr & 0xFF);
	for (i=0; i<len; i++)
	{
	  buf[i] = W25Q16_ReadWriteDate(byDUMMY);
	}
	FLASH_CS_SET;
}
/*******************************************************************************
* Function Name  : W25q16_4KErase
* Description    : 4K片擦除
* Input          : addr:起始地址
* Output         : None
* Return         : None
*******************************************************************************/
void  W25q16_4KErase(uint32_t addr)
{
	W25q16_WriteEnable();
	FLASH_CS_RESET;
	W25Q16_ReadWriteDate(0x20);
	W25Q16_ReadWriteDate((addr & 0xFF0000)>>16);
	W25Q16_ReadWriteDate((addr & 0x00FF00)>>8);
	W25Q16_ReadWriteDate(addr & 0xFF);
	FLASH_CS_SET;
	W25Q16_WaitForWriteEnd();
}
/*******************************************************************************
* Function Name  : W25q16_32KErase
* Description    : 32K片擦除
* Input          : addr:起始地址
* Output         : None
* Return         : None
*******************************************************************************/
void  W25q16_32KErase(uint32_t addr)
{
	W25q16_WriteEnable();
	FLASH_CS_RESET;
	W25Q16_ReadWriteDate(0x52);
	W25Q16_ReadWriteDate((addr & 0xFF0000)>>16);
	W25Q16_ReadWriteDate((addr & 0x00FF00)>>8);
	W25Q16_ReadWriteDate(addr & 0xFF);
	FLASH_CS_SET;
	W25Q16_WaitForWriteEnd();
}
/*******************************************************************************
* Function Name  : W25q16_64KErase
* Description    : 64K片擦除
* Input          : addr:起始地址
* Output         : None
* Return         : None
* Date           : 2014-10-24
* Author         : ADT LL
*******************************************************************************/
void  W25q16_64KErase(uint32_t addr)
{
	W25q16_WriteEnable();
	FLASH_CS_RESET;
	W25Q16_ReadWriteDate(0xD8);
	W25Q16_ReadWriteDate((addr & 0xFF0000)>>16);
	W25Q16_ReadWriteDate((addr & 0x00FF00)>>8);
	W25Q16_ReadWriteDate(addr & 0xFF);
	FLASH_CS_SET;
	W25Q16_WaitForWriteEnd();
}
/*******************************************************************************
* Function Name  : W25q16_PageProgram
* Description    : 页写
* Input          : buf:要写内的数据
                   len:数据长度
                   add:起始地址
* Output         : None
* Return         : 状态码
*******************************************************************************/
static void W25q16_PageProgram(uint8_t *buf, uint16_t len, uint32_t addr)
{
    //是否判断len 与addr 超限问题？？？
	//......
	W25q16_WriteEnable();
	FLASH_CS_RESET;
	W25Q16_ReadWriteDate(0x02);
	W25Q16_ReadWriteDate((addr & 0xFF0000)>>16);
	W25Q16_ReadWriteDate((addr & 0x00FF00)>>8);
	W25Q16_ReadWriteDate(addr & 0xFF);
	while (len--)
	{
		W25Q16_ReadWriteDate(*buf);
		buf++;
	}
	FLASH_CS_SET;
	W25Q16_WaitForWriteEnd();
}
/*******************************************************************************
* Function Name  : W25q64_Write
* Description    : 数据存储
* Input          : buf:要写内的数据
                   len:数据长度
                   add:起始地址
* Output         : None
* Return         : None
*******************************************************************************/
void W25q16_Write(uint8_t *buf, uint16_t len, uint32_t addr)
{
	uint8_t pagenum;
	uint8_t addrbyte;//最低八位地址
	addrbyte = addr%W25Q16_PAGE_SIZE;
	if (len > (W25Q16_PAGE_SIZE - addrbyte))//跨页了
	{
		W25q16_PageProgram(buf, W25Q16_PAGE_SIZE - addrbyte, addr);//写满本页
		addr += W25Q16_PAGE_SIZE-addrbyte;
		buf += W25Q16_PAGE_SIZE-addrbyte;
		len -= W25Q16_PAGE_SIZE-addrbyte;
		pagenum = len/W25Q16_PAGE_SIZE;
		while (pagenum--)
		{
			W25q16_PageProgram(buf, W25Q16_PAGE_SIZE, addr);
			addr += W25Q16_PAGE_SIZE;
			buf += W25Q16_PAGE_SIZE;
			len -= W25Q16_PAGE_SIZE;
		}
		W25q16_PageProgram(buf, len, addr);
	}
	else
	{
		W25q16_PageProgram(buf, len, addr);
	}
}
