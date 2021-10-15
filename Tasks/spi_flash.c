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

#define W25Q16_PAGE_SIZE    256                          //һҳ�Ĵ�С��256�ֽ�
#define W25Q16_SECTOR_SIZE (4*1024)                      //������С���ֽ�
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
  * ����  GPIOx �����ʼ�����á�
  * ���� : GPIOx: ָ��GPIO�ڴ�ṹ��ָ�룬ȡֵΪGPIOA_SFR~GPIOH_SFR��
  *       GpioPin: �˿��������룬ȡֵΪGPIO_PIN_MASK_0~GPIO_PIN_MASK_15�е�һ��������ϡ�
  * ����  �ޡ�
  */
void GPIOInit_Output_Config(GPIO_SFRmap* GPIOx,uint16_t GpioPin)
{
	/*��ʼ����λGPIOx���裬ʹ��GPIOx����ʱ��*/
		GPIO_Reset(GPIOx);

	/* ���� Pxy��Ϊ���ģʽ���� */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_Struct_Init(&GPIO_InitStructure);
	GPIO_InitStructure.m_Pin = GpioPin;
	GPIO_InitStructure.m_Speed = GPIO_LOW_SPEED;          //��ʼ�� GPIO����ٶ�
	GPIO_InitStructure.m_Mode = GPIO_MODE_OUT;            //��ʼ�� GPIO����Ϊ���
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;            //��ʼ�� GPIO�Ƿ�����
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;          //��ʼ�� GPIO�Ƿ�����
	GPIO_Configuration(GPIOx,&GPIO_InitStructure);

	//GPIO_Set_Output_Data_Bits(GPIOx,GpioPin,Bit_SET);	 //������Ϊ�ߵ�ƽ

}

/**
  * ����  GPIOx �����ʼ�����á�
  * ���� : GPIOx: ָ��GPIO�ڴ�ṹ��ָ�룬ȡֵΪGPIOA_SFR~GPIOH_SFR��
  *       GpioPin: �˿��������룬ȡֵΪGPIO_PIN_MASK_0~GPIO_PIN_MASK_15�е�һ��������ϡ�
  * ����  �ޡ�
  */
void GPIOInit_Input_Config(GPIO_SFRmap* GPIOx,uint16_t GpioPin)
{
	/*��ʼ����λGPIOx���裬ʹ��GPIOx����ʱ��*/
		GPIO_Reset(GPIOx);

	/* ���� Pxy��Ϊ����ģʽ */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_Struct_Init(&GPIO_InitStructure);
	GPIO_InitStructure.m_Pin = GpioPin;
	GPIO_InitStructure.m_Speed = GPIO_LOW_SPEED;                   //��ʼ�� GPIO����ٶ�
	GPIO_InitStructure.m_Mode = GPIO_MODE_IN;                      //��ʼ�� GPIO����Ϊ����
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;                     //��ʼ�� GPIO�Ƿ����� ������
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;                   //��ʼ�� GPIO�Ƿ����� ������
	GPIO_Configuration(GPIOx,&GPIO_InitStructure);
}

void GPIO_SPI2(void)
{
	/*SPI1_IO����*/
	//PB0=SS,PB1=SCK,PB2=SDI,PB3=SDO

	GPIO_Write_Mode_Bits(GPIOC_SFR,GPIO_PIN_MASK_9  \
			                      |GPIO_PIN_MASK_10  \
			                      |GPIO_PIN_MASK_11,GPIO_MODE_RMP);//��ӳ��IO�ڹ���ģʽ
			                      //|GPIO_PIN_MASK_12,GPIO_MODE_RMP);//��ӳ��IO�ڹ���ģʽ
	//GPIO_Write_Mode_Bits(GPIOC_SFR,GPIO_PIN_MASK_12,GPIO_MODE_OUT);
	GPIOInit_Output_Config(GPIOC_SFR,GPIO_PIN_MASK_12);
	GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_9,GPIO_RMP_AF7_SPI2);//��ӳ��ΪSPI1
	GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_10,GPIO_RMP_AF7_SPI2);//��ӳ��ΪSPI1
	GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_11,GPIO_RMP_AF7_SPI2);//��ӳ��ΪSPI1
    //GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_12,GPIO_RMP_AF7_SPI2);//��ӳ��ΪSPI1

    SPI_Init_Configuration(SPI2_SFR);
    INT_Interrupt_Enable(INT_SPI2,TRUE);//SPI1�ж�ʹ��
	INT_All_Enable(TRUE);//���ж�ʹ��
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
  * ����    SPI����
  * ����   SPIx��ָ��SPI�ڴ�ṹ��ָ�룬ȡֵΪSPI0_SFR~SPI3_SFR
  * ����   ��
  */
void SPI_Init_Configuration(SPI_SFRmap* SPIx)
{
	SPI_InitTypeDef newStruct_SPI;
	/*SPI����*/
#if SPI_MASTER
	newStruct_SPI.m_Mode = SPI_MODE_MASTER_CLKDIV4;                   //��ģʽ��ʱ��4��Ƶ 18M
#else
	newStruct_SPI.m_Mode = SPI_MODE_SLAVE;                            //��ģʽ
#endif
 	newStruct_SPI.m_Clock = SPI_CLK_SCLK;							  //SPI��Ƶʱ��
	newStruct_SPI.m_FirstBit = SPI_FIRSTBIT_MSB;					  //MSB
	newStruct_SPI.m_CKP = SPI_CKP_LOW;                                //SCK����Ϊ��
	newStruct_SPI.m_CKE = SPI_CKE_1EDGE;                              //��һ��ʱ�ӿ�ʼ��������
	newStruct_SPI.m_DataSize = SPI_DATASIZE_8BITS;                    //8bit
	newStruct_SPI.m_BaudRate = 0x59;                                  //Fck_spi=Fck/2(m_BaudRate+1)=10us
    SPI_Reset(SPIx);											      //��λģ��
    SPI_Configuration(SPIx, &newStruct_SPI);					      //д��ṹ������
	SPI_Cmd(SPIx,TRUE);											      //ʹ��
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

void W23Q16_SendCommond(uint8_t commond)		//��������
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
* Description    : SPI��д����
* Input          : val 		��������
* Output         : RevData  ��������
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
* Description    : SPIFLASH ��д����
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
* Description    : дʹ��
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
* Description    : �ȴ�FLASH�ڲ�ʱ��������
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
* Description    : д״̬�Ĵ���1
* Input          : status:״ֵ̬
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
* Description    : ��״̬�Ĵ���
* Input          : None
* Output         : None
* Return         : ״̬�Ĵ���ֵ
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
* Description    : ��оƬID
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
* Description    : ��оƬID
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
* Description    : ������
* Input          : buf:���ݻ�����
                   len:����
                   addr:��ʼ��ַ
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
* Description    : 4KƬ����
* Input          : addr:��ʼ��ַ
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
* Description    : 32KƬ����
* Input          : addr:��ʼ��ַ
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
* Description    : 64KƬ����
* Input          : addr:��ʼ��ַ
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
* Description    : ҳд
* Input          : buf:Ҫд�ڵ�����
                   len:���ݳ���
                   add:��ʼ��ַ
* Output         : None
* Return         : ״̬��
*******************************************************************************/
static void W25q16_PageProgram(uint8_t *buf, uint16_t len, uint32_t addr)
{
    //�Ƿ��ж�len ��addr �������⣿����
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
* Description    : ���ݴ洢
* Input          : buf:Ҫд�ڵ�����
                   len:���ݳ���
                   add:��ʼ��ַ
* Output         : None
* Return         : None
*******************************************************************************/
void W25q16_Write(uint8_t *buf, uint16_t len, uint32_t addr)
{
	uint8_t pagenum;
	uint8_t addrbyte;//��Ͱ�λ��ַ
	addrbyte = addr%W25Q16_PAGE_SIZE;
	if (len > (W25Q16_PAGE_SIZE - addrbyte))//��ҳ��
	{
		W25q16_PageProgram(buf, W25Q16_PAGE_SIZE - addrbyte, addr);//д����ҳ
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
