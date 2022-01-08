/******************** (C) COPYRIGHT  Դ�ع����� ********************************
 * �ļ���  ��spi.c
 * ����    ��SPIģ��ĳ�ʼ�����룬���ó�����ģʽ
 * ����    ��lmz
 * �汾����: 2021-08-30
 * Ӳ������  :PC12-/CS   PC9--CLK   PC10--MISO  PC11--MOSI
 * ���Է�ʽ��KungFu KF32DP2
**********************************************************************************/

#include "spi.h"
#include "system_init.h"
#include "spi.h"

/**
  * ����   GPIO_SPI2()������ӳ��
  * ����   ��
  * ����   ��
  * PC12=SS,PC9=SCK,PC10=SDI,PC11=SDO
  */
void GPIO_SPI2()
{
	/*SPI2_IO����*/
	GPIO_Write_Mode_Bits(GPIOC_SFR,GPIO_PIN_MASK_9  \
			                      |GPIO_PIN_MASK_10  \
			                      |GPIO_PIN_MASK_11 ,GPIO_MODE_RMP);	//��ӳ��IO�ڹ���ģʽ

	GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_9,GPIO_RMP_AF7_SPI2);	//��ӳ��ΪSPI2
	GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_10,GPIO_RMP_AF7_SPI2);	//��ӳ��ΪSPI2
	GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_11,GPIO_RMP_AF7_SPI2);	//��ӳ��ΪSPI2

	GPIO_Write_Mode_Bits(GPIOC_SFR,GPIO_PIN_MASK_12,GPIO_MODE_OUT);		// cs
	// SET read and write SPEED
	GPIO_Speed_Config(GPIOC_SFR,GPIO_PIN_MASK_12,GPIO_LOW_SPEED);		// set speed 10MHz
	GPIO_Speed_Config(GPIOC_SFR,GPIO_Pin_Num_9,GPIO_HIGH_SPEED);		// set speed 50MHz
	GPIO_Speed_Config(GPIOC_SFR,GPIO_Pin_Num_10,GPIO_HIGH_SPEED);		// set speed 50MHz
	GPIO_Speed_Config(GPIOC_SFR,GPIO_Pin_Num_11,GPIO_HIGH_SPEED);		// set speed 50MHz
}
/**
  * ����   SPI2_Init() SPI2 init
  * ����   ��
  * ����   ��
  */
void SPI2_Init(void)
{
	/* init gpio */
	SPI_InitTypeDef newStruct_SPI;
	SPI_SFRmap* SPIx = SPI_COM;

	/*SPI����*/
//#if SPI_MASTER
	newStruct_SPI.m_Mode 		= SPI_MODE_MASTER_CLKDIV4;    	//��ģʽ��ʱ��4��Ƶ 30M
//#else
//	newStruct_SPI.m_Mode		= SPI_MODE_SLAVE;              	//��ģʽ
//#endif
 	newStruct_SPI.m_Clock 		= SPI_CLK_SCLK;					//SPI��Ƶʱ��
	newStruct_SPI.m_FirstBit 	= SPI_FIRSTBIT_MSB;				//MSB
	newStruct_SPI.m_CKP 		= SPI_CKP_HIGH;              	//SCK����Ϊ�� //����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ
	newStruct_SPI.m_CKE			= SPI_CKE_1EDGE;            	//��һ��ʱ�ӿ�ʼ�������� //����SPI��SCLK��ǰһ���ز�������һ���������
	newStruct_SPI.m_DataSize	= SPI_DATASIZE_8BITS;       	//8bit
	// 0x12B		20us	50MHz	��ʱ���Ƶ��
	// 0x176		25us	40MHz
	// 0x257		40us	25MHz
	// 0x2ED		50us	20MHz
	// 0x5DB		100us	10MHz	Ĭ��
	// 0xBB7		200us	5MHz
	newStruct_SPI.m_BaudRate 	= 0x12B;                    	//Fck_spi=Fck/2(m_BaudRate+1)=10us 20us
    SPI_Reset(SPIx);											//��λģ��
    SPI_Configuration(SPIx, &newStruct_SPI);					//д��ṹ������
	SPI_Cmd(SPIx,TRUE);											//ʹ��
}

/**
*��������:SPI2_ReadWriteByte
*���ܸ�Ҫ:SPI12��дһ���ֽ�
*��������:TxData:Ҫд����ֽ�
*��������:��ȡ�����ֽ�
*/
//=============================================================================
uint8_t SPI2_ReadWriteByte(uint8_t TxData)
{
    /*�ȴ�����buffer���� */
    while (SPI_Get_Transmit_Buf_Flag(SPI_COM) == SET);
    /*���� byte */
	SPI_I2S_SendData8(SPI_COM, TxData);						//MOSI
	/*�ȴ��������*/
	while (SPI_Get_BUSY_Flag(SPI_COM)==SET);
	/*�ȴ��������*/
//    while (SPI_Get_Receive_Buf_Flag(SPI_COM) == RESET);
    /*���ؽ�������*/
	return SPI_I2S_ReceiveData(SPI_COM);					//MISO
}






