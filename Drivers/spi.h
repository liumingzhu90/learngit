#ifndef __SPI_H
#define __SPI_H
#include "system_init.h"

//#define  SPI_MASTER  			1    			//SPIģʽѡ��1=��ģʽ  0=��ģʽ
#define  SPI_COM     			SPI2_SFR		//SPI1�Ĵ�����ڵ�ַ

/* -----------------------ȫ�ֺ�������------------------------------- */
void SPI2_Init(void);			 				//��ʼ��SPI2��	Ĭ�ϣ�10us
uint8_t SPI2_ReadWriteByte(uint8_t TxData);		//SPI2���߶�дһ���ֽ�
void GPIO_SPI2();

#endif

