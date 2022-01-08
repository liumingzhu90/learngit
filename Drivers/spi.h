#ifndef __SPI_H
#define __SPI_H
#include "system_init.h"

//#define  SPI_MASTER  			1    			//SPI模式选择，1=主模式  0=从模式
#define  SPI_COM     			SPI2_SFR		//SPI1寄存器入口地址

/* -----------------------全局函数声明------------------------------- */
void SPI2_Init(void);			 				//初始化SPI2口	默认：10us
uint8_t SPI2_ReadWriteByte(uint8_t TxData);		//SPI2总线读写一个字节
void GPIO_SPI2();

#endif

