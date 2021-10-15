/*
 * spi_flash.h
 *
 *  Created on: 2021-8-9
 *      Author: wangzhenbao
 */

#ifndef SPI_FLASH_H_
#define SPI_FLASH_H_

void GPIO_SPI2(void);
void SPI_Init_Configuration(SPI_SFRmap* SPIx);
uint8_t SPI2_Write_and_Read_Byte(uint8_t TxData);
uint8_t SPI2_Write_and_Read_Byte_Commond(uint8_t TxData);
void W25Q16_readID(void);
uint16_t W25q16_ReadDeviceID(void);
uint16_t W25q16_ReadJEDE_ID(void);
#endif /* SPI_FLASH_H_ */
