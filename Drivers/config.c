/*
 * config.c
 *
 *  Created on: 2021-7-7
 *      Author: chenq
 */
#include <stdio.h>
#include "system_init.h"
#include "config.h"
#include "flash.h"
#include "usart.h"

uint32_t data[256] ;

void FLASH_Read_NByte(uint32_t page_address,uint8_t *p_DataBuffer,uint32_t offset,uint32_t leng)
{
	uint32_t i;
	for(i=0;i<256;i++)//一页1024byte
	{
		data[i]=FLASH_ReadWord(page_address+i*4,FLASH_PROGRAM_CODE);
	}
	memcpy ( p_DataBuffer,(uint8_t *)data+offset, leng);

}
void FLASH_Modify_NByte(uint32_t page_address,uint8_t *p_DataBuffer,uint32_t offset,uint32_t leng)
{

	uint32_t i;
	for(i=0;i<256;i++)//一页1024byte
	{
		data[i]=FLASH_ReadWord(page_address+i*4,FLASH_PROGRAM_CODE);
	}
	memcpy ( (uint8_t *)data +offset, p_DataBuffer, leng);

    //程序区整页写入数据
	FLASH_PageWrite_fun( page_address, data, 128);
}

void get_hardware_sn(uint8_t *hardware_sn)
{
	FLASH_Read_NByte(CONFIG_START_ADDRESS,hardware_sn,HARDWARE_SN_OFFSET,SN_LENGTH);
}

void get_hardware_ver(uint8_t *hardware_ver)
{
	FLASH_Read_NByte(CONFIG_START_ADDRESS,hardware_ver,HARDWARE_VER_OFFSET,VER_LENGTH);
}

void get_software_ver(uint8_t *software_ver)
{
	FLASH_Read_NByte(CONFIG_START_ADDRESS,software_ver,SOFTWARE_VER_OFFSET,VER_LENGTH);
}

void set_hardware_sn(uint8_t *hardware_sn)
{
	FLASH_Modify_NByte(CONFIG_START_ADDRESS,hardware_sn,HARDWARE_SN_OFFSET,SN_LENGTH);
}

void set_hardware_ver(uint8_t *hardware_ver)
{
	FLASH_Modify_NByte(CONFIG_START_ADDRESS,hardware_ver,HARDWARE_VER_OFFSET,VER_LENGTH);
}
void set_software_ver(uint8_t *software_ver)
{
	FLASH_Modify_NByte(CONFIG_START_ADDRESS,software_ver,SOFTWARE_VER_OFFSET,VER_LENGTH);
}
