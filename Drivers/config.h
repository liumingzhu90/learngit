/*
 * config.h
 *
 *  Created on: 2021-7-7
 *      Author: chenq
 */

#ifndef CONFIG_H_
#define CONFIG_H_

//地址和偏移都必须能被8整除
#define CONFIG_START_ADDRESS 0x78000//页起始地址
#define HARDWARE_SN_OFFSET 0x0
#define HARDWARE_VER_OFFSET 0x16
#define SOFTWARE_VER_OFFSET 0x32

#define SN_LENGTH 16
#define VER_LENGTH 16

void FLASH_Read_NByte(uint32_t page_address,uint8_t *p_DataBuffer,uint32_t offset,uint32_t leng);
void FLASH_Modify_NByte(uint32_t page_address,uint8_t *p_DataBuffer,uint32_t offset,uint32_t leng);
void set_hardware_sn(uint8_t *hardware_sn);
void get_hardware_sn(uint8_t *hardware_sn);
void set_hardware_ver(uint8_t *hardware_ver);
void set_software_ver(uint8_t *software_ver);
void get_hardware_ver(uint8_t *hardware_ver);
void get_software_ver(uint8_t *software_ver);

#endif /* CONFIG_H_ */
