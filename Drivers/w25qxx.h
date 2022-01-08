#ifndef __W25QXX_H
#define __W25QXX_H
#include "system_init.h"
#include "stdio.h"

#define W25X40B  				0XEF12
#define W25Q80 					0XEF13
#define W25Q16 					0XEF14
#define W25Q32 					0XEF15
#define W25Q64 					0XEF16
#define W25Q128					0XEF17
#define W25X_WriteEnable		0x06
#define W25X_WriteDisable		0x04
#define W25X_ReadStatusReg		0x05
#define W25X_WriteStatusReg		0x01
#define W25X_ReadData			0x03
#define W25X_FastReadData		0x0B
#define W25X_FastReadDual		0x3B
#define W25X_PageProgram		0x02
#define W25X_BlockErase			0xD8
#define W25X_SectorErase		0x20
#define W25X_ChipErase			0xC7		// 0x60
#define W25X_PowerDown			0xB9
#define W25X_ReleasePowerDown	0xAB
#define W25X_DeviceID			0xAB
#define W25X_ManufactDeviceID	0x90
#define W25X_JedecDeviceID		0x9F

#define SPI_FLASH_CS_LOW()		GPIO_Set_Output_Data_Bits(GPIOC_SFR,GPIO_PIN_MASK_12,Bit_RESET)
#define SPI_FLASH_CS_HIGH()		GPIO_Set_Output_Data_Bits(GPIOC_SFR,GPIO_PIN_MASK_12,Bit_SET)

/****************************** 结构体 *********************************************/
/* 版本号 */
typedef struct _version{
	uint8_t main_v;			// 主
	uint8_t sub_v;			// 次
	uint8_t rev_v;			// 修订
	uint8_t sensor_v[10];	// 传感器版本号
}AppVersion;
/* ----------------------- 函数声明 ------------------------------- */
void W25QXX_Init(void);
void W25QXX_Erase_Chip(void);
void W25QXX_Erase_Sector(uint32_t Dst_Addr);
void W25QXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);
void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);

uint8_t 	Version_Compare(uint8_t *current_v,uint8_t *upgrade_v);
void 		Print_Flash_bin_Content(uint32_t bin_size);			// debug print flash content

void 		Get_Upgrade_App_Version(uint8_t *read_version);
uint32_t 	Get_Upgrade_PKG_size(void);
void 		Get_Run_App_Version(uint8_t *read_version);
uint8_t		Get_Device_SN(char * sn);
void 		Get_Http_Download_Addr(uint8_t *addr);
uint8_t 	Get_AEB_Alg_Group_ID();
void		Get_AEB_Alg_Parameter(uint8_t *data,uint8_t id);

//AppVersion 	Get_APP_Version_Struct(void);
//AppVersion 	Get_Bootloader_Version_Struct();

void 	Set_Upgrade_App_Version(uint8_t *platform_version);
void  	Set_Upgrade_PKG_size(uint32_t len);
void 	Set_Run_App_Version(uint8_t *mcu_version);
void 	Set_Device_SN(uint8_t * sn);
void 	Set_Http_Download_Addr(uint8_t *addr);
void 	Set_AEB_Alg_Group_ID(uint8_t id);
void	Set_AEB_Alg_Parameter(uint8_t *data,uint8_t id);

#endif
