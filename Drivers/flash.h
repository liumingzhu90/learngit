/**
  ********************************************************************
  * 文件名  flash.h
  * 作  者   ChipON_AE/FAE_Group
  * 版  本  V2.1
  * 日  期  2019-11-16
  * 描  述  该文件提供了flash功能的相关读写函数功能定义
  *
  *********************************************************************
*/
#ifndef FLASH_H_
#define FLASH_H_
#include "system_init.h"

#define Flash_OK                 1
#define Flash_FAIL               0
#define			FLASH_BUFFER_MAX	    256         //整页的数据个数
#define			FLASH_BUFFER_Halfpage	128         //半页的数据个数
uint32_t     	FLASH_BUFFER[FLASH_BUFFER_MAX];     //程序区整页写时用到数组
uint32_t     	FLASH_BUFFER_CFG[FLASH_BUFFER_MAX];  //信息区整页读写时用到数组
uint32_t     	FLASH_BUFFER_HALFPAGE[FLASH_BUFFER_Halfpage];   //半页写用到的数组
uint32_t     	FLASH_BUFFER_Read[FLASH_BUFFER_MAX];            //整页读时用到数组
uint32_t Read_Flash_or_CFR_RAM (uint32_t address,uint32_t ZoneSelect);
void FLASH_HALFPAGE_WRITECODE_fun(uint32_t address,uint32_t *p_FlashBuffer,uint32_t length);
void FLASH_PageWrite_fun(uint32_t address,uint32_t *p_FlashBuffer,uint8_t length);
void FLASH_WriteCODE_ONE(uint32_t address,uint32_t *p_FlashBuffer);
void FLASH_READCODE_fun(uint32_t address,uint32_t *p_FlashBuffer,uint32_t length);


void FLASH_WriteCFG_ONE(uint32_t address,uint32_t *p_FlashBuffer);
void FLASH_HALFPAGE_WRITECFG_fun(uint32_t address,uint32_t *p_FlashBuffer,uint32_t length);
void FLASH_PageWrite_CFG_fun(uint32_t address,uint32_t *p_FlashBuffer,uint8_t length);

//地址必须为被8整除
void FLASH_WriteByte(uint32_t address,uint8_t p_FlashBuffer);     //写byte
void FLASH_WriteHalfWord(uint32_t address,uint16_t p_FlashBuffer);//写HalWord
void FLASH_WriteWord(uint32_t address,uint32_t p_FlashBuffer);    //写word
void FLASH_WriteNByte(uint32_t address,uint8_t *p_FlashBuffer,uint32_t leng);//写多Byte

uint32_t FLASH_ReadByte(uint32_t address,uint8_t *p_FlashBuffer);  //读byte
uint32_t FLASH_ReadHalWord(uint32_t address,uint16_t *p_FlashBuffer);//读HalWord
uint32_t FLASH_ReadWord(uint32_t address,uint32_t *p_FlashBuffer);//读Word
void FLASH_ReadNByte(uint32_t address,uint8_t *p_FlashBuffer,uint32_t leng);//读多Byte
void FLASH_WriteHalfWord_Cfg(uint32_t address,uint16_t p_FlashBuffer);
void FLASH_WriteCFG_ONE(uint32_t address,uint32_t *p_FlashBuffer);
/*参数地址划分*/
//第一块
#define  CAN0_RATE_ADDRESS	0x7f000			//can0波特率设置参数区
#define  CAN1_RATE_ADDRESS	0x7f010			//can1波特率设置参数区
#define  CAN2_RATE_ADDRESS	0x7f020			//can2波特率设置参数区
#define  CAN3_RATE_ADDRESS	0x7f030			//can3波特率设置参数区
#define  CAN4_RATE_ADDRESS	0x7f040			//can4波特率设置参数区
#define  CAN5_RATE_ADDRESS	0x7f050			//can5波特率设置参数区

#define  VEHICLE_WITH_ADDRESS	0x7f060			//车宽设置参数区
#define  VEHICLE_KEEP_DIS	 	0x7f070			//车距保持设置参数区
#define  VEHICLE_TTC1	 		0x7f080			//ttc1设置参数区
#define  VEHICLE_TTC2	 		0x7f090			//ttc2设置参数区
#define  VEHICLE_SENSITIVITY	0x7f0A0			//ttc2设置参数区
#define	 VEHICLE_COTROL_MODE	0x7f0B0
#define	 VEHICLE_GEAR_NUM		0x7f0C0
#define	 VEHICLE_HMW1		0x7f0D0
#define	 VEHICLE_HMW2		0x7f0E0

//第二块
#define  VEHICLE_SPEED_MODE				0x7f100			//车速模式
#define  VEHICLE_SPEED_ID	 			0x7f110			//车速ID
#define  VEHICLE_SPEED_BYTETYPE	 		0x7f120			//车速字节模式
#define  VEHICLE_SPEED_BYTESTART	 	0x7f130			//车速开始字节
#define  VEHICLE_SPEED_BYTENUM			0x7f140			//车速字节个数

#define	 VEHICLE_SPEED_SCALE_FACTOR_A	0x7f150	//速度正向比例系数
#define	 VEHICLE_SPEED_SCALE_FACTOR_B	0x7f160	//速度负向比例系数

#define  VEHICLE_ABS_ID	 				0x7f170			//ABSID
#define  VEHICLE_ABS_TYPE	 			0x7f180			//ABS信号来源
			#define  VEHICLE_ABS_BYTENUM	 			0x7f190			//ABS字节序号			//新添加
#define  VEHICLE_ABS_STARTBIT	 		0x7f1A0			//ABS开始bit
#define  VEHICLE_ABS_BITNUM				0x7f1B0			//ABS bit个数
#define	 VEHICLE_ABS_WRONGNUM			0x7f1C0			//ABS错误值
//第三块
#define  VEHICLE_TURN_ID	 				0x7f200			//转向ID
#define  VEHICLE_TURN_BYPE	 			0x7f210			//转向信号来源
#define  VEHICLE_TURN_STARTBYTE	 		0x7f220			//转向字节
#define  VEHICLE_TURN_LSTARTBIT				0x7f230			//左转向起始bit
#define	 VEHICLE_TURN_LBITNUM			0x7f240			//左转向bit个数
#define  VEHICLE_TURN_LEFFECTIVE	 			0x7f250			//左转向有效值
//#define  VEHICLE_TURN_RSTARTBIT	 		0x7f260			//右转向起始bit
//#define  VEHICLE_TURN_RBITNUM				0x7f270			//右转向bit个数
//#define	 VEHICLE_TURN_REFFECTIVE			0x7f280			//右转向有效值

#define  VEHICLE_TURN_ID1	 				0x7f260			//转向ID
#define  VEHICLE_TURN_BYPE1	 			0x7f270			//转向信号来源
#define  VEHICLE_TURN_STARTBYTE1	 		0x7f280			//转向字节
//#define  VEHICLE_TURN_LSTARTBIT				0x7f2c0			//左转向起始bit
//#define	 VEHICLE_TURN_LBITNUM			0x7f2d0			//左转向bit个数
//#define  VEHICLE_TURN_LEFFECTIVE	 			0x7f2e0			//左转向有效值
#define  VEHICLE_TURN_RSTARTBIT	 		0x7f290			//右转向起始bit
#define  VEHICLE_TURN_RBITNUM				0x7f2a0			//右转向bit个数
#define	 VEHICLE_TURN_REFFECTIVE			0x7f2b0			//右转向有效值

//第四块
#define  VEHICLE_BRAKE_ID	 				0x7f300			//刹车ID
#define  VEHICLE_BRAKE_BYPE	 			0x7f310			//刹车信号来源
#define  VEHICLE_BRAKE_STARTBYTE	 		0x7f320			//刹车字节
#define  VEHICLE_BRAKE_STARTBIT				0x7f330			//刹车起始bit
#define	 VEHICLE_BRAKE_BITNUM			0x7f340			//刹车bit个数
#define  VEHICLE_BRAKE_NUMTYPE	 			0x7f350			//刹车值类型
#define  VEHICLE_BRAKE_LEFFECTIVE	 		0x7f360			//刹车有效值
#define  VEHICLE_BRAKE_SCALE_FACTOR_A			0x7f370			//刹车正向系数A
#define	 VEHICLE_BRAKE_SCALE_FACTOR_B			0x7f380			//刹车负向系数B
//第五块
#define  FCW_LINKAGE_ADDRESS	0x7f400			//FCW车速关联设置参数区
#define  FCW_LINKAGE_SPEED_ADDRESS	0x7f410			//FCW关联车速设置参数区
#define  LDW_LINKAGE_ADDRESS	0x7f420			//LDW车速关联设置参数区
#define  LDW_LINKAGE_SPEED_ADDRESS	0x7f430			//LDW关联车速设置参数区
#define  HMW_LINKAGE_ADDRESS	0x7f440			//HMW车速关联设置参数区
#define  HMW_LINKAGE_SPEED_ADDRESS	0x7f450			//HMW关联车速设置参数区
#define  AEB_LINKAGE_ADDRESS	0x7f460			//AEB车速关联设置参数区
#define  AEB_LINKAGE_SPEED_ADDRESS	0x7f470			//AEB关联车速设置参数区

#define  SWA_IID_ADDRESS	0x7f480			//方向盘转角id设置参数区
#define  SWA_BYTENUM_ADDRESS	0x7f490			//方向盘转角起始字节设置参数区
#define  SWA_STARTBIT_ADDRESS	0x7f4a0			//方向盘转角开始bit设置参数区
#define  SWA_BITLEGH_ADDRESS	0x7f4b0			//方向盘转角bit个数设置参数区
//第六块
#define  URADER_HOST_ONE_ONE_ADDRESS	0x7f500			//超声波雷达第一主机第一编号雷达设置参数区
#define  URADER_HOST_ONE_TWO_ADDRESS	0x7f510			//超声波雷达第一主机第二编号雷达设置参数区
#define  URADER_HOST_ONE_THREE_ADDRESS	0x7f520			//超声波雷达第一主机第三编号雷达设置参数区
#define  URADER_HOST_ONE_FOUR_ADDRESS	0x7f530			//超声波雷达第一主机第四编号雷达设置参数区

#define  URADER_HOST_ONE_FIVE_ADDRESS	0x7f540			//超声波雷达第一主机第五编号雷达设置参数区
#define  URADER_HOST_ONE_SIX_ADDRESS	0x7f550			//超声波雷达第一主机第六编号雷达设置参数区
#define  URADER_HOST_ONE_SEVEN_ADDRESS	0x7f560			//超声波雷达第一主机第七编号雷达设置参数区
#define  URADER_HOST_ONE_EIGHT_ADDRESS	0x7f570			//超声波雷达第一主机第八编号雷达设置参数区

#define  URADER_HOST_ONE_NINE_ADDRESS	0x7f580			//超声波雷达第一主机第九编号雷达设置参数区
#define  URADER_HOST_ONE_TEN_ADDRESS	0x7f590			//超声波雷达第一主机第十编号雷达设置参数区
#define  URADER_HOST_ONE_ELEVEN_ADDRESS	0x7f5a0			//超声波雷达第一主机第十一编号雷达设置参数区
#define  URADER_HOST_ONE_TWELVE_ADDRESS	0x7f5b0			//超声波雷达第一主机第十二编号雷达设置参数区
//第七块
#define  URADER_HOST_TWO_ONE_ADDRESS	0x7f600			//超声波雷达第二主机第一编号雷达设置参数区
#define  URADER_HOST_TWO_TWO_ADDRESS	0x7f610			//超声波雷达第二主机第二编号雷达设置参数区
#define  URADER_HOST_TWO_THREE_ADDRESS	0x7f620			//超声波雷达第二主机第三编号雷达设置参数区
#define  URADER_HOST_TWO_FOUR_ADDRESS	0x7f630			//超声波雷达第二主机第四编号雷达设置参数区

#define  URADER_HOST_TWO_FIVE_ADDRESS	0x7f640			//超声波雷达第二主机第五编号雷达设置参数区
#define  URADER_HOST_TWO_SIX_ADDRESS	0x7f650			//超声波雷达第二主机第六编号雷达设置参数区
#define  URADER_HOST_TWO_SEVEN_ADDRESS	0x7f660			//超声波雷达第二主机第七编号雷达设置参数区
#define  URADER_HOST_TWO_EIGHT_ADDRESS	0x7f670			//超声波雷达第二主机第八编号雷达设置参数区

#define  URADER_HOST_TWO_NINE_ADDRESS	0x7f680			//超声波雷达第二主机第九编号雷达设置参数区
#define  URADER_HOST_TWO_TEN_ADDRESS	0x7f690			//超声波雷达第二主机第十编号雷达设置参数区
#define  URADER_HOST_TWO_ELEVEN_ADDRESS	0x7f6a0			//超声波雷达第二主机第十一编号雷达设置参数区
#define  URADER_HOST_TWO_TWELVE_ADDRESS	0x7f6b0			//超声波雷达第二主机第十二编号雷达设置参数区

//第八快
#define URADER_SYS_MESSAGE_FIRST_DIMENSIONAL_COEFFICIENT 0x7f700
#define URADER_SYS_MESSAGE_SECOND_DIMENSIONAL_COEFFICIENT 0x7f710
#define URADER_SYS_MESSAGE_0SPEED_THRESHOLD 0x7f720
#define URADER_SYS_MESSAGE_5SPEED_THRESHOLD 0x7f730
#define URADER_SYS_MESSAGE_10SPEED_THRESHOLD 0x7f740
#define URADER_SYS_MESSAGE_15SPEED_THRESHOLD 0x7f750
#define URADER_SYS_MESSAGE_20SPEED_THRESHOLD 0x7f760
#define URADER_SYS_MESSAGE_LINKAGE_STATA 0x7f770
#define URADER_SYS_MESSAGE_LINKAGE_SPEED 0x7f780
#define URADER_SYS_MESSAGE_25SPEED_THRESHOLD 0x7f790
#endif /* FLASH_H_ */
