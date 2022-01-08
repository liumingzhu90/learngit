/*
 * upgrade_common.h
 *
 *  Created on: 2021-12-6
 *      Author: Administrator
 * 说明:3种升级模式
 * ota_p.up_mode		ota_p.info.mode		控制模式
 * -------------------------------------------------------------
 * UPGRADE_OTA		UPGRADE_OTA				上电OTA升级
 * UPGRADE_OTA		UPGRADE_OTA_CMD			指令OTA升级
 * UPGRADE_USART1							串口升级
 * -------------------------------------------------------------
 */

#ifndef UPGRADE_COMMON_H_
#define UPGRADE_COMMON_H_
#include "system_init.h"
#include "utc_time.h"
/*******************************宏定义**********************************************/
#define BUFFER_SIZE					4096			//4096			// 分包存储大小，也是外部FLASH的扇区的大小
#define BUFFER_SIZE_I				4101			// (4096+5)		// 手动升级时，+4B校验+1B编号
#define SECTOR_SIZE					BUFFER_SIZE
#define PAGE_SIZE					256
//#define DATAUPLOAD_VERSION			"2.0"

/* 外部FLASH分区地址及大小，共2048KB */
#define OUT_FLASH_APP_START			0x00			// 平台程序存储的起始地址，大小为200KB
#define OUT_FLASH_RESERVED1_START	0x32000			// 平台程序存储的起始地址，大小为312KB
#define OUT_FLASH_PARAM_START		0x80000			// 升级版本信息的起始地址，版本+bin长度，大小为4KB
#define OUT_FLASH_SN_START			0x81000			// SN起始地址，大小为4KB，保证唯一性
#define OUT_FLASH_ALGPPARMGROUP_START 0x82000		// AEB ALG 参数配置 版本号，或者参数数组ID
#define OUT_FLASH_ALGPPARM_START	0x83000			// 算法端起始地址，大小为4KB，算法配置参数,随后会分配100*4KB进行存储配置参数
#define OUT_FLASH_RESERVED2_START	0xE6000			// 平台程序存储的起始地址，大小为1528KB

/* 内部FLASH分区地址及大小，共512KB */
#define IN_FLASH_BOOTLOADER_START	0x00000			// 存储Bootloader，大小为64KB
#define IN_FLASH_APP_START			0x10000			// 存储app，大小为200KB
#define IN_FLASH_RESERVED_START		0x42000			// 预留大小为244KB
#define IN_FLASH_PARAM_START		0x7F000			// 留有4KB写参数

#define VERSION_SIZE		  		50
#define APP_VERSION_OFFSET			0				// storage application version

#define PKG_SIZE_SIZE				10				// storage upgrade package size
#define PKG_SIZE_OFFSET				VERSION_SIZE

//#define BOOT_VERSION_SIZE		  	50				// storage bootloader's app version
#define BOOT_VERSION_OFFSET			(VERSION_SIZE+PKG_SIZE_SIZE)

#define HTTP_ADDR_SIZE				200				// storage upgrade https address
#define HTTP_ADDR_OFFSET			(VERSION_SIZE+PKG_SIZE_SIZE+VERSION_SIZE)

#define DEV_SN_SIZE					50				// storage device sn
#define DEV_SN_OFFSET				0

#define ALG_PARA_GROUPID_SIZE		5
#define ALG_PARA_SIZE				400				// AEB ALG 参数长度

#define CONNECT_ID					0				// id
#define CONNECT_PROTO_TYPE			"\"TCP\""
#define	SERVER_DEBUG 				0
#if	SERVER_DEBUG
	#define CONNECT_IP				"\"180.76.243.232\""	// "\"43.249.193.233\""
	#define CONNECT_PORT			9818
#else
	#define CONNECT_IP				"\"39.102.42.22\""
	//#define CONNECT_IP			"\"www.gxsq.com\""
	#define CONNECT_PORT			9818
#endif


/* ------------------------枚举------------------------------- */
typedef enum{
	RECV_CMD=0,
	RECV_BIN
}RECV_MODE;
typedef enum{
	UPGRADE_NO,		// 不升级
	UPGRADE_USART1,	// 串口升级
	UPGRADE_OTA,	// 4G OTA升级
	UPGRADE_OTA_CMD,// 4G OTA指令升级
}Upgrade_Mode;

/* 4G通讯状态 */
typedef enum{
	OTA_WAIT,
	OTA_RDY,			// 4G 模块被使能后，收到起始字符RDY
	OTA_UTCTIME,		// 计算UTC时间
	OTA_AT,				// AT指令
	OTA_ATI,			// ATI指令,获取EC200U-CN SN码
	OTA_SIM,			// SIM卡状态
	OTA_GSM,			// 注册GSM网络
	OTA_CONNECT,		// 连接服务器
	OTA_SECRET,			// 获取秘钥请求 和 获取秘钥发送数据
	OTA_REGISTER,		// 注册
	OTA_REQUEST,		// 请求
	OTA_UPGRADE,		// 升级
	OTA_RESPONSE,		// 升级后回复
}Quectel4GStatus;

typedef enum{
	BIN_RECV_WAIT=0,
	BIN_RECV_READY=1,
	BIN_RECV_GO=2,
	BIN_RECV_END=3,
	BIN_STORE_CHECK=4
}BIN_STATUS;

// 【1：FCW】、【2：HMW】、【3：LDW】、【4：AEB】
typedef enum {
	NO_EVENT,
	FCW_EVENT,
	HMW_EVENT,
	LDW_EVENT,
	AEB_EVENT,
	AEB_BI_EVENT,	// AEB 双目
	AEB_MMW_EVENT,	// AEB 毫米波
	AEB_ULT_EVENT,	// AEB 超声波
	AEB_END_EVENT,	// AEB 事件结束
}TRIGGER_EVENT;
//enum{
//	EVENT_START,
//	EVENT_END
//};
/* ------------------------结构体------------------------------- */
typedef struct _Event_T{
	FunctionalState 	fcw_t;
	FunctionalState 	hmw_t;
	FunctionalState 	ldw_t;
	FunctionalState 	aeb_t;
	TRIGGER_EVENT 		event;
	uint16_t 			fcw_id;	// 事件触发ID，便于平台检索
	uint16_t 			hmw_id;
	uint16_t 			ldw_id;
	uint16_t 			aeb_id;
}Event_T;

// OTA接收数据包
typedef struct _OTA_RX_PKG{
	BIN_STATUS 	ota_rx;					// 升级过程中的状态标志
	uint32_t 	cnt;					// 计数
	uint32_t 	total;					// 升级包总长度
	uint8_t 	times;					// 升级包分几次接收
	uint16_t 	remain;					// 升级包最后剩余数量
	uint8_t 	data_i[BUFFER_SIZE_I];	// 与上位机进行交互
	uint8_t 	data1[BUFFER_SIZE];		// OTA升级BUFFER1
	uint8_t 	data2[BUFFER_SIZE];		// OTA升级BUFFER2
	uint8_t 	sw_12;					// 双buffer缓存切换标志位
	RetStatus 	is_write;				// 写标志位
	uint32_t 	pkg_crc;				// 升级包的校验值
	uint32_t 	rx_crc;					// 接收校验值
	uint32_t 	addr;					// 写地址
	uint32_t	len;					// 长度
}OTA_RX_PKG;

// OTA升级包信息
typedef struct _OTA_INFO{
	char 		secret[10];				// 秘钥
	uint8_t 	itinerary_id[20];		// 行程ID，上传报警和GPS时需要携带
	uint8_t 	https[HTTP_ADDR_SIZE];	// 下载平台地址
	uint8_t 	https_hu[HTTP_ADDR_SIZE];// 下载平台地址,手动升级
	int32_t 	size;					// 接收升级包的大小
	char 		md5[33];				// MD5加密
	char 		upgrade_v[VERSION_SIZE];// 升级版本号
	char 		run_v[VERSION_SIZE];	// 现在版本号
	char 		sn[DEV_SN_SIZE];		// 设备SN码
	uint8_t 	id;						// 请求平台ID
	Upgrade_Mode mode;					// 自动升级 | 发送指令让其升级
//	uint8_t 	version[20];			// 平台对接协议，1.0 | 2.0
}OTA_Info;

// 升级包参数
typedef struct _UpgradePara{
	FunctionalState isFinish;	// OTA TRUE完成,FALSE未完成
	RECV_MODE		rx_mode;	// 接收模式
	Upgrade_Mode	up_mode;	// 升级模式
	OTA_RX_PKG 		pkg;		// OTA接收包
	OTA_Info		info;		// OTA与平台交互信息
	RetStatus 		recv_ok;	// SUCCESS接收到以\r\n结尾的指令
}UpgradePara;

// 平台参数
typedef struct _PlatformServer{
	FunctionalState sevIsConnect;// 服务器连接状态：TRUE已连接,FALSE未连接
	FunctionalState gpsIsConnect;// GPS连接状态：TRUE已连接,FALSE未连接
	FunctionalState btIsConnect;// BT连接状态：TRUE已连接,FALSE未连接
	Quectel4GStatus	_4G_status;	// 步骤标识
	uint8_t 		stage;		// 内部接收指令，划分阶段变量[0-3]
	FunctionalState	startOTA;	// TRUE开始OTA，FALSE不启动OTA
}PlatfromServerPara;
// 字符串截取
typedef struct _strtokStr{
	uint8_t data[10][20];
	uint8_t cnt;
}StrtokStr;
/* ------------------------全局变量------------------------------- */
extern uint32_t 		utctime;			// UTCTIME时间
extern PlatfromServerPara server_p;			// 4G连接服务器
extern UpgradePara		upgrade_p;			// 升级参数：OTA升级|串口升级
/* ------------------------全局函数------------------------------- */
void 		Connect_Server_Start(FunctionalState startOTA);// 连接服务器
void 		Clear_Usart1_Buffer();			// 清空USART1用户缓冲区
void 		Clear_Buffer(void);				// 清空USART0缓冲区
void 		AT();							//
void 		ATI();							// 获取EC200U-CN SN码，作为设备的SN码
StrtokStr 	StrtokString(uint8_t *string);	// 字符串解析，按照,
void 		Upgrade_Warning();				// 升级中，警示
TimeType 	Get_CurrentTime_Stamp();		// 获取当前时间戳
void 		Get_Device_Sign_Md5(uint8_t *sign);	// 获取设备的md5加密后的sign编码
#endif /* UPGRADE_COMMON_H_ */
