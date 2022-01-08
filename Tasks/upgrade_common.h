/*
 * upgrade_common.h
 *
 *  Created on: 2021-12-6
 *      Author: Administrator
 * ˵��:3������ģʽ
 * ota_p.up_mode		ota_p.info.mode		����ģʽ
 * -------------------------------------------------------------
 * UPGRADE_OTA		UPGRADE_OTA				�ϵ�OTA����
 * UPGRADE_OTA		UPGRADE_OTA_CMD			ָ��OTA����
 * UPGRADE_USART1							��������
 * -------------------------------------------------------------
 */

#ifndef UPGRADE_COMMON_H_
#define UPGRADE_COMMON_H_
#include "system_init.h"
#include "utc_time.h"
/*******************************�궨��**********************************************/
#define BUFFER_SIZE					4096			//4096			// �ְ��洢��С��Ҳ���ⲿFLASH�������Ĵ�С
#define BUFFER_SIZE_I				4101			// (4096+5)		// �ֶ�����ʱ��+4BУ��+1B���
#define SECTOR_SIZE					BUFFER_SIZE
#define PAGE_SIZE					256
//#define DATAUPLOAD_VERSION			"2.0"

/* �ⲿFLASH������ַ����С����2048KB */
#define OUT_FLASH_APP_START			0x00			// ƽ̨����洢����ʼ��ַ����СΪ200KB
#define OUT_FLASH_RESERVED1_START	0x32000			// ƽ̨����洢����ʼ��ַ����СΪ312KB
#define OUT_FLASH_PARAM_START		0x80000			// �����汾��Ϣ����ʼ��ַ���汾+bin���ȣ���СΪ4KB
#define OUT_FLASH_SN_START			0x81000			// SN��ʼ��ַ����СΪ4KB����֤Ψһ��
#define OUT_FLASH_ALGPPARMGROUP_START 0x82000		// AEB ALG �������� �汾�ţ����߲�������ID
#define OUT_FLASH_ALGPPARM_START	0x83000			// �㷨����ʼ��ַ����СΪ4KB���㷨���ò���,�������100*4KB���д洢���ò���
#define OUT_FLASH_RESERVED2_START	0xE6000			// ƽ̨����洢����ʼ��ַ����СΪ1528KB

/* �ڲ�FLASH������ַ����С����512KB */
#define IN_FLASH_BOOTLOADER_START	0x00000			// �洢Bootloader����СΪ64KB
#define IN_FLASH_APP_START			0x10000			// �洢app����СΪ200KB
#define IN_FLASH_RESERVED_START		0x42000			// Ԥ����СΪ244KB
#define IN_FLASH_PARAM_START		0x7F000			// ����4KBд����

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
#define ALG_PARA_SIZE				400				// AEB ALG ��������

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


/* ------------------------ö��------------------------------- */
typedef enum{
	RECV_CMD=0,
	RECV_BIN
}RECV_MODE;
typedef enum{
	UPGRADE_NO,		// ������
	UPGRADE_USART1,	// ��������
	UPGRADE_OTA,	// 4G OTA����
	UPGRADE_OTA_CMD,// 4G OTAָ������
}Upgrade_Mode;

/* 4GͨѶ״̬ */
typedef enum{
	OTA_WAIT,
	OTA_RDY,			// 4G ģ�鱻ʹ�ܺ��յ���ʼ�ַ�RDY
	OTA_UTCTIME,		// ����UTCʱ��
	OTA_AT,				// ATָ��
	OTA_ATI,			// ATIָ��,��ȡEC200U-CN SN��
	OTA_SIM,			// SIM��״̬
	OTA_GSM,			// ע��GSM����
	OTA_CONNECT,		// ���ӷ�����
	OTA_SECRET,			// ��ȡ��Կ���� �� ��ȡ��Կ��������
	OTA_REGISTER,		// ע��
	OTA_REQUEST,		// ����
	OTA_UPGRADE,		// ����
	OTA_RESPONSE,		// ������ظ�
}Quectel4GStatus;

typedef enum{
	BIN_RECV_WAIT=0,
	BIN_RECV_READY=1,
	BIN_RECV_GO=2,
	BIN_RECV_END=3,
	BIN_STORE_CHECK=4
}BIN_STATUS;

// ��1��FCW������2��HMW������3��LDW������4��AEB��
typedef enum {
	NO_EVENT,
	FCW_EVENT,
	HMW_EVENT,
	LDW_EVENT,
	AEB_EVENT,
	AEB_BI_EVENT,	// AEB ˫Ŀ
	AEB_MMW_EVENT,	// AEB ���ײ�
	AEB_ULT_EVENT,	// AEB ������
	AEB_END_EVENT,	// AEB �¼�����
}TRIGGER_EVENT;
//enum{
//	EVENT_START,
//	EVENT_END
//};
/* ------------------------�ṹ��------------------------------- */
typedef struct _Event_T{
	FunctionalState 	fcw_t;
	FunctionalState 	hmw_t;
	FunctionalState 	ldw_t;
	FunctionalState 	aeb_t;
	TRIGGER_EVENT 		event;
	uint16_t 			fcw_id;	// �¼�����ID������ƽ̨����
	uint16_t 			hmw_id;
	uint16_t 			ldw_id;
	uint16_t 			aeb_id;
}Event_T;

// OTA�������ݰ�
typedef struct _OTA_RX_PKG{
	BIN_STATUS 	ota_rx;					// ���������е�״̬��־
	uint32_t 	cnt;					// ����
	uint32_t 	total;					// �������ܳ���
	uint8_t 	times;					// �������ּ��ν���
	uint16_t 	remain;					// ���������ʣ������
	uint8_t 	data_i[BUFFER_SIZE_I];	// ����λ�����н���
	uint8_t 	data1[BUFFER_SIZE];		// OTA����BUFFER1
	uint8_t 	data2[BUFFER_SIZE];		// OTA����BUFFER2
	uint8_t 	sw_12;					// ˫buffer�����л���־λ
	RetStatus 	is_write;				// д��־λ
	uint32_t 	pkg_crc;				// ��������У��ֵ
	uint32_t 	rx_crc;					// ����У��ֵ
	uint32_t 	addr;					// д��ַ
	uint32_t	len;					// ����
}OTA_RX_PKG;

// OTA��������Ϣ
typedef struct _OTA_INFO{
	char 		secret[10];				// ��Կ
	uint8_t 	itinerary_id[20];		// �г�ID���ϴ�������GPSʱ��ҪЯ��
	uint8_t 	https[HTTP_ADDR_SIZE];	// ����ƽ̨��ַ
	uint8_t 	https_hu[HTTP_ADDR_SIZE];// ����ƽ̨��ַ,�ֶ�����
	int32_t 	size;					// �����������Ĵ�С
	char 		md5[33];				// MD5����
	char 		upgrade_v[VERSION_SIZE];// �����汾��
	char 		run_v[VERSION_SIZE];	// ���ڰ汾��
	char 		sn[DEV_SN_SIZE];		// �豸SN��
	uint8_t 	id;						// ����ƽ̨ID
	Upgrade_Mode mode;					// �Զ����� | ����ָ����������
//	uint8_t 	version[20];			// ƽ̨�Խ�Э�飬1.0 | 2.0
}OTA_Info;

// ����������
typedef struct _UpgradePara{
	FunctionalState isFinish;	// OTA TRUE���,FALSEδ���
	RECV_MODE		rx_mode;	// ����ģʽ
	Upgrade_Mode	up_mode;	// ����ģʽ
	OTA_RX_PKG 		pkg;		// OTA���հ�
	OTA_Info		info;		// OTA��ƽ̨������Ϣ
	RetStatus 		recv_ok;	// SUCCESS���յ���\r\n��β��ָ��
}UpgradePara;

// ƽ̨����
typedef struct _PlatformServer{
	FunctionalState sevIsConnect;// ����������״̬��TRUE������,FALSEδ����
	FunctionalState gpsIsConnect;// GPS����״̬��TRUE������,FALSEδ����
	FunctionalState btIsConnect;// BT����״̬��TRUE������,FALSEδ����
	Quectel4GStatus	_4G_status;	// �����ʶ
	uint8_t 		stage;		// �ڲ�����ָ����ֽ׶α���[0-3]
	FunctionalState	startOTA;	// TRUE��ʼOTA��FALSE������OTA
}PlatfromServerPara;
// �ַ�����ȡ
typedef struct _strtokStr{
	uint8_t data[10][20];
	uint8_t cnt;
}StrtokStr;
/* ------------------------ȫ�ֱ���------------------------------- */
extern uint32_t 		utctime;			// UTCTIMEʱ��
extern PlatfromServerPara server_p;			// 4G���ӷ�����
extern UpgradePara		upgrade_p;			// ����������OTA����|��������
/* ------------------------ȫ�ֺ���------------------------------- */
void 		Connect_Server_Start(FunctionalState startOTA);// ���ӷ�����
void 		Clear_Usart1_Buffer();			// ���USART1�û�������
void 		Clear_Buffer(void);				// ���USART0������
void 		AT();							//
void 		ATI();							// ��ȡEC200U-CN SN�룬��Ϊ�豸��SN��
StrtokStr 	StrtokString(uint8_t *string);	// �ַ�������������,
void 		Upgrade_Warning();				// �����У���ʾ
TimeType 	Get_CurrentTime_Stamp();		// ��ȡ��ǰʱ���
void 		Get_Device_Sign_Md5(uint8_t *sign);	// ��ȡ�豸��md5���ܺ��sign����
#endif /* UPGRADE_COMMON_H_ */
