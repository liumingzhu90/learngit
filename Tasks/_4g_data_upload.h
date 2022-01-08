/*
 * _4g_data_upload.h
 *
 *  Created on: 2021-12-13
 *      Author: Administrator
 */

#ifndef _4G_DATA_UPLOAD_H_
#define _4G_DATA_UPLOAD_H_
#include "upgrade_common.h"
/* ------------------------宏 枚举------------------------------- */
typedef enum{
	NORTH,
	SOUTH,
	WEST,
	EAST
}Orientation_e;

typedef enum{
	_NO_NMEA,	// 无状态
	_GPGGA,		// AT+QGPSGNMEA="GGA"
	_GPRMC,		// AT+QGPSGNMEA="GRMC"
	_GPGSV,		// AT+QGPSGNMEA="GSV"
	_GPGSA,		// AT+QGPSGNMEA="GSA"
	_GPVTG,		// AT+QGPSGNMEA="VTG"
	_GPSLOC,	// AT+QGPSLOC=0 获取定位信息
}GPSNMEA_TYPE;
typedef enum{
	_NO_MODE,		// 无状态
	GPS_AUTO,		// 自主定位
	GPS_DIFF,		// 差分
	GPS_ESTIMATE,	// 估算
	GPS_INVALID,	// 数据无效
	GPS_MANUAL,		// 手动
}GPS_Mode;

#define JSON_ID			2
#define FIVE_MINUTE		10	// 10
/* 数据上传信息类型定义 */
typedef enum{
	NO_UPLOAD_TYPE,
	UPLOAD_VBASE,		// 车辆基本信息
	UPLOAD_FCW,
	UPLOAD_LDW,
	UPLOAD_HMW,
	UPLOAD_AEB,
	UPLOAD_BRAKE,		// 刹车信息
	UPLOAD_DDRIVE,		// 危险驾驶信息
	UPLOAD_VBODY,		// 车身信息
}DataUpload_Type;

typedef enum{
	NO_STATE,
	ENABLE_GNSS,
	DISABLE_GNSS,
	ENABLE_QGPSNMEA,
	ENABLE_QGPSNMEATYPE,
	DISABLE_QGPSNMEA,
	GET_QGPSNMEA,
	GET_QGPSLOC,
	MANUAL_CMD,			// 手动输入指令
}GPS_Stage;

/* ------------------------结构体------------------------------- */
typedef struct _UpData{
	uint8_t 		data[700];	// 数据
	uint32_t 		clk;		// 时钟
	FunctionalState	interval;	// 时间间隔
	FunctionalState	trigger;	// 触发标志
}Up_Data;

typedef struct _DataUpload{
	Up_Data vBase;
	Up_Data vBody;
	Up_Data warn;
	Up_Data gps;
}Data_Upload;

/*
 * GPGGA:head,经度，纬度，UCT时间，GPS状态，卫星数，hdop，海拔
 * GPRMC：head,经度，纬度，UCT时间，地面速率，模式
 * GPGSA:head,模式，定位类型
 * GPGSV:没有解析
 * GPVTG:head，地面速率，模式
 * GPSLOC：head,UTC时间，经度，纬度，hdop，海拔，定位类型，对地速度，date，卫星数
 */
typedef struct _GPS_INFO{
	// GPGGA有效信息+GPRMC
	uint8_t			head[7];		// GPGGA|GPVTG
	uint8_t 		lat[12];		// 纬度
	uint8_t 		lon[12];		// 经度
	uint8_t 		ns[2];			// 南北
	uint8_t 		utc[10];		// UTC时间，时分秒格式
	uint8_t			ew[2];			// 东西
	uint8_t 		status;			// GPS状态，0=未定位，1=单点定位，2=差分定位，3=无效PPS，4=固定解，5=浮点解，6=正在估算
	uint8_t			sateNum;		// 卫星数
	float			hdop;			// 水平精度因子hdop
	float 			altitude;		// 海拔
	// GPGSA有效信息
//	GPS_Mode 		smode;			// 模式，M=手动，A=自动
	uint8_t			fs;				// 定位类型，1=没有定位，2=2D 定位，3=3D 定位
	// GPVTG
//	float 			cogt;	// 以真北为参考基准的地面航向，单位（度），范围（0~359.999）
//	float 			cogm;	// 以磁北为参考基准的地面航向，单位（度），范围（0-359.999）
	float			sog;	// 地面速率，单位（节）;1节（kn）=1海里/小时=（1852/3600）m/s 是速度单位
	float 			kph;	// 地面速率，单位（KM/H），与sog性质一样，不解析
	GPS_Mode 		mode;	// 模式指示（A=自主定位，D=差分，E=估算，N=数据无效）
	// GPRMC
//	float			spd;	// 地面速率，单位（节）; 1节（kn）=1海里/小时=（1852/3600）m/s 是速度单位
//	float			cog;	// 地面航向（0~359.99），单位（度），以真北为参考基准
	// GPGSV
	// 因GPGSV为卫星分布，没有对自驾有效数据，故不解析
	// GPSLOC
//	float 			spkn;	// 对地速度。精确到小数点后一位。单位：节（引自GPVTG语句）
	uint8_t 		date[8];// UTC日期。格式：ddmmyy（引自GPRMC语句） 日月年
//	uint8_t 		errcode;// 操作错误码
}GPS_INFO;
/* ------------------------全局函数声明------------------------------- */
// 数据上传
void 	_4G_Data_Uploading_Init();
void 	_4G_Data_Uploading_In_MainLoop();

// GPS
void 	GPS_Start();
void 	GPS_End();
uint8_t Get_GPGGA_String(uint8_t *string);	// 返回$GPGGA字符串
uint8_t Gps_Interaction_And_Obtain_Info();
GPS_INFO Get_GPS_Info();
void 	Set_NMEA_Way(GPSNMEA_TYPE type);
uint8_t GPS_UserCmd_Analysis_In_USART1IT();
void 	GPS_Analysis_In_USART0IT();

#endif /* _4G_DATA_UPLOAD_H_ */
