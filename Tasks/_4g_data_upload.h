/*
 * _4g_data_upload.h
 *
 *  Created on: 2021-12-13
 *      Author: Administrator
 */

#ifndef _4G_DATA_UPLOAD_H_
#define _4G_DATA_UPLOAD_H_
#include "upgrade_common.h"
/* ------------------------�� ö��------------------------------- */
typedef enum{
	NORTH,
	SOUTH,
	WEST,
	EAST
}Orientation_e;

typedef enum{
	_NO_NMEA,	// ��״̬
	_GPGGA,		// AT+QGPSGNMEA="GGA"
	_GPRMC,		// AT+QGPSGNMEA="GRMC"
	_GPGSV,		// AT+QGPSGNMEA="GSV"
	_GPGSA,		// AT+QGPSGNMEA="GSA"
	_GPVTG,		// AT+QGPSGNMEA="VTG"
	_GPSLOC,	// AT+QGPSLOC=0 ��ȡ��λ��Ϣ
}GPSNMEA_TYPE;
typedef enum{
	_NO_MODE,		// ��״̬
	GPS_AUTO,		// ������λ
	GPS_DIFF,		// ���
	GPS_ESTIMATE,	// ����
	GPS_INVALID,	// ������Ч
	GPS_MANUAL,		// �ֶ�
}GPS_Mode;

#define JSON_ID			2
#define FIVE_MINUTE		10	// 10
/* �����ϴ���Ϣ���Ͷ��� */
typedef enum{
	NO_UPLOAD_TYPE,
	UPLOAD_VBASE,		// ����������Ϣ
	UPLOAD_FCW,
	UPLOAD_LDW,
	UPLOAD_HMW,
	UPLOAD_AEB,
	UPLOAD_BRAKE,		// ɲ����Ϣ
	UPLOAD_DDRIVE,		// Σ�ռ�ʻ��Ϣ
	UPLOAD_VBODY,		// ������Ϣ
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
	MANUAL_CMD,			// �ֶ�����ָ��
}GPS_Stage;

/* ------------------------�ṹ��------------------------------- */
typedef struct _UpData{
	uint8_t 		data[700];	// ����
	uint32_t 		clk;		// ʱ��
	FunctionalState	interval;	// ʱ����
	FunctionalState	trigger;	// ������־
}Up_Data;

typedef struct _DataUpload{
	Up_Data vBase;
	Up_Data vBody;
	Up_Data warn;
	Up_Data gps;
}Data_Upload;

/*
 * GPGGA:head,���ȣ�γ�ȣ�UCTʱ�䣬GPS״̬����������hdop������
 * GPRMC��head,���ȣ�γ�ȣ�UCTʱ�䣬�������ʣ�ģʽ
 * GPGSA:head,ģʽ����λ����
 * GPGSV:û�н���
 * GPVTG:head���������ʣ�ģʽ
 * GPSLOC��head,UTCʱ�䣬���ȣ�γ�ȣ�hdop�����Σ���λ���ͣ��Ե��ٶȣ�date��������
 */
typedef struct _GPS_INFO{
	// GPGGA��Ч��Ϣ+GPRMC
	uint8_t			head[7];		// GPGGA|GPVTG
	uint8_t 		lat[12];		// γ��
	uint8_t 		lon[12];		// ����
	uint8_t 		ns[2];			// �ϱ�
	uint8_t 		utc[10];		// UTCʱ�䣬ʱ�����ʽ
	uint8_t			ew[2];			// ����
	uint8_t 		status;			// GPS״̬��0=δ��λ��1=���㶨λ��2=��ֶ�λ��3=��ЧPPS��4=�̶��⣬5=����⣬6=���ڹ���
	uint8_t			sateNum;		// ������
	float			hdop;			// ˮƽ��������hdop
	float 			altitude;		// ����
	// GPGSA��Ч��Ϣ
//	GPS_Mode 		smode;			// ģʽ��M=�ֶ���A=�Զ�
	uint8_t			fs;				// ��λ���ͣ�1=û�ж�λ��2=2D ��λ��3=3D ��λ
	// GPVTG
//	float 			cogt;	// ���汱Ϊ�ο���׼�ĵ��溽�򣬵�λ���ȣ�����Χ��0~359.999��
//	float 			cogm;	// �Դű�Ϊ�ο���׼�ĵ��溽�򣬵�λ���ȣ�����Χ��0-359.999��
	float			sog;	// �������ʣ���λ���ڣ�;1�ڣ�kn��=1����/Сʱ=��1852/3600��m/s ���ٶȵ�λ
	float 			kph;	// �������ʣ���λ��KM/H������sog����һ����������
	GPS_Mode 		mode;	// ģʽָʾ��A=������λ��D=��֣�E=���㣬N=������Ч��
	// GPRMC
//	float			spd;	// �������ʣ���λ���ڣ�; 1�ڣ�kn��=1����/Сʱ=��1852/3600��m/s ���ٶȵ�λ
//	float			cog;	// ���溽��0~359.99������λ���ȣ������汱Ϊ�ο���׼
	// GPGSV
	// ��GPGSVΪ���Ƿֲ���û�ж��Լ���Ч���ݣ��ʲ�����
	// GPSLOC
//	float 			spkn;	// �Ե��ٶȡ���ȷ��С�����һλ����λ���ڣ�����GPVTG��䣩
	uint8_t 		date[8];// UTC���ڡ���ʽ��ddmmyy������GPRMC��䣩 ������
//	uint8_t 		errcode;// ����������
}GPS_INFO;
/* ------------------------ȫ�ֺ�������------------------------------- */
// �����ϴ�
void 	_4G_Data_Uploading_Init();
void 	_4G_Data_Uploading_In_MainLoop();

// GPS
void 	GPS_Start();
void 	GPS_End();
uint8_t Get_GPGGA_String(uint8_t *string);	// ����$GPGGA�ַ���
uint8_t Gps_Interaction_And_Obtain_Info();
GPS_INFO Get_GPS_Info();
void 	Set_NMEA_Way(GPSNMEA_TYPE type);
uint8_t GPS_UserCmd_Analysis_In_USART1IT();
void 	GPS_Analysis_In_USART0IT();

#endif /* _4G_DATA_UPLOAD_H_ */
