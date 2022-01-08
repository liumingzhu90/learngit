/*
 * common.h
 *
 *  Created on: 2021-8-6
 *      Author: wangzhenbao
 */

#ifndef COMMON_H_
#define COMMON_H_
#include "can_task.h"
#include "stereo_camera.h"
#include "data_center_services.h"
#include "canhl.h"
#include "upgrade_common.h"

#define     RADAR_CAN_OFFLINE_PERIOD			3000           /*�״���߳�ʱʱ��1000ms*/
#define     HRADAR_CAN_OFFLINE_PERIOD			3000           /*�������״���߳�ʱʱ��1000ms*/
#define     CAMERA_CAN_OFFLINE_PERIOD			3000            /*����ͷ���߳�ʱʱ��1000ms*/
#define     VEHICLE_CAN_OFFLINE_PERIOD			3000          /*��can���߳�ʱʱ��1000ms*/
#define     VALVE_CAN_OFFLINE_PERIOD			3000            /* ������ȷ�ϵ���ʱ��1000mS */
#define		VALVE_BLUTH_OFFLINE_PERIOD			3000				/*��������ʱ��*/
#define		VALVE_NET_OFFLINE_PERIOD			3000			/*�������ӳ�ʱʱ��*/
//#define		VALVE_

#define 	VEHICLE_SPEED_NIOSE_Q 		0.25
#define 	VEHICLE_SPEED_NIOSE_R 		1
#define 	VEHICLE_MAX_SPEED_ERROR 	0.55f
#define     VEHICLE_MAX_SPEED_VALID     150.0f
#define     VEHICLE_SPEED_JUMP_THRESHOLD     15.0f
#define     VEHICLE_SPEED_JUMP_HOLD_TIME     5000
#define		SAM_ANA_BLINK_PERIOD		50			// �ɼ�ģ��ת���źŵ�����,mS

#define	SPEED_HIGH_BYTE_FISRT			0		// ���ֽ���ǰ
#define	SPEED_LOW_BYTE_FISRT			1		// ���ֽ���ǰ

#define	BLUETOOTH_STREAM				USART4_STREAM

extern uint32_t Time14_CNT;
extern uint32_t SystemtimeClock;
extern Camera_All_Info CameraMessage;
extern Camera_LDW_data camera_LDW_data;	// add lmz 20211011
enum Gear_MODE
{
    D_MODE  = 0x00,              /* ǰ��*/
    N_MODE  = 0x01,              /* ͣ����*/
    P_MODE  = 0x02,             /* ���� */
    R_MODE  = 0x03                /* �յ�*/
};

/* �ṹ��--������Ϣ */
typedef struct
{
	uint32_t 	TimeStamp;		/* time stamp: 0x00000000-0xffffffff,LSB=1[mS] */
	float	fVehicleSpeed;		/* vehicle speed in m/s ,-100-+100*/
	uint8_t 	RightFlag;		/* blink on:0x01;blink off:0x00 */
	uint8_t 	RightFlagTemp;
	uint8_t 	LeftFlag;			/* blink on:0x01;blink off:0x00 */
	uint8_t 	LeftFlagTemp;
	uint8_t 	BrakeFlag;		/* The brake pedal is depressed:0x01 */
	uint8_t	AbsErrorFlag;		/* The ABS error status, 0-No error 1-Error */
	uint8_t		shoudBrakeFlag;
	uint8_t		kongdangFlag;
	uint8_t		BreakState;		//0 ���ƶ���1˫Ŀ�ƶ���2���ײ��ƶ���3�������ƶ�
	uint8_t		BreakUreaderDirection;		//0��Ч	1ǰ��	2�Ҳ�	3��	4���
	uint8_t		BreakUreaderLevel;			//0��Ч	1һ��Ԥ��0.5m		2����Ԥ��1m		3����Ԥ��1.5m
	enum	Gear_MODE	Car_Gear;	//��λ��Ϣ
	uint8_t		ThrottleOpening;	//���ſ���
	uint8_t		BrakeOpening;		//ɲ��̤�忪��
	uint8_t		AEBbriking;			//AEB�ƶ�
	uint8_t		NeutralGear;		//�յ�
	uint8_t		ReverseGear;		//����
	uint8_t		Ultrasonicdistance;	//����������
	uint32_t	VehicleOdom_Total;	// ����̼�-�ۼ�,��λ��
	uint32_t	VehicleOdom_Current;// ����̼�-��ǰ��ʻ��̣���λ��
}_VEHICLE_PARA;

typedef struct
{
	uint32_t 	SWADegree;				//������ת��
	uint8_t		Direction;				//������״̬
}_SWADEGREE_PARA;

typedef struct
{
	uint8_t	can0rate;						//0��125k
											//1��250k
											//2��500k
											//3��1000k
	uint8_t	can1rate;
	uint8_t	can2rate;
	uint8_t	can3rate;
	uint8_t	can4rate;
	uint8_t	can5rate;
	uint8_t rs232_1_rate;					//����1
	uint8_t rs232_2_rate;					//����2
}_CAN_RATA_PARA;

typedef struct
{
	uint8_t	ucControlMode;						//���Ʒ�ʽ��0���Ʊ�������1���ͼ��ٶȣ�2����Ŀ����ٶ�
	uint8_t	ucVehicleWidth;						// ����,��λ ����.
	uint8_t	ucBrakeSense;						// ɲ��������.
	uint8_t	ucDistaceKeep;						// �������ֲ���.
	float	ttc1;								//ttc1ɲ����ֵ
	float	ttc2;								//ttc2ɲ����ֵ
	uint8_t	GearNum;							//ɲ�����ȵ�λ��Ϣ
	float hmw1;
	float hmw2;
}_SYS_PARA;

//--------------------------------------------------------------------------------------------------
// �ṹ��--�������ò���
typedef struct
{
	uint32_t	lId;								// ����ID.
	uint8_t	uSingnalType;							//�ź���Դ	0:CAN���ģ�1��I/O����
	uint8_t	ucByteOrder;						// �ֽ�˳��. 0:���ֽ���ǰ;1:���ֽ���ǰ.
	uint8_t	ucByteLth;							// �ֽ���. ȡֵ1,2.
	uint8_t	ucStartByte;						// ��ʼ�ֽ�. ȡֵ0-7.
	uint16_t	ucCoeffA;							// ϵ��A, ȡֵ1-100.
	uint16_t	ucCoeffB;							// ϵ��B, ȡֵ1-256.
	uint32_t	data;
}_SPEED_PARA;
typedef struct
{
	uint32_t	lId;
	uint8_t		uSingnalType;
	uint8_t		ucStartbit;
	uint8_t		ucBitnum;
	uint32_t	ucFact;
	uint8_t		ucOffset;
}_PARA_TYPE;
//--------------------------------------------------------------------------------------------------
// �ṹ��--ת�����ò���
typedef struct
{
	uint32_t	lId;								// ת��ID.
	uint8_t	Source;							// �ź���Դ. 0:CAN����; 1:I/0����.
	uint8_t	uclByteOrder;
	uint8_t	ucrByteOrder;
	uint8_t	ucByteNo;							// �ֽ����. ȡֵ0-7.
	uint8_t	ucLeftStartBit;						// ��ת�ź���ʼλ. ȡֵ0-7.
	uint8_t	ucRightStartBit;					// ��ת�ź���ʼλ. ȡֵ0-7.
	uint8_t	ucLeftBitLth;						// ��ת�ź�λ��. ȡֵ1-4.

	uint32_t	lId1;								// ת��ID.
	uint8_t	Source1;							// �ź���Դ. 0:CAN����; 1:I/0����.
	uint8_t	ucByteNo1;							// �ֽ����. ȡֵ0-7.

	uint8_t	ucRightBitLth;						// ��ת�ź�λ��. ȡֵ1-4.
	uint8_t	ucLeftValid;						// ��ת�ź���Чֵ.ȡֵ0x00-0x0f.
	uint8_t	ucRightValid;						// ��ת�ź���Чֵ.ȡֵ0x00-0x0f.

	uint32_t	ldata;							//��ת����
	uint32_t	rdata;							//��ת����

}_TURN_PARA;


typedef struct
{
	uint32_t	lId;								// ����ID.
	uint8_t	ucByteNo;							// �ֽ����. ȡֵ0-7.
	uint8_t	ucStartBit;						// �����ź���ʼλ. ȡֵ0-7.
}_WARNING_PARA;

//--------------------------------------------------------------------------------------------------
// �ṹ��--ɲ�����ò���
typedef struct
{
	uint32_t	lId;								// ת��ID.
	uint8_t	Source;							// �ź���Դ. 0:CAN����; 1:I/0����.
	uint8_t	Type;								// ����. 0:��ֵ; 1:�ٷֱ�.
	uint8_t	ucByteNo;							// �ֽ����. ȡֵ0-7.
	uint8_t	ucStartBit;							// �ź���ʼλ. ȡֵ0-7.
	uint8_t	ucBitLth;							// �ź�λ��. ȡֵ1-4.
	uint8_t	ucValid;							// �ź���Чֵ.ȡֵ0x00-0x0f.
	uint8_t	ucCoeffA;							// ϵ��A, ȡֵ1-100.
	uint8_t	ucCoeffB;							// ϵ��B, ȡֵ1-100.
	uint32_t	data;
}_BRAKE_PARA;

typedef struct
{
	uint32_t	lId;								// ������ת��ID.
	uint8_t	ucSource;
	uint8_t	ucByteNo;							// �ֽ����. ȡֵ0-7.
	uint8_t	ucStartBit;							// �ź���ʼλ. ȡֵ0-7.
	uint8_t	ucBitLth;							// �ź�λ��.
	uint16_t	ucFactor;
	uint16_t	ucOffset;
	uint8_t	ucByteOrder;

	uint32_t	SourceID;
	uint32_t	data4;
	//uint32_t	data5;
	//uint32_t	data6;
	//uint32_t	data7;
}_SWA_PARA;

typedef struct
{
	uint32_t	lId;							// Byte[0-3],ABS����ID.
	uint8_t	Source;							// Byte[4],�ź���Դ. 0:I/0����; 1:CAN����; 2:��֧��.
	uint8_t	ucByteNo;							// Byte[5],�ֽ����. ȡֵ0-7.
	uint8_t	ucAbsWarningStartBit;				// Byte[6],Abs�����ź���ʼλ. ȡֵ0-7.
	uint8_t	ucAbsWarningBitLth;					// Byte[7]Abs�����ź�λ��. ȡֵ1-4.
	uint8_t	ucAbsWarningValid;
	// Byte[8]Abs�����ź���Чֵ.ȡֵ0x00-0x0f.
}_ABS_PARA;

typedef struct
{
	uint32_t	wirlessOldtime;				//������
	uint32_t	VehicleOldtime;				//����can
	uint32_t	CameraOldtime;				//����ͷcan
	uint32_t	RadarOldtime;				//�״�can
	uint32_t	HRadarOldtime;				//�������״�can
	uint32_t	SpeedOldtime;				//����can
	uint32_t	ProportOldtime;				//������
}_OLD_TIME;

typedef struct{
	uint8_t		AEBstatus;					//AEBstatus
	uint8_t		LDWstatus;					//����ƫ��
}_WARN_STATUS;


enum Valid_Num
{
	INVALID	= 	0x00,			//��Ч
	VALID	= 	0x01				//��Ч
};

typedef struct
{
	enum Valid_Num	valid;			//�Ƿ���Ч ��ЧΪ1����ЧΪ0	Ĭ��Ϊ1
	uint8_t	quantity;			//������Ĭ��Ϊ1
	enum Valid_Num	Correlation_speed;	//�Ƿ����ٶ������	����Ϊ1��������Ϊ0	Ĭ��Ϊ1
	float	MaxSpeed;		//�����			Ĭ��150.0km/h
	float	MinSpeed;		//��С����			Ĭ��0.0km/h
	enum	Gear_MODE	Car_Gear;	//��λ��Ϣ
}SENSOR_CONFIG;


typedef struct
{
	uint8_t 	Forward_Camera_Num;
	uint8_t		Tail_Carmera_Num;
	uint8_t		Left_Front_Num;
	uint8_t		Right_Front_Num;
	uint8_t		Left_Rear_Num;
	uint8_t		Right_Rear_Num;
}URadar_distribution;

typedef struct
{
	uint8_t				quantity;			//������Ĭ��Ϊ1
	enum Valid_Num 		Car_speed;		//Ϊ1ʱ�복�ٹ�����Ϊ0ʱ���복�ٹ���
	float					Max_distance;		//���ٹ���ʱ��������
	float					Min_distance;		//���ٹ���ʱ����С����
	float					threshold;		//���ٲ�����ʱ��ֵΪ������������
	enum	Valid_Num	Gear_Mode;		//��λ���
	enum	Gear_MODE	Car_Gear;		//������λ
	enum	Valid_Num	Steering_Angle_Mode;		//ת���
	float					Max_Steering_Angle;		//���ת���
	float				Min_Steering_Angle;		//��Сת���
}URADER_CONFIG;

enum _URADER_POSITION
{
	FRONT=0,
	AFTER=1,
	LEFT=2,
	RIGHT=3,
	FLEFT=4,
	FLSIDE=5,
	FLMIDE=6,
	FRIGHT=7,
	FRSIDE=8,
	FRMIDE=9,
	NONEP
};

enum _URADER_STATA
{
	ACCESSIBILITY=0,		//0xfc δ��⵽�ϰ���
	NOURADER=1,				//0xfd	δ�����״�
	NOWORK=2,				//0xfe �״ﲻ��Ҫ����
	URWORK
};

typedef struct
{
	enum _URADER_POSITION Uposition[12];	//�״﷽λ
	uint8_t				Company;			//��λ
	enum _URADER_STATA Urader_work_stata[12];
	float				distance[12];
}URADER_MESSAGE;

typedef struct
{
	uint8_t	SSSlinkage;
	uint8_t SSSlinkageSpeed;
	uint8_t speed25distance;
	uint8_t speed20distance;
	uint8_t speed15distance;
	uint8_t speed10distance;
	uint8_t speed5distance;
	uint8_t speed0distance;
	uint8_t firststage;
	uint8_t secondstage;
}_URADER_SYS_MESSAGE;

typedef struct
{
	uint8_t				FCWlinkage;				//fcw�Ƿ��복�ٹ���
	uint8_t				FCWlinkageSpeed;		//fcw��������
	uint8_t				LDWlinkage;				//ldw�Ƿ��복�ٹ���
	uint8_t				LDWlinkageSpeed;		//ldw��������
	uint8_t				HMWlinkage;				//hmw�Ƿ��������
	uint8_t				HMWlinkageSpeed;		//hmw��������
	uint8_t				AEBlinkage;				//AEB�Ƿ��������
	uint8_t				AEBlinkageSpeed;		//AEB��������
	uint8_t				SSSlinkage;
	uint8_t				SSSlinkageSpeed;
}Alarm_linkage_PARA;

typedef struct
{
	uint8_t first;
	uint8_t second;
	uint8_t three;
	uint8_t four;
	uint8_t five;
	uint8_t six;
	uint8_t seven;
	uint8_t	eight;
}__SET_PARA;

extern __SET_PARA parameterset;
extern struct can_frame can_recv_valve_control;
extern _URADER_SYS_MESSAGE stUraderSysMessage;
extern Alarm_linkage_PARA	stWarninglinkage;
extern URADER_MESSAGE	Urader_Company_Message[2];
extern struct can_frame	stCammeraCanData;
extern struct can_frame  stVehicleCanData;
extern struct can_frame  stVehicleCanSpeedData;
extern struct can_frame  stVehicleCanOdomData;		// ���ô����̼�
extern struct can_frame  stVehicleCanTurnData;
extern struct can_frame  stVehicleCanBreakData;
extern struct can_frame	stVehicleCanAbsData;
extern struct can_frame	stVehicleCanWarningData;
extern struct can_frame	stVehicleCanSWAData;
extern uint8_t 	SN[21];
extern uint8_t		SN_Display[3];

extern uint32_t first_time;
//extern double doubulespeedsensor;
extern _WARN_STATUS	warning_status;
extern Camera_Essential_Data camera_data;
extern Obstacle_Basic_Data obstacle_Basic_data;
extern Obstacle_Information obstacle_cipv_data;
extern _CAN_RATA_PARA		stCanPara;
extern _SYS_PARA			stSysPara;					// ϵͳ���ò����ṹ��
extern _SPEED_PARA			stSpeedPara;					// �������ò����ṹ��
extern _TURN_PARA			stTurnPara;					// ת�����ò����ṹ��
extern _BRAKE_PARA			stBrakePara;					// ɲ�����ò����ṹ��
extern _WARNING_PARA			stWarningPara;					// ����
extern _SWA_PARA			stSwaPara;					// �����̹ս�
extern _ABS_PARA			stAbsPara;					// ABS���ò����ṹ��
extern _VEHICLE_PARA		stVehicleParas;						// ���������ṹ�����
extern _STRUCT_CAN_COMM_STA 	stCanCommSta;
extern _SWADEGREE_PARA		stSWAParas;
extern Camera_Info camera_share;
extern struct can_frame tx_valve_control;
extern struct can_frame rx_obstacle_info_b;
extern struct can_frame	stUraderCanData[4];
//extern float UraderSpeed[5];
extern struct can_frame rx_obstacle_info_tland; //0x7A3
extern struct can_frame rx_obstacle_info_lbll; //0x7A4
extern struct can_frame rx_obstacle_info_lbrl; //0x7A5
extern struct can_frame rx_obstacle_info_rbll; //0x7A6
extern struct can_frame rx_obstacle_info_rbrl; //0x7A7

void Set_ValveComm_StaStamp(uint32_t stamp);	//����can�ź�ʱ��
void Set_Valvespeed_Stamp(uint32_t stamp);	//������Ϣcan�ź�
void Set_ValveProportionalComm_StaStamp(uint32_t stamp);
void Bluetooth_Send_Data(void);
void Send_Display_Message(void);
void GPIO_State(void);
void Get_Flash_Parameter(void);
void Send_Break_Control(void);
void Set_Camera_Stamp(uint32_t stamp);
void DisPlay_Message_Analy(uint8_t *buf);
void Sys_Brake_Ctrl(void);
void Sys_Send_Break(void);
void Check_SysErr_Alarm(void);
void Proprot_RxValve_Data_Analysis(uint32_t ulSysTime);
void Urader_RxValve_Data_Analysis(uint32_t ulSysTime);
void Send_Warning_Message(void);
void Send_Targt_Message(void);			//Ŀ����Ϣ
void Send_Sys_Status(void);			//ϵͳ��Ϣ
void Send_Vehicle_Status(void);			//������Ϣ
void Send_Soft_Version(void);			//�����Ϣ
void GPS_Data_Analysis(uint8_t *data);
void Ureader_receive(struct can_frame rx_frame);
double readSpeedSensorValue(void);
void Delay(volatile uint32_t a, volatile uint32_t b);
BaseType_t Valve_AEBdecActive_Get(struct can_frame *tx_frame);
uint32_t Bluetooth_Set_Vehicle_With(uint8_t *data,uint32_t flag1,uint32_t flag2);
void ProcessHostCmd(void);
void Can_Set_Speed_Parameter(uint8_t *data);
void ReWrite_one(void);
void ReWrite_two(void);
void ReWrite_three(void);
void ReWrite_four(void);
void ReWrite_five(void);
void ReWrite_six(void);
void ReWrite_seven(void);
void ReWrite_eight(void);
void Reaskc_Set_Commond(void);
uint8_t Bluetooth_Get_URadP_Position(uint8_t a,uint8_t b);
void Report_Vehicle_Base_Info(void);
void Report_Warning_Info(void);
void Data_Uploading_In_MainLoop();
void Early_Warning_Info_start(void);
void Early_Warning_Info_start1(uint8_t type,uint8_t Serialnumber);
#endif /* COMMON_H_ */
