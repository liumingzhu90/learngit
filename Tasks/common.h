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

#define     RADAR_CAN_OFFLINE_PERIOD			3000           /*雷达掉线超时时间1000ms*/
#define     HRADAR_CAN_OFFLINE_PERIOD			3000           /*超声波雷达掉线超时时间1000ms*/
#define     CAMERA_CAN_OFFLINE_PERIOD			3000            /*摄像头掉线超时时间1000ms*/
#define     VEHICLE_CAN_OFFLINE_PERIOD			3000          /*车can掉线超时时间1000ms*/
#define     VALVE_CAN_OFFLINE_PERIOD			3000            /* 比例阀确认掉线时间1000mS */
#define		VALVE_BLUTH_OFFLINE_PERIOD			3000				/*蓝牙发送时间*/
#define		VALVE_NET_OFFLINE_PERIOD			3000			/*网络链接超时时间*/
//#define		VALVE_

#define 	VEHICLE_SPEED_NIOSE_Q 		0.25
#define 	VEHICLE_SPEED_NIOSE_R 		1
#define 	VEHICLE_MAX_SPEED_ERROR 	0.55f
#define     VEHICLE_MAX_SPEED_VALID     150.0f
#define     VEHICLE_SPEED_JUMP_THRESHOLD     15.0f
#define     VEHICLE_SPEED_JUMP_HOLD_TIME     5000
#define		SAM_ANA_BLINK_PERIOD		50			// 采集模拟转向信号的周期,mS

#define	SPEED_HIGH_BYTE_FISRT			0		// 高字节在前
#define	SPEED_LOW_BYTE_FISRT			1		// 低字节在前

#define	BLUETOOTH_STREAM				USART4_STREAM

extern uint32_t Time14_CNT;
extern uint32_t SystemtimeClock;
extern Camera_All_Info CameraMessage;
extern Camera_LDW_data camera_LDW_data;	// add lmz 20211011
enum Gear_MODE
{
    D_MODE  = 0x00,              /* 前当*/
    N_MODE  = 0x01,              /* 停车档*/
    P_MODE  = 0x02,             /* 倒挡 */
    R_MODE  = 0x03                /* 空挡*/
};

/* 结构体--车辆信息 */
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
	uint8_t		BreakState;		//0 无制动、1双目制动、2毫米波制动、3超声波制动
	uint8_t		BreakUreaderDirection;		//0无效	1前方	2右侧	3后方	4左侧
	uint8_t		BreakUreaderLevel;			//0无效	1一级预警0.5m		2二级预警1m		3三级预警1.5m
	enum	Gear_MODE	Car_Gear;	//挡位信息
	uint8_t		ThrottleOpening;	//油门开度
	uint8_t		BrakeOpening;		//刹车踏板开度
	uint8_t		AEBbriking;			//AEB制动
	uint8_t		NeutralGear;		//空挡
	uint8_t		ReverseGear;		//倒挡
	uint8_t		Ultrasonicdistance;	//超声波距离
	uint32_t	VehicleOdom_Total;	// 车里程计-累计,单位米
	uint32_t	VehicleOdom_Current;// 车里程计-当前行驶里程，单位米
}_VEHICLE_PARA;

typedef struct
{
	uint32_t 	SWADegree;				//方向盘转角
	uint8_t		Direction;				//方向盘状态
}_SWADEGREE_PARA;

typedef struct
{
	uint8_t	can0rate;						//0、125k
											//1、250k
											//2、500k
											//3、1000k
	uint8_t	can1rate;
	uint8_t	can2rate;
	uint8_t	can3rate;
	uint8_t	can4rate;
	uint8_t	can5rate;
	uint8_t rs232_1_rate;					//串口1
	uint8_t rs232_2_rate;					//串口2
}_CAN_RATA_PARA;

typedef struct
{
	uint8_t	ucControlMode;						//控制方式，0控制比例阀，1发送减速度，2发送目标加速度
	uint8_t	ucVehicleWidth;						// 车宽,单位 分米.
	uint8_t	ucBrakeSense;						// 刹车灵敏度.
	uint8_t	ucDistaceKeep;						// 车辆保持参数.
	float	ttc1;								//ttc1刹车阈值
	float	ttc2;								//ttc2刹车阈值
	uint8_t	GearNum;							//刹车力度挡位信息
	float hmw1;
	float hmw2;
}_SYS_PARA;

//--------------------------------------------------------------------------------------------------
// 结构体--车速设置参数
typedef struct
{
	uint32_t	lId;								// 车速ID.
	uint8_t	uSingnalType;							//信号来源	0:CAN报文；1：I/O输入
	uint8_t	ucByteOrder;						// 字节顺序. 0:高字节在前;1:低字节在前.
	uint8_t	ucByteLth;							// 字节数. 取值1,2.
	uint8_t	ucStartByte;						// 起始字节. 取值0-7.
	uint16_t	ucCoeffA;							// 系数A, 取值1-100.
	uint16_t	ucCoeffB;							// 系数B, 取值1-256.
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
// 结构体--转向设置参数
typedef struct
{
	uint32_t	lId;								// 转向ID.
	uint8_t	Source;							// 信号来源. 0:CAN报文; 1:I/0输入.
	uint8_t	uclByteOrder;
	uint8_t	ucrByteOrder;
	uint8_t	ucByteNo;							// 字节序号. 取值0-7.
	uint8_t	ucLeftStartBit;						// 左转信号起始位. 取值0-7.
	uint8_t	ucRightStartBit;					// 右转信号起始位. 取值0-7.
	uint8_t	ucLeftBitLth;						// 左转信号位长. 取值1-4.

	uint32_t	lId1;								// 转向ID.
	uint8_t	Source1;							// 信号来源. 0:CAN报文; 1:I/0输入.
	uint8_t	ucByteNo1;							// 字节序号. 取值0-7.

	uint8_t	ucRightBitLth;						// 右转信号位长. 取值1-4.
	uint8_t	ucLeftValid;						// 左转信号有效值.取值0x00-0x0f.
	uint8_t	ucRightValid;						// 右转信号有效值.取值0x00-0x0f.

	uint32_t	ldata;							//左转数据
	uint32_t	rdata;							//右转数据

}_TURN_PARA;


typedef struct
{
	uint32_t	lId;								// 报警ID.
	uint8_t	ucByteNo;							// 字节序号. 取值0-7.
	uint8_t	ucStartBit;						// 报警信号起始位. 取值0-7.
}_WARNING_PARA;

//--------------------------------------------------------------------------------------------------
// 结构体--刹车设置参数
typedef struct
{
	uint32_t	lId;								// 转向ID.
	uint8_t	Source;							// 信号来源. 0:CAN报文; 1:I/0输入.
	uint8_t	Type;								// 类型. 0:数值; 1:百分比.
	uint8_t	ucByteNo;							// 字节序号. 取值0-7.
	uint8_t	ucStartBit;							// 信号起始位. 取值0-7.
	uint8_t	ucBitLth;							// 信号位长. 取值1-4.
	uint8_t	ucValid;							// 信号有效值.取值0x00-0x0f.
	uint8_t	ucCoeffA;							// 系数A, 取值1-100.
	uint8_t	ucCoeffB;							// 系数B, 取值1-100.
	uint32_t	data;
}_BRAKE_PARA;

typedef struct
{
	uint32_t	lId;								// 方向盘转角ID.
	uint8_t	ucSource;
	uint8_t	ucByteNo;							// 字节序号. 取值0-7.
	uint8_t	ucStartBit;							// 信号起始位. 取值0-7.
	uint8_t	ucBitLth;							// 信号位长.
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
	uint32_t	lId;							// Byte[0-3],ABS报文ID.
	uint8_t	Source;							// Byte[4],信号来源. 0:I/0输入; 1:CAN报文; 2:不支持.
	uint8_t	ucByteNo;							// Byte[5],字节序号. 取值0-7.
	uint8_t	ucAbsWarningStartBit;				// Byte[6],Abs故障信号起始位. 取值0-7.
	uint8_t	ucAbsWarningBitLth;					// Byte[7]Abs故障信号位长. 取值1-4.
	uint8_t	ucAbsWarningValid;
	// Byte[8]Abs故障信号有效值.取值0x00-0x0f.
}_ABS_PARA;

typedef struct
{
	uint32_t	wirlessOldtime;				//无线网
	uint32_t	VehicleOldtime;				//整车can
	uint32_t	CameraOldtime;				//摄像头can
	uint32_t	RadarOldtime;				//雷达can
	uint32_t	HRadarOldtime;				//超声波雷达can
	uint32_t	SpeedOldtime;				//车速can
	uint32_t	ProportOldtime;				//比例阀
}_OLD_TIME;

typedef struct{
	uint8_t		AEBstatus;					//AEBstatus
	uint8_t		LDWstatus;					//车道偏离
}_WARN_STATUS;


enum Valid_Num
{
	INVALID	= 	0x00,			//无效
	VALID	= 	0x01				//有效
};

typedef struct
{
	enum Valid_Num	valid;			//是否有效 有效为1，无效为0	默认为1
	uint8_t	quantity;			//数量，默认为1
	enum Valid_Num	Correlation_speed;	//是否与速度相关联	关联为1，不关联为0	默认为1
	float	MaxSpeed;		//最大车速			默认150.0km/h
	float	MinSpeed;		//最小车速			默认0.0km/h
	enum	Gear_MODE	Car_Gear;	//挡位信息
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
	uint8_t				quantity;			//数量，默认为1
	enum Valid_Num 		Car_speed;		//为1时与车速关联，为0时不与车速关联
	float					Max_distance;		//车速关联时：最大距离
	float					Min_distance;		//车速关联时：最小距离
	float					threshold;		//车速不关联时阈值为触发的最大距离
	enum	Valid_Num	Gear_Mode;		//挡位相关
	enum	Gear_MODE	Car_Gear;		//关联挡位
	enum	Valid_Num	Steering_Angle_Mode;		//转向角
	float					Max_Steering_Angle;		//最大转向角
	float				Min_Steering_Angle;		//最小转向角
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
	ACCESSIBILITY=0,		//0xfc 未检测到障碍物
	NOURADER=1,				//0xfd	未插入雷达
	NOWORK=2,				//0xfe 雷达不需要工作
	URWORK
};

typedef struct
{
	enum _URADER_POSITION Uposition[12];	//雷达方位
	uint8_t				Company;			//单位
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
	uint8_t				FCWlinkage;				//fcw是否与车速关联
	uint8_t				FCWlinkageSpeed;		//fcw关联车速
	uint8_t				LDWlinkage;				//ldw是否与车速关联
	uint8_t				LDWlinkageSpeed;		//ldw关联车速
	uint8_t				HMWlinkage;				//hmw是否关联车速
	uint8_t				HMWlinkageSpeed;		//hmw关联车速
	uint8_t				AEBlinkage;				//AEB是否关联车速
	uint8_t				AEBlinkageSpeed;		//AEB关联车速
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
extern struct can_frame  stVehicleCanOdomData;		// 金旅大巴里程计
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
extern _SYS_PARA			stSysPara;					// 系统设置参数结构体
extern _SPEED_PARA			stSpeedPara;					// 车速设置参数结构体
extern _TURN_PARA			stTurnPara;					// 转向设置参数结构体
extern _BRAKE_PARA			stBrakePara;					// 刹车设置参数结构体
extern _WARNING_PARA			stWarningPara;					// 报警
extern _SWA_PARA			stSwaPara;					// 方向盘拐角
extern _ABS_PARA			stAbsPara;					// ABS设置参数结构体
extern _VEHICLE_PARA		stVehicleParas;						// 车辆参数结构体变量
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

void Set_ValveComm_StaStamp(uint32_t stamp);	//整车can信号时间
void Set_Valvespeed_Stamp(uint32_t stamp);	//车速信息can信号
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
void Send_Targt_Message(void);			//目标信息
void Send_Sys_Status(void);			//系统信息
void Send_Vehicle_Status(void);			//车辆信息
void Send_Soft_Version(void);			//软件信息
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
