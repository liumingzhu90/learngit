/*
 * _4g_data_upload.c
 *
 *  Created on: 2021-12-13
 *      Author: Administrator
 */

#include "_4g_data_upload.h"
#include "usart.h"
#include "stdlib.h"
//#include "utc_time.h"
#include "w25qxx.h"
#include "md5.h"
#include "common.h"
#include "gpio.h"
#include "EC200U.h"
#include "cqueue.h"

typedef struct _Event_Single{
	FunctionalState isStart;
	uint16_t		cnt;		// 发送计数
	FunctionalState	interval;	// 时间间隔
}Event_Single;
typedef struct _AEB_Ev_Single{
	Event_Single 	biCamera;	// 双目
	Event_Single 	mmw;		// 毫米波
	Event_Single 	ultrasonic;	// 超声波
	uint8_t 		endFlag;
}AEB_EV_Single;
typedef struct _Event_4G{
	Event_Single 	fcw;
	Event_Single 	hmw;
	Event_Single 	ldw;
	AEB_EV_Single 	aeb;	// 双目|毫米波|超声波 触发
	TRIGGER_EVENT	event;
	uint32_t 		clk;	//	时钟
}Event_4G;
typedef enum{
	NO_GPS,
	NMEA,
	GLOC
}GPS_Status;
typedef struct _GPS_4G{
	GPS_INFO		info;
	GPSNMEA_TYPE 	nmea_type;
	uint8_t			rx_data[100];
	GPS_Status 		gps_status;			//解析正常
	GPS_Stage		stage;
	uint32_t 		clk;	//	时钟
	uint8_t 		times;
}GPS_4G;
// 数据上传
LinkQueue 	dataUp_q;
Data_Upload data_up 			= {0};
Event_4G 	event_4g 			= {0};		// 预警中的触发事件类型
uint8_t 	cmd[30]				= {0};		// 发送命令
FunctionalState queue_send_t 	= FALSE;	// queue send times
FunctionalState	queue_remove_f 	= FALSE;	// queue remove front flag
Queue q;

// GPS
GPS_4G		gps_4g				={0};
GPS_Stage	gps_flag_tmp 		= NO_STATE;	// 用于备份GPS当前状态
uint8_t 	gpgga_string[100] 	= {0};
uint8_t		BreakState 			= 0;
uint8_t 	reConnect 			= 0;	// 服务器重新链接
uint8_t 	reConnect_Time		= 10;	// (10 20 40 10 20 40...)次-->(5 10 20 5 10 20...)min
// 数据上传
void 	In_Queue_Upload_Data();
void 	Uploading_VehicleBaseInfo();
void 	Uploading_VechileBodyInfo();
void 	Uploading_WarningInfo();

void 	Uploading_Data(uint8_t *data);
uint8_t Uploading_VehicleBaseInfo_Cmd();
uint8_t Uploading_VechileBodyInfo_Cmd();
void 	Uploading_WarningInfo_Cmd(TRIGGER_EVENT event,FunctionalState startEnd,uint16_t event_id);

void 	Upload_Warn_Condition(uint8_t condition,Event_Single *event_h,TRIGGER_EVENT event);
void 	Upload_Warn_Execute(Event_Single *event_h,TRIGGER_EVENT event);

// GPS
void 	Uploading_GPSInfo();
void 	Enable_GNSS();
void 	Disable_GNSS();
void 	Enable_QGPSNMEA();
void 	Enable_QGPSNMEATYPE();
void 	Disable_QGPSNMEA();
void 	Analysis_GPGGA_String(uint8_t *string);
void 	Analysis_GPRMC_String(uint8_t *string);
uint8_t Analysis_ManalCmd(char *str,uint8_t *m_cmd);
void 	Clear_Gps_Info();
void 	Send_Gps_Type_Cmd(GPSNMEA_TYPE type);
void 	Uploading_GPSInfo_Cmd();
float 	GPS_Format_To_Degree(uint8_t *dms);
FunctionalState GPGGA_Logic(char *strx);
FunctionalState GPRMC_Logic(char *strx);
FunctionalState GPGSV_Logic(char *strx);
FunctionalState GPGSA_Logic(char *strx);
FunctionalState GPVTG_Logic(char *strx);
FunctionalState GPSLOC_Logic(char *strx);

/****************************数据上传****************************************/
/*
 * 上传数据
 */
void Uploading_Data(uint8_t *data)
{
	Clear_Buffer();
//	fprintf(USART1_STREAM,">%s",data);
	EC200U_SendData(data,strlen(data));
}
/*
 * 上传车辆、设备基本数据信息
 * 参数：
 * 返回：
 */
uint8_t Uploading_VehicleBaseInfo_Cmd()
{
	// 设备状态故障码, 默认值为0（正常）, 参考双目设备故障码协议暂定
	uint8_t deviceStatus = 0;
	if(camera_data.ErrorCode == 0){								// 无异常，检测通讯异常
		if(stCanCommSta.stRadar.status == OFFLINE)	deviceStatus = 128;	// 毫米波通讯异常
		if(stCanCommSta.stOBD.status == OFFLINE)	deviceStatus = 129;	// OBD通讯异常
		if(stCanCommSta.stHRadar.status == OFFLINE)	deviceStatus = 130;	// 超声波通讯异常
		if(stCanCommSta.stVehicle.status == OFFLINE)deviceStatus = 131;	// AEBS控制器内部通信异常
		if(stCanCommSta.stWireless.status == OFFLINE)deviceStatus = 132;// AEBS与GPS通讯(4G模块)异常
	}else{
		deviceStatus = camera_data.ErrorCode;
	}

	uint8_t AebsStatus = 0;										// AEB制动状态
	if(stVehicleParas.BreakState == 0x01) 	AebsStatus = 1;		// 【1：双目制动】
	if(stVehicleParas.BreakState == 0x02) 	AebsStatus = 2;		// 【2：毫米波制动】
	if(stVehicleParas.BreakState == 0x04) 	AebsStatus = 3;		// 【3：超声波制动】
	if(stVehicleParas.BreakState == 0) 		AebsStatus = 0;		// 【0：无制动】
	uint8_t AebsOn 			= Read_AEB_switch() << 5;
	uint8_t SignalRightOn 	= stVehicleParas.RightFlagTemp;
	uint8_t SignalLeftOn	= stVehicleParas.LeftFlagTemp;
	uint8_t SignalBreakOn 	= stVehicleParas.BrakeFlag;
	uint8_t GearStatus 		= stVehicleParas.Car_Gear;
	float Mileage 			= 0.0;//stVehicleParas.VehicleOdom_Total/1000.0;	// 里程数,KM
	float Lat 				= GPS_Format_To_Degree(gps_4g.info.lat);	// 纬度
	float Lon 				= GPS_Format_To_Degree(gps_4g.info.lon);	// 经度
	float Speed 			= stVehicleParas.fVehicleSpeed;			// m/s转成km/h

	sprintf(data_up.vBase.data,"\{\"method\":\"vehicleBaseInfo\",\"params\":{\"lat\":%0.6f,\"lon\":%0.6f,\"date\":%ld,\"itinerary_id\":\"%s\",\"sn\":\"%s\",\"speed\":%0.2f,"
				"\"deviceStatus\":%d,\"AebsStatus\":%d,\"AebsOn\":%d,\"SignalRightOn\":%d,\"SignalLeftOn\":%d,\"SignalBreakOn\":%d,\"GearStatus\":%d,"
				"\"mileage\":%0.2f},\"id\":%d}\r\n",Lat,Lon,utctime,upgrade_p.info.itinerary_id,upgrade_p.info.sn,Speed,deviceStatus,AebsStatus,AebsOn,SignalRightOn,SignalLeftOn,
				SignalBreakOn,GearStatus,Mileage,JSON_ID+1);

	sprintf(cmd,"AT+QISEND=%d,%d\r\n",CONNECT_ID,strlen(data_up.vBase.data));
//	fprintf(USART1_STREAM,"base:%s\r\n",cmd);
	insertQueue(&dataUp_q, cmd,data_up.vBase.data);
//	Clear_Buffer();
//	EC200U_SendData(cmd,strlen(cmd));

	return 1;
}

/*
 * 上传车身信息
 * 参数：
 * 返回：
 */
uint8_t Uploading_VechileBodyInfo_Cmd()
{
//	uint8_t sign[34] = {0};
//	Get_Device_Sign_Md5(sign);								// 获取签名

	// 车速信息，km/h
	float speed 			= stVehicleParas.fVehicleSpeed;			// m/s转成km/h
	uint8_t AebsOn 			= Read_AEB_switch();
	uint8_t SignalRightOn 	= stVehicleParas.RightFlagTemp;				// 右车道偏离
	uint8_t SignalLeftOn 	= stVehicleParas.LeftFlagTemp;				// 左车道偏离
	uint8_t SignalBreakOn 	= stVehicleParas.BrakeFlag&0x01;			// 刹车灯状态
	uint8_t GearStatus 		= stVehicleParas.Car_Gear;					// 档位【0：无效】、【1：N档】、【2：D档】、【3：R档】
	// 设备状态故障码, 默认值为0（正常）, 参考双目设备故障码协议暂定
	uint8_t deviceStatus = 0;
	if(camera_data.ErrorCode == 0){										// 无异常，检测通讯异常
		if(stCanCommSta.stRadar.status == OFFLINE)	deviceStatus = 128;	// 毫米波通讯异常
		if(stCanCommSta.stOBD.status == OFFLINE)	deviceStatus = 129;	// OBD通讯异常
		if(stCanCommSta.stHRadar.status == OFFLINE)	deviceStatus = 130;	// 超声波通讯异常
		if(stCanCommSta.stVehicle.status == OFFLINE)deviceStatus = 131;	// AEBS控制器内部通信异常
		if(stCanCommSta.stWireless.status == OFFLINE)deviceStatus = 132;// AEBS与GPS通讯(4G模块)异常
	}else
		deviceStatus 		= camera_data.ErrorCode;

	uint8_t throttle 		=  stVehicleParas.ThrottleOpening;		// 油门开度
	uint8_t brake 			= stVehicleParas.BrakeOpening;			// 刹车踏板开度
	uint16_t steeringWheel 	= stSWAParas.SWADegree;					// [0,2000]
	uint8_t steeringWheelStatus = stSWAParas.Direction;				// 方向盘角度
	float lat 				= GPS_Format_To_Degree(gps_4g.info.lat);// 纬度
	float lon 				= GPS_Format_To_Degree(gps_4g.info.lon);// 经度

	sprintf(data_up.vBody.data,"\{\"method\":\"signInfo\",\"params\":{\"lat\":%0.6f,\"lon\":%0.6f,\"date\":%ld,\"itinerary_id\":\"%s\",\"sn\":\"%s\",\"speed\":%0.2f,"
			"\"AebsOn\":%d,\"SignalRightOn\":%d,\"SignalLeftOn\":%d,\"SignalBreakOn\":%d,\"GearStatus\":%d,\"deviceStatus\":%d,"
			"\"throttle\":%d,\"brake\":%d,\"steeringWheel\":%d,\"steeringWheelStatus\":%d},\"id\":%d}\r\n",lat,lon,utctime,upgrade_p.info.itinerary_id,
			upgrade_p.info.sn,speed,AebsOn,SignalRightOn,SignalLeftOn,SignalBreakOn,GearStatus,deviceStatus,throttle,brake,steeringWheel,steeringWheelStatus,JSON_ID+1);

	sprintf(cmd,"AT+QISEND=%d,%d\r\n",CONNECT_ID,strlen(data_up.vBody.data));
//	fprintf(USART1_STREAM,"body:%s\r\n",cmd);
	insertQueue(&dataUp_q, cmd,data_up.vBody.data);
//	EC200U_SendData(cmd,strlen(cmd));
	return 1;
}

/*
 * 上传数据初始化
 */
void _4G_Data_Uploading_Init()
{
	initQueue(&dataUp_q,10);				// init data upload queue
	// 上传--预警
	event_4g.fcw.isStart 	= FALSE;
	event_4g.fcw.interval	= FALSE;
	event_4g.fcw.cnt 		= 0;

	event_4g.hmw.isStart 	= FALSE;
	event_4g.hmw.interval	= FALSE;
	event_4g.hmw.cnt 		= 0;

	event_4g.ldw.isStart 	= FALSE;
	event_4g.ldw.interval	= FALSE;
	event_4g.ldw.cnt 		= 0;

	event_4g.aeb.biCamera.isStart 	= FALSE;
	event_4g.aeb.biCamera.interval	= FALSE;
	event_4g.aeb.biCamera.cnt 		= 0;

	event_4g.aeb.mmw.isStart 		= FALSE;
	event_4g.aeb.mmw.interval		= FALSE;
	event_4g.aeb.mmw.cnt 			= 0;

	event_4g.aeb.ultrasonic.isStart = FALSE;
	event_4g.aeb.ultrasonic.interval= FALSE;
	event_4g.aeb.ultrasonic.cnt 	= 0;

	// 数据上传
	data_up.vBase.interval 	= FALSE;
	data_up.vBody.interval 	= FALSE;
	data_up.warn.interval 	= FALSE;
	data_up.gps.interval 	= FALSE;
	data_up.vBase.trigger	=  FALSE;
	data_up.vBody.trigger	=  FALSE;
	data_up.warn.trigger	=  FALSE;
	data_up.gps.trigger		=  FALSE;
	memset(data_up.vBase.data,0,sizeof(data_up.vBase.data));
	memset(data_up.vBody.data,0,sizeof(data_up.vBody.data));
	memset(data_up.warn.data,0,sizeof(data_up.warn.data));
	memset(data_up.gps.data,0,sizeof(data_up.gps.data));
	// gps
	Set_NMEA_Way(_GPGGA);		// set GPS
	gps_4g.stage 			= NO_STATE;
	gps_4g.times			= 0;
	memset(gps_4g.rx_data,0,sizeof(gps_4g.rx_data));

	gps_4g.info.status		= 0;
	gps_4g.info.status 		= 0;
	gps_4g.info.sateNum 	= 0;
	gps_4g.info.fs 			= 0;
	gps_4g.info.hdop 		= 0.0;
	gps_4g.info.altitude 	= 0.0;
	gps_4g.info.sog 		= 0.0;
	gps_4g.info.kph 		= 0.0;
	memset(gps_4g.info.lat,0,sizeof(gps_4g.info.lat));
	memset(gps_4g.info.lon,0,sizeof(gps_4g.info.lon));
	memset(gps_4g.info.head,0,sizeof(gps_4g.info.head));
}
/*
 * 用队列方式上报数据
 * 说明：刷新频率：50Hz
 */
uint32_t queue_clk = 0;
uint8_t  queue_time = 0;
void In_Queue_Upload_Data()
{
	if(SystemtimeClock - queue_clk >= 20){
		queue_clk = SystemtimeClock;
		if(!queue_remove_f){
			if(!queue_send_t){
				if(!isEmpty(&dataUp_q)){
					queue_send_t = TRUE;
					q = dataUp_q.front->next;
//					fprintf(USART1_STREAM,"info:%s",q->data_info);
					Clear_Buffer();
					EC200U_SendData(q->data_info,strlen(q->data_info));
				}
			}else if(queue_send_t){
				queue_send_t = FALSE;
				queue_remove_f = TRUE;
//				fprintf(USART1_STREAM,"data:%s",q->data);
				Clear_Buffer();
				EC200U_SendData(q->data,strlen(q->data));
			}
		}else{
			queue_time++;
			if(queue_time >= 50){	// 1S 还没有 重发，就重发
				queue_time = 0;
				queue_remove_f = FALSE;
//				fprintf(USART1_STREAM,"Time's up.\r\n");
//				traversal(dataUp_q);
			}
		}
	}
}
/*
 * 数据上传入口
 * 说明：条件：在OTA升级完成后才执行
 */
void _4G_Data_Uploading_In_MainLoop()
{
	// 若开启使能了OTA升级，就等待完毕后在执行GPS获取和数据上报工作；
	// 否则仅判断是否已连接服务器
	if(server_p.startOTA){
		if(upgrade_p.isFinish && server_p.sevIsConnect){
			Gps_Interaction_And_Obtain_Info();// GPS获取信息
			Uploading_VehicleBaseInfo();	// 车基本数据 10min
			Uploading_VechileBodyInfo();	// 车身信息 触发
			Uploading_WarningInfo();		// 预警 触发
			Uploading_GPSInfo();			// GPS信息 30s

			if(strlen(EC200U_Rxbuffer) > 5 ){
				GPS_Analysis_In_USART0IT();
			}
		}
	}else{
		if(server_p.sevIsConnect){
			Gps_Interaction_And_Obtain_Info();// GPS获取信息
			Uploading_VehicleBaseInfo();	// 车基本数据 10min
			Uploading_VechileBodyInfo();	// 车身信息 触发
			Uploading_WarningInfo();		// 预警 触发
			Uploading_GPSInfo();			// GPS信息 30s

			if(strlen(EC200U_Rxbuffer) > 5 ){
				GPS_Analysis_In_USART0IT();
			}
		}
	}
	// 所有上报数据都采用了队列方式上报
	In_Queue_Upload_Data();
}
/*
 * 车辆基本信息上传
 * 说明：时间间隔10min
 */
void Uploading_VehicleBaseInfo()
{
	// 发送指令 600000
	if(SystemtimeClock - data_up.vBase.clk >= 600000){	// 10min=600s
		data_up.vBase.clk 		= SystemtimeClock;
//		data_up.vBase.interval 	= TRUE;
		Uploading_VehicleBaseInfo_Cmd();
	}
	// 发送数据
//	if(data_up.vBase.interval == TRUE){
//		if(SystemtimeClock - data_up.vBase.clk >= 10){
//			data_up.vBase.clk 		= SystemtimeClock;
//			data_up.vBase.interval 	= FALSE;
//			Uploading_Data(data_up.vBase.data);
//		}
//	}
}

/*
 * 车身信息上传
 * 说明：触发条件,触发后就上报一次
 */
FunctionalState tureLight_one 	= FALSE;	// 转向灯事件，车身信息发送一次
FunctionalState brake_one 		= FALSE;	// 踩刹车事件，车身信息发送一次
void Uploading_VechileBodyInfo()
{
	if(SystemtimeClock - data_up.vBody.clk  < 50) return ;	// 检测频率20Hz
	data_up.vBody.clk  = SystemtimeClock;

//	static uint8_t GearStatus 	= 0;
//	static uint32_t	swa_degree	= 0;
	// 条件:转向灯
	if(stVehicleParas.LeftFlagTemp || stVehicleParas.RightFlagTemp){
		if(!tureLight_one){
			tureLight_one = TRUE;
			data_up.vBody.trigger = TRUE;
		}
	}else
		tureLight_one = FALSE;

	// 条件:刹车灯亮(踩下刹车)
	if(stVehicleParas.BrakeFlag&0x01){
		if(!brake_one){
			brake_one = TRUE;
			data_up.vBody.trigger = TRUE;
		}
	}else
		brake_one = FALSE;

//	// 条件：换挡
//	if(stVehicleParas.Car_Gear != GearStatus){
//		GearStatus = stVehicleParas.Car_Gear;
//		data_up.vBody.trigger = TRUE;
//	}
//	// 条件：方向盘,允许误差在[-10,10]角度
//	if(stSWAParas.SWADegree-swa_degree>10 || swa_degree-stSWAParas.SWADegree>10){
//		swa_degree = stSWAParas.SWADegree;
//		data_up.vBody.trigger = TRUE;
//	}
	// 发送
	if(data_up.vBody.trigger){
//		fprintf(USART1_STREAM,"Uploading_VechileBodyInfo()\r\n");
		// 发送指令
//		if(data_up.vBody.interval == FALSE){
			Uploading_VechileBodyInfo_Cmd();
//			data_up.vBody.interval 	= TRUE;
//		}else{
//			// 发送数据
			data_up.vBody.trigger 	= FALSE;
//			data_up.vBody.interval 	= FALSE;
//			Uploading_Data(data_up.vBody.data);
//		}
	}
}
/*
 * 上传GPS信息
 * 参数：无
 * 返回：0无异常；1异常
 * 说明：间隔为30秒
 */
void Uploading_GPSInfo()
{
	// 没有定位，不发送
	if(strlen(gps_4g.info.lat) < 5) return ;
	// 发送指令
	if(SystemtimeClock - data_up.gps.clk > 30000){	// 30000
		data_up.gps.clk = SystemtimeClock;
		// 判断超时,5min,10min,20min,5min,10min,20min...
		if(reConnect > reConnect_Time){		// 5分钟没有连接，就重连
			fprintf(USART1_STREAM,"reconnect server.\r\n");

			if(reConnect_Time >= 40) reConnect_Time = 10;	// 10 20 40
			else reConnect_Time *= 2;

			reConnect = 0;
			Module_4G_ReStart();
			Connect_Server_Start(FALSE);
			return ;
		}else reConnect++;
		// 上报GPS
		Uploading_GPSInfo_Cmd();
//		data_up.gps.interval = TRUE;
	}
	// 发送数据
//	if(data_up.gps.interval == TRUE){
//		if(SystemtimeClock - data_up.gps.clk >= 10){
//			data_up.gps.clk = SystemtimeClock;
//			data_up.gps.interval = FALSE;
//			Uploading_Data(data_up.gps.data);
//		}
//	}
}

/*
 * 数据上传，判断条件
 * 参数：触发条件，事件句柄指针，事件
 */
void Upload_Warn_Condition(uint8_t condition,Event_Single *event_h,TRIGGER_EVENT event)
{
	if(condition && !event_h->isStart){			// start
		event_h->isStart 	= TRUE;
		event_h->interval	= TRUE;
		event_4g.event	 	= event;
		event_h->cnt++;
		if(event_h->cnt == 65536) event_h->cnt = 0;		// 防止计数溢出
	}else if(!condition && event_h->isStart){		// end
		event_h->isStart 	= FALSE;
		event_h->interval	= TRUE;
		event_4g.event 		= event;
	}
}
/*
 * 数据上传，对条件执行
 * 参数：事件句柄指针，事件
 */
void Upload_Warn_Execute(Event_Single *event_h,TRIGGER_EVENT event)
{
	if(event_h->interval){
		if(event>AEB_EVENT) event = AEB_EVENT;
		Uploading_WarningInfo_Cmd(event, event_h->isStart,event_h->cnt);
		event_h->interval = FALSE;
	}else{
//		Uploading_Data(data_up.warn.data);
		event_4g.event = NO_EVENT;
		if(event > AEB_EVENT)
			event_4g.aeb.endFlag = 0;
	}
}
/*
 * 上传预警信息
 * 说明：通过4G发送，一条信息需要发送2条指令;
 */
uint8_t aeb_end_flag = 0;
void Uploading_WarningInfo()
{
	if(SystemtimeClock - event_4g.clk < 50) return ;
	event_4g.clk = SystemtimeClock;

	// 条件
	if(event_4g.event == NO_EVENT){
		Upload_Warn_Condition(camera_data.FCWStatus,&event_4g.fcw,FCW_EVENT);
		Upload_Warn_Condition(camera_data.HMWGrade,&event_4g.hmw,HMW_EVENT);
		Upload_Warn_Condition((camera_data.LeftLDW || camera_data.RightLDW),&event_4g.ldw,LDW_EVENT);
		// AEB中分为双目制动、毫米波雷达制动、超声波雷达制动
		BreakState = stVehicleParas.BreakState;
		switch(BreakState){
		case 0:	// 双目 | 毫米波 | 超声波 end
			if(event_4g.aeb.biCamera.isStart){
				event_4g.aeb.endFlag = 1;
				Upload_Warn_Condition(0,&event_4g.aeb.biCamera,AEB_END_EVENT);
			}else if(event_4g.aeb.mmw.isStart){
				event_4g.aeb.endFlag = 2;
				Upload_Warn_Condition(0,&event_4g.aeb.mmw,AEB_END_EVENT);
			}else if(event_4g.aeb.ultrasonic.isStart){
				event_4g.aeb.endFlag = 3;
				Upload_Warn_Condition(0,&event_4g.aeb.ultrasonic,AEB_END_EVENT);
			}
			break;
		case 1:Upload_Warn_Condition(1,&event_4g.aeb.biCamera,AEB_BI_EVENT);break;		// 双目 start
		case 2:Upload_Warn_Condition(2,&event_4g.aeb.mmw,AEB_MMW_EVENT);break;			// 毫米波 start
		case 3:Upload_Warn_Condition(3,&event_4g.aeb.ultrasonic,AEB_ULT_EVENT);break;	// 超声波雷达 start
		}
	}
	// switch 执行
	switch(event_4g.event)
	{
	case FCW_EVENT:	Upload_Warn_Execute(&event_4g.fcw,FCW_EVENT);break;
	case HMW_EVENT:	Upload_Warn_Execute(&event_4g.hmw,HMW_EVENT);break;
	case LDW_EVENT:	Upload_Warn_Execute(&event_4g.ldw,LDW_EVENT);break;
	case AEB_BI_EVENT:Upload_Warn_Execute(&event_4g.aeb.biCamera,AEB_BI_EVENT);break;
	case AEB_MMW_EVENT:Upload_Warn_Execute(&event_4g.aeb.mmw,AEB_MMW_EVENT);break;
	case AEB_ULT_EVENT:Upload_Warn_Execute(&event_4g.aeb.ultrasonic,AEB_ULT_EVENT);break;
	case AEB_END_EVENT:
		switch(event_4g.aeb.endFlag){
		case 1:	Upload_Warn_Execute(&event_4g.aeb.biCamera,AEB_END_EVENT);break;
		case 2:	Upload_Warn_Execute(&event_4g.aeb.mmw,AEB_END_EVENT);break;
		case 3:	Upload_Warn_Execute(&event_4g.aeb.ultrasonic,AEB_END_EVENT);break;
		}
		break;
	}
}

/*
 * 上传预警指令信息
 * 参数：事件，开始|结束,事件ID
 * 返回：无
 */
void Uploading_WarningInfo_Cmd(TRIGGER_EVENT event,FunctionalState startEnd,uint16_t event_id)
{
	float lat 				= GPS_Format_To_Degree(gps_4g.info.lat);// 纬度
	float lon 				= GPS_Format_To_Degree(gps_4g.info.lon);// 经度
	float speed 			= stVehicleParas.fVehicleSpeed;		// m/s转成km/h
	float ttc 				= CameraMessage.ttc;
	float hmw 				= camera_data.HMW;
	float verticalDistance 	= camera_share.ObsInormation.DistanceZ;			//[0,300]
	float verticalSpeed 	= camera_share.ObsInormation.RelativeSpeedZ;	//[-127.93,127.93]
	uint16_t horizontalDistance = (uint16_t)(camera_share.ObsInormation.DistanceX);	//[-150,150] -> [0,300]
	uint8_t throttle 		= stVehicleParas.ThrottleOpening;		// 0～100 %
	uint8_t brake 			=  stVehicleParas.BrakeOpening;			// 0～100 %
	uint16_t steeringWheel 	= stSWAParas.SWADegree;					//[0,2000]
	uint8_t steeringWheelStatus = stSWAParas.Direction&0x01;
	uint8_t LeftLaneStyle 	= camera_LDW_data.LeftLaneStyle;
	uint8_t RightLaneStyle 	= camera_LDW_data.RightLaneStyle;
	uint8_t fcwLevel 		= camera_data.FCWLevel & 0x03;
	uint8_t obstacleType 	= camera_share.ObsInormation.ObstacleType;
	uint8_t warningSide 	= 0;
	if(camera_data.RightLDW) warningSide = 2;
	else if(camera_data.LeftLDW) warningSide = 1;

	// AEB制动状态【0：无制动】、【1：双目制动】、【2：毫米波制动】、【3：超声波制动】
	uint8_t AebsStatus = 0;
	if(BreakState == 1) 	AebsStatus = 1;
	if(BreakState == 2) 	AebsStatus = 2;
	if(BreakState == 3) 	AebsStatus = 3;
	if(BreakState == 0) 	AebsStatus = 0;
//	if(stVehicleParas.BreakState == 1) 	AebsStatus = 1;
//	if(stVehicleParas.BreakState == 2) 	AebsStatus = 2;
//	if(stVehicleParas.BreakState == 3) 	AebsStatus = 3;
//	if(stVehicleParas.BreakState == 0) 	AebsStatus = 0;
	//【0：无效】、【1：N档】、【2：D档】、【3：R档】
	uint8_t GearStatus 		= stVehicleParas.Car_Gear;
	uint8_t warningType 	= event;		// 触发事件 string【1：fcw】、【2：hmw】、【3：ldw】、【4：aeb】
	uint8_t isBegin 		= startEnd;		// 【0：结束】、【1：开始】
	//  超声波距离 范围: 0～250 分米
	uint8_t ultrasonicDistance = stVehicleParas.Ultrasonicdistance;
	uint16_t alarmID 		= event_id;		// 报警id， 同一次报警的开始和结束 报警id应相同

	sprintf(data_up.warn.data,"\{\"method\":\"alarm\",\"params\":{\"lat\":%0.6f,\"lon\":%0.6f,\"date\":%ld,\"itinerary_id\":\"%s\",\"sn\":\"%s\",\"speed\":%0.2f,"
				"\"ttc\":%0.1f,\"hmw\":%0.1f,\"verticalDistance\":%0.2f,\"verticalSpeed\":%0.4f,\"horizontalDistance\":%d,\"throttle\":%d,\"brake\":%d,"
				"\"steeringWheel\":%d,\"steeringWheelStatus\":%d,\"LeftLaneStyle\":%d,\"RightLaneStyle\":%d,\"fcwLevel\":%d,\"obstacleType\":%d,"
				"\"warningSide\":%d,\"AebsStatus\":%d,\"GearStatus\":%d,\"isBegin\":%d,\"warningType\":%d,\"ultrasonicDistance\":%d,"
				"\"alarmID\":%d},\"id\":%d}\r\n",lat,lon,utctime,upgrade_p.info.itinerary_id,upgrade_p.info.sn,speed,ttc,hmw,verticalDistance,verticalSpeed,horizontalDistance,throttle,brake,
				steeringWheel,steeringWheelStatus,LeftLaneStyle,RightLaneStyle,fcwLevel,obstacleType,warningSide,AebsStatus,GearStatus,isBegin,warningType,ultrasonicDistance,alarmID,JSON_ID+1);

	sprintf(cmd,"AT+QISEND=%d,%d\r\n",CONNECT_ID,strlen(data_up.warn.data));

	insertQueue(&dataUp_q, cmd,data_up.warn.data);

//	Clear_Buffer();
//	EC200U_SendData(cmd,strlen(cmd));
}

/*****************************GPS****************************************/
/*
 * 使能GNSS
 */
void Enable_GNSS()
{
	gps_4g.stage 	= ENABLE_GNSS;
	uint8_t cmd[20] = "AT+QGPS=1\r\n";
	Clear_Buffer();
	EC200U_SendData(cmd,strlen(cmd));
}
/*
 * 关闭使能GNSS
 */
void Disable_GNSS()
{
	gps_4g.stage	= DISABLE_GNSS;
	uint8_t cmd[20] = "AT+QGPSEND\r\n";
	Clear_Buffer();
	EC200U_SendData(cmd,strlen(cmd));
}

/*
 * 使能通过AT+QGPSNMEA获取NMEA语句
 */
void Enable_QGPSNMEA()
{
	gps_4g.stage 	= ENABLE_QGPSNMEA;
	uint8_t cmd[40] = "AT+QGPSCFG=\"nmeasrc\",1\r\n";	// gpsnmeatype
	Clear_Buffer();
//	fprintf(USART1_STREAM,"%s",cmd);
	EC200U_SendData(cmd,strlen(cmd));
}
/*
 * 配置 GPS NMEA 语句的数据类型
 */
void Enable_QGPSNMEATYPE()
{
	gps_4g.stage 	= ENABLE_QGPSNMEATYPE;
	uint8_t cmd[40] = "AT+QGPSCFG=\"gpsnmeatype\",31\r\n";	// gpsnmeatype
	Clear_Buffer();
//	fprintf(USART1_STREAM,"%s",cmd);
	EC200U_SendData(cmd,strlen(cmd));
}
/*
 * 禁用通过AT+QGPSNMEA获取NMEA语句
 */
void Disable_QGPSNMEA()
{
	gps_4g.stage	= DISABLE_QGPSNMEA;
	uint8_t cmd[40] = "AT+QGPSCFG=\"nmeasrc\",0\r\n";
	Clear_Buffer();
//	fprintf(USART1_STREAM,"%s",cmd);
	EC200U_SendData(cmd,strlen(cmd));
}

/*
 * 获取GPxxx语句
 */
void Send_Gps_Type_Cmd(GPSNMEA_TYPE type)
{
	uint8_t cmd[20] = {0};
	switch(type)
	{
	case _GPGGA: strcpy(cmd,"AT+QGPSGNMEA=\"GGA\"\r\n");break;
	case _GPRMC: strcpy(cmd,"AT+QGPSGNMEA=\"RMC\"\r\n");break;
	case _GPGSV: strcpy(cmd,"AT+QGPSGNMEA=\"GSV\"\r\n");break;
	case _GPGSA: strcpy(cmd,"AT+QGPSGNMEA=\"GSA\"\r\n");break;
	case _GPVTG: strcpy(cmd,"AT+QGPSGNMEA=\"VTG\"\r\n");break;
	case _GPSLOC: strcpy(cmd,"AT+QGPSLOC=0\r\n");break;
	}
	if(type == _GPSLOC)	gps_4g.stage = GET_QGPSLOC;
	else gps_4g.stage = GET_QGPSNMEA;

	Clear_Buffer();
//	fprintf(USART1_STREAM,"ss>%s",cmd);
	EC200U_SendData(cmd,strlen(cmd));
}
/*
 * 获取GPGGA的协议字符串
 * 说明：协议需要
 */
uint8_t Get_GPGGA_String(uint8_t *string)
{
	if(strlen(gpgga_string) > 60)
	{
//		fprintf(USART1_STREAM,"Get>%s\r\n",gpgga_string);
		strcpy(string,gpgga_string);
	}else{
//		fprintf(USART1_STREAM,"gga string is empty.\r\n");
		string = NULL;
	}
}
/*
 * GPS开始
 */
void GPS_Start()
{
	Set_NMEA_Way(_GPGGA);	// _GPSLOC _GPGGA
	Enable_GNSS();
	reConnect_Time = 10;	// 连接服务器后，必然使能GPS,故在此处复位5分钟检测服务器是否在线机制
}
/*
 * GPS结束
 */
void GPS_End()
{
	Disable_QGPSNMEA();
}

/*
 * 对手动输入GPS指令解析
 * 参数：
 * 返回：0无异常；1异常
 * 说明：
 */
char * g_strx = NULL;
uint8_t GPS_UserCmd_Analysis_In_USART1IT()
{
	// 获取当前GPS的经纬度信息
	if(strstr((const char*)User_Rxbuffer,(const char*)"ZKHYCHK*GPSINFO")){
		Clear_Usart1_Buffer();// 清空接收buffer
		GPS_INFO gpsInfo = Get_GPS_Info();
		uint8_t status[14] = {0};
		switch(gpsInfo.status){
		case 0:strcpy(status,"未定位");break;
		case 1:strcpy(status,"单点定位");break;
		case 2:strcpy(status,"差分定位");break;
		case 3:strcpy(status,"无效PPS");break;
		case 4:strcpy(status,"固定解");break;
		case 5:strcpy(status,"浮点解");break;
		case 6:strcpy(status,"正在估算");break;
		}
		if(gpsInfo.status == 0){
			switch(gpsInfo.fs){
			case 2:strcpy(status,"2D定位");break;
			case 3:strcpy(status,"3D定位");break;
			}
			if(gpsInfo.fs == 0){
				switch(gpsInfo.mode){
				case GPS_AUTO:strcpy(status,"自主定位");break;
				case GPS_DIFF:strcpy(status,"差分");break;
				case GPS_ESTIMATE:strcpy(status,"估算");break;
				case GPS_INVALID:strcpy(status,"数据无效");break;
				}
			}
		}
		if(strcmp(gpsInfo.head,"GPSLOC") == 0){
			strcpy(status,"单点定位");
		}
		fprintf(USART1_STREAM,">> (%0.6f,%s,%0.6f,%s,%s,sat:%d,alt:%0.1f,sog:%0.2f,hdop:%0.2f)\r\n",GPS_Format_To_Degree(gps_4g.info.lon),gpsInfo.ew,GPS_Format_To_Degree(gps_4g.info.lat),gpsInfo.ns,
				status,gpsInfo.sateNum,gpsInfo.altitude,gpsInfo.sog*1852.0/3600,gpsInfo.hdop);
	}else
	//	设置GPS的配置信息
	if((g_strx = strstr((const char*)User_Rxbuffer,(const char*)"ZKHYSET*GPSCONFIG"))){
		uint8_t m_cmd[50] = {0};
		if(Analysis_ManalCmd(g_strx,m_cmd)){
			gps_flag_tmp = gps_4g.stage;
			gps_4g.stage = MANUAL_CMD;
//			fprintf(USART1_STREAM,"%d,%d,m_cmd:%s\r\n",gps_flag_tmp,gps_4g.stage,m_cmd);
			Clear_Buffer();
			EC200U_SendData(m_cmd,strlen(m_cmd));
		}
		Clear_Usart1_Buffer();// 清空接收buffer
	}else
	// 方便预警触发测试 <ZKHYSET*WARNING:0,1>
	// 0FCW,1HMW,2LDW,3AEB,4刹车灯   |   1事件开始，0事件结束
	if((g_strx = strstr((const char*)User_Rxbuffer,(const char*)"ZKHYSET*WARNING"))){
		StrtokStr str = StrtokString(g_strx+16);
		Clear_Usart1_Buffer();// 清空接收buffer
		if(str.cnt == 2){
			uint8_t event	= atoi(str.data[0]);
			uint8_t sw		= atoi(str.data[1]);
//			fprintf(USART1_STREAM,"%d,%d\n",event,sw);
			switch(event){
			case 0:	camera_data.FCWStatus 		= sw;break;// FCW
			case 1:	camera_data.HMWGrade 		= sw;break;// hmw
			case 2:	camera_data.LeftLDW 		= sw;break;// ldw
			case 3:	stVehicleParas.BreakState 	= sw;break;// aeb
			case 4:	stVehicleParas.LeftFlagTemp = sw;break;// 刹车灯
			}
		}
	}
}
/*
 * GPS针对返回信息解析
 */
uint32_t gps_alyis_clk = 0;
void GPS_Analysis_In_USART0IT()
{
	if(SystemtimeClock - gps_alyis_clk < 10) return ;	// 100Hz
	gps_alyis_clk = SystemtimeClock;

	char * strx = NULL;
	if(gps_4g.stage == MANUAL_CMD){	// 手动输入查询的结果，直接输出
		gps_4g.stage = gps_flag_tmp;
		fprintf(USART1_STREAM,">> %s\r\n",EC200U_Rxbuffer);	// 输出给用户
		Clear_Buffer();
		return ;
	}
//	fprintf(USART1_STREAM,"result:%s",EC200U_Rxbuffer);
	// 上传数据，服务器回复，返回值解析 {"result":0,"id":3}
	if((strx = strstr((const char*)EC200U_Rxbuffer,(const char*)"result"))){
		uint8_t successId = atoi(&strx[8]);	// 0
		if(successId == 0){
			reConnect = 0;				// 服务器在线
			if(queue_remove_f){
				queue_remove_f = FALSE;
				deleteQueue(&dataUp_q);
//				fprintf(USART1_STREAM,"------deleteQueue()-------\r\n");
			}
//			fprintf(USART1_STREAM,"server is online.\r\n");
		}
		Clear_Buffer();
	}else
	// AT+QGPSGNMEA: $GPGGA,013857.00,3858.1823,N,11713.5614,E,1,05,3.93,17.3,M,,M,,*7EOK
	if((strx = strstr((const char*)EC200U_Rxbuffer,(const char*)"QGPSGNMEA:"))){	// +QGPSGNMEA:
//		fprintf(USART1_STREAM,"NMEA:%s",strx);
		memcpy(gps_4g.rx_data,strx,strlen(strx));
		gps_4g.gps_status = NMEA;
		Clear_Buffer();
	}else
	// AT+QGPSLOC=0+QGPSLOC: 100603.000,3858.1765N,11713.5745E,2.9,3.6,3,000.00,1.3,0.7,201221,04OK
	if((strx = strstr((const char*)EC200U_Rxbuffer,(const char*)"QGPSLOC:"))){		//+QGPSLOC:
//		fprintf(USART1_STREAM,"GLOC:%s",strx);
		memcpy(gps_4g.rx_data,strx,strlen(strx));
		gps_4g.gps_status = GLOC;
		Clear_Buffer();
	}
}
/*
 * 解析GPS数据
 * 参数：无
 * 返回：
 * 说明：2s检索一次,会在OTA升级完成后，才执行
 */
uint8_t Gps_Interaction_And_Obtain_Info()
{
	if(SystemtimeClock - gps_4g.clk < 2000) return 0;	// 2s
	gps_4g.clk = SystemtimeClock;

//	fprintf(USART1_STREAM,"gps:%s",EC200U_Rxbuffer);
	switch(gps_4g.stage)
	{
	case ENABLE_GNSS:
		server_p.gpsIsConnect = FALSE;
//		fprintf(USART1_STREAM,"%s",EC200U_Rxbuffer);
		// 回话正在进行，就发送获取命令
		if(upgrade_p.recv_ok == SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"504")){
			if(gps_4g.nmea_type == _GPSLOC)
				Send_Gps_Type_Cmd(gps_4g.nmea_type);
			else
				Enable_QGPSNMEA();
		}else{
			if(upgrade_p.recv_ok == SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"OK")){
				if(gps_4g.nmea_type == _GPSLOC)
					Send_Gps_Type_Cmd(gps_4g.nmea_type);
				else
					Enable_QGPSNMEA();
			}else
				Enable_GNSS();
		}
		break;
	case ENABLE_QGPSNMEA:
		server_p.gpsIsConnect = FALSE;
		if(upgrade_p.recv_ok == SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"OK"))
//			Enable_QGPSNMEATYPE();
			Send_Gps_Type_Cmd(gps_4g.nmea_type);
		else
			Enable_QGPSNMEA();
		break;
	case ENABLE_QGPSNMEATYPE:
		server_p.gpsIsConnect = FALSE;
		if(upgrade_p.recv_ok == SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"OK"))
			Send_Gps_Type_Cmd(gps_4g.nmea_type);
		else
			Enable_QGPSNMEATYPE();
	case GET_QGPSNMEA:
		if(gps_4g.gps_status == NMEA){
//			fprintf(USART1_STREAM,"gga:%s\r\n",gps_4g.rx_data);
			gps_4g.gps_status = NO_GPS;
			switch(gps_4g.nmea_type){
			case _GPGGA:GPGGA_Logic(gps_4g.rx_data);break;
			case _GPRMC:GPRMC_Logic(gps_4g.rx_data);break;
			case _GPGSV:GPGSV_Logic(gps_4g.rx_data);break;
			case _GPGSA:GPGSA_Logic(gps_4g.rx_data);break;
			case _GPVTG:GPVTG_Logic(gps_4g.rx_data);break;
			default:break;
			}
		}
		Send_Gps_Type_Cmd(gps_4g.nmea_type);
		break;
	case GET_QGPSLOC:
		if(gps_4g.gps_status == GLOC){
//				fprintf(USART1_STREAM,"gga:%s\r\n",gps_4g.rx_data);
			gps_4g.gps_status = NO_GPS;
			GPSLOC_Logic(gps_4g.rx_data);
		}
		Send_Gps_Type_Cmd(gps_4g.nmea_type);
		break;
	case DISABLE_QGPSNMEA:
		server_p.gpsIsConnect = FALSE;
		if(upgrade_p.recv_ok == SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"OK")){
			Disable_GNSS();
		}else
			Disable_QGPSNMEA();
		break;
	case DISABLE_GNSS:
		server_p.gpsIsConnect = FALSE;
		if(upgrade_p.recv_ok == SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"OK")){
			gps_4g.stage = NO_STATE;
		}else
			Disable_GNSS();
		break;
	default:
		break;
	}

	return 0;
}

/*
 * GPS 上传指令
 */
void Uploading_GPSInfo_Cmd()
{
	uint8_t gps_cmd[20] = {0};

	memset(data_up.gps.data,0,sizeof(data_up.gps.data));
	memset(gps_cmd,0,sizeof(gps_cmd));
	float lat 			= GPS_Format_To_Degree(gps_4g.info.lat);	// 纬度
	float lon 			= GPS_Format_To_Degree(gps_4g.info.lon);	// 经度
	float speed 		= stVehicleParas.fVehicleSpeed;			// m/s转成km/h
	float mileage 		= stVehicleParas.VehicleOdom_Total/1000.0;	// 里程数,KM

	sprintf(data_up.gps.data,"\{\"method\":\"gps\",\"params\":{\"lat\":%0.6f,\"lon\":%0.6f,\"date\":%ld,\"itinerary_id\":\"%s\""
			",\"speed\":%0.2f,\"sn\":\"%s\",\"mileage\":%0.2f},\"id\":%d}\r\n",
			lat,lon,utctime,upgrade_p.info.itinerary_id,speed,upgrade_p.info.sn,mileage,JSON_ID);//
	sprintf(gps_cmd,"AT+QISEND=%d,%d\r\n",CONNECT_ID,strlen(data_up.gps.data));
//	fprintf(USART1_STREAM,"gps:%s\r\n",gps_cmd);
//	fprintf(USART1_STREAM,"%s\r\n",data_up.gps.data);

	insertQueue(&dataUp_q, gps_cmd,data_up.gps.data);
//	Clear_Buffer();
//	EC200U_SendData(gps_cmd,strlen(gps_cmd));
}

/*
 * GPGGA字符串解析
 * 参数：gpgga_string
 * 返回：无
 * 说明：$GPGGA,075147.00,3858.1836,N,11713.5747,E,1,06,2.61,9.8,M,,M,,*4E
 */
void Analysis_GPGGA_String(uint8_t *string)
{
	uint8_t pos = 0;
	uint8_t tmp[100] = {0};
	strcpy(tmp,string);				// 防止将原始字符串给破坏了
	uint8_t *p = strtok(tmp, ",");	// gpgga
	while(p)
	{
		switch(pos){
		case 0:		// $GPGGA
			strcpy(gps_4g.info.head,p+1);
			gps_4g.info.head[strlen(gps_4g.info.head)] = '\0';
			break;
		case 1:		// UTC
			strcpy(gps_4g.info.utc,p);
			gps_4g.info.utc[strlen(gps_4g.info.utc)] = '\0';
			break;
		case 2:		// 纬度
			strcpy(gps_4g.info.lat,p);
			gps_4g.info.lat[strlen(gps_4g.info.lat)] = '\0';
			break;
		case 3:		// N S
			strcpy(gps_4g.info.ns,p);
			gps_4g.info.ns[strlen(gps_4g.info.ns)] = '\0';
			break;
		case 4:		// 经度
			strcpy(gps_4g.info.lon,p);
			gps_4g.info.lon[strlen(gps_4g.info.lon)] = '\0';
			break;
		case 5:		// E W
			strcpy(gps_4g.info.ew,p);
			gps_4g.info.ew[strlen(gps_4g.info.ew)] = '\0';
			break;
		case 6:		// GPS状态
			gps_4g.info.status = atoi(p);
			break;
		case 7:		// 卫星数目
			gps_4g.info.sateNum = atoi(p);
			break;
		case 8:		// 水平精度因子hdop
			gps_4g.info.hdop = atof(p);
			break;
		case 9:		// 海波高度
			gps_4g.info.altitude = atof(p);
			break;
		default:break;
		}
		p = strtok(NULL, ",");
		pos++;
	}
	pos = 0;
	server_p.gpsIsConnect = TRUE;
//	fprintf(USART1_STREAM,"%s\r\n%s\r\n%s\r\n%s\r\n%s\r\n%d\r\n%d\r\n%0.2f\r\n%0.1f\r\n----\r\n",gps_4g.info.head,gps_4g.info.lat,gps_4g.info.lon,gps_4g.info.ns,
//			gps_4g.info.ew,	gps_4g.info.status,gps_4g.info.sateNum,gps_4g.info.hdop,gps_4g.info.altitude);
}
/*
 * GPRMC字符串解析
 * 参数：需字符串
 * 返回：无
 * 说明：$GPRMC,060321.00,A,3858.1798,N,11713.5727,E,0.327,,301021,,,A,V*08
 */
void Analysis_GPRMC_String(uint8_t *string)
{
	uint8_t pos = 0;
	uint8_t *p = strtok(string, ",");//gpgga
	while(p)
	{
		switch(pos){
		case 0:		// $GPRMC
			strcpy(gps_4g.info.head,p+1);
			gps_4g.info.head[strlen(gps_4g.info.head)] = '\0';
			break;
		case 1:		// UTC
			strcpy(gps_4g.info.utc,p);
			gps_4g.info.utc[strlen(gps_4g.info.utc)] = '\0';
			break;
		case 3:		// 纬度
			strcpy(gps_4g.info.lat,p);
			gps_4g.info.lat[strlen(gps_4g.info.lat)] = '\0';
			break;
		case 4:		// N S
			strcpy(gps_4g.info.ns,p);
			gps_4g.info.ns[strlen(gps_4g.info.ns)] = '\0';
			break;
		case 5:		// 经度
			strcpy(gps_4g.info.lon,p);
			gps_4g.info.lon[strlen(gps_4g.info.lon)] = '\0';
			break;
		case 6:		// E W
			strcpy(gps_4g.info.ew,p);
			gps_4g.info.ew[strlen(gps_4g.info.ew)] = '\0';
			break;
		case 7:		// 地面速率，单位(节)
			gps_4g.info.sog = atof(p);
			break;
		case 9:		// 模式指示 （仅 A=自主定位，D=差分，E=估算，N=数据无效
			if(p[0] == 'A'){
				gps_4g.info.mode = GPS_AUTO;
			}else if(p[0] == 'D'){
				gps_4g.info.mode = GPS_DIFF;
			}else if(p[0] == 'E'){
				gps_4g.info.mode = GPS_ESTIMATE;
			}else if(p[0] == 'N'){
				gps_4g.info.mode = GPS_INVALID;
			}
			break;
		}
		p = strtok(NULL, ",");
		pos++;
	}
	pos = 0;
	server_p.gpsIsConnect = TRUE;
//	fprintf(USART1_STREAM,"%s\r\n%s\r\n%s\r\n%s\r\n%s\r\n%s\r\n%0.3f\r\n%d\r\n----\r\n",gps_4g.info.head,gps_4g.info.lat,gps_4g.info.lon,gps_4g.info.ns,gps_4g.info.utc,
//			gps_4g.info.ew,gps_4g.info.sog,gps_4g.info.mode);
}
/*
 * 清空GPS信息变量
 */
void Clear_Gps_Info()
{
	memset(gps_4g.info.lon,  0, sizeof(gps_4g.info.lon));
	memset(gps_4g.info.lat,  0, sizeof(gps_4g.info.lat));
	memset(gps_4g.info.ew,   0, sizeof(gps_4g.info.ew));
	memset(gps_4g.info.ns,   0, sizeof(gps_4g.info.ns));
	memset(gps_4g.info.head, 0, sizeof(gps_4g.info.head));
	memset(gps_4g.info.utc,  0, sizeof(gps_4g.info.utc));
	memset(gpgga_string,  0, sizeof(gpgga_string));
	gps_4g.info.status 		= 0;
	gps_4g.info.sateNum 	= 0;
	gps_4g.info.hdop 		= 0.0;
	gps_4g.info.altitude 	= 0.0;
}
/*
 * 获取GPS的信息
 */
GPS_INFO Get_GPS_Info()
{
	return gps_4g.info;
}
/*
 * 设置 NMEA方式
 * 说明：设置了那种方式，就会循环检测那种方式。中途不能修改
 */
void Set_NMEA_Way(GPSNMEA_TYPE type)
{
	gps_4g.nmea_type = type;
}
/*
 * 获取GGA逻辑
 * 参数：接收字符串
 * 返回：FALSE失败；TRUE成功
 * 说明：$GPGGA,070416.80,3858.1759,N,11713.5803,E,1,04,3.06,-6.6,M,,M,,*65
 */
FunctionalState GPGGA_Logic(char *strx)
{
	if(strx){
//		fprintf(USART1_STREAM,"%s\r\n",strx);
		// 截取$GPGGA字符串
		// +QGPSGNMEA: $GNGGA,021238.30,,,,,0,00,99.99,,,,,,*71 开始时不完整数据
		// +QGPSGNMEA: $GPGGA,103647.0,3150.721154,N,11711.925873,E,1,02,4.7,59.8,M,-2.0,M,,*77
        if(strlen(strx) <= 62){		// 开始未定位时，无定位信息
//        	fprintf(USART1_STREAM,"1>>%s\r\n",gpgga_string);
        }else{
			for(uint8_t i=0; i<strlen(strx)-11; i++){
				if(strx[11+i] != '\r')		// 去掉尾巴
					gpgga_string[i] = strx[11+i];
				else{
					gpgga_string[i] = '\0';
					break;
				}
			}
			Analysis_GPGGA_String(gpgga_string);
			return TRUE;
        }
	}
	return FALSE;
}
/*
 * 获取RMC逻辑
 * 参数：接收字符串
 * 返回：FALSE失败；TRUE成功
 * 说明：$GPRMC,055615.00,A,3858.1794,N,11713.5767,E,1.323,,301021,,,A,V*01
 +QGPSGNMEA: $GPRMC,060321.00,A,3858.1798,N,11713.5727,E,0.327,,301021,,,A,V*08
 */
FunctionalState GPRMC_Logic(char *strx)
{
	if(strx){
		// 截取$GPGGA字符串
		// +QGPSGNMEA:  $GPRMC,055559.87,V,,,,,,,301021,,,N,V*00 开始时不完整数据
		//				$GPRMC,101119.11,V,,,,,,,011121,,,N,V*0C
		// +QGPSGNMEA: $GPRMC,060321.00,A,3858.1798,N,11713.5727,E,0.327,,301021,,,A,V*08
        uint8_t string[100]={0};
		if(strlen(strx) <= 65){					// 不完整的信息
        	return FALSE;
        }else{
			for(uint8_t i=0; i<strlen(strx)-12; i++){
				if(strx[12+i] != '\r')			// 去掉尾巴
					string[i] = strx[12+i];
				else{
					string[i] = '\0';
					break;
				}
			}
			Analysis_GPRMC_String(string);
			return TRUE;
        }
	}
	return FALSE;
}
/*
 * 获取GSV逻辑
 * 参数：接收字符串
 * 返回：FALSE失败；TRUE成功
 * 说明： +QGPSGNMEA: $GPGSV,2,1,07,10,,,41,12,,,37,23,,,26,25,,,33,0*60
		 +QGPSGNMEA: $GPGSV,2,2,07,31,,,34,193,,,32,194,,,35,0*67
+QGPSGNMEA: $GPGSV,2,1,08,12,31,118,42,18,15,194,27,23,71,147,38,25,22,154,32,0*6D
+QGPSGNMEA: $GPGSV,2,2,08,24,48,050,,193,62,100,30,194,42,137,37,199,43,164,31,0*53
 */
FunctionalState GPGSV_Logic(char *strx)
{
	// $GPGSV,2,1,08,23,,,37,12,,,37,18,,,25,25,,,28,0*6F
	// $GPGSV,2,2,08,32,,,27,193,,,30,194,,,37,199,,,31,0*5A
	return FALSE;
}
/*
 * 获取GSA逻辑
 * 参数：接收字符串
 * 返回：FALSE失败；TRUE成功
 * 说明：$GPGSA,A,3,23,12,25,193,194,199,,,,,,,9.29,6.36,6.77,1*2B
 */
FunctionalState GPGSA_Logic(char *strx)
{
	if(strx){
		// 截取$GPGGA字符串
		// +QGPSGNMEA: $GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,1*2D 开始时不完整数据
		// +QGPSGNMEA: $GPGSA,A,3,23,12,25,193,194,199,,,,,,,9.29,6.36,6.77,1*2B
		uint8_t string[70]={0};
		if(strlen(strx) <= 60){					// 不完整的信息
			return FALSE;
		}else{
			for(uint8_t i=0; i<strlen(strx)-12; i++){
				if(strx[12+i] != '\r')			// 去掉尾巴
					string[i] = strx[12+i];
				else{
					string[i] = '\0';
					break;
				}
			}
			// 解析
			uint8_t pos = 0;
			uint8_t *p = strtok(string, ",");
			while(p)
			{
				switch(pos){
				case 0:		// $GPGSA
					strcpy(gps_4g.info.head,p+1);
					gps_4g.info.head[strlen(gps_4g.info.head)] = '\0';
					break;
				case 1:		// 模式，M=手动，A=自动
					if(p[0] == 'M'){
						gps_4g.info.mode = GPS_MANUAL;
					}else if(p[0] == 'A'){
						gps_4g.info.mode = GPS_AUTO;
					}else {
						gps_4g.info.mode = _NO_MODE;
					}
					break;
				case 2:		// 定位类型，1=没有定位，2=2D 定位，3=3D 定位
					gps_4g.info.fs = atoi(p);
					break;
				default:break;
				}
				p = strtok(NULL, ",");
				pos++;
			}
			pos = 0;
			server_p.gpsIsConnect = TRUE;
//			fprintf(USART1_STREAM,"%s\r\n%d\r\n%d\r\n----\r\n",gps_4g.info.head,gps_4g.info.mode,gps_4g.info.fs);
			return TRUE;
		}
	}
	return FALSE;
}
/*
 * 获取VTG逻辑
 * 参数：接收字符串
 * 返回：FALSE失败；TRUE成功
 * 说明： $GPVTG,,T,,M,0.848,N,1.571,K,A*25
 */
FunctionalState GPVTG_Logic(char *strx)
{
	if(strx){
		// 截取$GPGGA字符串
		// +QGPSGNMEA: $GPVTG,,,,,,,,,N*30 开始时不完整数据
		// +QGPSGNMEA: $GPVTG,,T,,M,0.848,N,1.571,K,A*25
        uint8_t string[50]={0};
		if(strlen(strx) <= 40){					// 不完整的信息
        	return FALSE;
        }else{
			for(uint8_t i=0; i<strlen(strx)-12; i++){
				if(strx[12+i] != '\r')			// 去掉尾巴
					string[i] = strx[12+i];
				else{
					string[i] = '\0';
					break;
				}
			}
			// 解析
			uint8_t pos = 0;
			uint8_t *p = strtok(string, ",");
			while(p)
			{
				switch(pos){
				case 0:		// $GPVTG
					strcpy(gps_4g.info.head,p+1);
					gps_4g.info.head[strlen(gps_4g.info.head)] = '\0';
					break;
				case 3:		// 地面速率，单位(节)
					gps_4g.info.sog = atof(p);
					break;
				case 7:		// 模式指示 （仅 A=自主定位，D=差分，E=估算，N=数据无效
					if(p[0] == 'A'){
						gps_4g.info.mode = GPS_AUTO;
					}else if(p[0] == 'D'){
						gps_4g.info.mode = GPS_DIFF;
					}else if(p[0] == 'E'){
						gps_4g.info.mode = GPS_ESTIMATE;
					}else if(p[0] == 'N'){
						gps_4g.info.mode = GPS_INVALID;
					}
					break;
				}
				p = strtok(NULL, ",");
				pos++;
			}
			pos = 0;
			server_p.gpsIsConnect = TRUE;
//			fprintf(USART1_STREAM,"%s\r\n%0.3f\r\n%d\r\n----\r\n",gps_4g.info.head,gps_4g.info.sog,gps_4g.info.mode);
			return TRUE;
        }
	}
	return FALSE;
}
/*
 * 获取VTG逻辑
 * 参数：接收字符串
 * 返回：FALSE失败；TRUE成功
 * 说明：+QGPSLOC: 070807.000,3858.1866N,11713.5805E,2.0,2.4,3,000.00,0.7,0.4,301021,07
 */
FunctionalState GPSLOC_Logic(char *strx)
{
	if(strx){
		// 截取$GPGGA字符串
		// +QGPSLOC: 070807.000,3858.1866N,11713.5805E,2.0,2.4,3,000.00,0.7,0.4,301021,07
		// +QGPSLOC: <UTC>,<latitude>,<longitude>,<HDOP>,<altitude>,<fix>,<COG>,<spkm>,<1>,<date>,<nsat>
		//  		  0		1   		2			3		4		 5		6 	  7		8		9		10
        uint8_t string[80]={0};
		for(uint8_t i=0; i<strlen(strx)-10; i++){
			if(strx[9+i] != '\r')			// 去掉尾巴
				string[i] = strx[9+i];
			else{
				string[i] = '\0';
				break;
			}
		}
		// 解析
		uint8_t pos = 0;
		uint8_t *p = strtok(string, ",");
		while(p)
		{
			switch(pos){
			case 0:		// UTC，引自GPGGA
				strcpy(gps_4g.info.utc,p);
				gps_4g.info.utc[strlen(gps_4g.info.utc)-1] = '\0';
				break;
			case 1:		// 纬度+N
				strncpy(gps_4g.info.lat,p,strlen(p)-1);
				gps_4g.info.lat[strlen(gps_4g.info.lat)] = '\0';
				gps_4g.info.ns[0] = p[strlen(p)-1];
				gps_4g.info.ns[1] = '\0';
				break;
			case 2:		// 经度+E
				strncpy(gps_4g.info.lon,p,strlen(p)-1);
				gps_4g.info.lon[strlen(gps_4g.info.lon)] = '\0';
				gps_4g.info.ew[0] = p[strlen(p)-1];
				gps_4g.info.ew[1] = '\0';
				break;
			case 3:		// HDOP
				gps_4g.info.hdop = atof(p);
				break;
			case 4:		// 海拔
				gps_4g.info.altitude = atof(p);
				break;
			case 5:		// fix
				gps_4g.info.fs = atoi(p);
				break;
//			case 6:		// COG
//				break;
			case 7:		// spkm，对地速度。精确到小数点后一位。单位：千米/时（引自GPVTG语句）。
				gps_4g.info.kph = atof(p);
				break;
			case 8:		// spkn，对地速度。精确到小数点后一位。单位：节（引自GPVTG语句）。
				gps_4g.info.sog = atof(p);
				break;
			case 9:		// date，ddmmyy,引自GPRMC
				strcpy(gps_4g.info.date,p);
				gps_4g.info.date[strlen(gps_4g.info.date)] = '\0';
				break;
			case 10:	// nsat,卫星数量。固定两位数，前导位数不足则补0（引自GPGGA语句）
				gps_4g.info.sateNum = atoi(p);
				break;
			default:break;
			}
			p = strtok(NULL, ",");
			pos++;
		}
		strcpy(gps_4g.info.head,"GPSLOC");
		pos = 0;
		server_p.gpsIsConnect = TRUE;
//		fprintf(USART1_STREAM,"%s\r\n%s\r\n%s\r\n%s\r\n%s\r\n%s\r\n%0.1f\r\n%0.1f\r\n%d\r\n%0.1f\r\n%0.1f\r\n%s\r\n%d\r\n----\r\n",
//				gps_4g.info.head,gps_4g.info.utc,gps_4g.info.lat,gps_4g.info.ns,
//				gps_4g.info.lon,gps_4g.info.ew,gps_4g.info.hdop,gps_4g.info.altitude,gps_4g.info.fs,gps_4g.info.kph,
//				gps_4g.info.sog,gps_4g.info.date,gps_4g.info.sateNum);
		return TRUE;
	}
	return FALSE;
}

uint8_t Analysis_ManalCmd(char *str,uint8_t *m_cmd)
{
	uint8_t flag = 0,j=0;
	for(uint8_t i=0;i<strlen(str);i++){
		if(flag == 0){
			if(str[i] == ':') flag = 1;
		}else if(flag == 1){
			if(str[i] != '>'){
				m_cmd[j] 	= str[i];
				j++;
			}else{
				m_cmd[j] 	= '\r';
				m_cmd[j+1] 	= '\n';
				m_cmd[j+2] 	= '\0';
				flag = 2;
				break;
			}
		}
	}
	if(flag == 2) return 1;
	else return 0;
}

/*
 * GPS格式转换
 * 参数：度分秒参数
 * 说明：将GPS的 度分 格式转化成 度 格式
 * ddmm.mmmm	--->    dd.dddddd
 */
float GPS_Format_To_Degree(uint8_t *dms)
{
	// 拆分数据
	uint8_t dot_id = 0;
	for(uint8_t i=0;i<strlen(dms);i++){
		if(dms[i] == '.'){	// 找到小数点的下标ID
			dot_id = i;
			break;
		}
	}
	uint8_t deg_t[4] = {0};
	uint8_t min_t[10] = {0};
//	uint8_t sec_t[5] = {0};
	switch(dot_id){
	case 3:			// 858.1730
		memcpy(deg_t,dms,1);
		deg_t[1] = '\0';
		break;
	case 4:			// 3858.1730
		memcpy(deg_t,dms,2);
		deg_t[2] = '\0';
		break;
	case 5:			// 11713.5696
		memcpy(deg_t,dms,3);
		deg_t[3] = '\0';
		break;
	}
	memcpy(min_t,dms+dot_id-2,strlen(dms) - dot_id + 2);
	min_t[strlen(dms) - dot_id + 2] = '\0';
//	memcpy(sec_t,dms+dot_id+1,4);
//	sec_t[4] = '\0';
	// 取数据
	int degree = atoi(deg_t);
	float minute = atof(min_t)/60;
//	float second = atof(sec_t)/100;
	// 计算
//	return (float)(degree + minute / 60 + second / 3600);
	return (float)(degree + minute);
}





