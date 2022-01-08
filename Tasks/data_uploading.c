/*
 * data_uploading.c
 *
 *  Created on: 2021-10-10
 *      Author: Administrator
 *  说明：直接与才库4G模块对接，详细协议请查阅《AEBS设备与GPS设备通讯协议V2.4.docx》
 */
#include "data_uploading.h"
#include "common.h"
#include "usart.h"
#include "stdio.h"
#include "gpio.h"
#include "upgrade_common.h"
/**************************宏定义*****************************************/
#define PROTOCOL_FRONT			0x2E
#define VECHICLE_DEVICE_TYPE	0x11
#define EARLY_WARNING_TYPE		0x24
#define CAR_BODY_TYPE			0x30
#define CAR_VELOCITY_TYPE		0x88
#define _0X0F					0x0F
#define _0X3F					0x3F
#define _0XFF					0xFF
//#define _0x3FFF					0x3FFF
/**************************结构体、枚举*****************************************/
typedef struct _CaiKuReply
{
	RetStatus vehicleDevice_res;	// 车体设备基本信息，返回结果
	RetStatus earlyWarning_res;		// 预警信息，返回结果
	RetStatus carBody_res;			// 车体信息，返回结果
	RetStatus gpsVelocity_res;		// GPS车速结果
}CaiKuReply;
/**************************全局变量*****************************************/
uint8_t replay_res[7]			= {0};
uint8_t replay_cnt				= 0;
uint8_t checkout 				= 0;
CaiKuReply carKu_replay			= {0};
uint8_t car_vechicle			= 0;
uint8_t vechicleBodyInfo_flag 	= 0;
enum Gear_MODE Car_Gear_tmp 	= P_MODE;		// 默认P档
uint32_t SWADegree_tmp			= 0;			// 方向盘转向角度
RetStatus gps_reponse	 		= FAILURE;
Event_T event_t 				= {0};			// 才库4G数据上传-事件

uint8_t sysSerialnum = 0;
uint8_t URaderSerialnum = 0;
uint8_t CmeraSerialnum = 0;
uint8_t FcwSerialnum = 0;
uint8_t HmwSerialnum = 0;
uint8_t LeftLdwSerialnum = 0;
uint8_t RightLdwSerialnum = 0;
/**************************函数声明*****************************************/
void Response(uint8_t dataType,RetStatus status);
uint8_t Checkout(uint8_t *data);
void UpLoad_Vehicle_Device_Info();
void UpLoad_Early_Warning_Info();
void Early_Warning_Info();
void Test_Early_Warning_Info();
void Early_Warning_Info_start(void);

#define LED_ON_OFF()    GPIO_Toggle_Output_Data_Config (GPIOE_SFR, GPIO_PIN_MASK_6)

//void delay_ms(volatile uint32_t nms)
//{
//	volatile uint32_t i,j;
//	for(i=0;i<nms;i++)
//	{
//		j=7000;
//		while(j--);
//	}
//}
/*
 * 数据上传-车辆设备基本信息
 * 参数：无
 * 返回：无
 * 说明：频率：100ms
 */
uint32_t dataUploading_TickTime = 0;
void UpLoad_Vehicle_Device_Info()
{
	uint8_t vehicleDevice_info[9] = {PROTOCOL_FRONT};
	uint8_t BreakFlag = 0;

	if(SystemtimeClock - dataUploading_TickTime > 100)
	{
		dataUploading_TickTime = SystemtimeClock;

		vehicleDevice_info[1] = VECHICLE_DEVICE_TYPE;
		vehicleDevice_info[2] = 0x05;
		// 获取车速--字节4
		vehicleDevice_info[3] = (uint8_t)stVehicleParas.fVehicleSpeed;;
		// 获取AEB制动状态--字节5 D8-D7	【0：无制动】、【1：双目制动】、【2：毫米波制动】、【3：超声波制动】

		if((stVehicleParas.BreakState & 0x01) == 0x01)
			BreakFlag = 1;
		if((stVehicleParas.BreakState & 0x02) == 0x02)
			BreakFlag = 1;
		if((stVehicleParas.BreakState & 0x04) == 0x04)
			BreakFlag = 3;
		if(stVehicleParas.BreakState == 0)
			BreakFlag = 0;
//		BreakFlag = 1;
		vehicleDevice_info[4] = BreakFlag << 6;
		// 获取AEB开关状态--字节5 D6	【0：否]、【1：是】
		//vehicleDevice_info[4] |= Read_AEB_switch() << 5;
		// 获取右转向灯状态--字节5 D5	【0：否]、【1：是】
		vehicleDevice_info[4] |= (stVehicleParas.RightFlagTemp&0x01) << 4;
		// 获取左转向灯状态--字节5 D4	【0：否]、【1：是】
		vehicleDevice_info[4] |= (stVehicleParas.LeftFlagTemp&0x01) << 3;
		// 获取刹车灯状态--字节5 D3	【0：否]、【1：是】
		vehicleDevice_info[4] |= (stVehicleParas.BrakeFlag&0x01) << 2;
		// 获取档位状态--字节5 D2-D1	【0：无效】、【1：N档】、【2：D档】、【3：R档】
		vehicleDevice_info[4] |= stVehicleParas.Car_Gear;
		// 获取故障码--字节6
		if(camera_data.ErrorCode == 0){								// 无异常，检测通讯异常
			//if(stCanCommSta.stRadar.status == OFFLINE)				// 毫米波通讯异常
			//	vehicleDevice_info[5] = 128;
			//if(stCanCommSta.stOBD.status == OFFLINE)				// OBD通讯异常
			//	vehicleDevice_info[5] = 129;
			if(stCanCommSta.stHRadar.status == OFFLINE)				// 超声波通讯异常
				vehicleDevice_info[5] = 130;
			if(stCanCommSta.stVehicle.status == OFFLINE)			// AEBS控制器内部通信异常
				vehicleDevice_info[5] = 131;
			//if(stCanCommSta.stWireless.status == OFFLINE)			// AEBS与GPS通讯(4G模块)异常
			//	vehicleDevice_info[5] = 132;
		}else{
			vehicleDevice_info[5] = camera_data.ErrorCode;
		}
		// 预留--字节7
		vehicleDevice_info[6] = _0XFF;
		// 预留--字节8
		vehicleDevice_info[7] = _0XFF;
		// 异或校验--字节9
		for(uint8_t i=0;i<8;i++)
			vehicleDevice_info[8] ^= vehicleDevice_info[i];

//		for(uint8_t i=0;i<9;i++)
//			fprintf(USART0_STREAM,"%02X ",vehicleDevice_info[i]);
//		fprintf(USART0_STREAM,"\r\n");
		// 发送
		USART_Send(USART2_SFR,vehicleDevice_info,9);
		memset(vehicleDevice_info,0,sizeof(vehicleDevice_info));

		// 发送车身信息：打开左右转向灯、制动灯亮、AEBS系统开关
/*
		if(stVehicleParas.RightFlagTemp==1 || stVehicleParas.LeftFlagTemp==1
				|| stVehicleParas.BrakeFlag==1)// || Read_AEB_switch())
			{
			vechicleBodyInfo_flag = 1;
		}
		// 发送车身信息：判断方向盘是否转动
		if(SWADegree_tmp != stSWAParas.SWADegree){
			SWADegree_tmp = stSWAParas.SWADegree;
			vechicleBodyInfo_flag = 1;
		}
		// 发送车身信息：切换档位
		if(Car_Gear_tmp != stVehicleParas.Car_Gear){
			Car_Gear_tmp = stVehicleParas.Car_Gear;
			vechicleBodyInfo_flag = 1;
		}
*/
//		vechicleBodyInfo_flag = 1;
	}
}
/*
 * 数据上传-预警信息
 * 参数：无
 * 返回：无
 */
void UpLoad_Early_Warning_Info()
{
	uint8_t type = 0;
	static uint32_t	Warning_time = 0;
	static uint8_t state = 0;
	static uint8_t Bflag = 0;	//刹车信息
	static uint8_t	Lflag = 0;	//左车道偏离
	static uint8_t	Rflag = 0;	//右车道偏离
	static uint8_t	fflag = 0;	//fcw预警
	static uint8_t	hflag = 0;	//hmw预警
	// AEBS事件 开始与结束各触发一次

	if(camera_data.LeftLDW != 0)
	{
		if(Lflag == 0)
		{
			Lflag = 1;
			type = 0x0B;
			LeftLdwSerialnum = sysSerialnum;
			Early_Warning_Info_start1(type,LeftLdwSerialnum);
			if(sysSerialnum == 255)
				sysSerialnum = 0;
			else
				sysSerialnum ++;

			//Early_Warning_Info_start();
		}
	}
	else
	{
		if(Lflag == 1)
		{
			Lflag = 0;
			type = 0x03;
			Early_Warning_Info_start1(type,LeftLdwSerialnum);
			//Early_Warning_Info_start();
		}
	}

	if(camera_data.RightLDW != 0)
	{
		if(Rflag == 0)
		{
			Rflag = 1;
			type = 0x0B;
			RightLdwSerialnum = sysSerialnum;
			Early_Warning_Info_start1(type,RightLdwSerialnum);
			if(sysSerialnum == 255)
				sysSerialnum = 0;
			else
				sysSerialnum ++;

			//Early_Warning_Info_start();
		}
	}
	else
	{
		if(Rflag == 1)
		{
			Rflag = 0;
			type = 0x03;
			Early_Warning_Info_start1(type,RightLdwSerialnum);
			//Early_Warning_Info_start();
		}
	}

	if(camera_data.FCWLevel != 0)
	{
		if(fflag == 0)
		{
			fflag = 1;
			type = 0x09;
			FcwSerialnum = sysSerialnum;
			Early_Warning_Info_start1(type,FcwSerialnum);
			if(sysSerialnum == 255)
				sysSerialnum = 0;
			else
				sysSerialnum ++;

			//Early_Warning_Info_start();
		}
	}
	else
	{
		if(fflag == 1)
		{
			fflag = 0;
			type = 0x01;
			Early_Warning_Info_start1(type,FcwSerialnum);
			//Early_Warning_Info_start();
		}
	}

	if((stVehicleParas.fVehicleSpeed != 0) && ((camera_data.HMWGrade == 1) || (camera_data.HMWGrade == 2)))
	{
		if(hflag == 0)
		{
			hflag = 1;
			type = 0x0A;
			HmwSerialnum = sysSerialnum;
			Early_Warning_Info_start1(type,HmwSerialnum);
			if(sysSerialnum == 255)
				sysSerialnum = 0;
			else
				sysSerialnum ++;

			//Early_Warning_Info_start();
		}
	}
	else
	{
		if(hflag == 1)
		{
			hflag = 0;
			type = 0x02;
			Early_Warning_Info_start1(type,HmwSerialnum);
			//Early_Warning_Info_start();
		}
	}
	//if((0 < CameraMessage.ttc) && (CameraMessage.ttc < stSysPara.ttc1)){
	//	if(state == 0){
	//if(SystemtimeClock - Warning_time > 100)
	{
	//	Warning_time = SystemtimeClock;
	//		state = 1;
	//		Early_Warning_Info();
	}
	//	}
	//}else{
	//	if(state == 1){
	//		state = 0;
	//		Early_Warning_Info();
	//	}
	//}

}
void Test_Early_Warning_Info()
{
	if(SystemtimeClock - dataUploading_TickTime > 1000)
	{
		dataUploading_TickTime = SystemtimeClock;
		Early_Warning_Info();
	}
}

/*
 * 数据上传-车体信息
 * 参数：无
 * 返回：
 */
uint32_t upload_CarBody_time = 0;
void UpLoad_Car_Body_Info()
{
	uint8_t carBody_info[13]		= {PROTOCOL_FRONT};
	static uint8_t Rflag = 0;
	static uint8_t Lflag = 0;
	static uint8_t Bflag = 0;
	static uint8_t Dflag = 0;

	if(SystemtimeClock - upload_CarBody_time > 100){	// 与其他发送延时10ms,否则GPS设备无返回信息号
		upload_CarBody_time = SystemtimeClock;

		//if(vechicleBodyInfo_flag)
		if(((stVehicleParas.LeftFlagTemp == 1) & (Lflag == 0))|| ((stVehicleParas.RightFlagTemp == 1) & (Rflag == 0)) || ((stVehicleParas.BrakeFlag == 1) & (Bflag == 0)) )//|| (stSWAParas.Direction != Dflag))
		{
			Lflag = stVehicleParas.LeftFlagTemp;
			Rflag = stVehicleParas.RightFlagTemp;
			Bflag = stVehicleParas.BrakeFlag;
			Dflag = stSWAParas.Direction;

			vechicleBodyInfo_flag = 0;
			carBody_info[1] = CAR_BODY_TYPE;
			carBody_info[2] = 0x09;
			// 获取车速--字节4，无效0XFF
			carBody_info[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
			// 字节5，D8-D7，不解析
			// 获取AEB开关状态--字节5，D6
			carBody_info[4] = Read_AEB_switch() << 5;
			// 获取右转向灯状态--字节5，D5
			carBody_info[4] |= (stVehicleParas.RightFlagTemp&0x01) << 4;
			// 获取左转向灯状态--字节5，D4
			carBody_info[4] |= (stVehicleParas.LeftFlagTemp&0x01) << 3;
			// 获取刹车灯状态--字节5，D3
			carBody_info[4] |= (stVehicleParas.BrakeFlag&0x01) << 2;
			// 获取档位状态--字节5，D2-D1
			carBody_info[4] |=  stVehicleParas.Car_Gear;			// 需修改
			// 获取故障码--字节6，D8-D1
			if(camera_data.ErrorCode == 0){								// 无故障码，就报如下异常
				//if(stCanCommSta.stRadar.status == OFFLINE)				// 毫米波通讯异常
				//	carBody_info[5] = 128;
				//if(stCanCommSta.stOBD.status == OFFLINE)				// OBD通讯异常
				//	carBody_info[5] = 129;
				if(stCanCommSta.stHRadar.status == OFFLINE)				// 超声波通讯异常
					carBody_info[5] = 130;
				if(stCanCommSta.stVehicle.status == OFFLINE)			// AEBS控制器内部通信异常
					carBody_info[5] = 131;
				//if(stCanCommSta.stWireless.status == OFFLINE)			// AEBS与GPS通讯(4G模块)异常
					//carBody_info[5] = 132;
			}else{
				carBody_info[5] = camera_data.ErrorCode;
			}
			// 获取油门踏板开度--字节7，无效0XFF
			carBody_info[6] = stVehicleParas.ThrottleOpening;// * 2.5;
			// 获取刹车踏板开度--字节8，无效0xFF
			carBody_info[7] = stVehicleParas.BrakeOpening;// * 2.5;
			// 获取方向盘角度(LSB)|(MSB)--字节9、字节10，无效0x7FF
			carBody_info[8] = stSWAParas.SWADegree & _0XFF;
			carBody_info[9] = (stSWAParas.SWADegree&0x07) >> 8;
			// 方向盘状态 D4	【0：向左打】、【1：向右打】Direction
			carBody_info[9] |= stSWAParas.Direction << 3;
			// 备用--字节11，0xFF
			carBody_info[10] = _0XFF;
			// 备用--字节12，0xFF
			carBody_info[11] = _0XFF;
			// 异或校验--字节13
			for(uint8_t i=0;i<12;i++)
				carBody_info[12] ^= carBody_info[i];
			// 发送
			USART_Send(USART2_SFR,carBody_info,13);
			memset(carBody_info,0,sizeof(carBody_info));
		}

		else
		{
			Lflag = stVehicleParas.LeftFlagTemp;
			Rflag = stVehicleParas.RightFlagTemp;
			Bflag = stVehicleParas.BrakeFlag;
		}
	}
}
//uint32_t data_time = 0;
//uint8_t  data_flag = 0;
//void Data_Uploading_In_SysTick()
//{
//	data_time++;
//	if(data_time >= 500){
//		data_time = 0;
//		data_flag = 1;
//	}
//}
/*
 * 才库4G模块数据上传功能在中断函数处理逻辑
 * 参数：data接收数据
 * 返回：无
 */
volatile uint8_t gps_times 		= 0;			// 1秒回复一次GPS设备车速
void Data_Uploading_In_KFIT(uint8_t data)
{
	uint8_t checkout 		= 0;
	replay_res[replay_cnt] 	= data;
	replay_cnt ++;
	if(replay_res[0] == PROTOCOL_FRONT)
	{
		switch(replay_res[1])
		{
#if 1
		case VECHICLE_DEVICE_TYPE:{
			if(replay_cnt >= 5){
				replay_cnt = 0;
				for(uint8_t i=0;i<4;i++){
					checkout ^= replay_res[i];
				}
				if(checkout == replay_res[4])
					carKu_replay.vehicleDevice_res = SUCCESS;
				else
					carKu_replay.vehicleDevice_res = FAILURE;

////				fprintf(USART0_STREAM,"VECHICLE_DEVICE_TYPE\r\n");
				memset(replay_res,0,sizeof(replay_res));
			}
		}break;
		case EARLY_WARNING_TYPE:{
			if(replay_cnt >= 5){
				replay_cnt = 0;
				for(uint8_t i=0;i<4;i++){
					checkout ^= replay_res[i];
				}
				if(checkout == replay_res[4])
					carKu_replay.earlyWarning_res = SUCCESS;
				else
					carKu_replay.earlyWarning_res = FAILURE;
//				fprintf(USART0_STREAM,"EARLY_WARNING_TYPE\r\n");
				memset(replay_res,0,sizeof(replay_res));
			}
		}break;
		case CAR_BODY_TYPE:{
			if(replay_cnt >= 5){
				replay_cnt = 0;
				for(uint8_t i=0;i<4;i++){
					checkout ^= replay_res[i];
				}
				if(checkout == replay_res[4])
					carKu_replay.carBody_res = SUCCESS;
				else
					carKu_replay.carBody_res = FAILURE;
//				fprintf(USART0_STREAM,"CAR_BODY_TYPE\r\n");
				memset(replay_res,0,sizeof(replay_res));
			}
		}break;
#endif
		case CAR_VELOCITY_TYPE:{		// 50ms接受一次车速信息，但1s回复一次
			if(replay_cnt >= 6){
				replay_cnt = 0;
				if(gps_times++ > 20){	// 50ms，1秒回复一次,避免程序堵塞
					gps_times = 0;
					gps_reponse = SUCCESS;
					for(uint8_t i=0;i<5;i++){
						checkout ^= replay_res[i];
					}
					if(checkout == replay_res[5]){
						// 有效数据，并解析出车速
						if(replay_res[4] == 1)
							car_vechicle = replay_res[3];
						carKu_replay.gpsVelocity_res = SUCCESS;
					}else{
						carKu_replay.gpsVelocity_res = FAILURE;
					}
					memset(replay_res,0,sizeof(replay_res));
				}
			}
		}break;
		default:
			if(replay_cnt >= 5){
				replay_cnt = 0;
			}
			break;
		}
	}else if(replay_cnt >= 6){			// 避免异常
		replay_cnt = 0;
	}
}
/*
 * 才库4G模块在main函数循环体
 */
void Data_Uploading_In_MainLoop()
{
	// 上传车辆设备基本信息
	UpLoad_Vehicle_Device_Info();
//	// 上传车身信息
	UpLoad_Car_Body_Info();
	// 上传预警信息
	UpLoad_Early_Warning_Info();
//	Test_Early_Warning_Info();
	if(gps_reponse == SUCCESS)
	{
		gps_reponse = FAILURE;
		Response(CAR_VELOCITY_TYPE,carKu_replay.gpsVelocity_res);
	}
}
/*
 * 回复才库4G模块
 * 参数：dataType数据类型；status数据状态
 * 返回：无
 */
void Response(uint8_t dataType,RetStatus status)
{
	uint8_t response_res[5] = {PROTOCOL_FRONT};
	response_res[1] = dataType;
	response_res[2] = 1;
	response_res[3] = status;
	for(uint8_t i=0; i<4; i++)
		response_res[4] ^= response_res[i];
//	for(uint8_t i=0;i<5;i++)
//		fprintf(USART0_STREAM,"%02X ",response_res[i]);
	// 发送
	USART_Send(USART2_SFR,response_res,5);
	memset(response_res,0,sizeof(response_res));
}

/*
 * 接收才库4G模块数据校验
 * 参数：接收数据
 * 返回：0校验失败；1数据内容失败；2数据内容成功
 */
uint8_t Checkout(uint8_t *data)
{
	for(uint8_t i=0;i<5;i++){
		checkout ^= data[i];
	}
	if(checkout == data[4]){
		// 有效数据，并解析出车速
		if(data[3] == 1)
			return 2;	// 成功
		else
			return 1;	// 失败
	}else{
		return 0;		// 校验失败
	}
}

void Early_Warning_Info_start1(uint8_t type,uint8_t Serialnumber)
{
	uint8_t earlyWarning_info[24] = {PROTOCOL_FRONT};
	earlyWarning_info[1] = EARLY_WARNING_TYPE;
	earlyWarning_info[2] = 0x14;
	// 获取车速--字节4，无效0xFF
	earlyWarning_info[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
	// 获取TTC时间--字节5，无效0x3F
	/*
	if((rx_obstacle_info_b.data[4] & _0X3F) == _0X3F)
		earlyWarning_info[4] = _0X3F;
	else
		earlyWarning_info[4] = (uint8_t)(CameraMessage.ttc * 10);
//	earlyWarning_info[4] = 60;
	// 获取HMW时间--字节6，无效0x3F
	if(((uint8_t)(camera_data.HMW * 10) & _0X3F) == _0X3F)
		earlyWarning_info[5] = _0X3F;
	else
		earlyWarning_info[5] = (uint8_t)(camera_data.HMW * 10);
	*/
	earlyWarning_info[4] = (uint8_t)(CameraMessage.ttc * 10);
	earlyWarning_info[5] = (uint8_t)(CameraMessage.hmw * 10);
//	earlyWarning_info[5] = 40;
	// 获取纵向相对距离Z(LSB)|Z(MSB)--字节7、字节8，无效0x7FFF
	earlyWarning_info[6] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceZ*100) & _0XFF);
	earlyWarning_info[7] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceZ*100) >> 8 & _0XFF);
	// 获取纵向相对速度(LSB)|(MSB)--字节9、字节10，无效0xFFF//obstacle_cipv_data.RelativeSpeedZ
	earlyWarning_info[8] = (uint8_t)((uint16_t)((obstacle_cipv_data.RelativeSpeedZ + 127.9375)/0.0625) & _0XFF);
	earlyWarning_info[9] = (uint8_t)((uint16_t)((obstacle_cipv_data.RelativeSpeedZ + 127.9375)/0.0625)>>8 & _0XFF);
	// 获取横向相对距离X(LSB)|X(MSB)--字节11、字节12，无效0x7FFF
	earlyWarning_info[10] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceX*100) & _0XFF);
	earlyWarning_info[11] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceX*100)>>8 & _0XFF);
	// 获取油门踏板开度--字节13，无效0xFF
	earlyWarning_info[12] = stVehicleParas.ThrottleOpening;// * 2.5;
	// 获取刹车踏板开度--字节14，无效0xFF
	earlyWarning_info[13] = stVehicleParas.BrakeOpening;// * 2.5;
	// 获取方向盘角度(LSB)|(MSB)--字节15、字节16，无效0x7FF
	earlyWarning_info[14] = stSWAParas.SWADegree & _0XFF;				// 方向盘角度低8位
	earlyWarning_info[15] = (stSWAParas.SWADegree>>8) & 0x07;			// 方向盘角度高3位
	earlyWarning_info[15] |= (stSWAParas.Direction&0x01) << 3;			// 方向盘状态
	// 获取左车道线类型--字节17 D8-D5 【0：无】、【1：预测】、【2：虚线】、【3：实线】、【4：双虚线】、【5：双实线】、【6：三线】、【7～15：保留】
	earlyWarning_info[16] = (camera_LDW_data.LeftLaneStyle&_0X0F) << 4;
	// 获取右车道线类型--字节17 D4-D1 【0：无】、【1：预测】、【2：虚线】、【3：实线】、【4：双虚线】、【5：双实线】、【6：三线】、【7～15：保留】
	earlyWarning_info[16] |= camera_LDW_data.RightLaneStyle  & _0X0F;
	// 获取FCWLevel前向碰撞预警等级--字节18 D2-D1	【0：否】、【1：一级预警】、【2：二级预警】
	earlyWarning_info[17] = camera_data.FCWLevel & 0x03;
	// 获取障碍物类型--字节18 D6-D3	【0：无效值】、【1：车辆】、【2：人】、【3-11：其他】
	earlyWarning_info[17] |= (obstacle_cipv_data.ObstacleType & _0X0F) << 2;
	// 获取右车道线偏离预警--字节18 D7	【0：否]、【1：是】
	earlyWarning_info[17] |= (camera_data.RightLDW&0x01) << 6;
	// 获取左车道线偏离预警--字节18 D8	【0：否]、【1：是】
	earlyWarning_info[17] |= (camera_data.LeftLDW&0x01) << 7;
	// 获取AEB制动状态--字节19 D8-D7	【0：无制动】、【1：双目制动】、【2：毫米波制动】、【3：超声波制动】
	/*
	uint8_t BreakFlag = 0;
	event_t.fcw_t = 0;
	event_t.ldw_t = 0;
	event_t.hmw_t = 0;
	event_t.aeb_t = 0;
	event_t.event = NO_EVENT;
	if((stVehicleParas.BreakState & 0x01) == 0x01)
	{
		BreakFlag = 1;
		event_t.hmw_t = 1;
		event_t.event = HMW_EVENT;
	}
	if((stVehicleParas.BreakState & 0x02) == 0x02)
	{
		BreakFlag = 1;
		event_t.fcw_t = 1;
		event_t.event = FCW_EVENT;
	}
	if((stVehicleParas.BreakState & 0x04) == 0x04)
	{
		BreakFlag = 3;
		event_t.aeb_t = 1;
		event_t.event = AEB_EVENT;
	}
	if(stVehicleParas.BreakState == 0)
	{
		BreakFlag = 0;
	}
//	BreakFlag = 1;
	earlyWarning_info[18] = BreakFlag << 6;
	// 获取档位状态--字节19 D6-D5	【0：无效】、【1：N档】、【2：D档】、【3：R档】
	earlyWarning_info[18] |= (stVehicleParas.Car_Gear) << 4;
	// 获取事件分类--字节19 D3-D1	【1：FCW】、【2：HMW】、【3：LDW】、【4：AEB】	待修改
	if(camera_data.LeftLDW || camera_data.RightLDW){	// 车道偏移
		event_t.ldw_t = 1;
		event_t.event = LDW_EVENT;
	}else event_t.ldw_t = 0;

	switch(event_t.event){
	case FCW_EVENT:
		earlyWarning_info[18] |= event_t.fcw_t<<3;		// 触发事件 起始时间 开始|结束
		earlyWarning_info[18] |= event_t.event & 0x07;	// 触发事件
		break;
	case HMW_EVENT:
		earlyWarning_info[18] |= event_t.hmw_t<<3;		// 触发事件 起始时间 开始|结束
		earlyWarning_info[18] |= event_t.event & 0x07;	// 触发事件
		break;
	case LDW_EVENT:
		earlyWarning_info[18] |= event_t.ldw_t<<3;		// 触发事件 起始时间 开始|结束
		earlyWarning_info[18] |= event_t.event & 0x07;	// 触发事件
		break;
	case AEB_EVENT:
		earlyWarning_info[18] |= event_t.aeb_t<<3;		// 触发事件 起始时间 开始|结束
		earlyWarning_info[18] |= event_t.event & 0x07;	// 触发事件
		break;
	}
	 */
	earlyWarning_info[18] = type;
	earlyWarning_info[18] |= (stVehicleParas.Car_Gear) << 4;
	// 获取超声波距离--字节20 D8-D1，无效0XFF
	if((type & 0xc0) == 0xc0)
		earlyWarning_info[19] = stVehicleParas.Ultrasonicdistance + 1;
	else
		earlyWarning_info[19] = 0xff;
	// 备用--字节21，0xFF
	earlyWarning_info[20] = _0XFF;
	// 备用--字节22，0xFF
	earlyWarning_info[21] = _0XFF;
	// 备用--字节23，0xFF
	earlyWarning_info[22] = Serialnumber;//_0XFF;
	// 异或校验--字节24
	for(uint8_t i=0;i<23;i++)
		earlyWarning_info[23] ^= earlyWarning_info[i];
//	for(uint8_t i=0;i<24;i++)
//		fprintf(USART0_STREAM,"%02X ",earlyWarning_info[i]);
//	fprintf(USART0_STREAM,"\r\n");
	// 发送
	USART_Send(USART2_SFR,earlyWarning_info,24);
	//memset(earlyWarning_info,0,sizeof(earlyWarning_info));
}

/*
 * 预警信息
 * 参数：无
 * 返回：无
 */
void Early_Warning_Info_start(void)
{
	uint8_t earlyWarning_info[24] = {PROTOCOL_FRONT};
	earlyWarning_info[1] = EARLY_WARNING_TYPE;
	earlyWarning_info[2] = 0x14;
	// 获取车速--字节4，无效0xFF
	earlyWarning_info[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
	// 获取TTC时间--字节5，无效0x3F
	/*
	if((rx_obstacle_info_b.data[4] & _0X3F) == _0X3F)
		earlyWarning_info[4] = _0X3F;
	else
		earlyWarning_info[4] = (uint8_t)(CameraMessage.ttc * 10);
//	earlyWarning_info[4] = 60;
	// 获取HMW时间--字节6，无效0x3F
	if(((uint8_t)(camera_data.HMW * 10) & _0X3F) == _0X3F)
		earlyWarning_info[5] = _0X3F;
	else
		earlyWarning_info[5] = (uint8_t)(camera_data.HMW * 10);
	*/
	earlyWarning_info[4] = (uint8_t)(CameraMessage.ttc * 10);
	earlyWarning_info[5] = (uint8_t)(CameraMessage.hmw * 10);
//	earlyWarning_info[5] = 40;
	// 获取纵向相对距离Z(LSB)|Z(MSB)--字节7、字节8，无效0x7FFF
	earlyWarning_info[6] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceZ*100) & _0XFF);
	earlyWarning_info[7] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceZ*100) >> 8 & _0XFF);
	// 获取纵向相对速度(LSB)|(MSB)--字节9、字节10，无效0xFFF
	earlyWarning_info[8] = (uint8_t)((uint16_t)(obstacle_cipv_data.RelativeSpeedZ*16) & _0XFF);
	earlyWarning_info[9] = (uint8_t)((uint16_t)(obstacle_cipv_data.RelativeSpeedZ*16)>>8 & _0XFF);
	// 获取横向相对距离X(LSB)|X(MSB)--字节11、字节12，无效0x7FFF
	earlyWarning_info[10] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceX*100) & _0XFF);
	earlyWarning_info[11] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceX*100)>>8 & _0XFF);
	// 获取油门踏板开度--字节13，无效0xFF
	earlyWarning_info[12] = stVehicleParas.ThrottleOpening;// * 2.5;
	// 获取刹车踏板开度--字节14，无效0xFF
	earlyWarning_info[13] = stVehicleParas.BrakeOpening;// * 2.5;
	// 获取方向盘角度(LSB)|(MSB)--字节15、字节16，无效0x7FF
	earlyWarning_info[14] = stSWAParas.SWADegree & _0XFF;				// 方向盘角度低8位
	earlyWarning_info[15] = (stSWAParas.SWADegree>>8) & 0x07;			// 方向盘角度高3位
	earlyWarning_info[15] |= (stSWAParas.Direction&0x01) << 3;			// 方向盘状态
	// 获取左车道线类型--字节17 D8-D5 【0：无】、【1：预测】、【2：虚线】、【3：实线】、【4：双虚线】、【5：双实线】、【6：三线】、【7～15：保留】
	earlyWarning_info[16] = (camera_LDW_data.LeftLaneStyle&_0X0F) << 4;
	// 获取右车道线类型--字节17 D4-D1 【0：无】、【1：预测】、【2：虚线】、【3：实线】、【4：双虚线】、【5：双实线】、【6：三线】、【7～15：保留】
	earlyWarning_info[16] |= camera_LDW_data.RightLaneStyle  & _0X0F;
	// 获取FCWLevel前向碰撞预警等级--字节18 D2-D1	【0：否】、【1：一级预警】、【2：二级预警】
	earlyWarning_info[17] = camera_data.FCWLevel & 0x03;
	// 获取障碍物类型--字节18 D6-D3	【0：无效值】、【1：车辆】、【2：人】、【3-11：其他】
	earlyWarning_info[17] |= (obstacle_cipv_data.ObstacleType & _0X0F) << 2;
	// 获取右车道线偏离预警--字节18 D7	【0：否]、【1：是】
	earlyWarning_info[17] |= (camera_data.RightLDW&0x01) << 6;
	// 获取左车道线偏离预警--字节18 D8	【0：否]、【1：是】
	earlyWarning_info[17] |= (camera_data.LeftLDW&0x01) << 7;
	// 获取AEB制动状态--字节19 D8-D7	【0：无制动】、【1：双目制动】、【2：毫米波制动】、【3：超声波制动】
	uint8_t BreakFlag = 0;
	event_t.fcw_t = 0;
	event_t.ldw_t = 0;
	event_t.hmw_t = 0;
	event_t.aeb_t = 0;
	event_t.event = NO_EVENT;
	if((stVehicleParas.BreakState & 0x01) == 0x01)
	{
		BreakFlag = 1;
		event_t.hmw_t = 1;
		event_t.event = HMW_EVENT;
	}
	if((stVehicleParas.BreakState & 0x02) == 0x02)
	{
		BreakFlag = 1;
		event_t.fcw_t = 1;
		event_t.event = FCW_EVENT;
	}
	if((stVehicleParas.BreakState & 0x04) == 0x04)
	{
		BreakFlag = 3;
		event_t.aeb_t = 1;
		event_t.event = AEB_EVENT;
	}
	if(stVehicleParas.BreakState == 0)
	{
		BreakFlag = 0;
	}
//	BreakFlag = 1;
	earlyWarning_info[18] = BreakFlag << 6;
	// 获取档位状态--字节19 D6-D5	【0：无效】、【1：N档】、【2：D档】、【3：R档】
	earlyWarning_info[18] |= (stVehicleParas.Car_Gear) << 4;
	// 获取事件分类--字节19 D3-D1	【1：FCW】、【2：HMW】、【3：LDW】、【4：AEB】	待修改
	/*
	if(camera_data.FCWStatus){ 			// FCW
		event_t.fcw_t = 1;
		event_t.event = FCW_EVENT;
	}else event_t.fcw_t = 0;

	if(camera_data.HMWGrade == 1) {			// HMW
		event_t.hmw_t = 1;
		event_t.event = HMW_EVENT;
	}else event_t.hmw_t = 0;
*/
	if(camera_data.LeftLDW || camera_data.RightLDW){	// 车道偏移
		event_t.ldw_t = 1;
		event_t.event = LDW_EVENT;
	}else event_t.ldw_t = 0;
/*
	if(stVehicleParas.BreakState != 0){	// AEB
		event_t.aeb_t = 1;
		event_t.event = AEB_EVENT;
	}else event_t.aeb_t = 0;
*/
	//event_t.event = HMW_EVENT;
	switch(event_t.event){
	case FCW_EVENT:
		earlyWarning_info[18] |= event_t.fcw_t<<3;		// 触发事件 起始时间 开始|结束
		earlyWarning_info[18] |= event_t.event & 0x07;	// 触发事件
		break;
	case HMW_EVENT:
		earlyWarning_info[18] |= event_t.hmw_t<<3;		// 触发事件 起始时间 开始|结束
		earlyWarning_info[18] |= event_t.event & 0x07;	// 触发事件
		break;
	case LDW_EVENT:
		earlyWarning_info[18] |= event_t.ldw_t<<3;		// 触发事件 起始时间 开始|结束
		earlyWarning_info[18] |= event_t.event & 0x07;	// 触发事件
		break;
	case AEB_EVENT:
		earlyWarning_info[18] |= event_t.aeb_t<<3;		// 触发事件 起始时间 开始|结束
		earlyWarning_info[18] |= event_t.event & 0x07;	// 触发事件
		break;
	}

	// 获取超声波距离--字节20 D8-D1，无效0XFF
	earlyWarning_info[19] = stVehicleParas.Ultrasonicdistance;
	// 备用--字节21，0xFF
	earlyWarning_info[20] = _0XFF;
	// 备用--字节22，0xFF
	earlyWarning_info[21] = _0XFF;
	// 备用--字节23，0xFF
	earlyWarning_info[22] = _0XFF;
	// 异或校验--字节24
	for(uint8_t i=0;i<23;i++)
		earlyWarning_info[23] ^= earlyWarning_info[i];
//	for(uint8_t i=0;i<24;i++)
//		fprintf(USART0_STREAM,"%02X ",earlyWarning_info[i]);
//	fprintf(USART0_STREAM,"\r\n");
	// 发送
	USART_Send(USART2_SFR,earlyWarning_info,24);
	//memset(earlyWarning_info,0,sizeof(earlyWarning_info));
}

void Early_Warning_Info()
{
	uint8_t earlyWarning_info[24] = {PROTOCOL_FRONT};
	earlyWarning_info[1] = EARLY_WARNING_TYPE;
	earlyWarning_info[2] = 0x14;
	// 获取车速--字节4，无效0xFF
	earlyWarning_info[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
	// 获取TTC时间--字节5，无效0x3F
	if((rx_obstacle_info_b.data[4] & _0X3F) == _0X3F)
		earlyWarning_info[4] = _0X3F;
	else
		earlyWarning_info[4] = (uint8_t)(CameraMessage.ttc * 10);
//	earlyWarning_info[4] = 60;
	// 获取HMW时间--字节6，无效0x3F
	if(((uint8_t)(camera_data.HMW * 10) & _0X3F) == _0X3F)
		earlyWarning_info[5] = _0X3F;
	else
		earlyWarning_info[5] = (uint8_t)(camera_data.HMW * 10);
//	earlyWarning_info[5] = 40;
	// 获取纵向相对距离Z(LSB)|Z(MSB)--字节7、字节8，无效0x7FFF
	earlyWarning_info[6] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceZ*100) & _0XFF);
	earlyWarning_info[7] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceZ*100) >> 8 & _0XFF);
	// 获取纵向相对速度(LSB)|(MSB)--字节9、字节10，无效0xFFF
	earlyWarning_info[8] = (uint8_t)((uint16_t)(obstacle_cipv_data.RelativeSpeedZ*16) & _0XFF);
	earlyWarning_info[9] = (uint8_t)((uint16_t)(obstacle_cipv_data.RelativeSpeedZ*16)>>8 & _0XFF);
	// 获取横向相对距离X(LSB)|X(MSB)--字节11、字节12，无效0x7FFF
	earlyWarning_info[10] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceX*100) & _0XFF);
	earlyWarning_info[11] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceX*100)>>8 & _0XFF);
	// 获取油门踏板开度--字节13，无效0xFF
	earlyWarning_info[12] = stVehicleParas.ThrottleOpening;// * 2.5;
	// 获取刹车踏板开度--字节14，无效0xFF
	earlyWarning_info[13] = stVehicleParas.BrakeOpening;// * 2.5;
	// 获取方向盘角度(LSB)|(MSB)--字节15、字节16，无效0x7FF
	earlyWarning_info[14] = stSWAParas.SWADegree & _0XFF;				// 方向盘角度低8位
	earlyWarning_info[15] = (stSWAParas.SWADegree>>8) & 0x07;			// 方向盘角度高3位
	earlyWarning_info[15] |= (stSWAParas.Direction&0x01) << 3;			// 方向盘状态
	// 获取左车道线类型--字节17 D8-D5 【0：无】、【1：预测】、【2：虚线】、【3：实线】、【4：双虚线】、【5：双实线】、【6：三线】、【7～15：保留】
	earlyWarning_info[16] = (camera_LDW_data.LeftLaneStyle&_0X0F) << 4;
	// 获取右车道线类型--字节17 D4-D1 【0：无】、【1：预测】、【2：虚线】、【3：实线】、【4：双虚线】、【5：双实线】、【6：三线】、【7～15：保留】
	earlyWarning_info[16] |= camera_LDW_data.RightLaneStyle  & _0X0F;
	// 获取FCWLevel前向碰撞预警等级--字节18 D2-D1	【0：否】、【1：一级预警】、【2：二级预警】
	earlyWarning_info[17] = camera_data.FCWLevel & 0x03;
	// 获取障碍物类型--字节18 D6-D3	【0：无效值】、【1：车辆】、【2：人】、【3-11：其他】
	earlyWarning_info[17] |= (obstacle_cipv_data.ObstacleType & _0X0F) << 2;
	// 获取右车道线偏离预警--字节18 D7	【0：否]、【1：是】
	earlyWarning_info[17] |= (camera_data.RightLDW&0x01) << 6;
	// 获取左车道线偏离预警--字节18 D8	【0：否]、【1：是】
	earlyWarning_info[17] |= (camera_data.LeftLDW&0x01) << 7;
	// 获取AEB制动状态--字节19 D8-D7	【0：无制动】、【1：双目制动】、【2：毫米波制动】、【3：超声波制动】
	uint8_t BreakFlag = 0;
	if((stVehicleParas.BreakState & 0x01) == 0x01)
		BreakFlag = 1;
	if((stVehicleParas.BreakState & 0x02) == 0x02)
		BreakFlag = 1;
	if((stVehicleParas.BreakState & 0x04) == 0x04)
		BreakFlag = 3;
	if(stVehicleParas.BreakState == 0)
		BreakFlag = 0;
//	BreakFlag = 1;
	earlyWarning_info[18] = BreakFlag << 6;
	// 获取档位状态--字节19 D6-D5	【0：无效】、【1：N档】、【2：D档】、【3：R档】

	earlyWarning_info[18] |= (stVehicleParas.Car_Gear) << 4;
	// 获取事件分类--字节19 D3-D1	【1：FCW】、【2：HMW】、【3：LDW】、【4：AEB】	待修改
	if(camera_data.FCWStatus){ 			// FCW
		event_t.fcw_t = 1;
		event_t.event = FCW_EVENT;
	}else event_t.fcw_t = 0;

	if(camera_data.HMWEnable) {			// HMW
		event_t.hmw_t = 1;
		event_t.event = HMW_EVENT;
	}else event_t.hmw_t = 0;

	if(camera_data.LeftLDW || camera_data.RightLDW){	// 车道偏移
		event_t.ldw_t = 1;
		event_t.event = LDW_EVENT;
	}else event_t.ldw_t = 0;

	if(stVehicleParas.BreakState != 0){	// AEB
		event_t.aeb_t = 1;
		event_t.event = AEB_EVENT;
	}else event_t.aeb_t = 0;
	switch(event_t.event){
	case FCW_EVENT:
		earlyWarning_info[18] |= event_t.fcw_t<<3;		// 触发事件 起始时间 开始|结束
		earlyWarning_info[18] |= event_t.event & 0x07;	// 触发事件
		break;
	case HMW_EVENT:
		earlyWarning_info[18] |= event_t.hmw_t<<3;		// 触发事件 起始时间 开始|结束
		earlyWarning_info[18] |= event_t.event & 0x07;	// 触发事件
		break;
	case LDW_EVENT:
		earlyWarning_info[18] |= event_t.ldw_t<<3;		// 触发事件 起始时间 开始|结束
		earlyWarning_info[18] |= event_t.event & 0x07;	// 触发事件
		break;
	case AEB_EVENT:
		earlyWarning_info[18] |= event_t.aeb_t<<3;		// 触发事件 起始时间 开始|结束
		earlyWarning_info[18] |= event_t.event & 0x07;	// 触发事件
		break;
	}
	// 获取超声波距离--字节20 D8-D1，无效0XFF
	earlyWarning_info[19] = stVehicleParas.Ultrasonicdistance;;
	// 备用--字节21，0xFF
	earlyWarning_info[20] = _0XFF;
	// 备用--字节22，0xFF
	earlyWarning_info[21] = _0XFF;
	// 备用--字节23，0xFF
	earlyWarning_info[22] = _0XFF;
	// 异或校验--字节24
	for(uint8_t i=0;i<23;i++)
		earlyWarning_info[23] ^= earlyWarning_info[i];
//	for(uint8_t i=0;i<24;i++)
//		fprintf(USART0_STREAM,"%02X ",earlyWarning_info[i]);
//	fprintf(USART0_STREAM,"\r\n");
	// 发送
	USART_Send(USART2_SFR,earlyWarning_info,24);
	//memset(earlyWarning_info,0,sizeof(earlyWarning_info));
}

