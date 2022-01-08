/*
 * data_uploading.c
 *
 *  Created on: 2021-10-10
 *      Author: Administrator
 *  ˵����ֱ����ſ�4Gģ��Խӣ���ϸЭ������ġ�AEBS�豸��GPS�豸ͨѶЭ��V2.4.docx��
 */
#include "data_uploading.h"
#include "common.h"
#include "usart.h"
#include "stdio.h"
#include "gpio.h"
#include "upgrade_common.h"
/**************************�궨��*****************************************/
#define PROTOCOL_FRONT			0x2E
#define VECHICLE_DEVICE_TYPE	0x11
#define EARLY_WARNING_TYPE		0x24
#define CAR_BODY_TYPE			0x30
#define CAR_VELOCITY_TYPE		0x88
#define _0X0F					0x0F
#define _0X3F					0x3F
#define _0XFF					0xFF
//#define _0x3FFF					0x3FFF
/**************************�ṹ�塢ö��*****************************************/
typedef struct _CaiKuReply
{
	RetStatus vehicleDevice_res;	// �����豸������Ϣ�����ؽ��
	RetStatus earlyWarning_res;		// Ԥ����Ϣ�����ؽ��
	RetStatus carBody_res;			// ������Ϣ�����ؽ��
	RetStatus gpsVelocity_res;		// GPS���ٽ��
}CaiKuReply;
/**************************ȫ�ֱ���*****************************************/
uint8_t replay_res[7]			= {0};
uint8_t replay_cnt				= 0;
uint8_t checkout 				= 0;
CaiKuReply carKu_replay			= {0};
uint8_t car_vechicle			= 0;
uint8_t vechicleBodyInfo_flag 	= 0;
enum Gear_MODE Car_Gear_tmp 	= P_MODE;		// Ĭ��P��
uint32_t SWADegree_tmp			= 0;			// ������ת��Ƕ�
RetStatus gps_reponse	 		= FAILURE;
Event_T event_t 				= {0};			// �ſ�4G�����ϴ�-�¼�

uint8_t sysSerialnum = 0;
uint8_t URaderSerialnum = 0;
uint8_t CmeraSerialnum = 0;
uint8_t FcwSerialnum = 0;
uint8_t HmwSerialnum = 0;
uint8_t LeftLdwSerialnum = 0;
uint8_t RightLdwSerialnum = 0;
/**************************��������*****************************************/
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
 * �����ϴ�-�����豸������Ϣ
 * ��������
 * ���أ���
 * ˵����Ƶ�ʣ�100ms
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
		// ��ȡ����--�ֽ�4
		vehicleDevice_info[3] = (uint8_t)stVehicleParas.fVehicleSpeed;;
		// ��ȡAEB�ƶ�״̬--�ֽ�5 D8-D7	��0�����ƶ�������1��˫Ŀ�ƶ�������2�����ײ��ƶ�������3���������ƶ���

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
		// ��ȡAEB����״̬--�ֽ�5 D6	��0����]����1���ǡ�
		//vehicleDevice_info[4] |= Read_AEB_switch() << 5;
		// ��ȡ��ת���״̬--�ֽ�5 D5	��0����]����1���ǡ�
		vehicleDevice_info[4] |= (stVehicleParas.RightFlagTemp&0x01) << 4;
		// ��ȡ��ת���״̬--�ֽ�5 D4	��0����]����1���ǡ�
		vehicleDevice_info[4] |= (stVehicleParas.LeftFlagTemp&0x01) << 3;
		// ��ȡɲ����״̬--�ֽ�5 D3	��0����]����1���ǡ�
		vehicleDevice_info[4] |= (stVehicleParas.BrakeFlag&0x01) << 2;
		// ��ȡ��λ״̬--�ֽ�5 D2-D1	��0����Ч������1��N��������2��D��������3��R����
		vehicleDevice_info[4] |= stVehicleParas.Car_Gear;
		// ��ȡ������--�ֽ�6
		if(camera_data.ErrorCode == 0){								// ���쳣�����ͨѶ�쳣
			//if(stCanCommSta.stRadar.status == OFFLINE)				// ���ײ�ͨѶ�쳣
			//	vehicleDevice_info[5] = 128;
			//if(stCanCommSta.stOBD.status == OFFLINE)				// OBDͨѶ�쳣
			//	vehicleDevice_info[5] = 129;
			if(stCanCommSta.stHRadar.status == OFFLINE)				// ������ͨѶ�쳣
				vehicleDevice_info[5] = 130;
			if(stCanCommSta.stVehicle.status == OFFLINE)			// AEBS�������ڲ�ͨ���쳣
				vehicleDevice_info[5] = 131;
			//if(stCanCommSta.stWireless.status == OFFLINE)			// AEBS��GPSͨѶ(4Gģ��)�쳣
			//	vehicleDevice_info[5] = 132;
		}else{
			vehicleDevice_info[5] = camera_data.ErrorCode;
		}
		// Ԥ��--�ֽ�7
		vehicleDevice_info[6] = _0XFF;
		// Ԥ��--�ֽ�8
		vehicleDevice_info[7] = _0XFF;
		// ���У��--�ֽ�9
		for(uint8_t i=0;i<8;i++)
			vehicleDevice_info[8] ^= vehicleDevice_info[i];

//		for(uint8_t i=0;i<9;i++)
//			fprintf(USART0_STREAM,"%02X ",vehicleDevice_info[i]);
//		fprintf(USART0_STREAM,"\r\n");
		// ����
		USART_Send(USART2_SFR,vehicleDevice_info,9);
		memset(vehicleDevice_info,0,sizeof(vehicleDevice_info));

		// ���ͳ�����Ϣ��������ת��ơ��ƶ�������AEBSϵͳ����
/*
		if(stVehicleParas.RightFlagTemp==1 || stVehicleParas.LeftFlagTemp==1
				|| stVehicleParas.BrakeFlag==1)// || Read_AEB_switch())
			{
			vechicleBodyInfo_flag = 1;
		}
		// ���ͳ�����Ϣ���жϷ������Ƿ�ת��
		if(SWADegree_tmp != stSWAParas.SWADegree){
			SWADegree_tmp = stSWAParas.SWADegree;
			vechicleBodyInfo_flag = 1;
		}
		// ���ͳ�����Ϣ���л���λ
		if(Car_Gear_tmp != stVehicleParas.Car_Gear){
			Car_Gear_tmp = stVehicleParas.Car_Gear;
			vechicleBodyInfo_flag = 1;
		}
*/
//		vechicleBodyInfo_flag = 1;
	}
}
/*
 * �����ϴ�-Ԥ����Ϣ
 * ��������
 * ���أ���
 */
void UpLoad_Early_Warning_Info()
{
	uint8_t type = 0;
	static uint32_t	Warning_time = 0;
	static uint8_t state = 0;
	static uint8_t Bflag = 0;	//ɲ����Ϣ
	static uint8_t	Lflag = 0;	//�󳵵�ƫ��
	static uint8_t	Rflag = 0;	//�ҳ���ƫ��
	static uint8_t	fflag = 0;	//fcwԤ��
	static uint8_t	hflag = 0;	//hmwԤ��
	// AEBS�¼� ��ʼ�����������һ��

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
 * �����ϴ�-������Ϣ
 * ��������
 * ���أ�
 */
uint32_t upload_CarBody_time = 0;
void UpLoad_Car_Body_Info()
{
	uint8_t carBody_info[13]		= {PROTOCOL_FRONT};
	static uint8_t Rflag = 0;
	static uint8_t Lflag = 0;
	static uint8_t Bflag = 0;
	static uint8_t Dflag = 0;

	if(SystemtimeClock - upload_CarBody_time > 100){	// ������������ʱ10ms,����GPS�豸�޷�����Ϣ��
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
			// ��ȡ����--�ֽ�4����Ч0XFF
			carBody_info[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
			// �ֽ�5��D8-D7��������
			// ��ȡAEB����״̬--�ֽ�5��D6
			carBody_info[4] = Read_AEB_switch() << 5;
			// ��ȡ��ת���״̬--�ֽ�5��D5
			carBody_info[4] |= (stVehicleParas.RightFlagTemp&0x01) << 4;
			// ��ȡ��ת���״̬--�ֽ�5��D4
			carBody_info[4] |= (stVehicleParas.LeftFlagTemp&0x01) << 3;
			// ��ȡɲ����״̬--�ֽ�5��D3
			carBody_info[4] |= (stVehicleParas.BrakeFlag&0x01) << 2;
			// ��ȡ��λ״̬--�ֽ�5��D2-D1
			carBody_info[4] |=  stVehicleParas.Car_Gear;			// ���޸�
			// ��ȡ������--�ֽ�6��D8-D1
			if(camera_data.ErrorCode == 0){								// �޹����룬�ͱ������쳣
				//if(stCanCommSta.stRadar.status == OFFLINE)				// ���ײ�ͨѶ�쳣
				//	carBody_info[5] = 128;
				//if(stCanCommSta.stOBD.status == OFFLINE)				// OBDͨѶ�쳣
				//	carBody_info[5] = 129;
				if(stCanCommSta.stHRadar.status == OFFLINE)				// ������ͨѶ�쳣
					carBody_info[5] = 130;
				if(stCanCommSta.stVehicle.status == OFFLINE)			// AEBS�������ڲ�ͨ���쳣
					carBody_info[5] = 131;
				//if(stCanCommSta.stWireless.status == OFFLINE)			// AEBS��GPSͨѶ(4Gģ��)�쳣
					//carBody_info[5] = 132;
			}else{
				carBody_info[5] = camera_data.ErrorCode;
			}
			// ��ȡ����̤�忪��--�ֽ�7����Ч0XFF
			carBody_info[6] = stVehicleParas.ThrottleOpening;// * 2.5;
			// ��ȡɲ��̤�忪��--�ֽ�8����Ч0xFF
			carBody_info[7] = stVehicleParas.BrakeOpening;// * 2.5;
			// ��ȡ�����̽Ƕ�(LSB)|(MSB)--�ֽ�9���ֽ�10����Ч0x7FF
			carBody_info[8] = stSWAParas.SWADegree & _0XFF;
			carBody_info[9] = (stSWAParas.SWADegree&0x07) >> 8;
			// ������״̬ D4	��0������򡿡���1�����Ҵ�Direction
			carBody_info[9] |= stSWAParas.Direction << 3;
			// ����--�ֽ�11��0xFF
			carBody_info[10] = _0XFF;
			// ����--�ֽ�12��0xFF
			carBody_info[11] = _0XFF;
			// ���У��--�ֽ�13
			for(uint8_t i=0;i<12;i++)
				carBody_info[12] ^= carBody_info[i];
			// ����
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
 * �ſ�4Gģ�������ϴ��������жϺ��������߼�
 * ������data��������
 * ���أ���
 */
volatile uint8_t gps_times 		= 0;			// 1��ظ�һ��GPS�豸����
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
		case CAR_VELOCITY_TYPE:{		// 50ms����һ�γ�����Ϣ����1s�ظ�һ��
			if(replay_cnt >= 6){
				replay_cnt = 0;
				if(gps_times++ > 20){	// 50ms��1��ظ�һ��,����������
					gps_times = 0;
					gps_reponse = SUCCESS;
					for(uint8_t i=0;i<5;i++){
						checkout ^= replay_res[i];
					}
					if(checkout == replay_res[5]){
						// ��Ч���ݣ�������������
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
	}else if(replay_cnt >= 6){			// �����쳣
		replay_cnt = 0;
	}
}
/*
 * �ſ�4Gģ����main����ѭ����
 */
void Data_Uploading_In_MainLoop()
{
	// �ϴ������豸������Ϣ
	UpLoad_Vehicle_Device_Info();
//	// �ϴ�������Ϣ
	UpLoad_Car_Body_Info();
	// �ϴ�Ԥ����Ϣ
	UpLoad_Early_Warning_Info();
//	Test_Early_Warning_Info();
	if(gps_reponse == SUCCESS)
	{
		gps_reponse = FAILURE;
		Response(CAR_VELOCITY_TYPE,carKu_replay.gpsVelocity_res);
	}
}
/*
 * �ظ��ſ�4Gģ��
 * ������dataType�������ͣ�status����״̬
 * ���أ���
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
	// ����
	USART_Send(USART2_SFR,response_res,5);
	memset(response_res,0,sizeof(response_res));
}

/*
 * ���ղſ�4Gģ������У��
 * ��������������
 * ���أ�0У��ʧ�ܣ�1��������ʧ�ܣ�2�������ݳɹ�
 */
uint8_t Checkout(uint8_t *data)
{
	for(uint8_t i=0;i<5;i++){
		checkout ^= data[i];
	}
	if(checkout == data[4]){
		// ��Ч���ݣ�������������
		if(data[3] == 1)
			return 2;	// �ɹ�
		else
			return 1;	// ʧ��
	}else{
		return 0;		// У��ʧ��
	}
}

void Early_Warning_Info_start1(uint8_t type,uint8_t Serialnumber)
{
	uint8_t earlyWarning_info[24] = {PROTOCOL_FRONT};
	earlyWarning_info[1] = EARLY_WARNING_TYPE;
	earlyWarning_info[2] = 0x14;
	// ��ȡ����--�ֽ�4����Ч0xFF
	earlyWarning_info[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
	// ��ȡTTCʱ��--�ֽ�5����Ч0x3F
	/*
	if((rx_obstacle_info_b.data[4] & _0X3F) == _0X3F)
		earlyWarning_info[4] = _0X3F;
	else
		earlyWarning_info[4] = (uint8_t)(CameraMessage.ttc * 10);
//	earlyWarning_info[4] = 60;
	// ��ȡHMWʱ��--�ֽ�6����Ч0x3F
	if(((uint8_t)(camera_data.HMW * 10) & _0X3F) == _0X3F)
		earlyWarning_info[5] = _0X3F;
	else
		earlyWarning_info[5] = (uint8_t)(camera_data.HMW * 10);
	*/
	earlyWarning_info[4] = (uint8_t)(CameraMessage.ttc * 10);
	earlyWarning_info[5] = (uint8_t)(CameraMessage.hmw * 10);
//	earlyWarning_info[5] = 40;
	// ��ȡ������Ծ���Z(LSB)|Z(MSB)--�ֽ�7���ֽ�8����Ч0x7FFF
	earlyWarning_info[6] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceZ*100) & _0XFF);
	earlyWarning_info[7] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceZ*100) >> 8 & _0XFF);
	// ��ȡ��������ٶ�(LSB)|(MSB)--�ֽ�9���ֽ�10����Ч0xFFF//obstacle_cipv_data.RelativeSpeedZ
	earlyWarning_info[8] = (uint8_t)((uint16_t)((obstacle_cipv_data.RelativeSpeedZ + 127.9375)/0.0625) & _0XFF);
	earlyWarning_info[9] = (uint8_t)((uint16_t)((obstacle_cipv_data.RelativeSpeedZ + 127.9375)/0.0625)>>8 & _0XFF);
	// ��ȡ������Ծ���X(LSB)|X(MSB)--�ֽ�11���ֽ�12����Ч0x7FFF
	earlyWarning_info[10] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceX*100) & _0XFF);
	earlyWarning_info[11] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceX*100)>>8 & _0XFF);
	// ��ȡ����̤�忪��--�ֽ�13����Ч0xFF
	earlyWarning_info[12] = stVehicleParas.ThrottleOpening;// * 2.5;
	// ��ȡɲ��̤�忪��--�ֽ�14����Ч0xFF
	earlyWarning_info[13] = stVehicleParas.BrakeOpening;// * 2.5;
	// ��ȡ�����̽Ƕ�(LSB)|(MSB)--�ֽ�15���ֽ�16����Ч0x7FF
	earlyWarning_info[14] = stSWAParas.SWADegree & _0XFF;				// �����̽Ƕȵ�8λ
	earlyWarning_info[15] = (stSWAParas.SWADegree>>8) & 0x07;			// �����̽Ƕȸ�3λ
	earlyWarning_info[15] |= (stSWAParas.Direction&0x01) << 3;			// ������״̬
	// ��ȡ�󳵵�������--�ֽ�17 D8-D5 ��0���ޡ�����1��Ԥ�⡿����2�����ߡ�����3��ʵ�ߡ�����4��˫���ߡ�����5��˫ʵ�ߡ�����6�����ߡ�����7��15��������
	earlyWarning_info[16] = (camera_LDW_data.LeftLaneStyle&_0X0F) << 4;
	// ��ȡ�ҳ���������--�ֽ�17 D4-D1 ��0���ޡ�����1��Ԥ�⡿����2�����ߡ�����3��ʵ�ߡ�����4��˫���ߡ�����5��˫ʵ�ߡ�����6�����ߡ�����7��15��������
	earlyWarning_info[16] |= camera_LDW_data.RightLaneStyle  & _0X0F;
	// ��ȡFCWLevelǰ����ײԤ���ȼ�--�ֽ�18 D2-D1	��0���񡿡���1��һ��Ԥ��������2������Ԥ����
	earlyWarning_info[17] = camera_data.FCWLevel & 0x03;
	// ��ȡ�ϰ�������--�ֽ�18 D6-D3	��0����Чֵ������1������������2���ˡ�����3-11��������
	earlyWarning_info[17] |= (obstacle_cipv_data.ObstacleType & _0X0F) << 2;
	// ��ȡ�ҳ�����ƫ��Ԥ��--�ֽ�18 D7	��0����]����1���ǡ�
	earlyWarning_info[17] |= (camera_data.RightLDW&0x01) << 6;
	// ��ȡ�󳵵���ƫ��Ԥ��--�ֽ�18 D8	��0����]����1���ǡ�
	earlyWarning_info[17] |= (camera_data.LeftLDW&0x01) << 7;
	// ��ȡAEB�ƶ�״̬--�ֽ�19 D8-D7	��0�����ƶ�������1��˫Ŀ�ƶ�������2�����ײ��ƶ�������3���������ƶ���
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
	// ��ȡ��λ״̬--�ֽ�19 D6-D5	��0����Ч������1��N��������2��D��������3��R����
	earlyWarning_info[18] |= (stVehicleParas.Car_Gear) << 4;
	// ��ȡ�¼�����--�ֽ�19 D3-D1	��1��FCW������2��HMW������3��LDW������4��AEB��	���޸�
	if(camera_data.LeftLDW || camera_data.RightLDW){	// ����ƫ��
		event_t.ldw_t = 1;
		event_t.event = LDW_EVENT;
	}else event_t.ldw_t = 0;

	switch(event_t.event){
	case FCW_EVENT:
		earlyWarning_info[18] |= event_t.fcw_t<<3;		// �����¼� ��ʼʱ�� ��ʼ|����
		earlyWarning_info[18] |= event_t.event & 0x07;	// �����¼�
		break;
	case HMW_EVENT:
		earlyWarning_info[18] |= event_t.hmw_t<<3;		// �����¼� ��ʼʱ�� ��ʼ|����
		earlyWarning_info[18] |= event_t.event & 0x07;	// �����¼�
		break;
	case LDW_EVENT:
		earlyWarning_info[18] |= event_t.ldw_t<<3;		// �����¼� ��ʼʱ�� ��ʼ|����
		earlyWarning_info[18] |= event_t.event & 0x07;	// �����¼�
		break;
	case AEB_EVENT:
		earlyWarning_info[18] |= event_t.aeb_t<<3;		// �����¼� ��ʼʱ�� ��ʼ|����
		earlyWarning_info[18] |= event_t.event & 0x07;	// �����¼�
		break;
	}
	 */
	earlyWarning_info[18] = type;
	earlyWarning_info[18] |= (stVehicleParas.Car_Gear) << 4;
	// ��ȡ����������--�ֽ�20 D8-D1����Ч0XFF
	if((type & 0xc0) == 0xc0)
		earlyWarning_info[19] = stVehicleParas.Ultrasonicdistance + 1;
	else
		earlyWarning_info[19] = 0xff;
	// ����--�ֽ�21��0xFF
	earlyWarning_info[20] = _0XFF;
	// ����--�ֽ�22��0xFF
	earlyWarning_info[21] = _0XFF;
	// ����--�ֽ�23��0xFF
	earlyWarning_info[22] = Serialnumber;//_0XFF;
	// ���У��--�ֽ�24
	for(uint8_t i=0;i<23;i++)
		earlyWarning_info[23] ^= earlyWarning_info[i];
//	for(uint8_t i=0;i<24;i++)
//		fprintf(USART0_STREAM,"%02X ",earlyWarning_info[i]);
//	fprintf(USART0_STREAM,"\r\n");
	// ����
	USART_Send(USART2_SFR,earlyWarning_info,24);
	//memset(earlyWarning_info,0,sizeof(earlyWarning_info));
}

/*
 * Ԥ����Ϣ
 * ��������
 * ���أ���
 */
void Early_Warning_Info_start(void)
{
	uint8_t earlyWarning_info[24] = {PROTOCOL_FRONT};
	earlyWarning_info[1] = EARLY_WARNING_TYPE;
	earlyWarning_info[2] = 0x14;
	// ��ȡ����--�ֽ�4����Ч0xFF
	earlyWarning_info[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
	// ��ȡTTCʱ��--�ֽ�5����Ч0x3F
	/*
	if((rx_obstacle_info_b.data[4] & _0X3F) == _0X3F)
		earlyWarning_info[4] = _0X3F;
	else
		earlyWarning_info[4] = (uint8_t)(CameraMessage.ttc * 10);
//	earlyWarning_info[4] = 60;
	// ��ȡHMWʱ��--�ֽ�6����Ч0x3F
	if(((uint8_t)(camera_data.HMW * 10) & _0X3F) == _0X3F)
		earlyWarning_info[5] = _0X3F;
	else
		earlyWarning_info[5] = (uint8_t)(camera_data.HMW * 10);
	*/
	earlyWarning_info[4] = (uint8_t)(CameraMessage.ttc * 10);
	earlyWarning_info[5] = (uint8_t)(CameraMessage.hmw * 10);
//	earlyWarning_info[5] = 40;
	// ��ȡ������Ծ���Z(LSB)|Z(MSB)--�ֽ�7���ֽ�8����Ч0x7FFF
	earlyWarning_info[6] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceZ*100) & _0XFF);
	earlyWarning_info[7] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceZ*100) >> 8 & _0XFF);
	// ��ȡ��������ٶ�(LSB)|(MSB)--�ֽ�9���ֽ�10����Ч0xFFF
	earlyWarning_info[8] = (uint8_t)((uint16_t)(obstacle_cipv_data.RelativeSpeedZ*16) & _0XFF);
	earlyWarning_info[9] = (uint8_t)((uint16_t)(obstacle_cipv_data.RelativeSpeedZ*16)>>8 & _0XFF);
	// ��ȡ������Ծ���X(LSB)|X(MSB)--�ֽ�11���ֽ�12����Ч0x7FFF
	earlyWarning_info[10] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceX*100) & _0XFF);
	earlyWarning_info[11] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceX*100)>>8 & _0XFF);
	// ��ȡ����̤�忪��--�ֽ�13����Ч0xFF
	earlyWarning_info[12] = stVehicleParas.ThrottleOpening;// * 2.5;
	// ��ȡɲ��̤�忪��--�ֽ�14����Ч0xFF
	earlyWarning_info[13] = stVehicleParas.BrakeOpening;// * 2.5;
	// ��ȡ�����̽Ƕ�(LSB)|(MSB)--�ֽ�15���ֽ�16����Ч0x7FF
	earlyWarning_info[14] = stSWAParas.SWADegree & _0XFF;				// �����̽Ƕȵ�8λ
	earlyWarning_info[15] = (stSWAParas.SWADegree>>8) & 0x07;			// �����̽Ƕȸ�3λ
	earlyWarning_info[15] |= (stSWAParas.Direction&0x01) << 3;			// ������״̬
	// ��ȡ�󳵵�������--�ֽ�17 D8-D5 ��0���ޡ�����1��Ԥ�⡿����2�����ߡ�����3��ʵ�ߡ�����4��˫���ߡ�����5��˫ʵ�ߡ�����6�����ߡ�����7��15��������
	earlyWarning_info[16] = (camera_LDW_data.LeftLaneStyle&_0X0F) << 4;
	// ��ȡ�ҳ���������--�ֽ�17 D4-D1 ��0���ޡ�����1��Ԥ�⡿����2�����ߡ�����3��ʵ�ߡ�����4��˫���ߡ�����5��˫ʵ�ߡ�����6�����ߡ�����7��15��������
	earlyWarning_info[16] |= camera_LDW_data.RightLaneStyle  & _0X0F;
	// ��ȡFCWLevelǰ����ײԤ���ȼ�--�ֽ�18 D2-D1	��0���񡿡���1��һ��Ԥ��������2������Ԥ����
	earlyWarning_info[17] = camera_data.FCWLevel & 0x03;
	// ��ȡ�ϰ�������--�ֽ�18 D6-D3	��0����Чֵ������1������������2���ˡ�����3-11��������
	earlyWarning_info[17] |= (obstacle_cipv_data.ObstacleType & _0X0F) << 2;
	// ��ȡ�ҳ�����ƫ��Ԥ��--�ֽ�18 D7	��0����]����1���ǡ�
	earlyWarning_info[17] |= (camera_data.RightLDW&0x01) << 6;
	// ��ȡ�󳵵���ƫ��Ԥ��--�ֽ�18 D8	��0����]����1���ǡ�
	earlyWarning_info[17] |= (camera_data.LeftLDW&0x01) << 7;
	// ��ȡAEB�ƶ�״̬--�ֽ�19 D8-D7	��0�����ƶ�������1��˫Ŀ�ƶ�������2�����ײ��ƶ�������3���������ƶ���
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
	// ��ȡ��λ״̬--�ֽ�19 D6-D5	��0����Ч������1��N��������2��D��������3��R����
	earlyWarning_info[18] |= (stVehicleParas.Car_Gear) << 4;
	// ��ȡ�¼�����--�ֽ�19 D3-D1	��1��FCW������2��HMW������3��LDW������4��AEB��	���޸�
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
	if(camera_data.LeftLDW || camera_data.RightLDW){	// ����ƫ��
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
		earlyWarning_info[18] |= event_t.fcw_t<<3;		// �����¼� ��ʼʱ�� ��ʼ|����
		earlyWarning_info[18] |= event_t.event & 0x07;	// �����¼�
		break;
	case HMW_EVENT:
		earlyWarning_info[18] |= event_t.hmw_t<<3;		// �����¼� ��ʼʱ�� ��ʼ|����
		earlyWarning_info[18] |= event_t.event & 0x07;	// �����¼�
		break;
	case LDW_EVENT:
		earlyWarning_info[18] |= event_t.ldw_t<<3;		// �����¼� ��ʼʱ�� ��ʼ|����
		earlyWarning_info[18] |= event_t.event & 0x07;	// �����¼�
		break;
	case AEB_EVENT:
		earlyWarning_info[18] |= event_t.aeb_t<<3;		// �����¼� ��ʼʱ�� ��ʼ|����
		earlyWarning_info[18] |= event_t.event & 0x07;	// �����¼�
		break;
	}

	// ��ȡ����������--�ֽ�20 D8-D1����Ч0XFF
	earlyWarning_info[19] = stVehicleParas.Ultrasonicdistance;
	// ����--�ֽ�21��0xFF
	earlyWarning_info[20] = _0XFF;
	// ����--�ֽ�22��0xFF
	earlyWarning_info[21] = _0XFF;
	// ����--�ֽ�23��0xFF
	earlyWarning_info[22] = _0XFF;
	// ���У��--�ֽ�24
	for(uint8_t i=0;i<23;i++)
		earlyWarning_info[23] ^= earlyWarning_info[i];
//	for(uint8_t i=0;i<24;i++)
//		fprintf(USART0_STREAM,"%02X ",earlyWarning_info[i]);
//	fprintf(USART0_STREAM,"\r\n");
	// ����
	USART_Send(USART2_SFR,earlyWarning_info,24);
	//memset(earlyWarning_info,0,sizeof(earlyWarning_info));
}

void Early_Warning_Info()
{
	uint8_t earlyWarning_info[24] = {PROTOCOL_FRONT};
	earlyWarning_info[1] = EARLY_WARNING_TYPE;
	earlyWarning_info[2] = 0x14;
	// ��ȡ����--�ֽ�4����Ч0xFF
	earlyWarning_info[3] = (uint8_t)stVehicleParas.fVehicleSpeed;
	// ��ȡTTCʱ��--�ֽ�5����Ч0x3F
	if((rx_obstacle_info_b.data[4] & _0X3F) == _0X3F)
		earlyWarning_info[4] = _0X3F;
	else
		earlyWarning_info[4] = (uint8_t)(CameraMessage.ttc * 10);
//	earlyWarning_info[4] = 60;
	// ��ȡHMWʱ��--�ֽ�6����Ч0x3F
	if(((uint8_t)(camera_data.HMW * 10) & _0X3F) == _0X3F)
		earlyWarning_info[5] = _0X3F;
	else
		earlyWarning_info[5] = (uint8_t)(camera_data.HMW * 10);
//	earlyWarning_info[5] = 40;
	// ��ȡ������Ծ���Z(LSB)|Z(MSB)--�ֽ�7���ֽ�8����Ч0x7FFF
	earlyWarning_info[6] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceZ*100) & _0XFF);
	earlyWarning_info[7] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceZ*100) >> 8 & _0XFF);
	// ��ȡ��������ٶ�(LSB)|(MSB)--�ֽ�9���ֽ�10����Ч0xFFF
	earlyWarning_info[8] = (uint8_t)((uint16_t)(obstacle_cipv_data.RelativeSpeedZ*16) & _0XFF);
	earlyWarning_info[9] = (uint8_t)((uint16_t)(obstacle_cipv_data.RelativeSpeedZ*16)>>8 & _0XFF);
	// ��ȡ������Ծ���X(LSB)|X(MSB)--�ֽ�11���ֽ�12����Ч0x7FFF
	earlyWarning_info[10] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceX*100) & _0XFF);
	earlyWarning_info[11] = (uint8_t)((uint16_t)(obstacle_cipv_data.DistanceX*100)>>8 & _0XFF);
	// ��ȡ����̤�忪��--�ֽ�13����Ч0xFF
	earlyWarning_info[12] = stVehicleParas.ThrottleOpening;// * 2.5;
	// ��ȡɲ��̤�忪��--�ֽ�14����Ч0xFF
	earlyWarning_info[13] = stVehicleParas.BrakeOpening;// * 2.5;
	// ��ȡ�����̽Ƕ�(LSB)|(MSB)--�ֽ�15���ֽ�16����Ч0x7FF
	earlyWarning_info[14] = stSWAParas.SWADegree & _0XFF;				// �����̽Ƕȵ�8λ
	earlyWarning_info[15] = (stSWAParas.SWADegree>>8) & 0x07;			// �����̽Ƕȸ�3λ
	earlyWarning_info[15] |= (stSWAParas.Direction&0x01) << 3;			// ������״̬
	// ��ȡ�󳵵�������--�ֽ�17 D8-D5 ��0���ޡ�����1��Ԥ�⡿����2�����ߡ�����3��ʵ�ߡ�����4��˫���ߡ�����5��˫ʵ�ߡ�����6�����ߡ�����7��15��������
	earlyWarning_info[16] = (camera_LDW_data.LeftLaneStyle&_0X0F) << 4;
	// ��ȡ�ҳ���������--�ֽ�17 D4-D1 ��0���ޡ�����1��Ԥ�⡿����2�����ߡ�����3��ʵ�ߡ�����4��˫���ߡ�����5��˫ʵ�ߡ�����6�����ߡ�����7��15��������
	earlyWarning_info[16] |= camera_LDW_data.RightLaneStyle  & _0X0F;
	// ��ȡFCWLevelǰ����ײԤ���ȼ�--�ֽ�18 D2-D1	��0���񡿡���1��һ��Ԥ��������2������Ԥ����
	earlyWarning_info[17] = camera_data.FCWLevel & 0x03;
	// ��ȡ�ϰ�������--�ֽ�18 D6-D3	��0����Чֵ������1������������2���ˡ�����3-11��������
	earlyWarning_info[17] |= (obstacle_cipv_data.ObstacleType & _0X0F) << 2;
	// ��ȡ�ҳ�����ƫ��Ԥ��--�ֽ�18 D7	��0����]����1���ǡ�
	earlyWarning_info[17] |= (camera_data.RightLDW&0x01) << 6;
	// ��ȡ�󳵵���ƫ��Ԥ��--�ֽ�18 D8	��0����]����1���ǡ�
	earlyWarning_info[17] |= (camera_data.LeftLDW&0x01) << 7;
	// ��ȡAEB�ƶ�״̬--�ֽ�19 D8-D7	��0�����ƶ�������1��˫Ŀ�ƶ�������2�����ײ��ƶ�������3���������ƶ���
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
	// ��ȡ��λ״̬--�ֽ�19 D6-D5	��0����Ч������1��N��������2��D��������3��R����

	earlyWarning_info[18] |= (stVehicleParas.Car_Gear) << 4;
	// ��ȡ�¼�����--�ֽ�19 D3-D1	��1��FCW������2��HMW������3��LDW������4��AEB��	���޸�
	if(camera_data.FCWStatus){ 			// FCW
		event_t.fcw_t = 1;
		event_t.event = FCW_EVENT;
	}else event_t.fcw_t = 0;

	if(camera_data.HMWEnable) {			// HMW
		event_t.hmw_t = 1;
		event_t.event = HMW_EVENT;
	}else event_t.hmw_t = 0;

	if(camera_data.LeftLDW || camera_data.RightLDW){	// ����ƫ��
		event_t.ldw_t = 1;
		event_t.event = LDW_EVENT;
	}else event_t.ldw_t = 0;

	if(stVehicleParas.BreakState != 0){	// AEB
		event_t.aeb_t = 1;
		event_t.event = AEB_EVENT;
	}else event_t.aeb_t = 0;
	switch(event_t.event){
	case FCW_EVENT:
		earlyWarning_info[18] |= event_t.fcw_t<<3;		// �����¼� ��ʼʱ�� ��ʼ|����
		earlyWarning_info[18] |= event_t.event & 0x07;	// �����¼�
		break;
	case HMW_EVENT:
		earlyWarning_info[18] |= event_t.hmw_t<<3;		// �����¼� ��ʼʱ�� ��ʼ|����
		earlyWarning_info[18] |= event_t.event & 0x07;	// �����¼�
		break;
	case LDW_EVENT:
		earlyWarning_info[18] |= event_t.ldw_t<<3;		// �����¼� ��ʼʱ�� ��ʼ|����
		earlyWarning_info[18] |= event_t.event & 0x07;	// �����¼�
		break;
	case AEB_EVENT:
		earlyWarning_info[18] |= event_t.aeb_t<<3;		// �����¼� ��ʼʱ�� ��ʼ|����
		earlyWarning_info[18] |= event_t.event & 0x07;	// �����¼�
		break;
	}
	// ��ȡ����������--�ֽ�20 D8-D1����Ч0XFF
	earlyWarning_info[19] = stVehicleParas.Ultrasonicdistance;;
	// ����--�ֽ�21��0xFF
	earlyWarning_info[20] = _0XFF;
	// ����--�ֽ�22��0xFF
	earlyWarning_info[21] = _0XFF;
	// ����--�ֽ�23��0xFF
	earlyWarning_info[22] = _0XFF;
	// ���У��--�ֽ�24
	for(uint8_t i=0;i<23;i++)
		earlyWarning_info[23] ^= earlyWarning_info[i];
//	for(uint8_t i=0;i<24;i++)
//		fprintf(USART0_STREAM,"%02X ",earlyWarning_info[i]);
//	fprintf(USART0_STREAM,"\r\n");
	// ����
	USART_Send(USART2_SFR,earlyWarning_info,24);
	//memset(earlyWarning_info,0,sizeof(earlyWarning_info));
}

