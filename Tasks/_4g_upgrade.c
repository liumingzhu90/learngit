/*
 * _4g_upgrade.c
 *
 *  Created on: 2021-12-3
 *      Author: Administrator
 * ����˵��������OTA����|�����ϴ�|��ȡGPS��λ��Ϣ|��������
 */
#include "_4g_upgrade.h"
#include "usart.h"
#include "stdlib.h"
#include "w25qxx.h"
#include "utc_time.h"
#include "md5.h"
#include "common.h"
#include "_4g_data_upload.h"
//#include "gpio.h"
/****************************�ꡢö�ٶ���**************************************/

/****************************�ṹ�嶨��**************************************/
typedef struct{	// ʱ��Ƭ�л���־λ
	AbleStatus t0;	// 10ms
//	AbleStatus t1;	// 100ms
//	AbleStatus t2;	// 200ms
//	AbleStatus t3;	// 300ms
//	AbleStatus t4;	// 400ms
	AbleStatus t5;	// 500ms
}Delay_Time;

/****************************ȫ�ֱ���**************************************/
// OTA
Delay_Time			dly_time 		= {0};
uint32_t 			utctime			= 0;		// UTCTIMEʱ�䣬��λ��
uint8_t 			pkg_times 		= 0;		// ����У�����
PlatfromServerPara 	server_p;
UpgradePara			upgrade_p;
/****************************��������**************************************/
// OTA
void 	Clear_Timing();
void 	AT_CCLK();
void 	AT_CPIN_To_Get_SIM_Status();
void 	AT_CREG_To_Judge_GSM();
void 	AT_QIOPEN_TO_Connect_Server();
void 	AT_QISEND_To_Get_Secret_Cmd();
void 	AT_QISEND_To_Get_Secret_Data();
void 	AT_QISEND_To_Register_Device_Cmd();
void 	AT_QISEND_To_Register_Device_Data();
void 	AT_QISEND_To_Request_Update_Cmd();
void 	AT_QISEND_To_Request_Update_Data();
void 	AT_QISEND_To_Replay_Success_Cmd();
void 	AT_QISEND_To_Replay_Success_Data();
uint8_t Analysis_UTCTIME(char * time_string);
int 	Get_Useful_Data(const char *buffer, char *input, char *output);
uint8_t Analysis_UpgradePackage_Info(char * strx);
uint8_t Check_For_Upgrade();
void 	AT_QHTTPURL_To_Upgrade_Stage1();
void 	AT_QHTTPURL_To_Upgrade_Stage2();
void 	AT_QHTTPURL_To_Upgrade_Stage3();
void 	AT_QHTTPURL_To_Upgrade_Stage4();
uint8_t TimeString_To_UTCTIME(char * time_string);
uint8_t Write_End_PKG(uint8_t *data,uint16_t len);
uint8_t OTA_Upgrade(Quectel4GStatus _ota_status);
void 	Analysis_Ec200u_SN(uint8_t *str);

/*
 * sscanf()��������øú���
 */
char getche(void)
{
	return getchar();
}
/****************************OTA****************************************/
/*
 * ��������������
 * ˵���������У�����4Gָʾ����˸
 */
uint32_t upgrade_Warning_Clk = 0;
void Upgrade_Warning()
{
	if(SystemtimeClock - upgrade_Warning_Clk > 50){
		upgrade_Warning_Clk = SystemtimeClock;
		GPIO_Toggle_Output_Data_Config(GPIOF_SFR, GPIO_PIN_MASK_0);	// 4Gָʾ��
	}
}
/*
 * 4Gģ���ʼ��
 */
void AT_4G_Module_Init()
{
	upgrade_p.rx_mode 		= RECV_CMD;
	upgrade_p.recv_ok 		= FAILURE;
	upgrade_p.pkg.ota_rx 	= BIN_RECV_WAIT;
	server_p.sevIsConnect  	= FALSE;
	server_p._4G_status 	= OTA_RDY;
	server_p.stage			= 0;


	EC200U_INIT();						// USART0 �����˽����жϣ�19200bps

	uint8_t version[VERSION_SIZE];
	Get_Upgrade_App_Version(version);			// ��ȡplatform�汾��
	strcpy(upgrade_p.info.run_v, (char *)version);
	Get_Run_App_Version(version);	// ��ȡplatform�汾��
	fprintf(USART1_STREAM,"------Upgrade_V---%s-------------\r\n",upgrade_p.info.run_v);
	fprintf(USART1_STREAM,"------Run_V-------%s-------------\r\n",version);

	upgrade_p.up_mode 		= UPGRADE_OTA;
	upgrade_p.info.mode		= UPGRADE_OTA;
	upgrade_p.recv_ok 		= FAILURE;
	// ����bin�ļ�����
	upgrade_p.pkg.cnt 		= 0;
	upgrade_p.pkg.remain 	= 0;
	upgrade_p.pkg.times 	= 0;
	upgrade_p.pkg.addr		= OUT_FLASH_APP_START;
	upgrade_p.pkg.rx_crc 	= 0;
	upgrade_p.pkg.pkg_crc 	= 0;
	upgrade_p.pkg.total		= 0;
	memset(upgrade_p.pkg.data1, 0, BUFFER_SIZE);
	memset(upgrade_p.pkg.data2, 0, BUFFER_SIZE);
	// ������Ϣ����
	upgrade_p.info.id 		= CONNECT_ID;
	upgrade_p.info.size 	= 0;
//	Set_Device_SN("21122700001");
	Get_Device_SN(upgrade_p.info.sn);
	// �������http·��
	Get_Http_Download_Addr(upgrade_p.info.https);
	memcpy(upgrade_p.info.https_hu,upgrade_p.info.https,strlen(upgrade_p.info.https));
	// ���ӷ�����������OTA���ӿ�ʼ����RDYλ�ÿ�ʼ
	Connect_Server_Start(TRUE);
}
/*
 * ��ʼ���ӷ�����
 * ������TRUE��ʼOTA���ܣ�FALSE������OTA����;restart��//TRUE��OTA_RDY������FALSE��OTA_AT����
 * ���أ���
 */
void Connect_Server_Start(FunctionalState startOTA)
{
	server_p.sevIsConnect	 		= FALSE;
	server_p._4G_status 			= OTA_RDY;
	server_p.startOTA				= startOTA;
}
/*
 * OTA��ʼ����
 * ������UPGRADE_OTA�ϵ�������UPGRADE_OTA_CMDָ������
 * ˵��:3������ģʽ
 * ota_p.up_mode	ota_p.info.mode		 	����ģʽ
 * -------------------------------------------------------------
 * UPGRADE_OTA		UPGRADE_OTA				�ϵ�OTA����
 * UPGRADE_OTA		UPGRADE_OTA_CMD			ָ��OTA����
 * UPGRADE_USART1							��������
 */
void OTA_Start(Upgrade_Mode upgrade_mode,uint32_t size)
{
	server_p._4G_status		= OTA_REQUEST;

	upgrade_p.up_mode 		= UPGRADE_OTA;
	upgrade_p.info.mode		= upgrade_mode;
	upgrade_p.isFinish 		= FALSE;
	upgrade_p.recv_ok 		= FAILURE;
	// ����bin�ļ�����
	upgrade_p.pkg.cnt 		= 0;
	upgrade_p.pkg.remain 	= 0;
	upgrade_p.pkg.times 	= 0;
	upgrade_p.pkg.addr		= OUT_FLASH_APP_START;
	upgrade_p.pkg.rx_crc 	= 0;
	upgrade_p.pkg.pkg_crc 	= 0;
	upgrade_p.pkg.total		= 0;
	upgrade_p.pkg.is_write 	= 0;
	upgrade_p.pkg.sw_12		= 0;
	memset(upgrade_p.pkg.data1, 0, BUFFER_SIZE);
	memset(upgrade_p.pkg.data2, 0, BUFFER_SIZE);
	// ������Ϣ����
	upgrade_p.info.id 		= CONNECT_ID;
	upgrade_p.info.size 	= size;
	Get_Device_SN(upgrade_p.info.sn);

	AT_QISEND_To_Request_Update_Cmd();// ����
}
/*
 * OTA��������
 */
void OTA_End(FunctionalState flag)
{
	upgrade_p.isFinish 		= flag;
	upgrade_p.rx_mode 		= RECV_CMD;		// ָ��ģʽ

	upgrade_p.up_mode		= UPGRADE_USART1;
	upgrade_p.info.mode 	= UPGRADE_NO;
	upgrade_p.pkg.ota_rx	= BIN_RECV_WAIT;
	upgrade_p.recv_ok 		= FAILURE;
	upgrade_p.pkg.cnt 		= 0;
	upgrade_p.pkg.total		= 0;
	upgrade_p.pkg.len		= 0;
	upgrade_p.pkg.rx_crc 	= 0;
	upgrade_p.pkg.pkg_crc 	= 0;
	server_p._4G_status		= OTA_WAIT;
	server_p.stage 			= 0;
	Clear_Buffer();
	GPS_Start();							// ����GPS
//	fprintf(USART1_STREAM,"OTA end.\n");
}
/*
 * ����ƽ̨����������
 * �������׶�״̬;�Ƿ��ota,TRUE,FALSE
 * ���أ�0ʧ�ܣ�1���ɹ�
 * ˵�������ӷ���������
 */
uint8_t ota_loop_times 		= 0;
uint8_t ota_err_times 		= 0;	// �쳣����2�μ�ⶼ���쳣���˳�
uint8_t ota_loop_err_times 	= 0;	//
char 	*strx 				= NULL;
uint32_t connect_server_clk = 0;
uint8_t Connect_Platform_Server(Quectel4GStatus _4G_status,FunctionalState ota_on)
{
	if(SystemtimeClock - connect_server_clk < 500) return 0;		// 500ms
	connect_server_clk = SystemtimeClock;

	if(server_p.sevIsConnect == TRUE) return 1;
//	fprintf(USART1_STREAM,"%s\r\n",EC200U_Rxbuffer);
	switch(_4G_status)
	{
	case OTA_RDY:
		if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"RDY")){
			ota_loop_times = 0;
			AT_CCLK();// ����UTCʱ��ָ��
		}else if(ota_loop_times > 10){	// 5s����
			ota_err_times++;
			if(ota_err_times > 4){		// ����4�Σ�����Ϊ�����������ߣ��Ϳ������������м��
				// �����������,���п����ڷ���������������,�����ֶ�> telnet serverip port
				ota_err_times = 0;
				fprintf(USART1_STREAM,"connect server failed.\r\n");
				OTA_End(FALSE);
				return 1;
			}
			ota_loop_times = 0;
			Module_4G_ReStart();
		}else ota_loop_times++;
		break;
	case OTA_UTCTIME:		//
		if(upgrade_p.recv_ok == SUCCESS){
//			fprintf(USART1_STREAM,"%s\r\n",EC200U_Rxbuffer);
			char *strx = strstr((const char*)EC200U_Rxbuffer,(const char*)"+CCLK:");
			if(strx){
				TimeString_To_UTCTIME(strx+8);// ����utctime
				AT();// ����ATָ��
			}else
				AT_CCLK();
		}
		break;
	case OTA_AT:
//		fprintf(USART1_STREAM,"OTA_AT:%s\r\n",EC200U_Rxbuffer);
		if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"OK")){
			AT_CPIN_To_Get_SIM_Status();//ATI();// ���ͻ�ȡSIM��״ָ̬��
		}else if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"AT")){
			AT_CPIN_To_Get_SIM_Status();//ATI();// ���ͻ�ȡSIM��״ָ̬��
		}else
			AT();
		break;
	case OTA_ATI:
		if(upgrade_p.recv_ok==SUCCESS && (strx = strstr((const char*)EC200U_Rxbuffer,(const char*)"Revision"))){
			Analysis_Ec200u_SN(strx);
			// ����SN��
			AT_CPIN_To_Get_SIM_Status();	// ���ͻ�ȡSIM��״ָ̬��
		}else
			ATI();
		break;
	case OTA_SIM:
		if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"+CPIN: READY")){
			AT_CREG_To_Judge_GSM();// �ж�GSM����ע������
		}else
			AT_CPIN_To_Get_SIM_Status();
		break;
	case OTA_GSM:
//		fprintf(USART1_STREAM,"%s\r\n",EC200U_Rxbuffer);
		if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"OK")){
			AT_QIOPEN_TO_Connect_Server();// ���ӷ�����
		}else
			AT_CREG_To_Judge_GSM();
		break;
	case OTA_CONNECT:
//		fprintf(USART1_STREAM,"%s\r\n",EC200U_Rxbuffer);
		if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"OK")){//+QIOPEN: 0,
			AT_QISEND_To_Get_Secret_Cmd();// ��ȡ��Կ
			return 1;
		}else
			AT_QIOPEN_TO_Connect_Server();
		break;
	case OTA_SECRET:
		if(server_p.stage == 0){	// ��ָ֤���
			// �쳣���� Invalid params
			ota_loop_err_times++;
			if(ota_loop_err_times > 30){
				ota_loop_err_times = 0;
				Module_4G_ReStart();
				Connect_Server_Start(server_p.startOTA);
				return 0;
			}
//			fprintf(USART1_STREAM,">>>>%s\r\n",EC200U_Rxbuffer);
			if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)">")){
				AT_QISEND_To_Get_Secret_Data();
			}else
				AT_QISEND_To_Get_Secret_Cmd();
		}else{							// �������ݷ���
			if(upgrade_p.recv_ok == SUCCESS){
				strx = strstr((const char*)EC200U_Rxbuffer,(const char*)"result");
				char *strxEx = strstr((const char*)strx,(const char*)"secret");
				if(strx!=NULL && strxEx!=NULL){
					Get_Useful_Data(strxEx,"secret",upgrade_p.info.secret);
//					fprintf(USART1_STREAM,"secret:%s\r\n",upgrade_p.info.secret);
					AT_QISEND_To_Register_Device_Cmd();// ע���豸
				}else
					AT_QISEND_To_Get_Secret_Cmd();
			}
		}
		break;
	case OTA_REGISTER:
		if(ota_loop_times >= 1){
			ota_loop_times = 0;
			if(server_p.stage == 0){	// ��ָ֤���
				if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)">")){
					AT_QISEND_To_Register_Device_Data();
				}else
					AT_QISEND_To_Register_Device_Cmd();
			}else{							// �������ݷ���
//				fprintf(USART1_STREAM,">>%s>>\r\n",EC200U_Rxbuffer+126);
				// �쳣����
				if(upgrade_p.recv_ok == SUCCESS){
					if(strstr((const char*)EC200U_Rxbuffer+126,(const char*)"Invalid Request")){// Invalid Request
						ota_loop_err_times++;
	//					fprintf(USART1_STREAM,"----------------\r\n");
						if(ota_loop_err_times > 6){
							ota_loop_err_times = 0;
							Module_4G_ReStart();
							Connect_Server_Start(server_p.startOTA);
							return 1;
						}
					}
					strx=strstr((const char*)EC200U_Rxbuffer+126,(const char*)"itinerary_id");
					if(strx){
						Get_Useful_Data((const char*)strx,"itinerary_id",upgrade_p.info.itinerary_id);
//						AT_QISEND_To_Request_Update_Cmd();// ����
						server_p.sevIsConnect 	= TRUE;
//						ota_err_times 		= 0;
						fprintf(USART1_STREAM,"connct server ok.\r\n");
						GPS_Start();
						if(ota_on)
							OTA_Start(UPGRADE_OTA,0);
					}else
						AT_QISEND_To_Register_Device_Cmd();
				}
			}
		}else ota_loop_times++;
		break;
	default:
		break;
	}
	return 0;
}
/*
 * OTA��������
 * ������4G�׶α�־λ
 * ���أ�0ʧ�ܣ�1�ɹ�
 * ˵���������Ϸ������󣬲�ִ��
 */
uint32_t ota_clk 		= 0;
uint8_t OTA_Upgrade(Quectel4GStatus _ota_status)
{
	if(SystemtimeClock - ota_clk < 1000) return 0;		// 500ms
	ota_clk = SystemtimeClock;

	if(server_p.sevIsConnect == FALSE) return 0;

	if(!upgrade_p.isFinish) fprintf(USART1_STREAM,"ota:%s\r\n",EC200U_Rxbuffer);	// ȥ���󣬲��񲻵�EC200U_Rxbuffer��ֵ
	switch(_ota_status)
	{
	case OTA_REQUEST:
		if(server_p.stage == 0){	// ��ָ֤���
			if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)">")){ //
				AT_QISEND_To_Request_Update_Data();
			}else
				AT_QISEND_To_Request_Update_Cmd();
		}else{						// �������ݷ���
			if(upgrade_p.recv_ok == SUCCESS){
				strx = strstr((const char*)EC200U_Rxbuffer,(const char*)"result");
				if(strx){
					if(upgrade_p.info.mode == UPGRADE_OTA_CMD){	// �ֶ�ָ������ģʽ
						if(Check_For_Upgrade()){				// ����Ƿ�����
							AT_QHTTPURL_To_Upgrade_Stage1();
						}else{
							OTA_End(TRUE);
						}
					}else{									// ��������������ģʽ
						switch(Analysis_UpgradePackage_Info(strx)){
						case 0:			// ��ȡʧ��
							AT_QISEND_To_Request_Update_Cmd();
							break;
						case 1:			// ��ȡ�ɹ�
							if(Check_For_Upgrade()){		// ����Ƿ�����
								AT_QHTTPURL_To_Upgrade_Stage1();
							}else
								OTA_End(TRUE);
							break;
						case 2:			// ��������
							fprintf(USART1_STREAM,"platform have been upgrade.\r\n");
							OTA_End(TRUE);
							break;
						}
					}
				}else
					AT_QISEND_To_Request_Update_Cmd();
			}
		}
		break;
	case OTA_UPGRADE:
//		fprintf(USART1_STREAM,"%s\r\n",EC200U_Rxbuffer);
		switch(server_p.stage)
		{
		case 0:	// �׶�1
			if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"CONNECT")){
				AT_QHTTPURL_To_Upgrade_Stage2();
			}else
				AT_QHTTPURL_To_Upgrade_Stage1();
			break;
		case 1:	// �׶�2
			if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"OK")){
				AT_QHTTPURL_To_Upgrade_Stage3();
			}else
				AT_QHTTPURL_To_Upgrade_Stage2();
			break;
		case 2:	// �׶�3
			if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"+QHTTPGET:")){
				AT_QHTTPURL_To_Upgrade_Stage4();
			}else
				AT_QHTTPURL_To_Upgrade_Stage3();
			break;
		case 3:	// �׶�4
			AT_QHTTPURL_To_Upgrade_Stage4();
			break;
		}
		break;
	case OTA_RESPONSE:
		switch(server_p.stage){
		case 0:
			if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)">")){
				AT_QISEND_To_Replay_Success_Data();
			}else
				AT_QISEND_To_Replay_Success_Cmd();
			break;
		case 1:
			if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"result")){
				OTA_End(TRUE);
				return 1;
			}else
				AT_QISEND_To_Replay_Success_Cmd();
			break;
		}
		break;
	default:
		break;
	}
}
/*
 * OTA���������߼�
 * ��������
 * ���أ�0��ִ�У�1�쳣����
 * ˵����ɨ������500ms
 */

uint8_t OTA_Upgrade_Interaction_In_MainLoop()
{
	// ����ƽ̨���񣬲�����OTA�������ܣ�Ĭ��ÿ�ο���������һ��
	if(Connect_Platform_Server(server_p._4G_status,server_p.startOTA)){
		OTA_Upgrade(server_p._4G_status);
	}

	return 0;
}
/*
 * дβ��
 * ���������ݣ����ݳ���
 * ���أ�0ʧ�ܣ�1�ɹ�
 * ˵����
 */
uint8_t Write_End_PKG(uint8_t *data,uint16_t len)
{
	pkg_times = 0;		// У���־��գ���ʼ��һ��
	// �������ݣ����ܻ��ң���Ҫ���²�ѯβ��֤��
	for(uint16_t i=len;i>2;i--){
		if(i==3){
			for(uint8_t j=0; j<=50; j++)
				W25QXX_Erase_Sector(j);
			OTA_End(FALSE);
			fprintf(USART1_STREAM,"Write failed.\n");	// ʧ�ܣ�������ظ�
			return 0;
		}
		if(data[i-1]==0xFF && data[i-2]==0xEE && data[i-3]==0xDD){
			len=i;
//			for(uint16_t j=0;j<len;j++)
//				fprintf(USART1_STREAM,"%02X ",data[j]);
			break;
		}
	}

//	upgrade_p.pkg.len += len;
	// �������һ��У��
	for(uint16_t i=0;i<len-12;i++)
		upgrade_p.pkg.rx_crc += data[i];

	if(len > 12){
//		fprintf(USART1_STREAM,">12\r\n");
		// write
		W25QXX_Write_NoCheck(data, upgrade_p.pkg.addr, len-12);
		// ȡУ��
		upgrade_p.pkg.pkg_crc  = (data[len-12]<<24);
		upgrade_p.pkg.pkg_crc += (data[len-11]<<16);
		upgrade_p.pkg.pkg_crc += (data[len-10]<<8);
		upgrade_p.pkg.pkg_crc += (data[len-9]);
	}else{
		fprintf(USART1_STREAM,"<12\r\n");
	}
//	fprintf(USART1_STREAM,"r_CRC:%08X,p_CRC:%08X\r\n",upgrade_p.pkg.rx_crc,upgrade_p.pkg.pkg_crc);
//	fprintf(USART1_STREAM,"pkg:%d,recv:%d\r\n",upgrade_p.info.size,upgrade_p.pkg.len);

	if(upgrade_p.pkg.rx_crc == upgrade_p.pkg.pkg_crc){	// У��ɹ�
		// �汾��Ϣ�Ͱ���С
//		fprintf(USART1_STREAM,"version:%s,len:%d\r\n",upgrade_p.info.upgrade_v,upgrade_p.info.size-12);
		Set_Upgrade_App_Version(upgrade_p.info.upgrade_v);
		Set_Upgrade_PKG_size(upgrade_p.info.size-12);
		Set_Http_Download_Addr(upgrade_p.info.https);	// https·����Ϊ�ֶ������ṩ·��
		memcpy(upgrade_p.info.https_hu,upgrade_p.info.https,strlen(upgrade_p.info.https));
		AT_QISEND_To_Replay_Success_Cmd();			// �ظ�ƽ̨���ɹ��������������
		fprintf(USART1_STREAM,"Write finish.\r\n");	// �ɹ�д��ظ�
		return 1;
		// ��д���-����4G����
//			GPIO_Set_Output_Data_Bits(GPIOF_SFR,GPIO_PIN_MASK_0, Bit_RESET);
	}else{											// У��ʧ��
		for(uint8_t j=0; j<=50; j++)
			W25QXX_Erase_Sector(j);
		OTA_End(FALSE);
		fprintf(USART1_STREAM,"Write failed.\n");	// ʧ�ܣ�������ظ�
	}
	return 0;
}

/*
 * д�ⲿFLASH
 */
uint8_t OTA_Upgrade_Write_In_MainLoop()
{
	if(upgrade_p.pkg.ota_rx == BIN_RECV_GO)
	{
		// ���һ��
		if(upgrade_p.pkg.total >= upgrade_p.info.size){
			upgrade_p.pkg.total = 0;
//			fprintf(USART1_STREAM,"------%d---%d---t:%ld--b:%ld--\r\n",upgrade_p.pkg.sw_12,upgrade_p.pkg.cnt,upgrade_p.pkg.total,upgrade_p.info.size);
			if(upgrade_p.pkg.sw_12 == 1){	// дdata1
				Write_End_PKG(upgrade_p.pkg.data1, upgrade_p.pkg.cnt);
			}else{					// дdata2
				Write_End_PKG(upgrade_p.pkg.data2, upgrade_p.pkg.cnt);
			}
			return 0;
		}
		// �������һ��
		if(upgrade_p.pkg.is_write == 1){
			upgrade_p.pkg.is_write = 0;
//			upgrade_p.pkg.len 	+= BUFFER_SIZE;

			if(upgrade_p.pkg.sw_12 == 1)	// дdata2
				W25QXX_Write_NoCheck(upgrade_p.pkg.data2, upgrade_p.pkg.addr, BUFFER_SIZE);
			else 					// дdata1
				W25QXX_Write_NoCheck(upgrade_p.pkg.data1, upgrade_p.pkg.addr, BUFFER_SIZE);

			upgrade_p.pkg.addr 	+= BUFFER_SIZE;
//			fprintf(USART1_STREAM,"--%d--%ld-%ld-\r\n",upgrade_p.pkg.sw_12,upgrade_p.pkg.addr,upgrade_p.info.size);
		}

		return 0;
	}
	return 2;
}

/*
 * ��USART0�жϵĺ���
 * ���������ֽ�����
 * ���أ���
 * ˵��������˫buffer��������
 */
void OTA_Upgrade_In_USART0IT(uint8_t data)
{
	if(upgrade_p.pkg.sw_12 == 1)
		upgrade_p.pkg.data1[upgrade_p.pkg.cnt] = data;
	else
		upgrade_p.pkg.data2[upgrade_p.pkg.cnt] = data;

	// У�飬���һ����У�飬��Ϊ��β��ʶ��У��
	if(upgrade_p.pkg.ota_rx == BIN_RECV_GO && pkg_times < upgrade_p.pkg.times){
		upgrade_p.pkg.rx_crc += data;
	}

	upgrade_p.pkg.cnt++;
	upgrade_p.pkg.total++;

	if(upgrade_p.pkg.ota_rx == BIN_RECV_GO){
		// ��4KB���л�����д
		if(upgrade_p.pkg.cnt == BUFFER_SIZE){
			upgrade_p.pkg.cnt 		= 0;
			upgrade_p.pkg.is_write 	= 1;
			if(upgrade_p.pkg.sw_12 == 1) upgrade_p.pkg.sw_12 = 2;
			else upgrade_p.pkg.sw_12 = 1;
			pkg_times ++;	// ����У��
		}
	}else if(upgrade_p.pkg.ota_rx == BIN_RECV_READY){
		// ������ͷ
		if(upgrade_p.pkg.data1[upgrade_p.pkg.cnt-1]==0x77 &&upgrade_p.pkg.data1[upgrade_p.pkg.cnt-2]==0x66 &&
				upgrade_p.pkg.data1[upgrade_p.pkg.cnt-7]==0x11 && upgrade_p.pkg.data1[upgrade_p.pkg.cnt-8]==0x00)
		{
			upgrade_p.pkg.cnt 		= 0;
			upgrade_p.pkg.total 	= 0;
			upgrade_p.pkg.rx_crc 	= 0;
			upgrade_p.pkg.ota_rx 	= BIN_RECV_GO;
		}
	}
}

/*
 * OTA�ڵδ�ʱ����ʱ
 * ˵�����ֱ�ʵ��100ms|200ms|300ms|400ms|500ms��ʱ,���䵱ǰʱ��(��)
 */
uint32_t utctime_t=0;	// t0,t5,	t1,t2,t3,t4,
void OTA_In_SysClick_IT(uint32_t sysTimeClock)
{
//	if(sysTimeClock - t0 >= 10){	// 10ms
//		t0 = sysTimeClock;
//		dly_time.t0 = ENABLE;
//	}
//	if(sysTimeClock - t1 >= 100){	// 100ms
//		t1 = sysTimeClock;
//		dly_time.t1 = ENABLE;
//	}
//	if(sysTimeClock-t2 >= 200){		// 200ms
//		t2 = sysTimeClock;
//		dly_time.t2 = ENABLE;
//	}
//	if(sysTimeClock-t3 >= 300){		// 300ms
//		t3 = sysTimeClock;
//		dly_time.t3 = ENABLE;
//	}
//	if(sysTimeClock-t4 >= 400){		// 400ms
//		t4 = sysTimeClock;
//		dly_time.t4 = ENABLE;
//	}
//	if(sysTimeClock-t5 >= 500){		// 500ms
//		t5 = sysTimeClock;
//		dly_time.t5 = ENABLE;
//	}
	// ʵʱʱ�䣬4G���ӳɹ��󣬻Ὣutctime���³�����ʱ��
	if(sysTimeClock - utctime_t >= 1000){	// 1s
		utctime_t = sysTimeClock;
		utctime++;
	}
}
/*
 * ���¼�ʱ��־
 */
void Clear_Timing()
{
	dly_time.t0 = DISABLE;
//	dly_time.t1 = DISABLE;
//	dly_time.t2 = DISABLE;
//	dly_time.t3 = DISABLE;
//	dly_time.t4 = DISABLE;
	dly_time.t5 = DISABLE;
}

/*
 * ʹ��ͨ��AT+CCLK��ȡUTCʱ��
 */
void AT_CCLK()
{
	server_p._4G_status = OTA_UTCTIME;
	upgrade_p.recv_ok = FAILURE;
	uint8_t cmd[12] = "AT+CCLK?\r\n";
//	fprintf(USART1_STREAM,"%s",cmd);
	Clear_Buffer();
	EC200U_SendData(cmd,strlen(cmd));
}

/*
 * ����AT
 */
void AT()
{
	server_p._4G_status 	= OTA_AT;
	upgrade_p.recv_ok 		= FAILURE;

	uint8_t cmd[5] 			= "AT\r\n";
//	fprintf(USART1_STREAM,"%s",cmd);
	Clear_Buffer();
	EC200U_SendData(cmd,strlen(cmd));
}

/*
 * ����AT
 */
void ATI()
{
	server_p._4G_status 	= OTA_ATI;
	upgrade_p.recv_ok 		= FAILURE;

	uint8_t cmd[5] 			= "ATI\r\n";
	Clear_Buffer();
	EC200U_SendData(cmd,strlen(cmd));
}
/*
 * ����AT+CPIN?��ȡSIM��״̬
 */
void AT_CPIN_To_Get_SIM_Status()
{
	server_p._4G_status = OTA_SIM;
	upgrade_p.recv_ok 	= FAILURE;
	uint8_t cmd[12] 	= "AT+CPIN?\r\n";
//	fprintf(USART1_STREAM,"%s",cmd);
	Clear_Buffer();
	EC200U_SendData(cmd,strlen(cmd));
}

/*
 * ����AT+CREG?�ж�GSM�����Ƿ�����
 */
void AT_CREG_To_Judge_GSM()
{
	server_p._4G_status = OTA_GSM;
	upgrade_p.recv_ok 	= FAILURE;
	uint8_t cmd[12] 	= "AT+CREG?\r\n";
//	fprintf(USART1_STREAM,"%s",cmd);
	Clear_Buffer();
	EC200U_SendData(cmd,strlen(cmd));
}

/*
 * ����AT+CPIN?��ȡSIM��״̬
 */
void AT_QIOPEN_TO_Connect_Server()
{
	server_p._4G_status = OTA_CONNECT;
	upgrade_p.recv_ok 	= FAILURE;
	uint8_t cmd[50] 	= {0};
	sprintf(cmd,"AT+QIOPEN=1,%d,%s,%s,%d,0,1\r\n",CONNECT_ID,CONNECT_PROTO_TYPE,CONNECT_IP,CONNECT_PORT);
//	fprintf(USART1_STREAM,"%s",cmd);
	Clear_Buffer();
	EC200U_SendData(cmd,strlen(cmd));
}
/*
 * ��ȡ��Կ������ָ��
 */
void AT_QISEND_To_Get_Secret_Cmd()
{
	server_p._4G_status = OTA_SECRET;
	upgrade_p.recv_ok 	= FAILURE;
	server_p.stage 		= 0;
	uint8_t data[120] 	= {0};

	sprintf(data,"{\"method\":\"secret\",\"params\":{\"sn\":\"%s\",\"date\":%ld},\"id\":%d}\r\n",upgrade_p.info.sn,utctime,upgrade_p.info.id);
//	fprintf(USART1_STREAM,"secret:%s",data);
	uint8_t cmd[30] = {0};
	sprintf(cmd,"AT+QISEND=%d,%d\r\n",CONNECT_ID,strlen(data));
//	fprintf(USART1_STREAM,"%s",cmd);
	Clear_Buffer();
	EC200U_SendData(cmd,strlen(cmd));
}
/*
 * ��ȡ��Կ����������
 */
void AT_QISEND_To_Get_Secret_Data()
{
	server_p._4G_status = OTA_SECRET;
	upgrade_p.recv_ok 	= FAILURE;
	server_p.stage 		= 1;
	uint8_t data[100] 	= {0};
	sprintf(data,"{\"method\":\"secret\",\"params\":{\"sn\":\"%s\",\"date\":%ld},\"id\":%d}\r\n",upgrade_p.info.sn,utctime,upgrade_p.info.id);
//	fprintf(USART1_STREAM,"%s",data);
	Clear_Buffer();
	EC200U_SendData(data,strlen(data));
}
/*
 * ע���豸������ָ��
 */
uint32_t time1 = 0;
void AT_QISEND_To_Register_Device_Cmd()
{
	server_p._4G_status = OTA_REGISTER;
	upgrade_p.recv_ok 	= FAILURE;
	server_p.stage 		= 0;
	char value[50]		="";
	char sign[34]={0};

	time1 = utctime;
	sprintf(value,"%s%ld%s",upgrade_p.info.sn, time1, upgrade_p.info.secret);
	cal_md5(sign,sizeof(sign), value, 1);
//	fprintf(USART1_STREAM,"sign:%s\r\n",sign);
	uint8_t data[200];
	sprintf(data,"{\"method\":\"register\",\"params\":{\"sn\":\"%s\",\"sign\":\"%s\",\"date\":%ld},\"id\":%d}\r\n",upgrade_p.info.sn, sign, time1, upgrade_p.info.id);
	uint8_t cmd[20];
	sprintf(cmd,"AT+QISEND=%d,%d\r\n",CONNECT_ID,strlen(data));
//	fprintf(USART1_STREAM,"register:%s\r\n",cmd);
	Clear_Buffer();
	EC200U_SendData(cmd,strlen(cmd));
}
/*
 * ע���豸����������
 */
void AT_QISEND_To_Register_Device_Data()
{
	server_p._4G_status = OTA_REGISTER;
	upgrade_p.recv_ok 	= FAILURE;
	server_p.stage 		= 1;
	char value[50]		="";
	char sign[34] 		= {0};

	sprintf(value,"%s%ld%s",upgrade_p.info.sn, time1, upgrade_p.info.secret);
	cal_md5(sign, sizeof(sign), value, 1);
//	fprintf(USART1_STREAM,"sign:%s\r\n",sign);
	uint8_t data[130] = {0};
	sprintf(data,"{\"method\":\"register\",\"params\":{\"sn\":\"%s\",\"sign\":\"%s\",\"date\":%ld},\"id\":%d}\r\n",upgrade_p.info.sn, sign, time1, upgrade_p.info.id);
//	fprintf(USART1_STREAM,"data:%s",data);
	Clear_Buffer();
	EC200U_SendData(data,strlen(data));
}
/*
 * ������£�����ָ��
 * ˵����Ҳ���ֶ�����OTA���������
 */
void AT_QISEND_To_Request_Update_Cmd()
{
	server_p._4G_status = OTA_REQUEST;
	upgrade_p.recv_ok 	= FAILURE;
	server_p.stage 		= 0;

	uint8_t data[200] 	= {0};
	sprintf(data,"{\"method\":\"ota.request_update\",\"params\":{\"type\":\"controller\",\"version\":\"%s\",\"date\":%ld,\"itinerary_id\":\"%s\",\"sn\":\"%s\"},\"id\":%d}\r\n",\
		upgrade_p.info.run_v,utctime,upgrade_p.info.itinerary_id, upgrade_p.info.sn,upgrade_p.info.id);
	uint8_t cmd[20];
	sprintf(cmd,"AT+QISEND=%d,%d\r\n",CONNECT_ID,strlen(data));
//	fprintf(USART1_STREAM,"update:%s\r\n",cmd);
	Clear_Buffer();
	EC200U_SendData(cmd,strlen(cmd));
}
/*
 * ������£���������
 */
void AT_QISEND_To_Request_Update_Data()
{
	server_p._4G_status = OTA_REQUEST;
	upgrade_p.recv_ok 	= FAILURE;
	server_p.stage 		= 1;
	uint8_t data[200] 	= {0};
	sprintf(data,"{\"method\":\"ota.request_update\",\"params\":{\"type\":\"controller\",\"version\":\"%s\",\"date\":%ld,\"itinerary_id\":\"%s\",\"sn\":\"%s\"},\"id\":%d}\r\n",\
		upgrade_p.info.run_v,utctime,upgrade_p.info.itinerary_id, upgrade_p.info.sn,upgrade_p.info.id);
//	fprintf(USART1_STREAM,"data:%s\r\n",data);
	Clear_Buffer();
	EC200U_SendData(data,strlen(data));
}

/*
 * HTTP�����������׶�1
 */
void AT_QHTTPURL_To_Upgrade_Stage1()
{
	server_p._4G_status = OTA_UPGRADE;
	upgrade_p.recv_ok	= FAILURE;
	server_p.stage 		= 0;
	uint8_t cmd[30]		= {0};
	sprintf(cmd,"AT+QHTTPURL=%d,80\r\n",strlen(upgrade_p.info.https));
//	fprintf(USART1_STREAM,"%s\r\n",cmd);	// ɾ����4Gģ��������ݲ���ȫ�����ڸĽ���������delay
	Clear_Buffer();
	EC200U_SendData(cmd,strlen(cmd));
}

/*
 * HTTP�����������׶�2
 */
void AT_QHTTPURL_To_Upgrade_Stage2()
{
	server_p._4G_status = OTA_UPGRADE;
	upgrade_p.recv_ok 	= FAILURE;
	server_p.stage 		= 1;
	uint8_t data[150] 	= {0};
	sprintf(data,"%s\r\n",upgrade_p.info.https);
//	fprintf(USART1_STREAM,"%s\r\n",data);
	Clear_Buffer();
	EC200U_SendData(data,strlen(data));
}
/*
 * HTTP�����������׶�3
 */
void AT_QHTTPURL_To_Upgrade_Stage3()
{
	server_p._4G_status = OTA_UPGRADE;
	upgrade_p.recv_ok 	= FAILURE;
	server_p.stage 		= 2;
	uint8_t cmd[30]		= {0};
	sprintf(cmd,"AT+QHTTPGET=80\r\n");
//	fprintf(USART1_STREAM,"%s\r\n",cmd);
	Clear_Buffer();
	EC200U_SendData(cmd,strlen(cmd));
}
/*
 * HTTP�����������׶�4
 */
void AT_QHTTPURL_To_Upgrade_Stage4()
{
	server_p._4G_status = OTA_UPGRADE;
	upgrade_p.recv_ok 	= FAILURE;
	server_p.stage 		= 3;
	// ��ʼ����������
//	upgrade_p.rx_mode 	= RECV_BIN;	// ��ʼ����bin�ļ���
	upgrade_p.pkg.times = upgrade_p.info.size / BUFFER_SIZE;
	upgrade_p.pkg.remain = upgrade_p.info.size % BUFFER_SIZE;
	upgrade_p.pkg.cnt 	= 0;
	upgrade_p.pkg.sw_12	= 1;
	upgrade_p.pkg.ota_rx = BIN_RECV_READY;
	upgrade_p.pkg.rx_crc = 0;

	uint8_t cmd[30]		= {0};
	sprintf(cmd,"AT+QHTTPREAD=80\r\n");
//	fprintf(USART1_STREAM,"%s\r\n",cmd);
	Clear_Buffer();
	EC200U_SendData(cmd,strlen(cmd));
	EC200U_SendData(cmd,strlen(cmd));
	EC200U_SendData(cmd,strlen(cmd));
}


/*
 * ��ȡ��Կ������ָ��
 */
void AT_QISEND_To_Replay_Success_Cmd()
{
	upgrade_p.rx_mode 	= RECV_CMD;			// �л���ָ��ģʽ
	server_p._4G_status = OTA_RESPONSE;		// OTA������ظ�
	upgrade_p.recv_ok 	= FAILURE;
	server_p.stage 		= 0;
	uint8_t data[200] 	= {0};
	sprintf(data,"{\"method\":\"ota.success\",\"params\":{\"type\":\"controller\",\"version\":\"%s\",\"date\":%ld,\"itinerary_id\":\"%s\",\"sn\":\"%s\"},\"id\":%d}\r\n",
		upgrade_p.info.upgrade_v,utctime,upgrade_p.info.itinerary_id,upgrade_p.info.sn,upgrade_p.info.id);

	uint8_t cmd[20];
	sprintf(cmd,"AT+QISEND=%d,%d\r\n",CONNECT_ID,strlen(data));
//	fprintf(USART1_STREAM,"replay:%s",cmd);
	Clear_Buffer();
	EC200U_SendData(cmd,strlen(cmd));
}

/*
 * ��ȡ��Կ������ָ��
 */
void AT_QISEND_To_Replay_Success_Data()
{
	server_p._4G_status = OTA_RESPONSE;
	upgrade_p.recv_ok 	= FAILURE;
	server_p.stage 		= 1;
	uint8_t data[300] 	= {0};
	sprintf(data,"{\"method\":\"ota.success\",\"params\":{\"type\":\"controller\",\"version\":\"%s\",\"date\":%ld,\"itinerary_id\":\"%s\",\"sn\":\"%s\"},\"id\":%d}\r\n",
		upgrade_p.info.upgrade_v,utctime,upgrade_p.info.itinerary_id,upgrade_p.info.sn,upgrade_p.info.id);
//	fprintf(USART1_STREAM,"%s",data);
	Clear_Buffer();
	EC200U_SendData(data,strlen(data));
}
/*
 * ��ʱ���ַ���ת��UTCʱ��
 * ������ʱ���ַ�����"21/11/01,09:50:40+00"
 * ���أ�UTCʱ��
 */
uint8_t TimeString_To_UTCTIME(char * time_string)
{
	int year, mon, day, hour, min, second,other;
	sscanf(time_string, "%d/%d/%d,%d:%d:%d+%d", &year, &mon, &day, &hour, &min, &second, &other);
	TimeType local_t;
	local_t.tm_year = year+2000;
	local_t.tm_mon 	= mon;
	local_t.tm_mday = day;
	local_t.tm_hour = hour+8;
	local_t.tm_min 	= min;
	local_t.tm_sec 	= second;
//			fprintf(USART1_STREAM,"20%d-%02d-%02d %02d:%02d:%02d\n", year, mon, day, hour+8, min, second);
	utctime = alg_LocalTime2Utc(local_t,8);
	fprintf(USART1_STREAM,"UTC:%ld\r\n",utctime);
}
/*
 * ��UTCʱ��ת��ʱ��ṹ�壬ʱ���
 * ������UTCʱ��
 * ���أ�ʱ��ṹ��
 */
TimeType Get_CurrentTime_Stamp()
{
	TimeType t;
	t = alg_Utc2LocalTime(utctime,8);
	return t;
}
/*
 * ��ȡ�豸�Ľ�MD5���ܺ��32λСд��sign��
 * ���������ص�sign�ַ���
 * ���أ���
 */
void Get_Device_Sign_Md5(uint8_t *sign)
{
	char value[50]	= "";

	sprintf(value,"%s%ld%s",upgrade_p.info.sn, utctime, upgrade_p.info.secret);
	cal_md5(sign, sizeof(sign), value, 1);	// Сд
	fprintf(USART1_STREAM,"sign:%s\n",sign);
}
/*
 * ���ַ�������ȡ���õ�����
 * buffer:�����ַ���
 * input:��Ҫ�������ַ���
 * output:������Ϻ�������ַ���
 * ����˵������buffer����ȡinput����������ַ�����Ϣ����ŵ�output��
 * ����ֵ��0û���ҵ���1��������ѯ����
 */
int Get_Useful_Data(const char *buffer, char *input, char *output)
{
	char* strx = strstr((const char*)buffer, (const char*)input);
	if (strx) {
		// ����������ֱ������','��'}'�ŷ���
		int init_len = strlen(input) + 2;
		if (strx[init_len] == '\"') {	// �ַ���
			int j = 0;
			for (int i = init_len+1; i < strlen(buffer); i++) {
				if (strx[i] == '\"') {
					output[j] = '\0';
//					fprintf(USART1_STREAM,"secret:%s\r\n",output);
					return 1;
				}
				else {
					output[j] = strx[i];
					j++;
				}
			}
		}
		else {	// ����
			int j = 0;
			for (int i = init_len; i < strlen(buffer); i++) {
				if (strx[i] == ',' || strx[i] == '}') {
					output[j] = '\0';
//					fprintf(USART1_STREAM,"secret:%s\r\n",output);
					return 1;
				}
				else {
					output[j] = strx[i];
					j++;
				}
			}
		}
	}
	return 0;
}
/*
 * ��Ⲣ��ȡ����������Ϣ
 * ������strx��Ч��Ϣ
 * ���أ�1��ȡ�ɹ���0ʧ��;2��������
 * ˵������ȡ����·�����ļ���С��md5���������汾
 */
//{"result":{"type":"controller","resource_path":"https:\/\/qupinmate.oss-cn-beijing.aliyuncs.com\/a\/v1.0.0.6.bin","size":34836,"md5":"8d3b591352d48c1a6fc192cb58d26c3d","version":"v1.0.0.7"},"id":0}
//����Ҫ������{"result":0,"id":0}
uint8_t Analysis_UpgradePackage_Info(char * strx)
{
//	fprintf(USART1_STREAM,"%s\r\n",strx);
	if(strx[8] == '0'){
		return 2;	// �Ѿ���ƽ̨��������
	}
	uint8_t data_size[10] 	= {0};
	uint8_t sign[33]		= {0};
	uint8_t http_path[100] 	= {0};
	Get_Useful_Data((const char*)strx,"resource_path",http_path);
	Get_Useful_Data((const char*)strx,"size",data_size);
	Get_Useful_Data((const char*)strx,"md5",sign);
	Get_Useful_Data((const char*)strx,"version",upgrade_p.info.upgrade_v);

	upgrade_p.info.size = atoi(data_size) - 8;	// ȥ����8���ֽ�
	if(strlen(upgrade_p.info.upgrade_v)>5 && strstr((const char*)http_path,(const char*)".bin") && 	strlen(sign)>30 && upgrade_p.info.size>1024)
	{
		uint8_t i = 0;
		/* �õ������ķ���·��  */
//		fprintf(USART1_STREAM,"http_path:%s\r\n",http_path);
		for(uint8_t len=0; len<strlen(http_path); len++)
		{
			if(http_path[len] != '\\'){
				upgrade_p.info.https[i] = http_path[len];
				i++;
			}
		}
		upgrade_p.info.https[i]='\0';
		memcpy(upgrade_p.info.https_hu,upgrade_p.info.https,strlen(upgrade_p.info.https));
//		fprintf(USART1_STREAM,"https:%s\r\nsize:%d\r\nmd5:%s\r\nversion:%s\r\n",upgrade_p.info.https, \
//				upgrade_p.info.size,sign,upgrade_p.info.upgrade_v);
		return 1;
	}else return 0;
}
/*
 * ����Ƿ�����
 * ������
 * ���أ�0��������1����
 */
uint8_t Check_For_Upgrade()
{
//	fprintf(USART1_STREAM,"c:%s,p:%s\r\n",upgrade_p.info.run_v,upgrade_p.info.upgrade_v);
	if(Version_Compare(upgrade_p.info.run_v, upgrade_p.info.upgrade_v)){
		// �����200KB �ⲿFLASH�ռ�
		for(uint32_t i=0;i<50;i++){
			W25QXX_Erase_Sector(i);
		}
		fprintf(USART1_STREAM,"OTA need upgrade.\r\n");
		return 1;
	}else{
		// ����Ҫ����
		fprintf(USART1_STREAM,"OTA no need upgrade.\r\n");
		return 0;
	}
}

void OTA_UserCmd_Analysis_In_USART1IT(uint8_t *User_Rxbuffer)
{
	char *strx_1 = NULL;
	// OTA ָ������ ,���������°汾�����¼��һ��
	if(strstr((const char*)User_Rxbuffer,(const char*)"ZKHYCHK*UPDATE:NEWEST")){
		// ��ȡ��ǰ�汾��
		memset(upgrade_p.info.upgrade_v, 0, sizeof(upgrade_p.info.upgrade_v));
		Get_Upgrade_App_Version(upgrade_p.info.run_v);
//		fprintf(USART1_STREAM,"%s\r\n",upgrade_p.info.run_v);
		OTA_Start(UPGRADE_OTA,0);
		fprintf(USART1_STREAM,">> Upgrade...\r\n");
		Clear_Usart1_Buffer();// ��ս���.buffer
	}else
	// <ZKHYSET*UPDATE:V1.1.0.L1C6,60436>
	// ������ָ���汾����������https·��+�汾����+.bin��(<ZKHYSET*UPDATE:�汾��,��������С>)
	if((strx_1 = strstr((const char*)User_Rxbuffer,(const char*)"ZKHYSET*UPDATE"))){
//		fprintf(USART1_STREAM,"%s\r\n",strx_1);
		StrtokStr str = StrtokString(strx_1+15);
		if(str.cnt == 2){
			uint8_t version[30] = {0};
			strcpy(upgrade_p.info.upgrade_v,str.data[0]);		// �����汾��
			sprintf(version,"%s.bin",str.data[0]);

			if(strlen(upgrade_p.info.https_hu) < 5){
				Clear_Usart1_Buffer();// ��ս���buffer
				fprintf(USART1_STREAM,">> This function cannot be used because the program has not been upgraded by OTA.\r\n");
				return ;
			}
//			IWDT_Feed_The_Dog();
			sprintf(upgrade_p.info.https,"%s%s",upgrade_p.info.https_hu,version);		// ����·��
//			strcat(upgrade_p.info.https,version);
			uint32_t size = atoi(str.data[1]) - 8;			// ���������ȣ�ȥ����β16���ֽ�
//			fprintf(USART1_STREAM,"h_a:%s,%d\r\n",upgrade_p.info.https,size);

			// ����OTAָ������
			OTA_Start(UPGRADE_OTA_CMD,size);
			fprintf(USART1_STREAM,">> Upgrade...\r\n");
		}else{
			fprintf(USART1_STREAM,">> Semantic Error...\r\n");
		}
		Clear_Usart1_Buffer();// ��ս���buffer
	}
}
/*
 * ����EC200U-CN ��SN��
 *
 */
void Analysis_Ec200u_SN(uint8_t *str)
{
	// Revision: EC200UCNAAR02A01M08OK
	int len = strlen(str+10);
//	uint8_t str_t[50] = {0};
//	strcpy(str_t,str);
	memset(upgrade_p.info.sn,0,sizeof(upgrade_p.info.sn));
//	memcpy(upgrade_p.info.sn,str+10,len-4);
	uint8_t i = 0;
	while(1){
		if(str[10+i] == 'O' && str[10+i+1]=='K'){
			upgrade_p.info.sn[i-2] = '\0';
			break;
		}else{
			upgrade_p.info.sn[i] = str[10+i];
		}
		i++;
	}
//	upgrade_p.info.sn[strlen(upgrade_p.info.sn)-2] = '\0';
	fprintf(USART1_STREAM,">%s<,%d,%d\r\n",upgrade_p.info.sn,strlen(upgrade_p.info.sn));//strlen(upgrade_p.info.sn)
}


