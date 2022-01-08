/*
 * _4g_upgrade.c
 *
 *  Created on: 2021-12-3
 *      Author: Administrator
 * 功能说明：包括OTA升级|数据上传|获取GPS定位信息|蓝牙功能
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
/****************************宏、枚举定义**************************************/

/****************************结构体定义**************************************/
typedef struct{	// 时间片切换标志位
	AbleStatus t0;	// 10ms
//	AbleStatus t1;	// 100ms
//	AbleStatus t2;	// 200ms
//	AbleStatus t3;	// 300ms
//	AbleStatus t4;	// 400ms
	AbleStatus t5;	// 500ms
}Delay_Time;

/****************************全局变量**************************************/
// OTA
Delay_Time			dly_time 		= {0};
uint32_t 			utctime			= 0;		// UTCTIME时间，单位秒
uint8_t 			pkg_times 		= 0;		// 计算校验计数
PlatfromServerPara 	server_p;
UpgradePara			upgrade_p;
/****************************函数声明**************************************/
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
 * sscanf()函数会调用该函数
 */
char getche(void)
{
	return getchar();
}
/****************************OTA****************************************/
/*
 * 升级过程中提醒
 * 说明：升级中，控制4G指示灯闪烁
 */
uint32_t upgrade_Warning_Clk = 0;
void Upgrade_Warning()
{
	if(SystemtimeClock - upgrade_Warning_Clk > 50){
		upgrade_Warning_Clk = SystemtimeClock;
		GPIO_Toggle_Output_Data_Config(GPIOF_SFR, GPIO_PIN_MASK_0);	// 4G指示灯
	}
}
/*
 * 4G模块初始化
 */
void AT_4G_Module_Init()
{
	upgrade_p.rx_mode 		= RECV_CMD;
	upgrade_p.recv_ok 		= FAILURE;
	upgrade_p.pkg.ota_rx 	= BIN_RECV_WAIT;
	server_p.sevIsConnect  	= FALSE;
	server_p._4G_status 	= OTA_RDY;
	server_p.stage			= 0;


	EC200U_INIT();						// USART0 开启了接收中断，19200bps

	uint8_t version[VERSION_SIZE];
	Get_Upgrade_App_Version(version);			// 获取platform版本号
	strcpy(upgrade_p.info.run_v, (char *)version);
	Get_Run_App_Version(version);	// 获取platform版本号
	fprintf(USART1_STREAM,"------Upgrade_V---%s-------------\r\n",upgrade_p.info.run_v);
	fprintf(USART1_STREAM,"------Run_V-------%s-------------\r\n",version);

	upgrade_p.up_mode 		= UPGRADE_OTA;
	upgrade_p.info.mode		= UPGRADE_OTA;
	upgrade_p.recv_ok 		= FAILURE;
	// 接收bin文件参数
	upgrade_p.pkg.cnt 		= 0;
	upgrade_p.pkg.remain 	= 0;
	upgrade_p.pkg.times 	= 0;
	upgrade_p.pkg.addr		= OUT_FLASH_APP_START;
	upgrade_p.pkg.rx_crc 	= 0;
	upgrade_p.pkg.pkg_crc 	= 0;
	upgrade_p.pkg.total		= 0;
	memset(upgrade_p.pkg.data1, 0, BUFFER_SIZE);
	memset(upgrade_p.pkg.data2, 0, BUFFER_SIZE);
	// 升级信息参数
	upgrade_p.info.id 		= CONNECT_ID;
	upgrade_p.info.size 	= 0;
//	Set_Device_SN("21122700001");
	Get_Device_SN(upgrade_p.info.sn);
	// 组合升级http路径
	Get_Http_Download_Addr(upgrade_p.info.https);
	memcpy(upgrade_p.info.https_hu,upgrade_p.info.https,strlen(upgrade_p.info.https));
	// 连接服务器：启动OTA，从开始接收RDY位置开始
	Connect_Server_Start(TRUE);
}
/*
 * 开始连接服务器
 * 参数：TRUE开始OTA功能，FALSE不开启OTA功能;restart：//TRUE从OTA_RDY启动，FALSE从OTA_AT启动
 * 返回：无
 */
void Connect_Server_Start(FunctionalState startOTA)
{
	server_p.sevIsConnect	 		= FALSE;
	server_p._4G_status 			= OTA_RDY;
	server_p.startOTA				= startOTA;
}
/*
 * OTA开始工作
 * 参数：UPGRADE_OTA上电升级；UPGRADE_OTA_CMD指令升级
 * 说明:3种升级模式
 * ota_p.up_mode	ota_p.info.mode		 	控制模式
 * -------------------------------------------------------------
 * UPGRADE_OTA		UPGRADE_OTA				上电OTA升级
 * UPGRADE_OTA		UPGRADE_OTA_CMD			指令OTA升级
 * UPGRADE_USART1							串口升级
 */
void OTA_Start(Upgrade_Mode upgrade_mode,uint32_t size)
{
	server_p._4G_status		= OTA_REQUEST;

	upgrade_p.up_mode 		= UPGRADE_OTA;
	upgrade_p.info.mode		= upgrade_mode;
	upgrade_p.isFinish 		= FALSE;
	upgrade_p.recv_ok 		= FAILURE;
	// 接收bin文件参数
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
	// 升级信息参数
	upgrade_p.info.id 		= CONNECT_ID;
	upgrade_p.info.size 	= size;
	Get_Device_SN(upgrade_p.info.sn);

	AT_QISEND_To_Request_Update_Cmd();// 请求
}
/*
 * OTA结束工作
 */
void OTA_End(FunctionalState flag)
{
	upgrade_p.isFinish 		= flag;
	upgrade_p.rx_mode 		= RECV_CMD;		// 指令模式

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
	GPS_Start();							// 启动GPS
//	fprintf(USART1_STREAM,"OTA end.\n");
}
/*
 * 连接平台服务器步骤
 * 参数：阶段状态;是否打开ota,TRUE,FALSE
 * 返回：0失败；1：成功
 * 说明：连接服务器步骤
 */
uint8_t ota_loop_times 		= 0;
uint8_t ota_err_times 		= 0;	// 异常处理，2次检测都报异常就退出
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
			AT_CCLK();// 发送UTC时间指令
		}else if(ota_loop_times > 10){	// 5s重连
			ota_err_times++;
			if(ota_err_times > 4){		// 控制4次，就认为服务器不在线，就可以依靠外层进行检测
				// 出现这种情况,很有可能在服务器连接有问题,可以手动> telnet serverip port
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
				TimeString_To_UTCTIME(strx+8);// 解析utctime
				AT();// 发送AT指令
			}else
				AT_CCLK();
		}
		break;
	case OTA_AT:
//		fprintf(USART1_STREAM,"OTA_AT:%s\r\n",EC200U_Rxbuffer);
		if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"OK")){
			AT_CPIN_To_Get_SIM_Status();//ATI();// 发送获取SIM卡状态指令
		}else if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"AT")){
			AT_CPIN_To_Get_SIM_Status();//ATI();// 发送获取SIM卡状态指令
		}else
			AT();
		break;
	case OTA_ATI:
		if(upgrade_p.recv_ok==SUCCESS && (strx = strstr((const char*)EC200U_Rxbuffer,(const char*)"Revision"))){
			Analysis_Ec200u_SN(strx);
			// 解析SN号
			AT_CPIN_To_Get_SIM_Status();	// 发送获取SIM卡状态指令
		}else
			ATI();
		break;
	case OTA_SIM:
		if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"+CPIN: READY")){
			AT_CREG_To_Judge_GSM();// 判断GSM网络注册正常
		}else
			AT_CPIN_To_Get_SIM_Status();
		break;
	case OTA_GSM:
//		fprintf(USART1_STREAM,"%s\r\n",EC200U_Rxbuffer);
		if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"OK")){
			AT_QIOPEN_TO_Connect_Server();// 连接服务器
		}else
			AT_CREG_To_Judge_GSM();
		break;
	case OTA_CONNECT:
//		fprintf(USART1_STREAM,"%s\r\n",EC200U_Rxbuffer);
		if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"OK")){//+QIOPEN: 0,
			AT_QISEND_To_Get_Secret_Cmd();// 获取秘钥
			return 1;
		}else
			AT_QIOPEN_TO_Connect_Server();
		break;
	case OTA_SECRET:
		if(server_p.stage == 0){	// 验证指令返回
			// 异常处理 Invalid params
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
		}else{							// 解析数据返回
			if(upgrade_p.recv_ok == SUCCESS){
				strx = strstr((const char*)EC200U_Rxbuffer,(const char*)"result");
				char *strxEx = strstr((const char*)strx,(const char*)"secret");
				if(strx!=NULL && strxEx!=NULL){
					Get_Useful_Data(strxEx,"secret",upgrade_p.info.secret);
//					fprintf(USART1_STREAM,"secret:%s\r\n",upgrade_p.info.secret);
					AT_QISEND_To_Register_Device_Cmd();// 注册设备
				}else
					AT_QISEND_To_Get_Secret_Cmd();
			}
		}
		break;
	case OTA_REGISTER:
		if(ota_loop_times >= 1){
			ota_loop_times = 0;
			if(server_p.stage == 0){	// 验证指令返回
				if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)">")){
					AT_QISEND_To_Register_Device_Data();
				}else
					AT_QISEND_To_Register_Device_Cmd();
			}else{							// 解析数据返回
//				fprintf(USART1_STREAM,">>%s>>\r\n",EC200U_Rxbuffer+126);
				// 异常处理
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
//						AT_QISEND_To_Request_Update_Cmd();// 请求
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
 * OTA升级步骤
 * 参数：4G阶段标志位
 * 返回：0失败；1成功
 * 说明：连接上服务器后，才执行
 */
uint32_t ota_clk 		= 0;
uint8_t OTA_Upgrade(Quectel4GStatus _ota_status)
{
	if(SystemtimeClock - ota_clk < 1000) return 0;		// 500ms
	ota_clk = SystemtimeClock;

	if(server_p.sevIsConnect == FALSE) return 0;

	if(!upgrade_p.isFinish) fprintf(USART1_STREAM,"ota:%s\r\n",EC200U_Rxbuffer);	// 去除后，捕获不到EC200U_Rxbuffer的值
	switch(_ota_status)
	{
	case OTA_REQUEST:
		if(server_p.stage == 0){	// 验证指令返回
			if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)">")){ //
				AT_QISEND_To_Request_Update_Data();
			}else
				AT_QISEND_To_Request_Update_Cmd();
		}else{						// 解析数据返回
			if(upgrade_p.recv_ok == SUCCESS){
				strx = strstr((const char*)EC200U_Rxbuffer,(const char*)"result");
				if(strx){
					if(upgrade_p.info.mode == UPGRADE_OTA_CMD){	// 手动指令升级模式
						if(Check_For_Upgrade()){				// 检测是否升级
							AT_QHTTPURL_To_Upgrade_Stage1();
						}else{
							OTA_End(TRUE);
						}
					}else{									// 开机自启动升级模式
						switch(Analysis_UpgradePackage_Info(strx)){
						case 0:			// 提取失败
							AT_QISEND_To_Request_Update_Cmd();
							break;
						case 1:			// 提取成功
							if(Check_For_Upgrade()){		// 检测是否升级
								AT_QHTTPURL_To_Upgrade_Stage1();
							}else
								OTA_End(TRUE);
							break;
						case 2:			// 无需升级
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
		case 0:	// 阶段1
			if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"CONNECT")){
				AT_QHTTPURL_To_Upgrade_Stage2();
			}else
				AT_QHTTPURL_To_Upgrade_Stage1();
			break;
		case 1:	// 阶段2
			if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"OK")){
				AT_QHTTPURL_To_Upgrade_Stage3();
			}else
				AT_QHTTPURL_To_Upgrade_Stage2();
			break;
		case 2:	// 阶段3
			if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"+QHTTPGET:")){
				AT_QHTTPURL_To_Upgrade_Stage4();
			}else
				AT_QHTTPURL_To_Upgrade_Stage3();
			break;
		case 3:	// 阶段4
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
 * OTA升级交互逻辑
 * 参数：无
 * 返回：0不执行；1异常返回
 * 说明：扫描周期500ms
 */

uint8_t OTA_Upgrade_Interaction_In_MainLoop()
{
	// 连接平台服务，并启动OTA升级功能，默认每次开机都需检测一下
	if(Connect_Platform_Server(server_p._4G_status,server_p.startOTA)){
		OTA_Upgrade(server_p._4G_status);
	}

	return 0;
}
/*
 * 写尾包
 * 参数：数据；数据长度
 * 返回：0失败；1成功
 * 说明：
 */
uint8_t Write_End_PKG(uint8_t *data,uint16_t len)
{
	pkg_times = 0;		// 校验标志清空，开始下一轮
	// 最后包数据，可能会乱，需要重新查询尾验证码
	for(uint16_t i=len;i>2;i--){
		if(i==3){
			for(uint8_t j=0; j<=50; j++)
				W25QXX_Erase_Sector(j);
			OTA_End(FALSE);
			fprintf(USART1_STREAM,"Write failed.\n");	// 失败，擦除后回复
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
	// 计算最后一包校验
	for(uint16_t i=0;i<len-12;i++)
		upgrade_p.pkg.rx_crc += data[i];

	if(len > 12){
//		fprintf(USART1_STREAM,">12\r\n");
		// write
		W25QXX_Write_NoCheck(data, upgrade_p.pkg.addr, len-12);
		// 取校验
		upgrade_p.pkg.pkg_crc  = (data[len-12]<<24);
		upgrade_p.pkg.pkg_crc += (data[len-11]<<16);
		upgrade_p.pkg.pkg_crc += (data[len-10]<<8);
		upgrade_p.pkg.pkg_crc += (data[len-9]);
	}else{
		fprintf(USART1_STREAM,"<12\r\n");
	}
//	fprintf(USART1_STREAM,"r_CRC:%08X,p_CRC:%08X\r\n",upgrade_p.pkg.rx_crc,upgrade_p.pkg.pkg_crc);
//	fprintf(USART1_STREAM,"pkg:%d,recv:%d\r\n",upgrade_p.info.size,upgrade_p.pkg.len);

	if(upgrade_p.pkg.rx_crc == upgrade_p.pkg.pkg_crc){	// 校验成功
		// 版本信息和包大小
//		fprintf(USART1_STREAM,"version:%s,len:%d\r\n",upgrade_p.info.upgrade_v,upgrade_p.info.size-12);
		Set_Upgrade_App_Version(upgrade_p.info.upgrade_v);
		Set_Upgrade_PKG_size(upgrade_p.info.size-12);
		Set_Http_Download_Addr(upgrade_p.info.https);	// https路径，为手动升级提供路径
		memcpy(upgrade_p.info.https_hu,upgrade_p.info.https,strlen(upgrade_p.info.https));
		AT_QISEND_To_Replay_Success_Cmd();			// 回复平台，成功后才算升级结束
		fprintf(USART1_STREAM,"Write finish.\r\n");	// 成功写入回复
		return 1;
		// 烧写完成-板载4G灯灭
//			GPIO_Set_Output_Data_Bits(GPIOF_SFR,GPIO_PIN_MASK_0, Bit_RESET);
	}else{											// 校验失败
		for(uint8_t j=0; j<=50; j++)
			W25QXX_Erase_Sector(j);
		OTA_End(FALSE);
		fprintf(USART1_STREAM,"Write failed.\n");	// 失败，擦除后回复
	}
	return 0;
}

/*
 * 写外部FLASH
 */
uint8_t OTA_Upgrade_Write_In_MainLoop()
{
	if(upgrade_p.pkg.ota_rx == BIN_RECV_GO)
	{
		// 最后一包
		if(upgrade_p.pkg.total >= upgrade_p.info.size){
			upgrade_p.pkg.total = 0;
//			fprintf(USART1_STREAM,"------%d---%d---t:%ld--b:%ld--\r\n",upgrade_p.pkg.sw_12,upgrade_p.pkg.cnt,upgrade_p.pkg.total,upgrade_p.info.size);
			if(upgrade_p.pkg.sw_12 == 1){	// 写data1
				Write_End_PKG(upgrade_p.pkg.data1, upgrade_p.pkg.cnt);
			}else{					// 写data2
				Write_End_PKG(upgrade_p.pkg.data2, upgrade_p.pkg.cnt);
			}
			return 0;
		}
		// 不是最后一包
		if(upgrade_p.pkg.is_write == 1){
			upgrade_p.pkg.is_write = 0;
//			upgrade_p.pkg.len 	+= BUFFER_SIZE;

			if(upgrade_p.pkg.sw_12 == 1)	// 写data2
				W25QXX_Write_NoCheck(upgrade_p.pkg.data2, upgrade_p.pkg.addr, BUFFER_SIZE);
			else 					// 写data1
				W25QXX_Write_NoCheck(upgrade_p.pkg.data1, upgrade_p.pkg.addr, BUFFER_SIZE);

			upgrade_p.pkg.addr 	+= BUFFER_SIZE;
//			fprintf(USART1_STREAM,"--%d--%ld-%ld-\r\n",upgrade_p.pkg.sw_12,upgrade_p.pkg.addr,upgrade_p.info.size);
		}

		return 0;
	}
	return 2;
}

/*
 * 在USART0中断的函数
 * 参数：单字节数据
 * 返回：无
 * 说明：采用双buffer缓冲数据
 */
void OTA_Upgrade_In_USART0IT(uint8_t data)
{
	if(upgrade_p.pkg.sw_12 == 1)
		upgrade_p.pkg.data1[upgrade_p.pkg.cnt] = data;
	else
		upgrade_p.pkg.data2[upgrade_p.pkg.cnt] = data;

	// 校验，最后一包不校验，因为有尾标识和校验
	if(upgrade_p.pkg.ota_rx == BIN_RECV_GO && pkg_times < upgrade_p.pkg.times){
		upgrade_p.pkg.rx_crc += data;
	}

	upgrade_p.pkg.cnt++;
	upgrade_p.pkg.total++;

	if(upgrade_p.pkg.ota_rx == BIN_RECV_GO){
		// 满4KB就切换，并写
		if(upgrade_p.pkg.cnt == BUFFER_SIZE){
			upgrade_p.pkg.cnt 		= 0;
			upgrade_p.pkg.is_write 	= 1;
			if(upgrade_p.pkg.sw_12 == 1) upgrade_p.pkg.sw_12 = 2;
			else upgrade_p.pkg.sw_12 = 1;
			pkg_times ++;	// 计算校验
		}
	}else if(upgrade_p.pkg.ota_rx == BIN_RECV_READY){
		// 解析包头
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
 * OTA在滴答定时器计时
 * 说明：分别实现100ms|200ms|300ms|400ms|500ms定时,记忆当前时间(秒)
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
	// 实时时间，4G连接成功后，会将utctime更新成网络时间
	if(sysTimeClock - utctime_t >= 1000){	// 1s
		utctime_t = sysTimeClock;
		utctime++;
	}
}
/*
 * 更新计时标志
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
 * 使能通过AT+CCLK获取UTC时间
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
 * 发送AT
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
 * 发送AT
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
 * 发送AT+CPIN?获取SIM卡状态
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
 * 发送AT+CREG?判断GSM网络是否正常
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
 * 发送AT+CPIN?获取SIM卡状态
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
 * 获取秘钥：发送指令
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
 * 获取秘钥：发送数据
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
 * 注册设备：发送指令
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
 * 注册设备：发送数据
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
 * 请求更新：发送指令
 * 说明：也是手动控制OTA升级的入口
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
 * 请求更新：发送数据
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
 * HTTP升级操作：阶段1
 */
void AT_QHTTPURL_To_Upgrade_Stage1()
{
	server_p._4G_status = OTA_UPGRADE;
	upgrade_p.recv_ok	= FAILURE;
	server_p.stage 		= 0;
	uint8_t cmd[30]		= {0};
	sprintf(cmd,"AT+QHTTPURL=%d,80\r\n",strlen(upgrade_p.info.https));
//	fprintf(USART1_STREAM,"%s\r\n",cmd);	// 删除后，4G模块接收数据不完全，后期改进，类似于delay
	Clear_Buffer();
	EC200U_SendData(cmd,strlen(cmd));
}

/*
 * HTTP升级操作：阶段2
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
 * HTTP升级操作：阶段3
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
 * HTTP升级操作：阶段4
 */
void AT_QHTTPURL_To_Upgrade_Stage4()
{
	server_p._4G_status = OTA_UPGRADE;
	upgrade_p.recv_ok 	= FAILURE;
	server_p.stage 		= 3;
	// 开始接收升级包
//	upgrade_p.rx_mode 	= RECV_BIN;	// 开始接收bin文件包
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
 * 获取秘钥：发送指令
 */
void AT_QISEND_To_Replay_Success_Cmd()
{
	upgrade_p.rx_mode 	= RECV_CMD;			// 切换到指令模式
	server_p._4G_status = OTA_RESPONSE;		// OTA升级后回复
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
 * 获取秘钥：发送指令
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
 * 将时间字符串转成UTC时间
 * 参数：时间字符串："21/11/01,09:50:40+00"
 * 返回：UTC时间
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
 * 将UTC时间转成时间结构体，时间戳
 * 参数：UTC时间
 * 返回：时间结构体
 */
TimeType Get_CurrentTime_Stamp()
{
	TimeType t;
	t = alg_Utc2LocalTime(utctime,8);
	return t;
}
/*
 * 获取设备的将MD5加密后的32位小写的sign码
 * 参数：返回的sign字符串
 * 返回：无
 */
void Get_Device_Sign_Md5(uint8_t *sign)
{
	char value[50]	= "";

	sprintf(value,"%s%ld%s",upgrade_p.info.sn, utctime, upgrade_p.info.secret);
	cal_md5(sign, sizeof(sign), value, 1);	// 小写
	fprintf(USART1_STREAM,"sign:%s\n",sign);
}
/*
 * 从字符串中提取有用的数据
 * buffer:输入字符串
 * input:需要检索的字符串
 * output:检索完毕后输出的字符串
 * 函数说明：从buffer中提取input后的完整的字符串信息，存放到output中
 * 返回值：0没有找到，1检索到查询数据
 */
int Get_Useful_Data(const char *buffer, char *input, char *output)
{
	char* strx = strstr((const char*)buffer, (const char*)input);
	if (strx) {
		// 遍历操作，直到遇到','和'}'才返回
		int init_len = strlen(input) + 2;
		if (strx[init_len] == '\"') {	// 字符串
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
		else {	// 数字
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
 * 检测并提取升级包的信息
 * 参数：strx有效信息
 * 返回：1提取成功；0失败;2无需升级
 * 说明：截取下载路径，文件大小，md5，升级包版本
 */
//{"result":{"type":"controller","resource_path":"https:\/\/qupinmate.oss-cn-beijing.aliyuncs.com\/a\/v1.0.0.6.bin","size":34836,"md5":"8d3b591352d48c1a6fc192cb58d26c3d","version":"v1.0.0.7"},"id":0}
//无需要升级：{"result":0,"id":0}
uint8_t Analysis_UpgradePackage_Info(char * strx)
{
//	fprintf(USART1_STREAM,"%s\r\n",strx);
	if(strx[8] == '0'){
		return 2;	// 已经在平台升级过了
	}
	uint8_t data_size[10] 	= {0};
	uint8_t sign[33]		= {0};
	uint8_t http_path[100] 	= {0};
	Get_Useful_Data((const char*)strx,"resource_path",http_path);
	Get_Useful_Data((const char*)strx,"size",data_size);
	Get_Useful_Data((const char*)strx,"md5",sign);
	Get_Useful_Data((const char*)strx,"version",upgrade_p.info.upgrade_v);

	upgrade_p.info.size = atoi(data_size) - 8;	// 去掉首8个字节
	if(strlen(upgrade_p.info.upgrade_v)>5 && strstr((const char*)http_path,(const char*)".bin") && 	strlen(sign)>30 && upgrade_p.info.size>1024)
	{
		uint8_t i = 0;
		/* 得到完整的访问路径  */
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
 * 检测是否升级
 * 参数：
 * 返回：0不升级；1升级
 */
uint8_t Check_For_Upgrade()
{
//	fprintf(USART1_STREAM,"c:%s,p:%s\r\n",upgrade_p.info.run_v,upgrade_p.info.upgrade_v);
	if(Version_Compare(upgrade_p.info.run_v, upgrade_p.info.upgrade_v)){
		// 先清空200KB 外部FLASH空间
		for(uint32_t i=0;i<50;i++){
			W25QXX_Erase_Sector(i);
		}
		fprintf(USART1_STREAM,"OTA need upgrade.\r\n");
		return 1;
	}else{
		// 不需要升级
		fprintf(USART1_STREAM,"OTA no need upgrade.\r\n");
		return 0;
	}
}

void OTA_UserCmd_Analysis_In_USART1IT(uint8_t *User_Rxbuffer)
{
	char *strx_1 = NULL;
	// OTA 指令升级 ,升级到最新版本，重新检测一下
	if(strstr((const char*)User_Rxbuffer,(const char*)"ZKHYCHK*UPDATE:NEWEST")){
		// 获取当前版本号
		memset(upgrade_p.info.upgrade_v, 0, sizeof(upgrade_p.info.upgrade_v));
		Get_Upgrade_App_Version(upgrade_p.info.run_v);
//		fprintf(USART1_STREAM,"%s\r\n",upgrade_p.info.run_v);
		OTA_Start(UPGRADE_OTA,0);
		fprintf(USART1_STREAM,">> Upgrade...\r\n");
		Clear_Usart1_Buffer();// 清空接收.buffer
	}else
	// <ZKHYSET*UPDATE:V1.1.0.L1C6,60436>
	// 升级到指定版本（命名规则：https路径+版本名称+.bin）(<ZKHYSET*UPDATE:版本号,升级包大小>)
	if((strx_1 = strstr((const char*)User_Rxbuffer,(const char*)"ZKHYSET*UPDATE"))){
//		fprintf(USART1_STREAM,"%s\r\n",strx_1);
		StrtokStr str = StrtokString(strx_1+15);
		if(str.cnt == 2){
			uint8_t version[30] = {0};
			strcpy(upgrade_p.info.upgrade_v,str.data[0]);		// 升级版本号
			sprintf(version,"%s.bin",str.data[0]);

			if(strlen(upgrade_p.info.https_hu) < 5){
				Clear_Usart1_Buffer();// 清空接收buffer
				fprintf(USART1_STREAM,">> This function cannot be used because the program has not been upgraded by OTA.\r\n");
				return ;
			}
//			IWDT_Feed_The_Dog();
			sprintf(upgrade_p.info.https,"%s%s",upgrade_p.info.https_hu,version);		// 下载路径
//			strcat(upgrade_p.info.https,version);
			uint32_t size = atoi(str.data[1]) - 8;			// 升级包长度，去掉收尾16个字节
//			fprintf(USART1_STREAM,"h_a:%s,%d\r\n",upgrade_p.info.https,size);

			// 启动OTA指令升级
			OTA_Start(UPGRADE_OTA_CMD,size);
			fprintf(USART1_STREAM,">> Upgrade...\r\n");
		}else{
			fprintf(USART1_STREAM,">> Semantic Error...\r\n");
		}
		Clear_Usart1_Buffer();// 清空接收buffer
	}
}
/*
 * 解析EC200U-CN 的SN码
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


