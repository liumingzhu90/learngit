/*
 * usart_upgrade.c
 *
 *  Created on: 2021-11-17
 *      Author: Administrator
 */
#include "usart_upgrade.h"
#include "usart.h"
#include "stdio.h"
#include "stdlib.h"
#include "common.h"
/* ------------------------全局变量------------------------------- */
uint16_t 			recv_bin_times = 0;
/* -----------------------全局函数声明------------------------------- */


/*
 * 用户升级开始
 */
void Usart_Upgrade_Start()
{
	upgrade_p.rx_mode 	= RECV_BIN;
	upgrade_p.pkg.ota_rx 	= BIN_RECV_READY;
	upgrade_p.up_mode 	= UPGRADE_USART1;

	upgrade_p.pkg.rx_crc 	= 0;
	upgrade_p.pkg.pkg_crc = 0;
	upgrade_p.pkg.cnt  	= 0;
	upgrade_p.pkg.addr 	= OUT_FLASH_APP_START;
	recv_bin_times 	= 0;
}
/*
 * 用户升级结束
 */
void Usart_Upgrade_End()
{
	upgrade_p.rx_mode = RECV_CMD;
	upgrade_p.pkg.ota_rx = BIN_RECV_WAIT;
	upgrade_p.pkg.cnt = 0;
}

/*
 * 用户通过usart1升级交互逻辑
 * 参数：无
 * 返回：无
 * 说明：传送方式：问答式存储
 */

void Usart1_UserUpgrade_Interactoin_In_MainLoop()
{
	uint16_t i = 0;
	// 先接收整包，后接收尾包
	if(recv_bin_times < upgrade_p.pkg.times){	// 接收整包
		if(upgrade_p.pkg.cnt >= BUFFER_SIZE_I){
			for(i=0;i<BUFFER_SIZE;i++){			// 取校验
				upgrade_p.pkg.rx_crc += upgrade_p.pkg.data_i[i];
			}
			upgrade_p.pkg.pkg_crc = upgrade_p.pkg.data_i[i];i++;
			upgrade_p.pkg.pkg_crc += upgrade_p.pkg.data_i[i]<<8;i++;
			upgrade_p.pkg.pkg_crc += upgrade_p.pkg.data_i[i]<<16;i++;
			upgrade_p.pkg.pkg_crc += upgrade_p.pkg.data_i[i]<<24;i++;

//			fprintf(USART1_STREAM,"r:%08X,p:%08X,%02X",upgrade_p.pkg.rx_crc,upgrade_p.pkg.pkg_crc);
			recv_bin_times++;
			if(upgrade_p.pkg.rx_crc == upgrade_p.pkg.pkg_crc){	// 比对校验
				// 正常包，写
				W25QXX_Write_NoCheck(upgrade_p.pkg.data_i, upgrade_p.pkg.addr, BUFFER_SIZE);
				upgrade_p.pkg.addr += BUFFER_SIZE;
				// 若为最后一包，写
				if(recv_bin_times==upgrade_p.pkg.times && upgrade_p.pkg.remain==0){
					// 版本信息和包大小
					Set_Upgrade_App_Version(upgrade_p.info.upgrade_v);
					Set_Upgrade_PKG_size(upgrade_p.info.size);
					Usart_Upgrade_End();
				}
				fprintf(USART1_STREAM,"AAAAAAAA\r\n");	// 成功写入回复
			}else{
				for(uint8_t j=0; j<recv_bin_times; j++){
					W25QXX_Erase_Sector(j);
				}
				recv_bin_times = 0;
				upgrade_p.pkg.cnt = 0;
				fprintf(USART1_STREAM,"BBBBBBBB\r\n");	// 失败，擦除后回复
			}
			upgrade_p.pkg.rx_crc = 0;
			upgrade_p.pkg.pkg_crc = 0;
			upgrade_p.pkg.cnt = 0;
			// 烧写过程-板载4G灯长亮
			GPIO_Set_Output_Data_Bits(GPIOF_SFR,GPIO_PIN_MASK_0, Bit_SET);
		}
	}else{			// 接收尾包
		if(upgrade_p.pkg.cnt >= (upgrade_p.pkg.remain+5)){	// +5 = 4B校验+1B包分组编号
			for(i=0;i<upgrade_p.pkg.remain;i++){		// 取校验
				upgrade_p.pkg.rx_crc += upgrade_p.pkg.data_i[i];
			}
			upgrade_p.pkg.pkg_crc = upgrade_p.pkg.data_i[i];i++;
			upgrade_p.pkg.pkg_crc += upgrade_p.pkg.data_i[i]<<8;i++;
			upgrade_p.pkg.pkg_crc += upgrade_p.pkg.data_i[i]<<16;i++;
			upgrade_p.pkg.pkg_crc += upgrade_p.pkg.data_i[i]<<24;i++;

//			fprintf(USART1_STREAM,"r:%08X,p:%08X\r\n",upgrade_p.pkg.rx_crc,upgrade_p.pkg.pkg_crc);
			if(upgrade_p.pkg.rx_crc == upgrade_p.pkg.pkg_crc){
				W25QXX_Write_NoCheck(upgrade_p.pkg.data_i, upgrade_p.pkg.addr, upgrade_p.pkg.remain);
				// 版本信息和包大小
				Set_Upgrade_App_Version(upgrade_p.info.upgrade_v);
				Set_Upgrade_PKG_size(upgrade_p.info.size);

				fprintf(USART1_STREAM,"AAAAAAAA\r\n");	// 成功写入回复
				Usart_Upgrade_End();
				// 烧写完成-板载4G灯灭
				GPIO_Set_Output_Data_Bits(GPIOF_SFR,GPIO_PIN_MASK_0, Bit_RESET);
			}else{
				for(uint8_t j=0; j<=recv_bin_times; j++){
					W25QXX_Erase_Sector(j);
				}
				recv_bin_times = 0;
				upgrade_p.pkg.cnt = 0;
				fprintf(USART1_STREAM,"BBBBBBBB\n");	// 失败，擦除后回复
			}
		}
	}
}

/*
 * 解析用户指令
 * 参数：无
 * 返回：无
 * 说明：(1)升级包信息：版本号，包大小；(2)获取版本号； (3)获取SN号
 */
char *strx_1 = NULL;
void Usart1_UserCmd_Analysis()
{
	// 手动串口升级，需先发送升级包的大小， <ZKHYSET*PACKAGESIZE:v1.0.0.1,26644>
	if((strx_1 = strstr((const char*)User_Rxbuffer,(const char*)"ZKHYSET*PACKAGESIZE"))){
		// 取数据
		StrtokStr str = StrtokString(strx_1+20);
		if(str.cnt == 2){
			// 设置变量
			upgrade_p.info.size 	= atoi(str.data[1]);
			upgrade_p.pkg.times 	= upgrade_p.info.size / BUFFER_SIZE;
			upgrade_p.pkg.remain 	= upgrade_p.info.size % BUFFER_SIZE;
			strcpy(upgrade_p.info.upgrade_v,str.data[0]);

			// 先清空200KB 外部FLASH空间
			for(uint32_t i=0;i<50;i++){
				W25QXX_Erase_Sector(i);
			}
//			fprintf(USART1_STREAM,"%d,%d,%d,%s\r\n",upgrade_p.info.size,upgrade_p.pkg.times,upgrade_p.pkg.remain,upgrade_p.info.upgrade_v);
			fprintf(USART1_STREAM,"11111111\r\n");	// OK
			Usart_Upgrade_Start();

			memset(upgrade_p.pkg.data_i,0,BUFFER_SIZE_I);
		}else{
			fprintf(USART1_STREAM,"22222222\r\n");	// ERR
		}
		Clear_Usart1_Buffer();
	}else
	// 设置设备的SN号 <ZKHYSET*DEVICESN:EC200UCNAAR02A01M08>
	if((strx_1 = strstr((const char*)User_Rxbuffer,(const char*)"ZKHYSET*DEVICESN"))){
		// 取数据
		StrtokStr str = StrtokString(strx_1 + 17);
		if(str.cnt == 1){
			Set_Device_SN(str.data[0]);
			fprintf(USART1_STREAM,">> SN:%s\r\n",str.data[0]);
			fprintf(USART1_STREAM,">> Set Success.\r\n");
		}
		Clear_Usart1_Buffer();
	}else
	// 获取设备的SN号 <ZKHYCHK*DEVICESN>
	if((strx_1 = strstr((const char*)User_Rxbuffer,(const char*)"ZKHYCHK*DEVICESN"))){
		char sn[30] = {0};
		Get_Device_SN(sn);
		fprintf(USART1_STREAM,">> SN:%s\r\n",sn);
		Clear_Usart1_Buffer();
	}else
	// 设置当前版本号 <ZKHYSET*VERSION:v1.0.0.5,v1.0.0.6>
	if((strx_1 = strstr((const char*)User_Rxbuffer,(const char*)"ZKHYSET*VERSION"))){
		StrtokStr str = StrtokString(strx_1+16);
		if(str.cnt == 2){
			Set_Upgrade_App_Version(str.data[0]);
			Set_Run_App_Version(str.data[1]);
			// 通过串口显示
			fprintf(USART1_STREAM,">> APP_Version:%s; MCU_Verson:%s\r\n",str.data[0],str.data[1]);
			fprintf(USART1_STREAM,">> Set Success.\r\n");
		}else{
			fprintf(USART1_STREAM,">> Set Version Failed.\r\n");
		}
		Clear_Usart1_Buffer();
	}else
	// 获取当前版本信息	(<ZKHYCHK*VERSION>)
	if(strstr((const char*)User_Rxbuffer,(const char*)"ZKHYCHK*VERSION")){
		// 获取当前app版本号、MCU版本号
		uint8_t app_v[VERSION_SIZE]={0}, mcu_v[VERSION_SIZE]={0};
		Get_Upgrade_App_Version(app_v);
		Get_Run_App_Version(mcu_v);

		// 通过串口显示
		fprintf(USART1_STREAM,">> APP_Version:%s;Boot_Verson:%s\r\n",app_v,mcu_v);
		Clear_Usart1_Buffer();
	}
}
/*
 * 按照,进行分割字符串
 * 参数：分割字符串
 * 返回：0失败，1成功
 * 说明： 1223,456,789,adc,456,85ad21,dfdsa00>
 */
StrtokStr StrtokString(uint8_t *string)
{
	StrtokStr strtokStr;
	strtokStr.cnt = 0;
	if(string == NULL) return strtokStr;
	// 如下代码，必须要打印，才能正常输出

//	fprintf(USART1_STREAM,"%s",string);	// 必须打印这句话，才有效，暂时舍弃
	uint8_t *p = strtok(string,",");
	while(p){
		strcpy(strtokStr.data[strtokStr.cnt], p);

		strtokStr.data[strtokStr.cnt][strlen(strtokStr.data[strtokStr.cnt])] = '\0';
		p = strtok(NULL, ",");
		strtokStr.cnt++;
	}
	// 去除解析最后一个字符串的'>'
	for(uint8_t i=0; i<strlen(strtokStr.data[strtokStr.cnt-1]); i++){
		if(strtokStr.data[strtokStr.cnt-1][i] == '>'){
			strtokStr.data[strtokStr.cnt-1][i] = '\0';
			break;
		}
	}
//	for(uint8_t i=0; i<strtokStr.cnt;i++){
//		fprintf(USART1_STREAM,"%s ",strtokStr.data[i]);
//	}
//	fprintf(USART1_STREAM,"\r\n");

	return strtokStr;
}

void Clear_Usart1_Buffer()
{
	memset(User_Rxbuffer,0,sizeof(User_Rxbuffer));
	User_Rxcount=0;
}

/*
 * 读取写入bin文件的数据，得出4B校验
 */
uint32_t Get_CRC(uint32_t size)
{
	uint32_t result = 0;

	uint32_t rw_addr 	= OUT_FLASH_APP_START;
	uint8_t sector_num 		= (size) / BUFFER_SIZE;
	uint16_t sector_remain 	= (size) % BUFFER_SIZE;
	uint8_t read_data[BUFFER_SIZE] = {0};
	/* 输出整个BUFFER_SIZE大小的数据 */
	if(sector_num > 0){
		for(int k=0; k<sector_num; k++){
			memset(read_data, 0, BUFFER_SIZE);
			W25QXX_Read(read_data,rw_addr,BUFFER_SIZE);
			rw_addr += BUFFER_SIZE;

			// print
			for(int j=0;j<BUFFER_SIZE;j++){	//
				if(k == 0){					// first package
					fprintf(USART1_STREAM, "%02X ",read_data[j]);

				}
				else if(k == (sector_num-1))	// end package
					fprintf(USART1_STREAM, "%02X ",read_data[j]);

//				result = (result+read_data[j])&0xFFFFFFFF;
			}
		}
	}
	fprintf(USART1_STREAM, "\r\n\r\n");
	if(sector_remain > 0){
		memset(read_data, 0, BUFFER_SIZE);
		/* 最后输出剩余大小的数据 */
		W25QXX_Read(read_data,rw_addr,sector_remain);
		for(int j=0;j<sector_remain;j++){	// end package
			fprintf(USART1_STREAM, "%02X",read_data[j]);
//			result = (result+read_data[j])&0xFFFFFFFF;
		}
	}
//	fprintf(USART1_STREAM, "\r\n%08X\r\n",result);
	return result;
}










