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
/* ------------------------ȫ�ֱ���------------------------------- */
uint16_t 			recv_bin_times = 0;
/* -----------------------ȫ�ֺ�������------------------------------- */


/*
 * �û�������ʼ
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
 * �û���������
 */
void Usart_Upgrade_End()
{
	upgrade_p.rx_mode = RECV_CMD;
	upgrade_p.pkg.ota_rx = BIN_RECV_WAIT;
	upgrade_p.pkg.cnt = 0;
}

/*
 * �û�ͨ��usart1���������߼�
 * ��������
 * ���أ���
 * ˵�������ͷ�ʽ���ʴ�ʽ�洢
 */

void Usart1_UserUpgrade_Interactoin_In_MainLoop()
{
	uint16_t i = 0;
	// �Ƚ��������������β��
	if(recv_bin_times < upgrade_p.pkg.times){	// ��������
		if(upgrade_p.pkg.cnt >= BUFFER_SIZE_I){
			for(i=0;i<BUFFER_SIZE;i++){			// ȡУ��
				upgrade_p.pkg.rx_crc += upgrade_p.pkg.data_i[i];
			}
			upgrade_p.pkg.pkg_crc = upgrade_p.pkg.data_i[i];i++;
			upgrade_p.pkg.pkg_crc += upgrade_p.pkg.data_i[i]<<8;i++;
			upgrade_p.pkg.pkg_crc += upgrade_p.pkg.data_i[i]<<16;i++;
			upgrade_p.pkg.pkg_crc += upgrade_p.pkg.data_i[i]<<24;i++;

//			fprintf(USART1_STREAM,"r:%08X,p:%08X,%02X",upgrade_p.pkg.rx_crc,upgrade_p.pkg.pkg_crc);
			recv_bin_times++;
			if(upgrade_p.pkg.rx_crc == upgrade_p.pkg.pkg_crc){	// �ȶ�У��
				// ��������д
				W25QXX_Write_NoCheck(upgrade_p.pkg.data_i, upgrade_p.pkg.addr, BUFFER_SIZE);
				upgrade_p.pkg.addr += BUFFER_SIZE;
				// ��Ϊ���һ����д
				if(recv_bin_times==upgrade_p.pkg.times && upgrade_p.pkg.remain==0){
					// �汾��Ϣ�Ͱ���С
					Set_Upgrade_App_Version(upgrade_p.info.upgrade_v);
					Set_Upgrade_PKG_size(upgrade_p.info.size);
					Usart_Upgrade_End();
				}
				fprintf(USART1_STREAM,"AAAAAAAA\r\n");	// �ɹ�д��ظ�
			}else{
				for(uint8_t j=0; j<recv_bin_times; j++){
					W25QXX_Erase_Sector(j);
				}
				recv_bin_times = 0;
				upgrade_p.pkg.cnt = 0;
				fprintf(USART1_STREAM,"BBBBBBBB\r\n");	// ʧ�ܣ�������ظ�
			}
			upgrade_p.pkg.rx_crc = 0;
			upgrade_p.pkg.pkg_crc = 0;
			upgrade_p.pkg.cnt = 0;
			// ��д����-����4G�Ƴ���
			GPIO_Set_Output_Data_Bits(GPIOF_SFR,GPIO_PIN_MASK_0, Bit_SET);
		}
	}else{			// ����β��
		if(upgrade_p.pkg.cnt >= (upgrade_p.pkg.remain+5)){	// +5 = 4BУ��+1B��������
			for(i=0;i<upgrade_p.pkg.remain;i++){		// ȡУ��
				upgrade_p.pkg.rx_crc += upgrade_p.pkg.data_i[i];
			}
			upgrade_p.pkg.pkg_crc = upgrade_p.pkg.data_i[i];i++;
			upgrade_p.pkg.pkg_crc += upgrade_p.pkg.data_i[i]<<8;i++;
			upgrade_p.pkg.pkg_crc += upgrade_p.pkg.data_i[i]<<16;i++;
			upgrade_p.pkg.pkg_crc += upgrade_p.pkg.data_i[i]<<24;i++;

//			fprintf(USART1_STREAM,"r:%08X,p:%08X\r\n",upgrade_p.pkg.rx_crc,upgrade_p.pkg.pkg_crc);
			if(upgrade_p.pkg.rx_crc == upgrade_p.pkg.pkg_crc){
				W25QXX_Write_NoCheck(upgrade_p.pkg.data_i, upgrade_p.pkg.addr, upgrade_p.pkg.remain);
				// �汾��Ϣ�Ͱ���С
				Set_Upgrade_App_Version(upgrade_p.info.upgrade_v);
				Set_Upgrade_PKG_size(upgrade_p.info.size);

				fprintf(USART1_STREAM,"AAAAAAAA\r\n");	// �ɹ�д��ظ�
				Usart_Upgrade_End();
				// ��д���-����4G����
				GPIO_Set_Output_Data_Bits(GPIOF_SFR,GPIO_PIN_MASK_0, Bit_RESET);
			}else{
				for(uint8_t j=0; j<=recv_bin_times; j++){
					W25QXX_Erase_Sector(j);
				}
				recv_bin_times = 0;
				upgrade_p.pkg.cnt = 0;
				fprintf(USART1_STREAM,"BBBBBBBB\n");	// ʧ�ܣ�������ظ�
			}
		}
	}
}

/*
 * �����û�ָ��
 * ��������
 * ���أ���
 * ˵����(1)��������Ϣ���汾�ţ�����С��(2)��ȡ�汾�ţ� (3)��ȡSN��
 */
char *strx_1 = NULL;
void Usart1_UserCmd_Analysis()
{
	// �ֶ��������������ȷ����������Ĵ�С�� <ZKHYSET*PACKAGESIZE:v1.0.0.1,26644>
	if((strx_1 = strstr((const char*)User_Rxbuffer,(const char*)"ZKHYSET*PACKAGESIZE"))){
		// ȡ����
		StrtokStr str = StrtokString(strx_1+20);
		if(str.cnt == 2){
			// ���ñ���
			upgrade_p.info.size 	= atoi(str.data[1]);
			upgrade_p.pkg.times 	= upgrade_p.info.size / BUFFER_SIZE;
			upgrade_p.pkg.remain 	= upgrade_p.info.size % BUFFER_SIZE;
			strcpy(upgrade_p.info.upgrade_v,str.data[0]);

			// �����200KB �ⲿFLASH�ռ�
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
	// �����豸��SN�� <ZKHYSET*DEVICESN:EC200UCNAAR02A01M08>
	if((strx_1 = strstr((const char*)User_Rxbuffer,(const char*)"ZKHYSET*DEVICESN"))){
		// ȡ����
		StrtokStr str = StrtokString(strx_1 + 17);
		if(str.cnt == 1){
			Set_Device_SN(str.data[0]);
			fprintf(USART1_STREAM,">> SN:%s\r\n",str.data[0]);
			fprintf(USART1_STREAM,">> Set Success.\r\n");
		}
		Clear_Usart1_Buffer();
	}else
	// ��ȡ�豸��SN�� <ZKHYCHK*DEVICESN>
	if((strx_1 = strstr((const char*)User_Rxbuffer,(const char*)"ZKHYCHK*DEVICESN"))){
		char sn[30] = {0};
		Get_Device_SN(sn);
		fprintf(USART1_STREAM,">> SN:%s\r\n",sn);
		Clear_Usart1_Buffer();
	}else
	// ���õ�ǰ�汾�� <ZKHYSET*VERSION:v1.0.0.5,v1.0.0.6>
	if((strx_1 = strstr((const char*)User_Rxbuffer,(const char*)"ZKHYSET*VERSION"))){
		StrtokStr str = StrtokString(strx_1+16);
		if(str.cnt == 2){
			Set_Upgrade_App_Version(str.data[0]);
			Set_Run_App_Version(str.data[1]);
			// ͨ��������ʾ
			fprintf(USART1_STREAM,">> APP_Version:%s; MCU_Verson:%s\r\n",str.data[0],str.data[1]);
			fprintf(USART1_STREAM,">> Set Success.\r\n");
		}else{
			fprintf(USART1_STREAM,">> Set Version Failed.\r\n");
		}
		Clear_Usart1_Buffer();
	}else
	// ��ȡ��ǰ�汾��Ϣ	(<ZKHYCHK*VERSION>)
	if(strstr((const char*)User_Rxbuffer,(const char*)"ZKHYCHK*VERSION")){
		// ��ȡ��ǰapp�汾�š�MCU�汾��
		uint8_t app_v[VERSION_SIZE]={0}, mcu_v[VERSION_SIZE]={0};
		Get_Upgrade_App_Version(app_v);
		Get_Run_App_Version(mcu_v);

		// ͨ��������ʾ
		fprintf(USART1_STREAM,">> APP_Version:%s;Boot_Verson:%s\r\n",app_v,mcu_v);
		Clear_Usart1_Buffer();
	}
}
/*
 * ����,���зָ��ַ���
 * �������ָ��ַ���
 * ���أ�0ʧ�ܣ�1�ɹ�
 * ˵���� 1223,456,789,adc,456,85ad21,dfdsa00>
 */
StrtokStr StrtokString(uint8_t *string)
{
	StrtokStr strtokStr;
	strtokStr.cnt = 0;
	if(string == NULL) return strtokStr;
	// ���´��룬����Ҫ��ӡ�������������

//	fprintf(USART1_STREAM,"%s",string);	// �����ӡ��仰������Ч����ʱ����
	uint8_t *p = strtok(string,",");
	while(p){
		strcpy(strtokStr.data[strtokStr.cnt], p);

		strtokStr.data[strtokStr.cnt][strlen(strtokStr.data[strtokStr.cnt])] = '\0';
		p = strtok(NULL, ",");
		strtokStr.cnt++;
	}
	// ȥ���������һ���ַ�����'>'
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
 * ��ȡд��bin�ļ������ݣ��ó�4BУ��
 */
uint32_t Get_CRC(uint32_t size)
{
	uint32_t result = 0;

	uint32_t rw_addr 	= OUT_FLASH_APP_START;
	uint8_t sector_num 		= (size) / BUFFER_SIZE;
	uint16_t sector_remain 	= (size) % BUFFER_SIZE;
	uint8_t read_data[BUFFER_SIZE] = {0};
	/* �������BUFFER_SIZE��С������ */
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
		/* ������ʣ���С������ */
		W25QXX_Read(read_data,rw_addr,sector_remain);
		for(int j=0;j<sector_remain;j++){	// end package
			fprintf(USART1_STREAM, "%02X",read_data[j]);
//			result = (result+read_data[j])&0xFFFFFFFF;
		}
	}
//	fprintf(USART1_STREAM, "\r\n%08X\r\n",result);
	return result;
}










