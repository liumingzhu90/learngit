/*
 * update_program.c
 *
 *  Created on: 2021-7-15
 *      Author: chenq
 */
#include "system_init.h"
#include "update.h"
#include "bluetooth.h"

uint32_t test[256];

int update_program(void)
{
	uint32_t FlashDestination = 0x8000;
	uint32_t Flashsource=PROGRAM_START_ADDRESS;
	while(FlashDestination<PROGRAM_START_ADDRESS+PROGRAM_SIZE)
	{
		memcpy ( (uint8_t *)test, (uint8_t *)Flashsource, 1024);
    	//��������ҳд������
    	FLASH_PageWrite_fun( FlashDestination, test, 128);     //��һҳ������д��0x8000 ���ڳ�������flash PAGE
    	FlashDestination +=0x400; //ÿд1K����ַ�ۼ�1K
    	Flashsource +=0x400;
	}
	return 100;
}
#define FLAG Bluetooth_Receive_flag

static int32_t Receive_Packet (uint8_t *data, int32_t *length)
{
	uint32_t timeout =5000,time=0;//����������
	uint8_t flag=0;//����������
	while(1)
	{
		if(FLAG==1){
			*length=Bluetooth_get_data(data,128);
			break;
		}
		else if(time<timeout)
		{
			vTaskDelay(100);
			timeout+=100;
		}
		else
			return -1;

	}
	return 0;
}

int download_program(void)
{
	uint32_t FlashDestination=PROGRAM_START_ADDRESS;
	uint8_t buf[128],file_name[50];
	char *file_ptr;
	int i;
	uint32_t packet_length;
	uint32_t packets_received =0 ,k=0;
    while(1) //��������Э�鴫���жϻ������ش��󣬷��߻�һֱ������ȴ�����
    {
      if(!Receive_Packet(buf, &packet_length ))//��ȷ�Ľ�����һ�����ݰ�
      {
    	  if (packets_received == 0)//����ǵ�һ�����ݰ�
    	  {
    		  for (i = 0, file_ptr = buf ; (*file_ptr != 0) && (i < 50);) //ȡ���ļ���
    		  {
    		      file_name[i++] = *file_ptr++;
    		  }
    		  file_name[i++] = '\0'; //�����ÿո����
    	  }
    	  else//ÿ�δ���128�ֽ�
    	  {
    		  k = (packets_received-1) % 8;
    		  memcpy ( (uint8_t *)test + k*128, buf, 128);//���յ������ݰ���˳������test[]��
    		  //��������ҳд������
    		  if(k==7){
    			  FLASH_PageWrite_fun( FlashDestination, test, 128);     //��һҳ������д��0x8000 ���ڳ�������flash PAGE
    			  FlashDestination +=0x400; //ÿд1K����ַ�ۼ�1K
    		  }
    	  }
    	  packets_received ++; //���հ���������
      }

	  if(packets_received> 1600) //�������3200 /8 = 200 * 1K byteδ��������������ֹͣ��������
	  {
	     set_flag();
	     return 100;
	  }
    }
}

int check_flag(void)
{
	if(*(volatile uint32_t*)0x7FFFA == 0xAAAA5555)
		return 1;
	else
		return 0;
}

void set_flag(void)
{
	memcpy ( (uint8_t *)test, (uint8_t *)0x7A000, 1024);
	//���ñ�־λ
	test[255]=0xAAAA5555;
	FLASH_PageWrite_fun( 0x7A000, test, 128);
}

void clear_flag(void)
{
	memcpy ( (uint8_t *)test, (uint8_t *)0x7A000, 1024);
	//�����־λ
	test[255]=0xFFFFFFFF;
	FLASH_PageWrite_fun( 0x7A000, test, 128);
}
