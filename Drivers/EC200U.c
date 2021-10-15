/*
 * 4G_model.h
 *
 *  Created on: 2021-7-10
 *      Author: chenq
 */

#include "usart.h"
#include "uart_task.h"
#include "EC200U.h"
#include "stdlib.h"
#include "string.h"

#define SN 1234567890
#define SECRET 897621

uint8_t EC200U_Receive_flag;
uint8_t EC200U_Rxbuffer[EC200U_Rxbuffer_MAX];
uint8_t EC200U_Rxcount;

void Clear_Buffer(void)
{
	uint8_t i;
    for(i=0;i<EC200U_Rxbuffer_MAX;i++)
    	EC200U_Rxbuffer[i]=0;
    EC200U_Rxcount=0;
}

void EC200U_INIT()
{
	GPIO_Write_Mode_Bits(GPIOD_SFR, GPIO_PIN_MASK_1, GPIO_MODE_OUT);//POWER_EN
	GPIO_Set_Output_Data_Bits(GPIOD_SFR,GPIO_PIN_MASK_1, 1);
	GPIO_Write_Mode_Bits(GPIOB_SFR, GPIO_PIN_MASK_8, GPIO_MODE_OUT);//POWERKEY
	GPIO_Set_Output_Data_Bits(GPIOB_SFR,GPIO_PIN_MASK_8, 0);
				//GPIO_Write_Mode_Bits(GPIOB_SFR, GPIO_PIN_MASK_7, GPIO_MODE_OUT);//RESET
				//GPIO_Set_Output_Data_Bits(GPIOB_SFR,GPIO_PIN_MASK_7, 1);
	GPIO_Write_Mode_Bits(GPIOB_SFR, GPIO_PIN_MASK_6, GPIO_MODE_OUT);//WAKEUP
	GPIO_Set_Output_Data_Bits(GPIOB_SFR,GPIO_PIN_MASK_6, 1);
	GPIO_Write_Mode_Bits(GPIOB_SFR, GPIO_PIN_MASK_9, GPIO_MODE_OUT);//DISABLE
	GPIO_Set_Output_Data_Bits(GPIOB_SFR,GPIO_PIN_MASK_9, 0);
}

void EC200U_SendData(uint8_t* Databuf, uint32_t length)
{
	USART_Send(USART0_SFR, Databuf, length);
}


int EC200U_Start(work_mode_eu mode)
{
	char *strx,*extstrx;
	uint8_t error;
	EC200U_POWERKEY(Bit_RESET);
	vTaskDelay(3000);
	EC200U_POWERKEY(Bit_SET);
	vTaskDelay(3000);

	EC200U_SendData("AT\r\n",4);
	vTaskDelay(300);
	strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
	error=0;
	while(strx==NULL&&error<10)
	{
	    Clear_Buffer();
	    EC200U_SendData("AT\r\n",4);
	    vTaskDelay(500);
	    strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
	    error++;
	}
	if(error>=10)
		return -1;
	EC200U_SendData("ATE0\r\n",6);
	vTaskDelay(500);
	Clear_Buffer();
	EC200U_SendData("AT+CSQ\r\n",8);
	vTaskDelay(500);
	EC200U_SendData("ATI\r\n",5);
	vTaskDelay(500);
	//判断SIM卡是否插入
	EC200U_SendData("AT+CPIN?\r\n",10);
	vTaskDelay(500);
	strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"+CPIN: READY");
	error=0;
	while(strx==NULL&&error<10)
	{
	    Clear_Buffer();
	    EC200U_SendData("AT+CPIN?\r\n",10);
	    vTaskDelay(500);
	    strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"+CPIN: READY");//如果没有卡 返回错误
	    error++;
	}
	if(error>=10)
	    return -1;
	Clear_Buffer();
    //查看是否注册GSM网络
	EC200U_SendData("AT+CREG?\r\n",10);
	vTaskDelay(500);
	strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"+CREG: 0,1");
	extstrx=strstr((const char*)EC200U_Rxbuffer,(const char*)"+CREG: 0,5");
	error=0;
	while(strx==NULL&&extstrx==NULL&&error<10)
	{
	    Clear_Buffer();
	    EC200U_SendData("AT+CREG?\r\n",10);//
	    vTaskDelay(500);
	    strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"+CREG: 0,1");
	    extstrx=strstr((const char*)EC200U_Rxbuffer,(const char*)"+CREG: 0,5");
	    error++;
	}
	if(error>=10)
	    return -1;
	Clear_Buffer();
	EC200U_SendData("AT+CGREG?\r\n",11);
	vTaskDelay(500);
	strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"+CGREG: 0,1");
	extstrx=strstr((const char*)EC200U_Rxbuffer,(const char*)"+CGREG: 0,5");
	error=0;
	while(strx==NULL&&extstrx==NULL&&error<10)
	{
	    Clear_Buffer();
	    EC200U_SendData("AT+CGREG?\r\n",11);
	    vTaskDelay(500);
	    strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"+CGREG: 0,1");
	    extstrx=strstr((const char*)EC200U_Rxbuffer,(const char*)"+CGREG: 0,5");
	    error++;
	}
	if(error>=10)
	    return -1;
	Clear_Buffer();

	EC200U_SendData("AT+COPS?\r\n",10);
	vTaskDelay(500);
	Clear_Buffer();
	EC200U_SendData("AT+QICLOSE=0\r\n",12);
	vTaskDelay(500);
	Clear_Buffer();
	EC200U_SendData("AT+QICSGP=1,1,\"CMNET\",\"\",\"\",0\r\n",31);
	vTaskDelay(500);
    strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
	error=0;
	while(strx==NULL&&error<10)
	{
		vTaskDelay(500);
	    strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
	    error++;
	}
	if(error>=10)
	   	return -1;
	Clear_Buffer();
	EC200U_SendData("AT+QIDEACT=1\r\n",14);
	vTaskDelay(500);
	strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
	while(strx==NULL)
	{
		vTaskDelay(500);
	    strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
	}
	Clear_Buffer();
	EC200U_SendData("AT+QIACT=1\r\n",12);
	vTaskDelay(500);
	strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
	error=0;
	while(strx==NULL&&error<10)
	{
		vTaskDelay(500);
	    strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
	    error++;
	}
	if(error>=10)
	   	return -1;
	Clear_Buffer();
	EC200U_SendData("AT+QIACT?\r\n",11);
	vTaskDelay(500);
	Clear_Buffer();
	switch(mode)
	{
	case EC200U_WORKMODE_NET:
		EC200U_SendData("AT+QIOPEN=1,0,\"TCP\",\"www.gxsq.com\",9818,0,1\r\n",55);
		vTaskDelay(500);
		strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"+QIOPEN: 0,0");
		error=0;
		while(strx==NULL&&error<10)
		{
			strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"+QIOPEN: 0,0");
			vTaskDelay(100);
			error++;
		}
		break;
	default:
		EC200U_SendData("AT+QIOPEN=1,0,\"TCP\",\"www.zhongkehuiyan.com\",19095,0,1\r\n",55);
		vTaskDelay(500);
		strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"+QIOPEN: 0,0");
		error=0;
		while(strx==NULL&&error<10)
		{
			strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"+QIOPEN: 0,0");
			vTaskDelay(100);
			error++;
		}
		break;
	}
	if(error>=10)
	   	return -1;

	vTaskDelay(500);
	Clear_Buffer();
	return 0;

}


//发送字符串
int EC200U_Send_StrData(char *bufferdata,uint32_t length)
{
	uint8_t untildata=0xff,error;
	char * strx;
    Clear_Buffer();
	EC200U_SendData("AT+QISEND=0\r\n",13);
	vTaskDelay(100);
	EC200U_SendData(bufferdata,length);
	vTaskDelay(100);
	USART_SendData(USART0_SFR,0x1a);
    vTaskDelay(100);
    strx=strstr((char*)EC200U_Rxbuffer,(char*)"SEND OK");
    error=0;
    while(strx==NULL&&error<10)
    {
        strx=strstr((char*)EC200U_Rxbuffer,(char*)"SEND OK");
        vTaskDelay(100);
	    error++;
	}
	if(error>=10)
	   	return -1;
	else
		return 0;
    /*
    Clear_Buffer();
    EC200U_SendData("AT+QISEND=0,0\r\n",15);//确认服务器是否接收完毕，可以选择不要
    vTaskDelay(200);
    strx=strstr((char*)EC200U_Rxbuffer,(char*)"+QISEND:");
    error=0;
    while(untildata&&error<100)
    {
        EC200U_SendData("AT+QISEND=0,0\r\n",15);
        vTaskDelay(200);
        strx=strstr((char*)EC200U_Rxbuffer,(char*)"+QISEND:");
        strx=strstr((char*)strx,(char*)",");
        strx=strstr((char*)(strx+1),(char*)",");
        untildata=*(strx+1)-0x30;
        error++;
        Clear_Buffer();
    }
    if(error>=100)
    	UART_Puts("4G send failed.\r\n");
    Clear_Buffer();
    */
}

//获取字符串
int EC200U_Get_StrData(char *bufferdata,uint32_t *length)
{
	uint8_t error=0;
	char * strx,*temp;
	*length=0;
	temp=bufferdata;
    strx=strstr((char*)EC200U_Rxbuffer,(char*)"+QIURC");
    while(strx==NULL&&error<10)
    {
           strx=strstr((char*)EC200U_Rxbuffer,(char*)"+QIURC");
           vTaskDelay(100);
           error++;
    }
	if(error>=10)
	   	return -1;
	while(*strx!='{'&&*strx!=0)
		strx++;
	while((*strx!='\n'||*strx!='\r')&&(*length<(EC200U_Rxbuffer_MAX-15)))
	{
		*temp=*strx;
		*length++;
		temp++;
		strx++;
	}
   	return 0;
}

//发送十六进制数 例如 AT+QISENDEX=0,"424344"  即为发送“BCD”
void EC20Send_HexData(char *bufferdata,uint32_t length)
{
	uint8_t untildata=0xff,error;
	char * strx;
    Clear_Buffer();
	EC200U_SendData("AT+QISENDEX=0,\"",16);
	EC200U_SendData(bufferdata,length);
	EC200U_SendData("\"\r\n",3);
	vTaskDelay(100);
    strx=strstr((char*)EC200U_Rxbuffer,(char*)"OK");
    error=0;
    while(strx==NULL&&error<10)
    {
        strx=strstr((char*)EC200U_Rxbuffer,(char*)"OK");
        vTaskDelay(10);
        error++;
    }

    Clear_Buffer();
    EC200U_SendData("AT+QISEND=0,0\r\n",15);
    vTaskDelay(200);
    strx=strstr((char*)EC200U_Rxbuffer,(char*)"+QISEND:");
    error=0;
    while(untildata&&error<100)//确认服务器是否接收完毕，可以选择不要
    {
        EC200U_SendData("AT+QISEND=0,0\r\n",15);
        vTaskDelay(200);
        strx=strstr((char*)EC200U_Rxbuffer,(char*)"+QISEND:");
        strx=strstr((char*)strx,(char*)",");
        strx=strstr((char*)(strx+1),(char*)",");
        untildata=*(strx+1)-0x30;
        error++;
        Clear_Buffer();
    }
    if(error>=100)
    	UART_Puts("4G send failed.\r\n");
}

int EC200U_Config_GPS()
{
	char *strx;
	uint8_t error=0;
	Clear_Buffer();
	EC200U_SendData("AT+QGPS?\r\n",10);
	vTaskDelay(500);
	strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"+QGPS: 1");
	if(strx==NULL)
	{
		EC200U_SendData("AT+QGPS=1\r\n",11);
		vTaskDelay(500);
		strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
		while(strx==NULL&&error<10)
		{
            strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
            vTaskDelay(100);
            error++;
        }
		Clear_Buffer();
		if(strx==NULL)
			return -1;
		else
			return 0;
	}
	else
		return 0;
}


void EC200U_Get_NMEA(char * NMEA_type)//GGA RMC GSV GSA VTG
{
	char *strx;
	uint8_t error=0;
	Clear_Buffer();
	EC200U_SendData("AT+QGPSCFG=\"nmeasrc\",1\r\n",24);
	vTaskDelay(500);
	strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
	while(strx==NULL)
	{
	    strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
	    vTaskDelay(100);
    }
	Clear_Buffer();
	EC200U_SendData("AT+QGPSGNMEA=",13);
	EC200U_SendData(NMEA_type,3);
	EC200U_SendData("\"\r\n",3);
	vTaskDelay(500);
    strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
    while(strx==NULL&&error<10)
	{
        strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
    	vTaskDelay(100);
    	error++;
	}
    UART_Puts((char*)EC200U_Rxbuffer);

}

void EC200U_Get_GPS()
{
	char *strx;
	uint8_t error=0;
	Clear_Buffer();
	EC200U_SendData("AT+QGPSLOC=0\r\n",20);//
	vTaskDelay(500);
    strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
    while(strx==NULL&&error<10)
	{
        strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
    	vTaskDelay(100);
    	error++;
	}
    UART_Puts((char*)EC200U_Rxbuffer);

}

int EC200U_Config_BT(int start)
{
	char *strx;
	uint8_t error=0;
	Clear_Buffer();
	if(start==1)
	{
		EC200U_SendData("AT+QBTPWR=1\r\n",13);
		vTaskDelay(500);
		strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
		while(strx==NULL&&error<10)
		{
            strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
            vTaskDelay(100);
            error++;
        }
		Clear_Buffer();
		if(strx==NULL)
			return -1;
		else
			return 0;
	}
	else
	{
		EC200U_SendData("AT+QBTPWR=0\r\n",13);
		vTaskDelay(500);
		strx=strstr((const char*)EC200U_Rxbuffer,(const char*)"OK");
		return 0;
	}

}
