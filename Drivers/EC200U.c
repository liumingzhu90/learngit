/*
 * 4G_model.h
 *
 *  Created on: 2021-7-10
 *      Author: chenq
 */

#include "usart.h"
#include "EC200U.h"
#include "w25qxx.h"
#include "stdlib.h"
#include "string.h"
//#include "cJSON.h"
#include "common.h"
#include "md5.h"
#include "utc_time.h"

#define JSON_ID			0		// id
//#define vTaskDelay(ms)	delay_ms(ms)

/* -----------------------局部函数声明------------------------------- */
int 	EC200U_Get_StrData(char *bufferdata,uint32_t *length);
void 	EC20Send_HexData(char *bufferdata,uint32_t length);
uint8_t Check_And_Update_Baud();
void 	Disable_Usart_Interrupt(USART_SFRmap *USARTx,InterruptIndex Periphera);

/* ------------------------全局变量定义区------------------------------- */
uint8_t 			EC200U_Rxbuffer[BUFFER_SIZE];
volatile uint16_t 	EC200U_Rxcount;
//延时函数 局部变量用volatile声明，否则可能会被优化
void delay_ms(uint32_t nms)
{
	volatile uint32_t i,j;
	for(i=0;i<nms;i++)
	{
		j=7000;
		while(j--);
	}
}
/*
 * 清空USART0缓冲区
 * 返回：无
 */
void Clear_Buffer(void)
{
	memset(EC200U_Rxbuffer,0,BUFFER_SIZE);
    EC200U_Rxcount=0;
}
/*
 * 4G模块使能
 */
void Module_4G_ReStart()
{
	// 软件重启上电
	EC200U_POWERON(Bit_RESET);
	delay_ms(100);
	EC200U_POWERON(Bit_SET);
	delay_ms(100);
	// 重新使能
	EC200U_POWERKEY(Bit_RESET);
	delay_ms(100);
	EC200U_POWERKEY(Bit_SET);
	delay_ms(100);
	fprintf(USART1_STREAM,"4G module restart.\r\n");
}
/*
 * EC200U-CN 4G模块初始化
 */
void EC200U_INIT()
{
	// PB6:WAKEUP | PB7:RESET | PB8:POWERKEY | PB9:DISABLE | PD1:POWER_EN
	GPIO_Write_Mode_Bits(GPIOD_SFR, GPIO_PIN_MASK_1, GPIO_MODE_OUT);//POWER_EN
	GPIO_Set_Output_Data_Bits(GPIOD_SFR,GPIO_PIN_MASK_1, 1);
	GPIO_Write_Mode_Bits(GPIOB_SFR, GPIO_PIN_MASK_8, GPIO_MODE_OUT);//POWERKEY
	GPIO_Set_Output_Data_Bits(GPIOB_SFR,GPIO_PIN_MASK_8, 0);
	GPIO_Write_Mode_Bits(GPIOB_SFR, GPIO_PIN_MASK_7, GPIO_MODE_OUT);//RESET
	GPIO_Set_Output_Data_Bits(GPIOB_SFR,GPIO_PIN_MASK_7, 1);
	GPIO_Write_Mode_Bits(GPIOB_SFR, GPIO_PIN_MASK_6, GPIO_MODE_OUT);//WAKEUP
	GPIO_Set_Output_Data_Bits(GPIOB_SFR,GPIO_PIN_MASK_6, 1);
	GPIO_Write_Mode_Bits(GPIOB_SFR, GPIO_PIN_MASK_9, GPIO_MODE_OUT);//DISABLE
	GPIO_Set_Output_Data_Bits(GPIOB_SFR,GPIO_PIN_MASK_9, 0);

	Module_4G_ReStart();

	 //AT+IPR=19200;&W
	User_USART0_Init(1);
	USART_Async_config(USART0_SFR, 230400);	// 14400 9600
	USART_ReceiveInt_config(USART0_SFR,INT_USART0);

	// 判断当前4G模块是不是115200；若不是，先改成19200改回来波特率，之后再将波特率改成115200
//	if(Check_And_Update_Baud()){
////		fprintf(USART1_STREAM,"波特率230400\r\n");
//	}else{
//		fprintf(USART1_STREAM,"未能识别波特率.\r\n");
//	}
}
/*
 * 4G模块发送数据
 * 参数：Databuf：发送数据；length:发送字符长度
 * 返回：无
 */
void EC200U_SendData(uint8_t* Databuf, uint32_t length)
{
	USART_Send(USART0_SFR, Databuf, length);
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
	while((*strx!='\n'||*strx!='\r')&&(*length<(BUFFER_SIZE-15)))
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
    if(error>=100){
//    	UART_Puts("4G send failed.\r\n");
    	fprintf(USART1_STREAM,"4G send failed.\r\n");
    }
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
//    UART_Puts((char*)EC200U_Rxbuffer);
//    fprintf(USART1_STREAM,(char*)EC200U_Rxbuffer);
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
//    UART_Puts((char*)EC200U_Rxbuffer);
//    fprintf(USART1_STREAM,(char*)EC200U_Rxbuffer);
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
/*
 * 关闭串口中断
 */
void Disable_Usart_Interrupt(USART_SFRmap *USARTx,InterruptIndex Peripheral)
{
	USART_RDR_INT_Enable(USARTx,FALSE);
	INT_Interrupt_Enable(Peripheral,FALSE);
	USART_ReceiveData(USARTx);//清接收标志位
	INT_All_Enable(FALSE);
}
/*
 * 校验与更新波特率
 * 参数：
 * 返回：0；1沿用115200
 * 说明：先用115200波特率发送AT指令，若收到回复OK，就执行后面升级逻辑；若5次都没有接收到就改波特率19200；
 *       发送AT指令，之后发送修改波特率指令AT+IPR=115200;&W，收到OK后；修改波特率115200，执行后面升级逻辑。
 */
#define CMD_TIMES		14	// 发送指令的次数
#define DELAY_TIME		500
uint8_t Check_And_Update_Baud()
{
	uint8_t times = CMD_TIMES;
	uint8_t cmd[20] = {0};
	char *strx = NULL;
	// 发送AT指令
	strcpy(cmd,"AT\r\n");
	while(times){
		times --;
		Clear_Buffer();
		EC200U_SendData(cmd,strlen(cmd));
		delay_ms(DELAY_TIME);	// 需要修改 2021-12-06
		if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"OK")){
//			fprintf(USART1_STREAM,"直接返回，%d\r\n",times);
			return 1;
		}
	}

	// 修改波特率 115200 默认
	USART_Async_config(USART0_SFR, 115200);
	USART_ReceiveInt_config(USART0_SFR,INT_USART0);

	// 发送AT指令
	times = CMD_TIMES;
	strcpy(cmd,"AT\r\n");
	while(times){
		times --;
		Clear_Buffer();
		EC200U_SendData(cmd,strlen(cmd));
		delay_ms(DELAY_TIME);
		if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"OK")){
//			fprintf(USART1_STREAM,"已修改115200\r\n");
			break;
		}
	}
	if(times == 0){
		fprintf(USART1_STREAM,"发送AT失败\r\n");
		return 0;
	}
	// 发送修改波特率 AT+IPR=115200;&W
	times = CMD_TIMES;
	strcpy(cmd," AT+IPR=230400;&W\r\n");
	while(times){
		times --;
		Clear_Buffer();
		EC200U_SendData(cmd,strlen(cmd));
		delay_ms(DELAY_TIME);
		if(upgrade_p.recv_ok==SUCCESS && strstr((const char*)EC200U_Rxbuffer,(const char*)"OK")){
			fprintf(USART1_STREAM,"修改了4G的baud 230400\r\n");
			break;
		}
	}
	if(times != 0){
		// 修改波特率
		USART_Async_config(USART0_SFR, 230400);
		USART_ReceiveInt_config(USART0_SFR,INT_USART0);
		return 1;
	}
//	fprintf(USART1_STREAM,"流程最后\r\n");
	return 0;
}
