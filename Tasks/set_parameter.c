/*
 * set_parameter.c
 *
 *  Created on: 2021-8-13
 *      Author: wangzhenbao
 */
#include "usart.h"
#include "flash.h"
#include <stdio.h>
#include "common.h"

_CAN_RATA_PARA		stCanPara;
_SYS_PARA			stSysPara;					// 系统参数
_SPEED_PARA			stSpeedPara;					// 车速参数
_PARA_TYPE			stGear;							//挡位
_PARA_TYPE			stThrottleOpening;				//油门开度
_PARA_TYPE			stBrakeOpening;					//刹车踏板开度
_TURN_PARA			stTurnPara;					// 转向参数
_BRAKE_PARA			stBrakePara;					// 刹车参数
_WARNING_PARA			stWarningPara;					// 报警
_SWA_PARA			stSwaPara;					// 方向盘拐角
_ABS_PARA			stAbsPara;					// ABS参数
_VEHICLE_PARA		stVehicleParas;						// 车辆参数
_SWADEGREE_PARA		stSWAParas;
_WARN_STATUS		warning_status;				//报警状态信息
SENSOR_CONFIG	camera_config[3];		//摄像头配置参数
SENSOR_CONFIG	rader_config[4];			//毫米波雷达配置参数
URADER_CONFIG		stUrader_config[42];
Alarm_linkage_PARA	stWarninglinkage;
_URADER_SYS_MESSAGE stUraderSysMessage;

uint8_t Bluetooth_Set_Canrata_Commond(uint8_t *data)
{
	uint8_t ret;

	ret = data[29] - 0x30;
	return ret;
}

uint32_t Bluetooth_Set_Commond_Annalyse(uint8_t *data)
{
	uint32_t ret;

	if((data[20] == 'C') &&
				(data[21] == 'A') &&
				(data[22] == 'N') &&
				(data[23] == 'R'))		//设置can0波特率命令
			ret = 0;
	else if((data[20] == 'C') &&
			(data[21] == 'A') &&
			(data[22] == 'N') &&
			(data[23] == '0'))		//设置can0波特率命令
		ret = 1;
	else if((data[20] == 'C') &&
			(data[21] == 'A') &&
			(data[22] == 'N') &&
			(data[23] == '1'))		//设置can1波特率命令
		ret = 2;
	else if((data[20] == 'C') &&
			(data[21] == 'A') &&
			(data[22] == 'N') &&
			(data[23] == '2'))		//设置can2波特率命令
		ret = 3;
	else if((data[20] == 'C') &&
			(data[21] == 'A') &&
			(data[22] == 'N') &&
			(data[23] == '3'))		//设置can3波特率命令
		ret = 4;
	else if((data[20] == 'C') &&
			(data[21] == 'A') &&
			(data[22] == 'N') &&
			(data[23] == '4'))		//设置can4波特率命令
		ret = 5;
	else if((data[20] == 'C') &&
			(data[21] == 'A') &&
			(data[22] == 'N') &&
			(data[23] == '5'))		//设置can5波特率命令
		ret = 6;
	else if((data[20] == 'V') &&
			(data[21] == 'E') &&
			(data[22] == 'H') &&
			(data[23] == 'I'))		//车辆参数设置
		ret = 7;
	else if((data[20] == 'S') &&
			(data[21] == 'P') &&
			(data[22] == 'E') &&
			(data[23] == 'E'))		//车辆参数设置
		ret = 8;
	else if((data[20] == 'A') &&
			(data[21] == 'B') &&
			(data[22] == 'S'))		//ABS参数设置
		ret = 9;
	else if((data[20] == 'T') &&
			(data[21] == 'U') &&
			(data[22] == 'R'))		//转向参数设置
		ret = 10;
	else if((data[20] == 'B') &&
			(data[21] == 'R') &&
			(data[22] == 'A'))		//刹车参数设置
		ret = 11;
	else if((data[20] == 'W') &&
			(data[21] == 'L') &&
			(data[22] == 'S'))		//联动报警参数设置
		ret = 12;
	else if((data[20] == 'S') &&
			(data[21] == 'W') &&
			(data[22] == 'A'))		//方向盘转角参数设置
		ret = 13;
	else if((data[20] == 'F') &&	//前
			(data[21] == 'R') &&
			(data[22] == 'O') &&
			(data[23] == 'N') &&
			(data[24] == 'T'))
		ret = 14;

	else if((data[20] == 'A') &&	//后
			(data[21] == 'F') &&
			(data[22] == 'T') &&
			(data[23] == 'O') &&
			(data[24] == 'R'))
		ret = 15;
	else if((data[20] == 'L') &&	//左
			(data[21] == 'E') &&
			(data[22] == 'F') &&
			(data[23] == 'T'))
		ret = 16;
	else if((data[20] == 'R') &&	//右
			(data[21] == 'I') &&
			(data[22] == 'G') &&
			(data[23] == 'H') &&
			(data[24] == 'T'))
		ret = 17;
	else if((data[20] == 'F') &&	//前左
			(data[21] == 'L') &&
			(data[22] == 'E') &&
			(data[23] == 'F') &&
			(data[24] == 'T'))
		ret = 18;
	else if((data[20] == 'F') &&	//前左边
			(data[21] == 'L') &&
			(data[22] == 'S') &&
			(data[23] == 'I') &&
			(data[24] == 'D'))
		ret = 19;
	else if((data[20] == 'F') &&	//前左中
			(data[21] == 'L') &&
			(data[22] == 'M') &&
			(data[23] == 'I') &&
			(data[24] == 'D'))
		ret = 20;
	else if((data[20] == 'F') &&	//前右
			(data[21] == 'R') &&
			(data[22] == 'I') &&
			(data[23] == 'G') &&
			(data[24] == 'H'))
		ret = 21;
	else if((data[20] == 'F') &&	//前左中
			(data[21] == 'R') &&
			(data[22] == 'M') &&
			(data[23] == 'I') &&
			(data[24] == 'D'))
		ret = 22;
	else if((data[20] == 'F') &&	//前左中
			(data[21] == 'R') &&
			(data[22] == 'M') &&
			(data[23] == 'I') &&
			(data[24] == 'D'))
		ret = 23;
	else if((data[20] == 'U') &&	//前左中
			(data[21] == 'R') &&
			(data[22] == 'S') &&
			(data[23] == 'E') &&
			(data[24] == 'T'))
		ret = 24;

	else if((data[20] == 'U') &&	//雷达第一维系数
			(data[21] == 'S') &&
			(data[22] == 'M') &&
			(data[23] == 'S') &&
			(data[24] == 'S'))
		ret = 25;
	else if((data[20] == 'U') &&	//雷达第二维系数
			(data[21] == 'S') &&
			(data[22] == 'M') &&
			(data[23] == 'S') &&
			(data[24] == 'S'))
		ret = 26;
	else if((data[20] == 'U') &&	//0车速关联参数
			(data[21] == 'S') &&
			(data[22] == '0') &&
			(data[23] == 'S') &&
			(data[24] == 'E'))
		ret = 27;
	else if((data[20] == 'U') &&	//5车速的关联参数
			(data[21] == 'S') &&
			(data[22] == '5') &&
			(data[23] == 'S') &&
			(data[24] == 'E'))
		ret = 28;
	else if((data[20] == 'U') &&	//10车速关联参数
			(data[21] == 'S') &&
			(data[22] == '1') &&
			(data[23] == '0') &&
			(data[24] == 'S'))
		ret = 29;
	else if((data[20] == 'U') &&	//15车速关联参数
			(data[21] == 'S') &&
			(data[22] == '1') &&
			(data[23] == '5') &&
			(data[24] == 'S'))
		ret = 30;
	else if((data[20] == 'U') &&	//20车速关联参数
			(data[21] == 'S') &&
			(data[22] == '2') &&
			(data[23] == '0') &&
			(data[24] == 'S'))
		ret = 31;

	else
		ret = 0;
	return ret;
}

uint32_t Bluetooth_Set_Vehicle_With(uint8_t *data,uint32_t flag1,uint32_t flag2)
{
	uint32_t ret = 0;
	uint32_t start = 0;
	uint32_t end = 0;
	uint32_t i = 0;

	for(i = 0;i < 1024;i ++)
	{
		if(data[i] == '*')
		{
			ret ++;
			if(ret == flag1)
				start = i - 1;
			if(ret == flag2)
			{
				end = i - 1;
				break;
			}
		}
	}
	if((end - start) == 2)
		ret = data[end] - 0x30;
	else if((end - start) == 3)
		ret = (data[end] - 0x30) +  (data[end - 1] - 0x30) * 10;
	else if((end - start) == 4)
		ret = (data[end] - 0x30) +  (data[end - 1] - 0x30) * 10  +  (data[end - 2] - 0x30) * 100;
	else
		ret = 0;

	return ret;
}


uint32_t Bluetooth_Set_Speed_SCALE_FACTOR(uint8_t *data,uint32_t flag1,uint32_t flag2)
{
	uint32_t ret = 0;
	uint32_t start = 0;
	uint32_t end = 0;
	uint32_t i = 0;

	for(i = 0;i < 1024;i ++)
	{
		if(data[i] == '*')
		{
			ret ++;
			if(ret == flag1)
				start = i - 1;
			if(ret == flag2)
			{
				end = i - 1;
				break;
			}
		}
	}
	if((end - start) == 2)
		ret = data[end] - 0x30;
	else if((end - start) == 3)
		ret = (data[end] - 0x30) +  (data[end - 1] - 0x30) * 10;
	else if((end - start) == 4)
		ret = (data[end] - 0x30) +  (data[end - 1] - 0x30) * 10  +  (data[end - 2] - 0x30) * 100;
	else
		ret = 0;

	return ret;
}


uint32_t Bluetooth_Set_Vehicle_Sensitivity(uint8_t *data,uint32_t flag1,uint32_t flag2)
{
	uint32_t ret = 0;
	uint32_t start = 0;
	uint32_t end = 0;
	uint32_t i = 0;

	for(i = 0;i < 1024;i ++)
	{
		if(data[i] == '*')
		{
			ret ++;
			if(ret == flag1)
				start = i - 1;
			if(ret == flag2)
			{
				end = i - 1;
				break;
			}
		}
	}
	if((end - start) == 2)
	{
		if((data[end] > 0x29) && (data[end] < 0x3A))
			ret = data[end] - 0x30;
		else if( data[end] == 0x40 )
			ret = 10;
		else if( data[end] == 0x41 )
			ret = 11;
		else if( data[end] == 0x42 )
			ret = 12;
		else if( data[end] == 0x43 )
			ret = 13;
		else if( data[end] == 0x44 )
			ret = 14;
		else if( data[end] == 0x45 )
			ret = 15;
		else
			ret = 0;
	}
	else
		ret = 0;

	return ret;
}

uint32_t ret = 0;
uint32_t start = 0;
uint32_t end = 0;
uint32_t i = 0;
float ret_num = 0;
uint8_t src[256];

float Bluetooth_Set_TTC(uint8_t *data,uint32_t flag1,uint32_t flag2)
{
	//for(i = 0;i )
	for(i = 0;i < 256;i ++)
	{
		if(data[i] == '*')
		{
			ret ++;
			if(ret == flag1)
			{
				start = i - 1;
			}
		}
	}
	ret_num = (data[start] - 0x30) +  (float)(data[start + 2] - 0x30) * 0.1;

	return ret;
	/*
	uint32_t ret = 0;
	uint32_t start = 0;
	uint32_t end = 0;
	uint32_t i = 0;

	for(i = 0;i < 1024;i ++)
	{
		if(data[i] == '*')
		{
			ret ++;
			if(ret == flag1)
				start = i - 1;
			if(ret == flag2)
			{
				end = i - 1;
				break;
			}
		}
	}
	if((end - start) == 2)
		ret = (data[start + 2] - 0x30)*10;
	else if((end - start) == 4)
		ret = (data[start + 2] - 0x30)*10 +  (data[start + 4] - 0x30);
	else
		ret = 0;

	return ret;
	*/
}

uint32_t Bluetooth_Set_Keep_Dis(uint8_t *data,uint32_t flag1)
{
	uint32_t ret = 0;
	uint32_t start = 0;
	uint32_t end = 0;
	uint32_t i = 0;

	for(i = 0;i < 1024;i ++)
	{
		if(data[i] == '*')
		{
			ret ++;
			if(ret == flag1)
			{
				start = i;
				break;
			}
		}
	}
	ret = data[start + 1] - 0x30;

	return ret;
}

uint32_t Char_To_Int(uint8_t data)
{
	uint32_t ret;

	switch(data)
	{
	case '0':
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':
		ret = data - 0x30;
		break;
	case 'a':
	case 'A':
		ret = 10;
		break;
	case 'b':
	case 'B':
		ret = 11;
		break;
	case 'c':
	case 'C':
		ret = 12;
		break;
	case 'd':
	case 'D':
		ret = 13;
		break;
	case 'e':
	case 'E':
		ret = 14;
		break;
	case 'f':
	case 'F':
		ret = 15;
		break;
	default:
		break;
	}
	return ret;
}

uint32_t Bluetooth_Set_Speed_ID(uint8_t *data)
{
	uint32_t num[8];
	uint32_t ret = 0;
	num[0] = Char_To_Int(data[28]);
	num[1] = Char_To_Int(data[29]);
	num[2] = Char_To_Int(data[30]);
	num[3] = Char_To_Int(data[31]);
	num[4] = Char_To_Int(data[32]);
	num[5] = Char_To_Int(data[33]);
	num[6] = Char_To_Int(data[34]);
	num[7] = Char_To_Int(data[35]);
	ret = ( num[0] << 28 ) |
			( num[1] << 24 ) |
			( num[2] << 20 ) |
			( num[3] << 16 ) |
			( num[4] << 12 ) |
			( num[5] << 8 ) |
			( num[6] << 4 ) |
			( num[7]);
	return ret;
}

/*
 * 函数名称：Bluetooth_Set_Break_ID
 * 函数功能：计算刹车 id信息
 * 函数输入：uint8_t *data	串口接收数据
 * 函数输出：刹车ID信息
 * */
uint32_t Bluetooth_Set_Break_ID(uint8_t *data)
{
	uint32_t num[8];
	uint32_t ret = 0;
	num[0] = Char_To_Int(data[26]);
	num[1] = Char_To_Int(data[27]);
	num[2] = Char_To_Int(data[28]);
	num[3] = Char_To_Int(data[29]);
	num[4] = Char_To_Int(data[30]);
	num[5] = Char_To_Int(data[31]);
	num[6] = Char_To_Int(data[32]);
	num[7] = Char_To_Int(data[33]);
	ret = ( num[0] << 28 ) |
			( num[1] << 24 ) |
			( num[2] << 20 ) |
			( num[3] << 16 ) |
			( num[4] << 12 ) |
			( num[5] << 8 ) |
			( num[6] << 4 ) |
			( num[7]);
	return ret;
}

/*
 * 函数名称：Bluetooth_Set_Abs_ID
 * 函数功能：计算ABS id信息
 * 函数输入：uint8_t *data	串口接收数据
 * 函数输出：转向ID信息
 * */
uint32_t Bluetooth_Set_Abs_ID(uint8_t *data)
{
	uint32_t num[8];
	uint32_t ret = 0;
	num[0] = Char_To_Int(data[24]);
	num[1] = Char_To_Int(data[25]);
	num[2] = Char_To_Int(data[26]);
	num[3] = Char_To_Int(data[27]);
	num[4] = Char_To_Int(data[28]);
	num[5] = Char_To_Int(data[29]);
	num[6] = Char_To_Int(data[30]);
	num[7] = Char_To_Int(data[31]);
	ret = ( num[0] << 28 ) |
			( num[1] << 24 ) |
			( num[2] << 20 ) |
			( num[3] << 16 ) |
			( num[4] << 12 ) |
			( num[5] << 8 ) |
			( num[6] << 4 ) |
			( num[7]);
	return ret;
}

/*
 * 函数名称：Bluetooth_Set_Turn_ID
 * 函数功能：计算转向ID信息
 * 函数输入：uint8_t *data	串口接收数据
 * 函数输出：转向ID信息
 * */
uint32_t Bluetooth_Set_Turn_ID(uint8_t *data)
{
	uint32_t num[8];
	uint32_t ret = 0;
	num[0] = Char_To_Int(data[25]);
	num[1] = Char_To_Int(data[26]);
	num[2] = Char_To_Int(data[27]);
	num[3] = Char_To_Int(data[28]);
	num[4] = Char_To_Int(data[29]);
	num[5] = Char_To_Int(data[30]);
	num[6] = Char_To_Int(data[31]);
	num[7] = Char_To_Int(data[32]);
	ret = ( num[0] << 28 ) |
			( num[1] << 24 ) |
			( num[2] << 20 ) |
			( num[3] << 16 ) |
			( num[4] << 12 ) |
			( num[5] << 8 ) |
			( num[6] << 4 ) |
			( num[7]);
	return ret;
}

uint16_t Get_Buff_Lengh(uint8_t *data)
{

	uint16_t ret = 0;

	for(ret = 0;;ret ++)
	{
		if(data[ret] == 0)
			break;
	}
	fprintf(USART1_STREAM,"ret = %d\n",ret);
	Delay(1000,1000);
	return ret;
}

void Copy_Buff(uint8_t *src,uint8_t *dst,uint16_t num)
{
	uint16_t ret;

	for(ret = 0;ret < num;ret ++)
		src[ret] = dst[ret];
}

void Left_Buff(uint8_t *src,uint8_t *dst,uint8_t flag)
{
	uint16_t ret = 0;

	for(ret = 0;;ret ++)
	{
		if(src[ret] == flag)
			break;
		dst[ret] = src[ret];
	}
}

uint8_t Buff_To_Init(uint8_t *src,uint16_t len)
{
	uint8_t ret = 0;

	if(len == 1)
		ret = src[0];
	if(len == 2)
		ret = src[0] + src[1] * 10;
	if(len == 3)
		ret = src[0] + src[1] * 10 + src[2] * 100;

	return ret;
}
/*
void Vechicle_Parameter_Set(uint8_t *data)
{

	uint8_t Send_Buff[64] = "USTART*SETPARAMETER*VEHICLE*1*USTOP";
	uint8_t	i;
	//char *p;
	//uint16_t Major_version;
	//uint16_t Minor_version;
	//uint16_t	Revision;
	uint8_t	Camera;
	uint8_t	URader_num;
	uint16_t Str_len;
	uint16_t Str_len2;
	uint8_t 	Sn[1024];
	uint8_t		Sn_Temp[1024];
	uint8_t 	Temp_Sn[1024];
	uint8_t		p[1024];
	fprintf(USART1_STREAM,"11111111");
	Delay(1000,1000);
	for(i = 0;i < 1024;i ++)
	{
		Sn[i] = 0;
		Sn_Temp[i] = 0;
		Temp_Sn[i] = 0;
		p[i] = 0;
	}
	Str_len = Get_Buff_Lengh(data);
	fprintf(USART1_STREAM,"22222");
	Delay(1000,1000);
	Copy_Buff(Sn,data,Str_len);
	fprintf(USART1_STREAM,"333333");
	Delay(1000,1000);
	Copy_Buff(Sn_Temp,data,Str_len);
	fprintf(USART1_STREAM,"444444");
	Delay(1000,1000);
	//p = strtok(Sn,"*");
	Left_Buff(Sn,p,'*');
	Str_len2 = Get_Buff_Lengh(p);
	Right(Temp_Sn,Sn_Temp,Str_len - Str_len2 - 1);		//USTART
	for(i = 0;i < 1024;i ++)
	{
		Sn[i] = '\0';
		Sn_Temp[i] = '\0';
	}

	Str_len = Get_Buff_Lengh(Temp_Sn);
	Copy_Buff(Sn,Temp_Sn,Str_len);
	Copy_Buff(Sn_Temp,Temp_Sn,Str_len);
	//p = strtok(Sn,"*");
	Left_Buff(Sn,p,'*');
	Str_len2 = Get_Buff_Lengh(p);
	for(i = 0;i < 1024;i ++)
	{
		Temp_Sn[i] = '\0';
	}
	Right(Temp_Sn,Sn_Temp,Str_len - Str_len2 - 1);		//SETPARAMETER

	for(i = 0;i < 1024;i ++)
	{
		Sn[i] = '\0';
		Sn_Temp[i] = '\0';
	}
	Str_len = Get_Buff_Lengh(Temp_Sn);
	Copy_Buff(Sn,Temp_Sn,Str_len);
	Copy_Buff(Sn_Temp,Temp_Sn,Str_len);
	//p = strtok(Sn,"*");
	Left_Buff(Sn,p,'*');
	Str_len2 = Get_Buff_Lengh(p);
	Right(Temp_Sn,Sn_Temp,Str_len - Str_len2 - 1);
	for(i = 0;i < 1024;i ++)
	{
		Sn[i] = '\0';
		Sn_Temp[i] = '\0';
		p[i] = '\0';
	}
	Str_len = Get_Buff_Lengh(Temp_Sn);
	Copy_Buff(Sn,Temp_Sn,Str_len);
	Copy_Buff(Sn_Temp,Temp_Sn,Str_len);
	//p = strtok(Sn,"*");
	Left_Buff(Sn,p,'*');
	Str_len2 = Get_Buff_Lengh(p);
	stSysPara.ucVehicleWidth = Buff_To_Init(p,Str_len2);
	//stSysPara.ucVehicleWidth = atoi(p);
	Right(Temp_Sn,Sn_Temp,Str_len - Str_len2 - 1);
	for(i = 0;i < 1024;i ++)
	{
		Sn[i] = '\0';
		Sn_Temp[i] = '\0';
		p[i] = '\0';
	}
	Str_len = Get_Buff_Lengh(Temp_Sn);
	Copy_Buff(Sn,Temp_Sn,Str_len);
	Copy_Buff(Sn_Temp,Temp_Sn,Str_len);
	//p = strtok(Sn,"*");
	Left_Buff(Sn,p,'*');
	Str_len2 = Get_Buff_Lengh(p);
	stSysPara.ucDistaceKeep = Buff_To_Init(p,Str_len2);
	//stSysPara.ucDistaceKeep = atoi(p);
	Right(Temp_Sn,Sn_Temp,Str_len - Str_len2 - 1);
	for(i = 0;i < 1024;i ++)
	{
		Sn[i] = '\0';
		Sn_Temp[i] = '\0';
		p[i] = '\0';
	}

	Str_len = Get_Buff_Lengh(Temp_Sn);
	Copy_Buff(Sn,Temp_Sn,Str_len);
	Copy_Buff(Sn_Temp,Temp_Sn,Str_len);
	//p = strtok(Sn,"*");
	Left_Buff(Sn,p,'*');
	Str_len2 = Get_Buff_Lengh(p);
	//stSysPara.ttc1 = atof(p);
	Right(Temp_Sn,Sn_Temp,Str_len - Str_len2 - 1);
	for(i = 0;i < 1024;i ++)
	{
		Sn[i] = '\0';
		Sn_Temp[i] = '\0';
	}
	Str_len = Get_Buff_Lengh(Temp_Sn);
	Copy_Buff(Sn,Temp_Sn,Str_len);
	Copy_Buff(Sn_Temp,Temp_Sn,Str_len);
	//p = strtok(Sn,"*");
	Left_Buff(Sn,p,'*');
	Str_len2 = Get_Buff_Lengh(p);
	//stSysPara.ttc2 = atof(p);
	Right(Temp_Sn,Sn_Temp,Str_len - Str_len2 - 1);
	for(i = 0;i < 1024;i ++)
	{
		Sn[i] = '\0';
		Sn_Temp[i] = '\0';
	}
	Str_len = Get_Buff_Lengh(Temp_Sn);
	Copy_Buff(Sn,Temp_Sn,Str_len);
	Copy_Buff(Sn_Temp,Temp_Sn,Str_len);
	//p = strtok(Sn,"*");
	Left_Buff(Sn,p,'*');
	Str_len2 = Get_Buff_Lengh(p);
	//stSysPara.ucBrakeSense = atoi(p);
	Right(Temp_Sn,Sn_Temp,Str_len - Str_len2 - 1);
	for(i = 0;i < 1024;i ++)
	{
		Sn[i] = '\0';
		Sn_Temp[i] = '\0';
	}
	Str_len = Get_Buff_Lengh(Temp_Sn);
	Copy_Buff(Sn,Temp_Sn,Str_len);
	Copy_Buff(Sn_Temp,Temp_Sn,Str_len);
	//p = strtok(Sn,"*");
	Left_Buff(Sn,p,'*');
	Str_len2 = Get_Buff_Lengh(p);
	//stSysPara.ucControlMode = atoi(p);
	Right(Temp_Sn,Sn_Temp,Str_len - Str_len2 - 1);

	for(i = 0;i < 1024;i ++)
	{
		Sn[i] = '\0';
		Sn_Temp[i] = '\0';
	}
	Str_len = Get_Buff_Lengh(Temp_Sn);
	Copy_Buff(Sn,Temp_Sn,Str_len);
	Copy_Buff(Sn_Temp,Temp_Sn,Str_len);
	//p = strtok(Sn,"*");
	Left_Buff(Sn,p,'*');
	Str_len2 = Get_Buff_Lengh(p);
	//stSysPara.hmw1 = atof(p);
	Right(Temp_Sn,Sn_Temp,Str_len - Str_len2 - 1);

	for(i = 0;i < 1024;i ++)
	{
		Sn[i] = '\0';
		Sn_Temp[i] = '\0';
	}
	Str_len = Get_Buff_Lengh(Temp_Sn);
	Copy_Buff(Sn,Temp_Sn,Str_len);
	Copy_Buff(Sn_Temp,Temp_Sn,Str_len);
	//p = strtok(Sn,"*");
	Left_Buff(Sn,p,'*');
	Str_len2 = Get_Buff_Lengh(p);
	//stSysPara.hmw2 = atof(p);
	Right(Temp_Sn,Sn_Temp,Str_len - Str_len2 - 1);

	for(i = 0;i < 1024;i ++)
	{
		Sn[i] = '\0';
		Sn_Temp[i] = '\0';
	}
	Str_len = Get_Buff_Lengh(Temp_Sn);
	Copy_Buff(Sn,Temp_Sn,Str_len);
	Copy_Buff(Sn_Temp,Temp_Sn,Str_len);
	//p = strtok(Sn,"*");
	Left_Buff(Sn,p,'*');
	Str_len2 = Get_Buff_Lengh(p);
	//stSysPara.GearNum = atoi(p);
	Right(Temp_Sn,Sn_Temp,Str_len - Str_len2 - 1);
	//Revision = atoi(SN);
*/
void Vechicle_Parameter_Set(uint8_t *data)
{
	uint32_t vechicle;
//	strcpy(Vehicle_Para_Buf,data);

	vechicle = Bluetooth_Set_Vehicle_With(data,3,4);		//车辆宽度
	stSysPara.ucVehicleWidth = (uint8_t)vechicle;
	vechicle = Bluetooth_Set_Vehicle_With(data,4,5);//Bluetooth_Set_Keep_Dis(data,4);			//车距保持
	stSysPara.ucDistaceKeep = (uint8_t) vechicle;

	stSysPara.ttc1 = (float)(data[33] - 0x30) + (float)(data[35] - 0x30) * 0.1;//Bluetooth_Set_TTC(data,5,6);
	//stSysPara.ttc1 = (float)(vechicle / 10) + (float)(vechicle % 10);
	stSysPara.ttc2 = (float)(data[37] - 0x30) + (float)(data[39] -0x30) * 0.1;//Bluetooth_Set_TTC(data,6,7);				//第二刹车阈值
	//stSysPara.ttc2 = (float)(vechicle / 10) + (float)(vechicle % 10);

	vechicle = Bluetooth_Set_Vehicle_With(data,7,8);
	stSysPara.ucBrakeSense = (uint8_t)vechicle;
	vechicle = Bluetooth_Set_Vehicle_With(data,8,9);
	stSysPara.ucControlMode = (uint8_t)vechicle;

	stSysPara.hmw1 = (float)(data[45] - 0x30) + (float)(data[47] - 0x30) * 0.1;//Bluetooth_Set_TTC(data,9,10);
			//stSysPara.hmw1 = (float)(vechicle / 10) + (float)(vechicle % 10);
	stSysPara.hmw2 = (float)(data[49] - 0x30) + (float)(data[51] - 0x30) * 0.1;//Bluetooth_Set_TTC(data,10,11);
			//stSysPara.hmw2 = (float)(vechicle / 10) + (float)(vechicle % 10);

	vechicle = Bluetooth_Set_Vehicle_With(data,11,12);
	stSysPara.GearNum = (uint8_t)vechicle;

//	stVehicleParas.WriteFlashFlag = 1;

	ReWrite_one();
	//fprintf(BLUETOOTH_STREAM,"USTART*SETPARAMETER*VEHICLE*1*USTOP");
	USART_Send(BLUETOOTH_STREAM,"USTART*SETPARAMETER*VEHICLE*1*USTOP",35);
}

void Break_Parameter_Set(uint8_t *data)
{
	uint32_t vechicle;
	uint8_t Send_Buff[64] = "USTART*SETPARAMETER*BRAKE*1*USTOP";

	vechicle = Bluetooth_Set_Break_ID(data);			//刹车ID		八个字节固定
	stBrakePara.lId = vechicle;
	vechicle = Char_To_Int(data[35]);
	stBrakePara.Source = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[37]);				//刹车开始字节
	stBrakePara.Type = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[39]);
	stBrakePara.ucStartBit = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[41]);
	stBrakePara.ucBitLth = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[43]);				//车速开始字节
	stBrakePara.ucByteNo = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[45]);
	stBrakePara.ucValid = (uint8_t)vechicle;
	vechicle = Bluetooth_Set_Speed_SCALE_FACTOR(data,10,11);				//车速开始字节
	stBrakePara.ucCoeffA = (uint8_t)vechicle;
	vechicle = Bluetooth_Set_Speed_SCALE_FACTOR(data,11,12);
	stBrakePara.ucCoeffB = (uint8_t)vechicle;
	ReWrite_four();
	//USART_Send(BLUETOOTH_STREAM,Send_Buff,strlen(Send_Buff));
	fprintf(BLUETOOTH_STREAM,"USTART*SETPARAMETER*BRAKE*1*USTOP");
}

void Turn_Parameter_Set(uint8_t *data)
{
	uint32_t vechicle;

	vechicle = Bluetooth_Set_Turn_ID(data);			//转向ID		八个字节固定
	stTurnPara.lId = vechicle;
	vechicle = Char_To_Int(data[34]);				//转向模式	can模式0，模拟1
	stTurnPara.Source = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[36]);				//转向字节
	stTurnPara.ucByteNo = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[38]);					//左转向开始bit
	stTurnPara.ucLeftStartBit = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[40]);					//左转向bit个数
	stTurnPara.ucLeftBitLth = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[42]);					//左转向有效值
	stTurnPara.ucLeftValid = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[44]);					//右转向开始bit
	stTurnPara.ucRightStartBit = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[46]);					//右转向bit个数
	stTurnPara.ucRightBitLth = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[48]);					//右转向有效值
	stTurnPara.ucRightValid = (uint8_t)vechicle;
	ReWrite_three();
	fprintf(BLUETOOTH_STREAM,"USTART*SETPARAMETER*TURN*1*USTOP");
}

void Abs_Parameter_Set(uint8_t *data)
{
	uint32_t vechicle;

	vechicle = Bluetooth_Set_Abs_ID(data);			//ABS ID		八个字节固定
	stAbsPara.lId = vechicle;
	vechicle = Char_To_Int(data[33]);
	stAbsPara.Source = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[35]);				//ABS开始字节
	stAbsPara.ucAbsWarningStartBit = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[37]);
	stAbsPara.ucAbsWarningBitLth = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[39]);				//ABS开始字节
	stAbsPara.ucAbsWarningValid = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[41]);
	stAbsPara.ucByteNo = (uint8_t)vechicle;
	ReWrite_two();
	fprintf(BLUETOOTH_STREAM,"USTART*SETPARAMETER*ABS*1*USTOP");
}

void Speed_Parameter_Set(uint8_t *data)
{
	uint32_t vechicle;

	vechicle = Bluetooth_Set_Vehicle_With(data,3,4);		//车速类型
	stSpeedPara.uSingnalType = (uint8_t)vechicle;
	vechicle = Bluetooth_Set_Speed_ID(data);			//车速ID		八个字节固定
	stSpeedPara.lId = vechicle;
	fprintf(BLUETOOTH_STREAM,"2222stSpeedPara.lId");
	vechicle = Char_To_Int(data[37]);
	stSpeedPara.ucByteOrder = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[39]);				//车速开始字节
	stSpeedPara.ucStartByte = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[41]);
	stSpeedPara.ucByteLth = (uint8_t)vechicle;
	vechicle = Bluetooth_Set_Speed_SCALE_FACTOR(data,8,9);				//车速开始字节
	stSpeedPara.ucCoeffA = (uint8_t)vechicle;
	vechicle = Bluetooth_Set_Speed_SCALE_FACTOR(data,9,10);
	stSpeedPara.ucCoeffB = (uint8_t)vechicle;
	ReWrite_two();
	fprintf(BLUETOOTH_STREAM,"USTART*SETPARAMETER*SPEED*1*USTOP");
}

void WlinkSpeed_Parameter_Set(uint8_t *data)
{
	uint32_t vechicle;

	vechicle = Bluetooth_Set_Vehicle_With(data,3,4);		//车速类型
	stWarninglinkage.FCWlinkage = (uint8_t)vechicle;
	vechicle = Bluetooth_Set_Vehicle_With(data,4,5);			//车速ID		八个字节固定
	stWarninglinkage.FCWlinkageSpeed = vechicle;

	vechicle = Bluetooth_Set_Vehicle_With(data,5,6);
	stWarninglinkage.LDWlinkage = (uint8_t)vechicle;
	vechicle = Bluetooth_Set_Vehicle_With(data,6,7);				//车速开始字节
	stWarninglinkage.LDWlinkageSpeed = (uint8_t)vechicle;

	vechicle = Bluetooth_Set_Vehicle_With(data,7,8);
	stWarninglinkage.AEBlinkage = (uint8_t)vechicle;
	vechicle = Bluetooth_Set_Vehicle_With(data,8,9);				//车速开始字节
	stWarninglinkage.AEBlinkageSpeed = (uint8_t)vechicle;

	vechicle = Bluetooth_Set_Vehicle_With(data,9,10);
	stWarninglinkage.HMWlinkage = (uint8_t)vechicle;
	vechicle = Bluetooth_Set_Vehicle_With(data,10,11);				//车速开始字节
	stWarninglinkage.HMWlinkageSpeed = (uint8_t)vechicle;
	ReWrite_five();
	fprintf(BLUETOOTH_STREAM,"USTART*SETPARAMETER*SPEED*1*USTOP");
}

void CAN_Rata_Set(uint8_t *data)
{
	stCanPara.can0rate = data[28] - 0x30;
	stCanPara.can1rate = data[30] - 0x30;
	stCanPara.can2rate = data[32] - 0x30;
	stCanPara.can3rate = data[34] - 0x30;
	stCanPara.can4rate = data[36] - 0x30;
	stCanPara.can5rate = data[38] - 0x30;
	ReWrite_one();
	fprintf(BLUETOOTH_STREAM,"USTART*SETPARAMETER*CANRATE*1*USTOP");
}

void CAN0_Rata_Set(uint8_t *data)
{
	uint8_t sencond_ret;

	sencond_ret = Bluetooth_Set_Canrata_Commond(data);
	stCanPara.can0rate = sencond_ret;
	ReWrite_one();
	data[29] = '1';
	fprintf(BLUETOOTH_STREAM,"%s",data);
}

void CAN1_Rata_Set(uint8_t *data)
{
	uint8_t sencond_ret;

	sencond_ret = Bluetooth_Set_Canrata_Commond(data);
	stCanPara.can1rate = sencond_ret;
	ReWrite_one();
	data[29] = '1';
	fprintf(BLUETOOTH_STREAM,"%s",data);
}

void CAN2_Rata_Set(uint8_t *data)
{
	uint8_t sencond_ret;

	sencond_ret = Bluetooth_Set_Canrata_Commond(data);
	stCanPara.can2rate = sencond_ret;
	ReWrite_one();
	data[29] = '1';
	fprintf(BLUETOOTH_STREAM,"%s",data);
}

void CAN3_Rata_Set(uint8_t *data)
{
	uint8_t sencond_ret;

	sencond_ret = Bluetooth_Set_Canrata_Commond(data);
	stCanPara.can3rate = sencond_ret;
	ReWrite_one();
	data[29] = '1';
	fprintf(BLUETOOTH_STREAM,"%s",data);

}

void CAN4_Rata_Set(uint8_t *data)
{
	uint8_t sencond_ret;

	sencond_ret = Bluetooth_Set_Canrata_Commond(data);
	stCanPara.can4rate = sencond_ret;
	ReWrite_one();
	data[29] = '1';
	fprintf(BLUETOOTH_STREAM,"%s",data);
}

void CAN5_Rata_Set(uint8_t *data)
{
	uint8_t sencond_ret;

	sencond_ret = Bluetooth_Set_Canrata_Commond(data);
	stCanPara.can5rate = sencond_ret;
	ReWrite_one();
	data[29] = '1';
	fprintf(BLUETOOTH_STREAM,"%s",data);
}

void SWSteering_Parameter_Set(uint8_t *data)
{
	uint32_t vechicle;

	vechicle = Bluetooth_Set_Abs_ID(data);			//转向 ID		八个字节固定
	stSwaPara.lId = vechicle;
	vechicle = Char_To_Int(data[33]);
	stSwaPara.ucByteNo = (uint8_t)vechicle;
	vechicle = Char_To_Int(data[35]);
	stSwaPara.ucStartBit = (uint8_t)vechicle;
	vechicle = Bluetooth_Set_Vehicle_With(data,6,7);
	stSwaPara.ucBitLth = (uint8_t)vechicle;
	ReWrite_five();
	fprintf(BLUETOOTH_STREAM,"USTART*SETPARAMETER*SWA*1*USTOP");

}


void URader_Position_Flash_Preser(uint8_t a,uint8_t b,uint8_t c)
{
	if(a == 1)
	{
		if(b == 1)
			FLASH_WriteWord(URADER_HOST_ONE_ONE_ADDRESS,(uint32_t)c);
		else if(b == 2)
			FLASH_WriteWord(URADER_HOST_ONE_TWO_ADDRESS,(uint32_t)c);
		else if(b == 3)
			FLASH_WriteWord(URADER_HOST_ONE_THREE_ADDRESS,(uint32_t)c);
		else if(b == 4)
			FLASH_WriteWord(URADER_HOST_ONE_FOUR_ADDRESS,(uint32_t)c);
		else if(b == 5)
			FLASH_WriteWord(URADER_HOST_ONE_FIVE_ADDRESS,(uint32_t)c);
		else if(b == 6)
			FLASH_WriteWord(URADER_HOST_ONE_SIX_ADDRESS,(uint32_t)c);
		else if(b == 7)
			FLASH_WriteWord(URADER_HOST_ONE_SEVEN_ADDRESS,(uint32_t)c);
		else if(b == 8)
			FLASH_WriteWord(URADER_HOST_ONE_EIGHT_ADDRESS,(uint32_t)c);
		else if(b == 9)
			FLASH_WriteWord(URADER_HOST_ONE_NINE_ADDRESS,(uint32_t)c);
		else if(b == 10)
			FLASH_WriteWord(URADER_HOST_ONE_TEN_ADDRESS,(uint32_t)c);
		else if(b == 11)
			FLASH_WriteWord(URADER_HOST_ONE_ELEVEN_ADDRESS,(uint32_t)c);
		else if(b == 12)
			FLASH_WriteWord(URADER_HOST_ONE_TWELVE_ADDRESS,(uint32_t)c);
	}
	else if(a == 2)
	{
		if(b == 1)
			FLASH_WriteWord(URADER_HOST_TWO_ONE_ADDRESS,(uint32_t)c);
		else if(b == 2)
			FLASH_WriteWord(URADER_HOST_TWO_TWO_ADDRESS,(uint32_t)c);
		else if(b == 3)
			FLASH_WriteWord(URADER_HOST_TWO_THREE_ADDRESS,(uint32_t)c);
		else if(b == 4)
			FLASH_WriteWord(URADER_HOST_TWO_FOUR_ADDRESS,(uint32_t)c);
		else if(b == 5)
			FLASH_WriteWord(URADER_HOST_TWO_FIVE_ADDRESS,(uint32_t)c);
		else if(b == 6)
			FLASH_WriteWord(URADER_HOST_TWO_SIX_ADDRESS,(uint32_t)c);
		else if(b == 7)
			FLASH_WriteWord(URADER_HOST_TWO_SEVEN_ADDRESS,(uint32_t)c);
		else if(b == 8)
			FLASH_WriteWord(URADER_HOST_TWO_EIGHT_ADDRESS,(uint32_t)c);
		else if(b == 9)
			FLASH_WriteWord(URADER_HOST_TWO_NINE_ADDRESS,(uint32_t)c);
		else if(b == 10)
			FLASH_WriteWord(URADER_HOST_TWO_TEN_ADDRESS,(uint32_t)c);
		else if(b == 11)
			FLASH_WriteWord(URADER_HOST_TWO_ELEVEN_ADDRESS,(uint32_t)c);
		else if(b == 12)
			FLASH_WriteWord(URADER_HOST_TWO_TWELVE_ADDRESS,(uint32_t)c);
	}
	else
		;
}

void URader_Position_Restore_Settings(uint8_t *data)
{
	uint8_t i,j;

	for(i = 0;i < 2;i ++)
	{
		for(j = 0;j < 12;j ++)
		{
			Urader_Company_Message[i].Uposition[j] = NONEP;
			URader_Position_Flash_Preser(i,j,NONEP);
		}
	}
}


void URader_Position_Front(uint8_t *data)
{
	uint32_t i,j;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	j = Bluetooth_Set_Vehicle_With(data,4,5);
	Urader_Company_Message[i].Uposition[j] = FRONT;
	if(i == 1)
	{
		ReWrite_six();
	}
	if(i == 2)
	{
		ReWrite_seven();
	}

	//URader_Position_Flash_Preser(i,j,FRONT);
}

void URader_Position_Aftor(uint8_t *data)
{
	uint32_t i,j;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	j = Bluetooth_Set_Vehicle_With(data,4,5);
	Urader_Company_Message[i].Uposition[j] = AFTER;
	if(i == 1)
	{
		ReWrite_six();
	}
	if(i == 2)
	{
		ReWrite_seven();
	}
	//URader_Position_Flash_Preser(i,j,AFTER);
}

void URader_Position_Left(uint8_t *data)
{
	uint32_t i,j;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	j = Bluetooth_Set_Vehicle_With(data,4,5);
	Urader_Company_Message[i].Uposition[j] = LEFT;
	if(i == 1)
	{
		ReWrite_six();
	}
	if(i == 2)
	{
		ReWrite_seven();
	}

	//URader_Position_Flash_Preser(i,j,LEFT);
}

void URader_Position_Right(uint8_t *data)
{
	uint32_t i,j;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	j = Bluetooth_Set_Vehicle_With(data,4,5);
	Urader_Company_Message[i].Uposition[j] = RIGHT;
	if(i == 1)
	{
		ReWrite_six();
	}
	if(i == 2)
	{
		ReWrite_seven();
	}

	//URader_Position_Flash_Preser(i,j,RIGHT);
}

void URader_Position_FLeft(uint8_t *data)
{
	uint32_t i,j;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	j = Bluetooth_Set_Vehicle_With(data,4,5);
	Urader_Company_Message[i].Uposition[j] = FLEFT;
	if(i == 1)
	{
		ReWrite_six();
	}
	if(i == 2)
	{
		ReWrite_seven();
	}

	//URader_Position_Flash_Preser(i,j,FLEFT);
}

void URader_Position_FLSide(uint8_t *data)
{
	uint32_t i,j;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	j = Bluetooth_Set_Vehicle_With(data,4,5);
	Urader_Company_Message[i].Uposition[j] = FLSIDE;
	if(i == 1)
	{
		ReWrite_six();
	}
	if(i == 2)
	{
		ReWrite_seven();
	}

	//URader_Position_Flash_Preser(i,j,FLSIDE);
}

void URader_Position_FLMide(uint8_t *data)
{
	uint32_t i,j;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	j = Bluetooth_Set_Vehicle_With(data,4,5);
	Urader_Company_Message[i].Uposition[j] = FLMIDE;
	if(i == 1)
	{
		ReWrite_six();
	}
	if(i == 2)
	{
		ReWrite_seven();
	}

	//URader_Position_Flash_Preser(i,j,FLMIDE);
}

void URader_Position_FRight(uint8_t *data)
{
	uint32_t i,j;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	j = Bluetooth_Set_Vehicle_With(data,4,5);
	Urader_Company_Message[i].Uposition[j] = FRIGHT;
	if(i == 1)
	{
		ReWrite_six();
	}
	if(i == 2)
	{
		ReWrite_seven();
	}

	//URader_Position_Flash_Preser(i,j,FRIGHT);
}

void URader_Position_FRSide(uint8_t *data)
{
	uint32_t i,j;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	j = Bluetooth_Set_Vehicle_With(data,4,5);
	Urader_Company_Message[i].Uposition[j] = FRSIDE;
	if(i == 1)
	{
		ReWrite_six();
	}
	if(i == 2)
	{
		ReWrite_seven();
	}

	//URader_Position_Flash_Preser(i,j,FRSIDE);
}

void URader_Position_FRMide(uint8_t *data)
{
	uint32_t i,j;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	j = Bluetooth_Set_Vehicle_With(data,4,5);
	Urader_Company_Message[i].Uposition[j] = FRMIDE;
	if(i == 1)
	{
		ReWrite_six();
	}
	if(i == 2)
	{
		ReWrite_seven();
	}

	//URader_Position_Flash_Preser(i,j,FRMIDE);
}

void URader_Sys_Message_First_Dimensional_Coefficient(uint8_t *data)
{
	uint32_t i;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	stUraderSysMessage.firststage = i;
	i = Bluetooth_Set_Vehicle_With(data,4,5);
	stUraderSysMessage.secondstage = i;
	i = Bluetooth_Set_Vehicle_With(data,5,6);
	stUraderSysMessage.speed0distance = i;
	i = Bluetooth_Set_Vehicle_With(data,6,7);
	stUraderSysMessage.speed5distance = i;
	i = Bluetooth_Set_Vehicle_With(data,7,8);
	stUraderSysMessage.speed10distance = i;
	i = Bluetooth_Set_Vehicle_With(data,8,9);
	stUraderSysMessage.speed15distance = i;
	i = Bluetooth_Set_Vehicle_With(data,9,10);
	stUraderSysMessage.speed20distance = i;
	i = Bluetooth_Set_Vehicle_With(data,10,11);
	stUraderSysMessage.SSSlinkage = i;
	i = Bluetooth_Set_Vehicle_With(data,11,12);
	stUraderSysMessage.SSSlinkageSpeed = i;
	i = Bluetooth_Set_Vehicle_With(data,12,13);
	stUraderSysMessage.speed25distance = i;

	ReWrite_eight();

	fprintf(BLUETOOTH_STREAM,"USTART*SETPARAMETER*USYSS*1*USTOP");
}

void URader_Sys_Message_Second_Dimensional_Coefficient(uint8_t *data)
{
	uint32_t i;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	stUraderSysMessage.secondstage = (float)i * 0.1;
	FLASH_WriteWord(URADER_SYS_MESSAGE_SECOND_DIMENSIONAL_COEFFICIENT,i);
}

void URader_Sys_Message_0Speed_Threshold(uint8_t *data)
{
	uint32_t i;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	stUraderSysMessage.speed0distance = (float)i * 0.1;
	FLASH_WriteWord(URADER_SYS_MESSAGE_0SPEED_THRESHOLD,i);
}

void URader_Sys_Message_5Speed_Threshold(uint8_t *data)
{
	uint32_t i;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	stUraderSysMessage.speed5distance = (float)i * 0.1;
	FLASH_WriteWord(URADER_SYS_MESSAGE_5SPEED_THRESHOLD,i);
}

void URader_Sys_Message_10Speed_Threshold(uint8_t *data)
{
	uint32_t i;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	stUraderSysMessage.speed10distance = (float)i * 0.1;
	FLASH_WriteWord(URADER_SYS_MESSAGE_10SPEED_THRESHOLD,i);
}

void URader_Sys_Message_15Speed_Threshold(uint8_t *data)
{
	uint32_t i;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	stUraderSysMessage.speed15distance = (float)i * 0.1;
	FLASH_WriteWord(URADER_SYS_MESSAGE_15SPEED_THRESHOLD,i);
}

void URader_Sys_Message_20Speed_Threshold(uint8_t *data)
{
	uint32_t i;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	stUraderSysMessage.speed20distance = (float)i * 0.1;
	FLASH_WriteWord(URADER_SYS_MESSAGE_20SPEED_THRESHOLD,i);
}

/*
 * 函数名称：Bluetooth_Set_Commond
 * 函数功能：接收到的参数信息
 * 函数输入：uint8_t *data	串口接收数据
 * 函数输出：无
 * */
void Bluetooth_Set_Commond(uint8_t *data)
{
	uint8_t send_data_correct[256] = "USTART*SETPARAMETER*VEHICLE*1*USTOP";
	uint8_t send_data_wrong[256] = "USTART*SETPARAMETER*VEHICLE*0*USTOP";
	uint32_t ret;
	uint8_t sencond_ret;
	uint32_t sencond_ret_buf[1];
	uint32_t vechicle;

	ret = Bluetooth_Set_Commond_Annalyse(data);
	switch(ret)
	{
	case 0:
		CAN_Rata_Set(data);
		break;
	case 1:
		CAN0_Rata_Set(data);
		break;
	case 2:
		CAN1_Rata_Set(data);
		break;
	case 3:
		CAN2_Rata_Set(data);
		break;
	case 4:
		CAN3_Rata_Set(data);
		break;
	case 5:
		CAN4_Rata_Set(data);
		break;
	case 6:
		CAN5_Rata_Set(data);
		break;
	case 7:	//车辆参数
		Vechicle_Parameter_Set(data);
		break;
	case 8:	//车速参数设置
		Speed_Parameter_Set(data);
		break;
	case 9:	//Abs参数设置
		Abs_Parameter_Set(data);
		break;
	case 10:	//转向参数设置
		Turn_Parameter_Set(data);
		break;
	case 11:
		Break_Parameter_Set(data);
		break;
	case 12:
		WlinkSpeed_Parameter_Set(data);
		break;
	case 13:
		SWSteering_Parameter_Set(data);
		break;
	case 14:
		URader_Position_Front(data);
		break;
	case 15:
		URader_Position_Aftor(data);
		break;
	case 16:
		URader_Position_Left(data);
		break;
	case 17:
		URader_Position_Right(data);
		break;
	case 18:
		break;
		URader_Position_FLeft(data);
	case 19:
		URader_Position_FLSide(data);
		break;
	case 20:
		URader_Position_FLMide(data);
		break;
	case 21:
		URader_Position_FRight(data);
		break;
	case 22:
		URader_Position_FRSide(data);
		break;
	case 23:
		URader_Position_FRMide(data);
		break;
	case 24:
		URader_Position_Restore_Settings(data);
		break;

	case 25:
		URader_Sys_Message_First_Dimensional_Coefficient(data);
		break;
	case 26:
		URader_Sys_Message_Second_Dimensional_Coefficient(data);
		break;
	case 27:
		URader_Sys_Message_0Speed_Threshold(data);
		break;
	case 28:
		URader_Sys_Message_5Speed_Threshold(data);
		break;
	case 29:
		URader_Sys_Message_10Speed_Threshold(data);
		break;
	case 30:
		URader_Sys_Message_15Speed_Threshold(data);
		break;
	case 31:
		URader_Sys_Message_20Speed_Threshold(data);
		break;
	default:
		break;
	}
}

/*
 * 函数名称：Init_SetParameter
 * 函数功能：初始化参数设置
 * 函数输入：无
 * 函数输出：无
 * */
void Init_SetParameter(void)
{
	uint8_t i,j;

	stSpeedPara.lId = 0x0CFE6CEE;		//车速ID 10000000
	stSpeedPara.uSingnalType = 0;			//默认为CAN通信
	stSpeedPara.ucByteOrder = 1;			//高字节在前
	stSpeedPara.ucStartByte = 6;			//起始字节
	stSpeedPara.ucByteLth = 2;			//默认一个字节
	stSpeedPara.ucCoeffA = 1;				//正向比例系数默认设置1
	stSpeedPara.ucCoeffB = 2;				//负向比例系数默认设置1

	stTurnPara.lId	= 0x18A70017;		// 转向ID.
	stTurnPara.Source = 0;				// 信号来源. 0:CAN报文; 1:I/0输入.
	stTurnPara.ucByteNo = 4;				// 字节序号. 取值0-7.
	stTurnPara.ucLeftStartBit = 0;
	stTurnPara.ucLeftBitLth = 2;
	stTurnPara.ucLeftValid = 1;
	stTurnPara.ucRightStartBit = 2;
	stTurnPara.ucRightBitLth = 2;
	stTurnPara.ucRightValid = 1;
	stTurnPara.lId1 = 0x18A70017;
	stTurnPara.Source1 = 0;
	stTurnPara.ucByteNo1 = 4;

	stAbsPara.lId = 0x18F0010B;
	stAbsPara.Source = 0;
	stAbsPara.ucByteNo = 5;
	stAbsPara.ucAbsWarningStartBit = 4;
	stAbsPara.ucAbsWarningBitLth = 2;
	stAbsPara.ucAbsWarningValid = 1;

	stBrakePara.lId = 0x18A70017;
	stBrakePara.Source = 0;
	stBrakePara.ucByteNo = 4;
	stBrakePara.ucStartBit = 4;
	stBrakePara.ucBitLth = 2;
	stBrakePara.Type = 0;
	stBrakePara.ucValid = 1;
	stBrakePara.ucCoeffA = 1;
	stBrakePara.ucCoeffB = 1;

	stSysPara.ucControlMode = 0;
	stSysPara.ucVehicleWidth = 22;
	stSysPara.ucDistaceKeep = 5;
	stSysPara.ucBrakeSense = 5;
	stSysPara.ttc1 = 2.5;
	stSysPara.ttc2 = 1.5;
	stSysPara.GearNum = 5;
	stSysPara.hmw1 = 1.0;
	stSysPara.hmw2 = 0.5;

	stCanPara.can0rate = 2;//相机500K
	stCanPara.can1rate = 2;//毫米波雷达
	stCanPara.can2rate = 1;//整车250
	stCanPara.can3rate = 1;//控制阀250
	stCanPara.can4rate = 2;//显示屏
	stCanPara.can5rate = 2;

	stSwaPara.lId = 0x1C01F022;//方向盘转角ID
	stSwaPara.ucByteNo = 0;		//方向盘转角开始字节
	stSwaPara.ucStartBit = 0;	//方向盘转角开始bit
	stSwaPara.ucBitLth = 16;	//方向盘转角bit个数

	warning_status.AEBstatus = 0;
	warning_status.LDWstatus = 0;

	stWarninglinkage.AEBlinkage = 1;
	stWarninglinkage.AEBlinkageSpeed = 10;
	stWarninglinkage.FCWlinkage = 1;
	stWarninglinkage.FCWlinkageSpeed = 10;
	stWarninglinkage.HMWlinkage = 1;
	stWarninglinkage.HMWlinkageSpeed = 10;
	stWarninglinkage.LDWlinkage = 1;
	stWarninglinkage.LDWlinkageSpeed = 50;

	stUraderSysMessage.firststage = 8;
	stUraderSysMessage.secondstage = 5;
	stUraderSysMessage.speed0distance = 4;
	stUraderSysMessage.speed5distance = 6;
	stUraderSysMessage.speed10distance = 8;
	stUraderSysMessage.speed15distance = 10;
	stUraderSysMessage.speed20distance = 12;
	stUraderSysMessage.SSSlinkage = 0;
	stUraderSysMessage.SSSlinkageSpeed = 20;
	stUraderSysMessage.speed25distance = 14;

	for(i = 0;i < 2;i ++)
	{
		for(j = 0;j < 12;j ++)
			Urader_Company_Message[i].Uposition[j] = NONEP;
	}

	stVehicleParas.ThrottleOpening = 0xff;		//油门开度
	stVehicleParas.BrakeOpening = 0xff;			//刹车踏板开度
	stVehicleParas.Car_Gear = 0;				//挡位信息
	stSWAParas.SWADegree = 0;//0x7FF;				//方向盘转角
	stSWAParas.Direction	= 0;				//方向盘状态			0默认为左转向
	stVehicleParas.AEBbriking = 0;				//AEB制动
	stVehicleParas.Ultrasonicdistance = 0;		//超声波距离	报警时最小距离
}

void Init_Flash_Speed_Parameter(void)
{
	//速度参数
	FLASH_WriteWord(VEHICLE_SPEED_MODE,(uint32_t)stSpeedPara.data);
	FLASH_WriteWord(VEHICLE_SPEED_ID,(uint32_t)stSpeedPara.lId);

	//FLASH_WriteWord(VEHICLE_SPEED_BYTETYPE,(uint32_t)stSpeedPara.ucByteOrder);
	//FLASH_WriteWord(VEHICLE_SPEED_BYTESTART,(uint32_t)stSpeedPara.ucStartByte);
	//FLASH_WriteWord(VEHICLE_SPEED_BYTENUM,(uint32_t)stSpeedPara.ucByteLth);
	//FLASH_WriteWord(VEHICLE_SPEED_SCALE_FACTOR_A,(uint32_t)stSpeedPara.ucCoeffA);
	//FLASH_WriteWord(VEHICLE_SPEED_SCALE_FACTOR_B,(uint32_t)stSpeedPara.ucCoeffB);
}

void Init_Flash_Turn_Parameter(void)
{
	//转向参数
	FLASH_WriteWord(VEHICLE_TURN_ID,(uint32_t)stTurnPara.lId);
	FLASH_WriteWord(VEHICLE_TURN_BYPE,(uint32_t)stTurnPara.ldata);
	FLASH_WriteWord(VEHICLE_TURN_ID1,(uint32_t)stTurnPara.lId1);
	FLASH_WriteWord(VEHICLE_TURN_BYPE1,(uint32_t)stTurnPara.rdata);
}

void Init_Flash_Abs_Parameter(void)
{
	//ABS参数
	FLASH_WriteWord(VEHICLE_ABS_ID,(uint32_t)stAbsPara.lId);
	FLASH_WriteWord(VEHICLE_ABS_TYPE,(uint32_t)stAbsPara.Source);
	FLASH_WriteWord(VEHICLE_ABS_STARTBIT,(uint32_t)stAbsPara.ucAbsWarningStartBit);
	FLASH_WriteWord(VEHICLE_ABS_BITNUM,(uint32_t)stAbsPara.ucAbsWarningBitLth);
	FLASH_WriteWord(VEHICLE_ABS_WRONGNUM,(uint32_t)stAbsPara.ucAbsWarningValid);
	FLASH_WriteWord(VEHICLE_ABS_BYTENUM,(uint32_t)stAbsPara.ucByteNo);

}

void Init_Flash_Break_Parameter(void)
{
	//刹车参数
	FLASH_WriteWord(VEHICLE_BRAKE_ID,(uint32_t)stBrakePara.lId);
	FLASH_WriteWord(VEHICLE_BRAKE_BYPE,(uint32_t)stBrakePara.data);
/*	FLASH_WriteWord(VEHICLE_BRAKE_BYPE,(uint32_t)stBrakePara.Source);
	FLASH_WriteWord(VEHICLE_BRAKE_STARTBYTE,(uint32_t)stBrakePara.ucByteNo);
	FLASH_WriteWord(VEHICLE_BRAKE_STARTBIT,(uint32_t)stBrakePara.ucStartBit);
	FLASH_WriteWord(VEHICLE_BRAKE_BITNUM,(uint32_t)stBrakePara.ucBitLth);
	FLASH_WriteWord(VEHICLE_BRAKE_NUMTYPE,(uint32_t)stBrakePara.Type);
	FLASH_WriteWord(VEHICLE_BRAKE_LEFFECTIVE,(uint32_t)stBrakePara.ucValid);
	FLASH_WriteWord(VEHICLE_BRAKE_SCALE_FACTOR_A,(uint32_t)stBrakePara.ucCoeffA);
	FLASH_WriteWord(VEHICLE_BRAKE_SCALE_FACTOR_B,(uint32_t)stBrakePara.ucCoeffB);
	*/
}

//uint32_t vehicle;
void Init_Flash_Sys_Parameter(void)
{
	//整车参数
	FLASH_WriteWord(VEHICLE_WITH_ADDRESS,(uint32_t)stSysPara.ucVehicleWidth);
	FLASH_WriteWord(VEHICLE_KEEP_DIS,(uint32_t)stSysPara.ucDistaceKeep);
	FLASH_WriteWord(VEHICLE_TTC1,(uint32_t)(stSysPara.ttc1 * 10));
	FLASH_WriteWord(VEHICLE_TTC2,(uint32_t)(stSysPara.ttc2 * 10));
	FLASH_WriteWord(VEHICLE_SENSITIVITY,(uint32_t)stSysPara.ucBrakeSense);
	FLASH_WriteWord(VEHICLE_COTROL_MODE,(uint32_t)stSysPara.ucControlMode);
	FLASH_WriteWord(VEHICLE_GEAR_NUM,(uint32_t)stSysPara.GearNum);
	//vehicle = (uint32_t)(stSysPara.hmw1 * 10);
	FLASH_WriteWord(VEHICLE_HMW1,(uint32_t)(stSysPara.hmw1 * 10));
	//vehicle = (uint32_t)(stSysPara.hmw2 * 10);
	FLASH_WriteWord(VEHICLE_HMW2,(uint32_t)(stSysPara.hmw2 * 10));
}


void Init_Flash_CANRata_Parameter(void)
{
	uint32_t rate;
	//can通信率
	FLASH_WriteWord(CAN0_RATE_ADDRESS,(uint32_t)stCanPara.can0rate);
	FLASH_WriteWord(CAN1_RATE_ADDRESS,(uint32_t)stCanPara.can1rate);
	FLASH_WriteWord(CAN2_RATE_ADDRESS,(uint32_t)stCanPara.can2rate);
	FLASH_WriteWord(CAN3_RATE_ADDRESS,(uint32_t)stCanPara.can3rate);
	FLASH_WriteWord(CAN4_RATE_ADDRESS,(uint32_t)stCanPara.can4rate);
	rate = (uint32_t)stCanPara.can5rate | ((uint32_t)stCanPara.rs232_1_rate) << 4| ((uint32_t)stCanPara.rs232_2_rate) << 8;
	FLASH_WriteWord(CAN5_RATE_ADDRESS,rate);
}

void Init_Flash_Linkage_Parameter(void)
{
	//报警联动数据
	FLASH_WriteWord(FCW_LINKAGE_ADDRESS,(uint32_t)stWarninglinkage.FCWlinkage);
	FLASH_WriteWord(FCW_LINKAGE_SPEED_ADDRESS,(uint32_t)stWarninglinkage.FCWlinkageSpeed);
	FLASH_WriteWord(LDW_LINKAGE_ADDRESS,(uint32_t)stWarninglinkage.LDWlinkage);
	FLASH_WriteWord(LDW_LINKAGE_SPEED_ADDRESS,(uint32_t)stWarninglinkage.LDWlinkageSpeed);
	FLASH_WriteWord(HMW_LINKAGE_ADDRESS,(uint32_t)stWarninglinkage.HMWlinkage);
	FLASH_WriteWord(HMW_LINKAGE_SPEED_ADDRESS,(uint32_t)stWarninglinkage.HMWlinkageSpeed);
	FLASH_WriteWord(AEB_LINKAGE_ADDRESS,(uint32_t)stWarninglinkage.AEBlinkage);
	FLASH_WriteWord(AEB_LINKAGE_SPEED_ADDRESS,(uint32_t)stWarninglinkage.AEBlinkageSpeed);
}

void Init_Flash_Swa_Parameter(void)
{
	//FLASH_WriteWord(SWA_IID_ADDRESS,(uint32_t)stSwaPara.lId);
	//FLASH_WriteWord(SWA_BYTENUM_ADDRESS,(uint32_t)stSwaPara.ucByteNo);
	//FLASH_WriteWord(SWA_STARTBIT_ADDRESS,(uint32_t)stSwaPara.ucStartBit);
	//FLASH_WriteWord(SWA_BITLEGH_ADDRESS,(uint32_t)stSwaPara.ucBitLth);
	FLASH_WriteWord(SWA_IID_ADDRESS,(uint32_t)stSwaPara.SourceID);
	FLASH_WriteWord(SWA_BYTENUM_ADDRESS,(uint32_t)stSwaPara.data4);
}

void Init_Flash_Urader_Position(uint8_t i)
{
	uint8_t j;
	//块6
	for(j = 0;j < 12;j ++)
		URader_Position_Flash_Preser(i + 1,j + 1,Urader_Company_Message[i].Uposition[j]);
	//块7
	Urader_Company_Message[0].Uposition[1] = FLMIDE;//FRSIDE;//FLSIDE;
	Urader_Company_Message[0].Uposition[2] = FRMIDE;//FLMIDE;
	Urader_Company_Message[0].Uposition[3] = FRSIDE;//FRMIDE;
	Urader_Company_Message[0].Uposition[4] = RIGHT;//FRSIDE;
	Urader_Company_Message[0].Uposition[5] = RIGHT;//FRIGHT;

}

void Init_Flash_Urader_Sys(void)
{
	FLASH_WriteWord(URADER_SYS_MESSAGE_FIRST_DIMENSIONAL_COEFFICIENT,(uint32_t)(stUraderSysMessage.firststage));
	FLASH_WriteWord(URADER_SYS_MESSAGE_SECOND_DIMENSIONAL_COEFFICIENT,(uint32_t)(stUraderSysMessage.secondstage));
	FLASH_WriteWord(URADER_SYS_MESSAGE_0SPEED_THRESHOLD,(uint32_t)(stUraderSysMessage.speed0distance));
	FLASH_WriteWord(URADER_SYS_MESSAGE_5SPEED_THRESHOLD,(uint32_t)(stUraderSysMessage.speed5distance));
	FLASH_WriteWord(URADER_SYS_MESSAGE_10SPEED_THRESHOLD,(uint32_t)(stUraderSysMessage.speed10distance));
	FLASH_WriteWord(URADER_SYS_MESSAGE_15SPEED_THRESHOLD,(uint32_t)(stUraderSysMessage.speed15distance));
	FLASH_WriteWord(URADER_SYS_MESSAGE_20SPEED_THRESHOLD,(uint32_t)(stUraderSysMessage.speed20distance));
	FLASH_WriteWord(URADER_SYS_MESSAGE_25SPEED_THRESHOLD,(uint32_t)(stUraderSysMessage.speed25distance));
	FLASH_WriteWord(URADER_SYS_MESSAGE_LINKAGE_STATA,(uint32_t)(stUraderSysMessage.SSSlinkage));
	FLASH_WriteWord(URADER_SYS_MESSAGE_LINKAGE_SPEED,(uint32_t)(stUraderSysMessage.SSSlinkageSpeed));
}

/*
 * 函数名称：Init_Flash_Parameter
 * 函数功能：参数写flash
 * 函数输入：无
 * 函数输出：无
 * */
void Init_Flash_Parameter(void)
{
	//块1
	Init_Flash_CANRata_Parameter();
	Init_Flash_Sys_Parameter();
	//块2
	Init_Flash_Speed_Parameter();
	Init_Flash_Abs_Parameter();
	//块3
	Init_Flash_Turn_Parameter();
	//块4
	Init_Flash_Break_Parameter();

	//块5
	Init_Flash_Linkage_Parameter();
	Init_Flash_Swa_Parameter();
	//块6  块7
	Init_Flash_Urader_Position(0);
	Init_Flash_Urader_Position(1);
	//块8
	Init_Flash_Urader_Sys();
}

/*
 * 函数名称：Init_Parameter
 * 函数功能：判断设备是否有参数
 * 函数输入：无
 * 函数输出：无
 * */
void Init_Config_Parameter(void)
{
	uint32_t sencond_ret;

	//FLASH_ReadWord(CAN0_RATE_ADDRESS,&sencond_ret);
	//if(sencond_ret == 0xffffffff)				//读取flash,判断参数
	{
		//fprintf(BLUETOOTH_STREAM,"Get_Flash_Parameter11111 %d\t\n");
		Init_SetParameter();
		Init_Flash_Parameter();
	}
	//else
	//{
		//fprintf(BLUETOOTH_STREAM,"Get_Flash_Parameter2222\t\n");
	//	Get_Flash_Parameter();
	//}
}

//通信参数初始化
void Init_Sys_Parameter(void)
{
	uint8_t i;
	uint8_t j;
	//超声波雷达初始化
	for(i = 0;i < 4;i ++)
	{
		stUraderCanData[i].flgBoxRxEnd = 0;
	}
	for(i = 0;i < 2;i ++)
	{
		Urader_Company_Message[i].Company = 0;
		for(j = 0;j < 12;j ++)
		{
			Urader_Company_Message[i].distance[j] = 4;
			Urader_Company_Message[i].Urader_work_stata[j] = NOWORK;
			//Urader_Company_Message[i].Uposition[j] = NONEP;
		}
	}
	//摄像头数据初始化
	stCammeraCanData.flgBoxRxEnd = 0;
	stVehicleCanData.flgBoxRxEnd = 0;
	stVehicleCanSpeedData.flgBoxRxEnd = 0;
	stVehicleCanTurnData.flgBoxRxEnd = 0;
	stVehicleCanBreakData.flgBoxRxEnd = 0;
	stVehicleCanAbsData.flgBoxRxEnd = 0;
	stVehicleCanWarningData.flgBoxRxEnd = 0;
	stVehicleCanSWAData.flgBoxRxEnd = 0;

	for(i = 0;i < 8;i ++)
	{
		stCammeraCanData.data[i] = 0;
		stVehicleCanData.data[i] = 0;
		stVehicleCanSpeedData.data[i] = 0;
		stVehicleCanTurnData.data[i] = 0;
		stVehicleCanBreakData.data[i] = 0;
		stVehicleCanAbsData.data[i] = 0;
		stVehicleCanWarningData.data[i] = 0;
		stVehicleCanSWAData.data[i] = 0;
	}

	stVehicleParas.AbsErrorFlag = 0;
	stVehicleParas.BrakeFlag = 0;
	stVehicleParas.LeftFlag = 0;
	stVehicleParas.LeftFlagTemp = 0;
	stVehicleParas.RightFlag = 0;
	stVehicleParas.RightFlagTemp = 0;
	stVehicleParas.TimeStamp = 0;
	stVehicleParas.fVehicleSpeed = 0;
	stVehicleParas.kongdangFlag = 0;
	stVehicleParas.shoudBrakeFlag = 0;

	//UraderSpeed[0] = (float)((float)stUraderSysMessage.speed0distance/stUraderSysMessage.speed5distance);
	//UraderSpeed[1] = (float)((float)stUraderSysMessage.speed5distance/stUraderSysMessage.speed10distance);
	//UraderSpeed[2] = (float)((float)stUraderSysMessage.speed10distance/stUraderSysMessage.speed15distance);
	//UraderSpeed[3] = (float)((float)stUraderSysMessage.speed15distance/stUraderSysMessage.speed20distance);
	//UraderSpeed[4] = (float)((float)stUraderSysMessage.speed20distance/stUraderSysMessage.speed25distance);
}



void ReWrite_one(void)
{
	FLASH_Wipe_Configuration_RAM(FLASH_WIPE_CODE_PAGE,CAN0_RATE_ADDRESS);
	//Init_Flash_CANRata_Parameter();
	//Init_Flash_Sys_Parameter();
	Init_Flash_Parameter();
}

void ReWrite_two(void)
{
	FLASH_Wipe_Configuration_RAM(FLASH_WIPE_CODE_PAGE,VEHICLE_SPEED_MODE);
	//Init_Flash_Speed_Parameter();
	//Init_Flash_Abs_Parameter();
	//Init_Flash_Parameter();
}

void ReWrite_three(void)
{
	FLASH_Wipe_Configuration_RAM(FLASH_WIPE_CODE_PAGE,0x7f200);
	//Init_Flash_Parameter();
	//Init_Flash_Turn_Parameter();
}

void ReWrite_four(void)
{
	FLASH_Wipe_Configuration_RAM(FLASH_WIPE_CODE_PAGE,0x7f300);
	//Init_Flash_Parameter();
	//Init_Flash_Break_Parameter();
}

void ReWrite_five(void)
{
	FLASH_Wipe_Configuration_RAM(FLASH_WIPE_CODE_PAGE,0x7f400);
	//Init_Flash_Parameter();
	//Init_Flash_Linkage_Parameter();
	//Init_Flash_Swa_Parameter();
}

void ReWrite_six(void)
{
	FLASH_Wipe_Configuration_RAM(FLASH_WIPE_CODE_PAGE,0x7f500);
	//Init_Flash_Parameter();
	//Init_Flash_Urader_Position(1);
}

void ReWrite_seven(void)
{
	FLASH_Wipe_Configuration_RAM(FLASH_WIPE_CODE_PAGE,0x7f600);
	//Init_Flash_Parameter();
	//Init_Flash_Urader_Position(2);
}

void ReWrite_eight(void)
{
	FLASH_Wipe_Configuration_RAM(FLASH_WIPE_CODE_PAGE,0x7f700);
	//Init_Flash_Parameter();
	//Init_Flash_Urader_Sys();
}

void Reaskc_Set_Commond(void)
{
	uint8_t i = 0;

	for(i = 0;i < 8;i ++)
		can_recv_valve_control.data[i] = 0xff;
	can_recv_valve_control.TargetID = 0x7c6;
	can_recv_valve_control.data[0] = 0x01;
	CAN_Transmit_DATA(CAN_PARAMETER,can_recv_valve_control);
}

void ProcessHostCmd(void)
{
	if(parameterset.first == 1)
	{
		parameterset.first = 0;
		ReWrite_one();
		can_recv_valve_control.TargetID = 0x7c6;
		can_recv_valve_control.data[0] = 0x01;
		CAN_Transmit_DATA(CAN_PARAMETER,can_recv_valve_control);
	}
	if(parameterset.second == 1)
	{
		parameterset.second = 0;
		ReWrite_two();
		can_recv_valve_control.TargetID = 0x7c6;
		can_recv_valve_control.data[0] = 0x01;
		CAN_Transmit_DATA(CAN_PARAMETER,can_recv_valve_control);
	}
	if(parameterset.three == 1)
	{
		parameterset.three = 0;
		ReWrite_three();
		can_recv_valve_control.TargetID = 0x7c6;
		can_recv_valve_control.data[0] = 0x01;
		CAN_Transmit_DATA(CAN_PARAMETER,can_recv_valve_control);
	}
	if(parameterset.four == 1)
	{
		parameterset.four = 0;
		ReWrite_four();
		can_recv_valve_control.TargetID = 0x7c6;
		can_recv_valve_control.data[0] = 0x01;
		CAN_Transmit_DATA(CAN_PARAMETER,can_recv_valve_control);
	}
	if(parameterset.five == 1)
	{
		parameterset.five = 0;
		ReWrite_five();
		can_recv_valve_control.TargetID = 0x7c6;
		can_recv_valve_control.data[0] = 0x01;
		CAN_Transmit_DATA(CAN_PARAMETER,can_recv_valve_control);
	}
	if(parameterset.six == 1)
	{
		parameterset.six = 0;
		ReWrite_six();
		can_recv_valve_control.TargetID = 0x7c6;
		can_recv_valve_control.data[0] = 0x01;
		CAN_Transmit_DATA(CAN_PARAMETER,can_recv_valve_control);
	}
	if(parameterset.seven == 1)
	{
		parameterset.seven = 0;
		ReWrite_seven();
		can_recv_valve_control.TargetID = 0x7c6;
		can_recv_valve_control.data[0] = 0x01;
		CAN_Transmit_DATA(CAN_PARAMETER,can_recv_valve_control);
	}
	if(parameterset.eight == 1)
	{
		parameterset.eight = 0;
		ReWrite_eight();
		can_recv_valve_control.TargetID = 0x7c6;
		can_recv_valve_control.data[0] = 0x01;
		CAN_Transmit_DATA(CAN_PARAMETER,can_recv_valve_control);
	}
}

