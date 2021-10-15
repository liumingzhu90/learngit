/*
 * get_parameter.c
 *
 *  Created on: 2021-8-13
 *      Author: wangzhenbao
 */
#include "usart.h"
#include "flash.h"
#include <stdio.h>
#include <get_parameter.h>
#include "common.h"

uint32_t num[9];
void Bluetooth_Get_Speed_Commond(void)
{

	uint32_t wordnum;

	num[0] = FLASH_ReadWord(VEHICLE_SPEED_MODE,&wordnum);
	num[1] = FLASH_ReadWord(VEHICLE_SPEED_ID,&wordnum);
	num[2] = FLASH_ReadWord(VEHICLE_SPEED_BYTETYPE,&wordnum);
	num[3] = FLASH_ReadWord(VEHICLE_SPEED_BYTESTART,&wordnum);
	num[4] = FLASH_ReadWord(VEHICLE_SPEED_BYTENUM,&wordnum);
	num[5] = FLASH_ReadWord(VEHICLE_SPEED_SCALE_FACTOR_A,&wordnum);
	num[6] = FLASH_ReadWord(VEHICLE_SPEED_SCALE_FACTOR_B,&wordnum);
	fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*SPEED*%d*%x*%d*%d*%d*%d*%d*USTOP",num[0],num[1],num[2],num[3],num[4],num[5],num[6]);

}

void Bluetooth_Get_URadSM_Sys_Commond(void)
{
	uint32_t num[9];
	uint32_t wordnum;

	num[0] = FLASH_ReadWord(URADER_SYS_MESSAGE_FIRST_DIMENSIONAL_COEFFICIENT,&wordnum);
	num[1] = FLASH_ReadWord(URADER_SYS_MESSAGE_SECOND_DIMENSIONAL_COEFFICIENT,&wordnum);
	num[2] = FLASH_ReadWord(URADER_SYS_MESSAGE_0SPEED_THRESHOLD,&wordnum);
	num[3] = FLASH_ReadWord(URADER_SYS_MESSAGE_5SPEED_THRESHOLD,&wordnum);
	num[4] = FLASH_ReadWord(URADER_SYS_MESSAGE_10SPEED_THRESHOLD,&wordnum);
	num[5] = FLASH_ReadWord(URADER_SYS_MESSAGE_15SPEED_THRESHOLD,&wordnum);
	num[6] = FLASH_ReadWord(URADER_SYS_MESSAGE_20SPEED_THRESHOLD,&wordnum);
	fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*URADSYS*%d*%x*%d*%d*%d*%d*%d*USTOP",num[0],num[1],num[2],num[3],num[4],num[5],num[6]);
}

void Bluetooth_Get_CAN_Commond(uint32_t ret)
{
	uint8_t send_data[1024]="USTART*GETPARAMETER*CAN0RATE*0*USTOP";
	uint16_t sencond_ret;

	Get_Can_Rata_Config(ret - 1,&sencond_ret);
	send_data[29] = sencond_ret + 0x30;
	send_data[23] = (ret - 1) + 0x30;
	fprintf(BLUETOOTH_STREAM,"%s",send_data);
}

void Bluetooth_Get_Sys_Commond(void)
{
	uint32_t num[9];
	uint32_t wordnum;
	float fnum[2];

	num[0] = FLASH_ReadWord(VEHICLE_WITH_ADDRESS,&wordnum);
	num[1] = FLASH_ReadWord(VEHICLE_KEEP_DIS,&wordnum);
	num[2] = FLASH_ReadWord(VEHICLE_TTC1,&wordnum);
	fnum[0] = (float)num[2]/10;
	num[3] = FLASH_ReadWord(VEHICLE_TTC2,&wordnum);
	fnum[1] = (float)num[3]/10;
	num[4] = FLASH_ReadWord(VEHICLE_SENSITIVITY,&wordnum);
	num[5] = FLASH_ReadWord(VEHICLE_COTROL_MODE,&wordnum);
	num[6] = FLASH_ReadWord(VEHICLE_GEAR_NUM,&wordnum);
	fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*VEHICLE*%d*%d*%.1lf*%.1lf*%d*%d*%d*USTOP",num[0],num[1],fnum[0],fnum[1],num[4],num[5],num[6]);
}

void Bluetooth_Get_Abs_Commond(void)
{
	uint32_t num[9];
	uint32_t wordnum;

	num[0] = FLASH_ReadWord(VEHICLE_ABS_ID,&wordnum);
	num[1] = FLASH_ReadWord(VEHICLE_ABS_TYPE,&wordnum);
	num[2] = FLASH_ReadWord(VEHICLE_ABS_STARTBIT,&wordnum);
	num[3] = FLASH_ReadWord(VEHICLE_ABS_BITNUM,&wordnum);
	num[4] = FLASH_ReadWord(VEHICLE_ABS_WRONGNUM,&wordnum);
	num[5] = FLASH_ReadWord(VEHICLE_ABS_BYTENUM,&wordnum);
	fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*ABS*%x*%d*%d*%d*%d*USTOP",num[0],num[1],num[2],num[3],num[4],num[5]);
}

void Bluetooth_Get_Turn_Commond(void)
{
	uint32_t num[9];
	uint32_t wordnum;

	num[0] = FLASH_ReadWord(VEHICLE_TURN_ID,&wordnum);
	num[1] = FLASH_ReadWord(VEHICLE_TURN_BYPE,&wordnum);
	num[2] = FLASH_ReadWord(VEHICLE_TURN_STARTBYTE,&wordnum);
	num[3] = FLASH_ReadWord(VEHICLE_TURN_LSTARTBIT,&wordnum);
	num[4] = FLASH_ReadWord(VEHICLE_TURN_LBITNUM,&wordnum);
	num[5] = FLASH_ReadWord(VEHICLE_TURN_LEFFECTIVE,&wordnum);
	num[6] = FLASH_ReadWord(VEHICLE_TURN_RSTARTBIT,&wordnum);
	num[7] = FLASH_ReadWord(VEHICLE_TURN_RBITNUM,&wordnum);
	num[8] = FLASH_ReadWord(VEHICLE_TURN_REFFECTIVE,&wordnum);

	fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*TURN*%x*%d*%d*%d*%d*%d*%d*%d*%d*USTOP",num[0],num[1],num[2],num[3],num[4],num[5],num[6],num[7],num[8]);

}

void Bluetooth_Get_Break_Commond(void)
{
	uint32_t num[9];
	uint32_t wordnum;

	num[0] = FLASH_ReadWord(VEHICLE_BRAKE_ID,&wordnum);
	num[1] = FLASH_ReadWord(VEHICLE_BRAKE_BYPE,&wordnum);
	num[2] = FLASH_ReadWord(VEHICLE_BRAKE_STARTBYTE,&wordnum);
	num[3] = FLASH_ReadWord(VEHICLE_BRAKE_STARTBIT,&wordnum);
	num[4] = FLASH_ReadWord(VEHICLE_BRAKE_BITNUM,&wordnum);
	num[5] = FLASH_ReadWord(VEHICLE_BRAKE_NUMTYPE,&wordnum);
	num[6] = FLASH_ReadWord(VEHICLE_BRAKE_LEFFECTIVE,&wordnum);
	num[7] = FLASH_ReadWord(VEHICLE_BRAKE_SCALE_FACTOR_A,&wordnum);
	num[8] = FLASH_ReadWord(VEHICLE_BRAKE_SCALE_FACTOR_B,&wordnum);

	fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*BRAKE*%x*%d*%d*%d*%d*%d*%d*%d*%d*USTOP",num[0],num[1],num[2],num[3],num[4],num[5],num[6],num[7],num[8]);

}

void Bluetooth_Get_Linkage_Commond(void)
{
	uint32_t num[9];
	uint32_t wordnum;

	num[0] = FLASH_ReadWord(FCW_LINKAGE_ADDRESS,&wordnum);
	num[1] = FLASH_ReadWord(FCW_LINKAGE_SPEED_ADDRESS,&wordnum);
	num[2] = FLASH_ReadWord(LDW_LINKAGE_ADDRESS,&wordnum);
	num[3] = FLASH_ReadWord(LDW_LINKAGE_SPEED_ADDRESS,&wordnum);
	num[4] = FLASH_ReadWord(HMW_LINKAGE_ADDRESS,&wordnum);
	num[5] = FLASH_ReadWord(HMW_LINKAGE_SPEED_ADDRESS,&wordnum);
	num[6] = FLASH_ReadWord(AEB_LINKAGE_ADDRESS,&wordnum);
	num[7] = FLASH_ReadWord(AEB_LINKAGE_SPEED_ADDRESS,&wordnum);

	fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*WLSPEED*%d*%d*%d*%d*%d*%d*%d*%d*USTOP",num[0],num[1],num[2],num[3],num[4],num[5],num[6],num[7]);

}

void Bluetooth_Get_Swa_Commond(void)
{
	uint32_t num[9];
	uint32_t wordnum;

	num[0] = FLASH_ReadWord(SWA_IID_ADDRESS,&wordnum);
	num[1] = FLASH_ReadWord(SWA_BYTENUM_ADDRESS,&wordnum);
	num[2] = FLASH_ReadWord(SWA_STARTBIT_ADDRESS,&wordnum);
	num[3] = FLASH_ReadWord(SWA_BITLEGH_ADDRESS,&wordnum);

	fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*SWA*%d*%d*%d*%d*USTOP",num[0],num[1],num[2],num[3]);

}

uint8_t Bluetooth_Get_URadP_Position(uint8_t a,uint8_t b)
{
	uint32_t ret;
	uint32_t wordnum;

	if(a == 1)
	{
		if(b == 1)
		{
			ret = FLASH_ReadWord(URADER_HOST_ONE_ONE_ADDRESS,&wordnum);
		}
		else if(b == 2)
		{
			ret = FLASH_ReadWord(URADER_HOST_ONE_TWO_ADDRESS,&wordnum);
		}
		else if(b == 3)
		{
			ret = FLASH_ReadWord(URADER_HOST_ONE_THREE_ADDRESS,&wordnum);
		}
		else if(b == 4)
		{
			ret = FLASH_ReadWord(URADER_HOST_ONE_FOUR_ADDRESS,&wordnum);
		}
		else if(b == 5)
		{
			ret = FLASH_ReadWord(URADER_HOST_ONE_FIVE_ADDRESS,&wordnum);
		}
		else if(b == 6)
		{
			ret = FLASH_ReadWord(URADER_HOST_ONE_SIX_ADDRESS,&wordnum);
		}
		else if(b == 7)
		{
			ret = FLASH_ReadWord(URADER_HOST_ONE_SEVEN_ADDRESS,&wordnum);
		}
		else if(b == 8)
		{
			ret = FLASH_ReadWord(URADER_HOST_ONE_EIGHT_ADDRESS,&wordnum);
		}
		else if(b == 9)
		{
			ret = FLASH_ReadWord(URADER_HOST_ONE_NINE_ADDRESS,&wordnum);
		}
		else if(b == 10)
		{
			ret = FLASH_ReadWord(URADER_HOST_ONE_TEN_ADDRESS,&wordnum);
		}
		else if(b == 11)
		{
			ret = FLASH_ReadWord(URADER_HOST_ONE_ELEVEN_ADDRESS,&wordnum);
		}
		else if(b == 12)
		{
			ret = FLASH_ReadWord(URADER_HOST_ONE_TWELVE_ADDRESS,&wordnum);
		}

	}
	else if(a == 2)
	{
		if(b == 1)
		{
			ret = FLASH_ReadWord(URADER_HOST_TWO_ONE_ADDRESS,&wordnum);
		}
		else if(b == 2)
		{
			ret = FLASH_ReadWord(URADER_HOST_TWO_TWO_ADDRESS,&wordnum);
		}
		else if(b == 3)
		{
			ret = FLASH_ReadWord(URADER_HOST_TWO_THREE_ADDRESS,&wordnum);
		}
		else if(b == 4)
		{
			ret = FLASH_ReadWord(URADER_HOST_TWO_FOUR_ADDRESS,&wordnum);
		}
		else if(b == 5)
		{
			ret = FLASH_ReadWord(URADER_HOST_TWO_FIVE_ADDRESS,&wordnum);
		}
		else if(b == 6)
		{
			ret = FLASH_ReadWord(URADER_HOST_TWO_SIX_ADDRESS,&wordnum);
		}
		else if(b == 7)
		{
			ret = FLASH_ReadWord(URADER_HOST_TWO_SEVEN_ADDRESS,&wordnum);
		}
		else if(b == 8)
		{
			ret = FLASH_ReadWord(URADER_HOST_TWO_EIGHT_ADDRESS,&wordnum);
		}
		else if(b == 9)
		{
			ret = FLASH_ReadWord(URADER_HOST_TWO_NINE_ADDRESS,&wordnum);
		}
		else if(b == 10)
		{
			ret = FLASH_ReadWord(URADER_HOST_TWO_TEN_ADDRESS,&wordnum);
		}
		else if(b == 11)
		{
			ret = FLASH_ReadWord(URADER_HOST_TWO_ELEVEN_ADDRESS,&wordnum);
		}
		else if(b == 12)
		{
			ret = FLASH_ReadWord(URADER_HOST_TWO_TWELVE_ADDRESS,&wordnum);
		}
	}
	else
		;

	return (uint8_t)ret;
}

void Bluetooth_Get_URadP_Commond(uint8_t *data)
{
	uint8_t i,j,k;

	i = Bluetooth_Set_Vehicle_With(data,3,4);
	j = Bluetooth_Set_Vehicle_With(data,4,5);
	k = Bluetooth_Get_URadP_Position(i,j);
	switch(k)
	{
	case 0:
		fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*FRONT*%d*%d*USTOP",i,j);
		break;
	case 1:
		fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*AFTER*%d*%d*USTOP",i,j);
		break;
	case 2:
		fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*LEFT*%d*%d*USTOP",i,j);
		break;
	case 3:
		fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*RIGHT*%d*%d*USTOP",i,j);
		break;
	case 4:
		fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*FLEFT*%d*%d*USTOP",i,j);
		break;
	case 5:
		fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*FLSIDE*%d*%d*USTOP",i,j);
		break;
	case 6:
		fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*FLMIDE*%d*%d*USTOP",i,j);
		break;
	case 7:
		fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*FRIGHT*%d*%d*USTOP",i,j);
		break;
	case 8:
		fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*FRSIDE*%d*%d*USTOP",i,j);
		break;
	case 9:
		fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*FRMID*%d*%d*USTOP",i,j);
		break;
	default:
		fprintf(BLUETOOTH_STREAM,"USTART*GETPARAMETER*UNONE*%d*%d*USTOP",i,j);
		break;
	}
}

void Bluetooth_Get_Commond(uint8_t *data)
{
	uint8_t send_data[1024]="USTART*GETPARAMETER*CAN0RATE*0*USTOP";
	uint32_t ret;
	uint16_t sencond_ret;
	uint32_t num[9];
	uint32_t wordnum;
	float fnum[2];

	ret = Bluetooth_Get_Commond_Annalyse(data);
	switch(ret)
	{
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
			Bluetooth_Get_CAN_Commond(ret);
			break;
		case 8:
			Bluetooth_Get_Speed_Commond();
			break;
		case 7:
			Bluetooth_Get_Sys_Commond();
			break;
		case 9:
			Bluetooth_Get_Abs_Commond();
			break;
		case 10:
			Bluetooth_Get_Turn_Commond();
			break;
		case 11:
			Bluetooth_Get_Break_Commond();
			break;
		case 12:
			Bluetooth_Get_Linkage_Commond();
			break;
		case 13:
			Bluetooth_Get_Swa_Commond();
			break;
		case 14:
			Bluetooth_Get_URadP_Commond(data);
			break;
		case 25:
			Bluetooth_Get_URadSM_Sys_Commond();
			break;
		default:
			break;
	}
}

uint32_t Bluetooth_Get_Commond_Annalyse(uint8_t *data)
{
	uint32_t ret;

	if((data[20] == 'C') &&
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
			(data[22] == 'R'))		//ABS参数设置
		ret = 10;
	else if((data[20] == 'B') &&
			(data[21] == 'R') &&
			(data[22] == 'A'))		//ABS参数设置
		ret = 11;
	else if((data[20] == 'W') &&
			(data[21] == 'L') &&
			(data[22] == 'S'))		//ABS参数设置
		ret = 12;
	else if((data[20] == 'S') &&
			(data[21] == 'W') &&
			(data[22] == 'A'))		//ABS参数设置
		ret = 13;

	else if((data[20] == 'U') &&	//前
			(data[21] == 'R') &&
			(data[22] == 'A') &&
			(data[23] == 'D') &&
			(data[24] == 'P'))
		ret = 14;

	else if((data[20] == 'U') &&	//雷达第一维系数
			(data[21] == 'R') &&
			(data[22] == 'A') &&
			(data[23] == 'D') &&
			(data[24] == 'S'))
		ret = 25;
	else
		ret = 0;
	return ret;
}

void Get_Can_Rata_Config(uint8_t cannum,uint16_t *canrata)
{
	uint32_t ret;
	if(cannum == 0)
		ret = FLASH_ReadHalWord(CAN0_RATE_ADDRESS,canrata);
	if(cannum == 1)
		ret = FLASH_ReadHalWord(CAN1_RATE_ADDRESS,canrata);
	if(cannum == 2)
		ret = FLASH_ReadHalWord(CAN2_RATE_ADDRESS,canrata);
	if(cannum == 3)
		ret = FLASH_ReadHalWord(CAN3_RATE_ADDRESS,canrata);
	if(cannum == 4)
		ret = FLASH_ReadHalWord(CAN4_RATE_ADDRESS,canrata);
	if(cannum == 5)
		ret = FLASH_ReadHalWord(CAN5_RATE_ADDRESS,canrata);
	//return ret;
}

void Get_Speed_Parameter(void)
{
	uint32_t valide = 0;
	uint32_t num[9];

	FLASH_ReadWord(VEHICLE_SPEED_ID,&valide);//0x10000000;		//车速ID 10000000
	stSpeedPara.lId = valide;
	FLASH_ReadWord(VEHICLE_SPEED_MODE,&valide);			//默认为CAN通信
	stSpeedPara.uSingnalType = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_SPEED_BYTETYPE,&valide);			//高字节在前
	stSpeedPara.ucByteOrder = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_SPEED_BYTESTART,&valide);			//起始字节
	stSpeedPara.ucStartByte = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_SPEED_BYTENUM,&valide);			//默认一个字节
	stSpeedPara.ucByteLth = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_SPEED_SCALE_FACTOR_A,&valide);;				//正向比例系数默认设置1
	stSpeedPara.ucCoeffA = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_SPEED_SCALE_FACTOR_B,&valide);				//负向比例系数默认设置1
	stSpeedPara.ucCoeffB = (uint8_t)valide;
}

void Get_Turn_Parameter(void)
{
	uint32_t valide = 0;

	FLASH_ReadWord(VEHICLE_TURN_ID,&valide);		// 转向ID.
	stTurnPara.lId = valide;
	FLASH_ReadWord(VEHICLE_TURN_BYPE,&valide);				// 信号来源. 0:CAN报文; 1:I/0输入.
	stTurnPara.Source = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_TURN_STARTBYTE,&valide);				// 字节序号. 取值0-7.
	stTurnPara.ucByteNo = (uint8_t)valide;
	//FLASH_ReadWord(VEHICLE_TURN_LSTARTBIT,&valide);
	//stTurnPara.ucLeftStartBit = (uint8_t)valide;
	//FLASH_ReadWord(VEHICLE_TURN_LBITNUM,&valide);
	//stTurnPara.ucLeftBitLth = (uint8_t)valide;
	//FLASH_ReadWord(VEHICLE_TURN_LEFFECTIVE,&valide);
	//stTurnPara.ucLeftValid = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_TURN_RSTARTBIT,&valide);
	stTurnPara.ucRightStartBit = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_TURN_RBITNUM,&valide);
	stTurnPara.ucRightBitLth = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_TURN_REFFECTIVE,&valide);
	stTurnPara.ucRightValid= (uint8_t)valide;

	FLASH_ReadWord(VEHICLE_TURN_ID1,&valide);		// 转向ID.
	stTurnPara.lId1 = valide;
	FLASH_ReadWord(VEHICLE_TURN_BYPE1,&valide);				// 信号来源. 0:CAN报文; 1:I/0输入.
	stTurnPara.Source1 = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_TURN_STARTBYTE1,&valide);				// 字节序号. 取值0-7.
	stTurnPara.ucByteNo1 = (uint8_t)valide;
	//FLASH_ReadWord(VEHICLE_TURN_LSTARTBIT,&valide);
	//stTurnPara.ucLeftStartBit = (uint8_t)valide;
	//FLASH_ReadWord(VEHICLE_TURN_LBITNUM,&valide);
	//stTurnPara.ucLeftBitLth = (uint8_t)valide;
	//FLASH_ReadWord(VEHICLE_TURN_LEFFECTIVE,&valide);
	//stTurnPara.ucLeftValid = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_TURN_RSTARTBIT,&valide);
	stTurnPara.ucRightStartBit = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_TURN_RBITNUM,&valide);
	stTurnPara.ucRightBitLth = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_TURN_REFFECTIVE,&valide);
	stTurnPara.ucRightValid= (uint8_t)valide;

}

void Get_Abs_Parameter(void)
{
	uint32_t valide = 0;

	FLASH_ReadWord(VEHICLE_ABS_ID,&valide);
	stAbsPara.lId = valide;
	FLASH_ReadWord(VEHICLE_ABS_TYPE,&valide);
	stAbsPara.Source = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_ABS_BYTENUM,&valide);
	stAbsPara.ucByteNo = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_ABS_STARTBIT,&valide);
	stAbsPara.ucAbsWarningStartBit = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_ABS_BITNUM,&valide);
	stAbsPara.ucAbsWarningBitLth= (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_ABS_WRONGNUM,&valide);
	stAbsPara.ucAbsWarningValid = (uint8_t)valide;
}

void Get_Break_Parameter(void)
{
	uint32_t valide = 0;

	FLASH_ReadWord(VEHICLE_BRAKE_ID,&valide);
	stBrakePara.lId = valide;
	FLASH_ReadWord(VEHICLE_BRAKE_BYPE,&valide);
	stBrakePara.Source = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_BRAKE_STARTBYTE,&valide);
	stBrakePara.ucByteNo = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_BRAKE_STARTBIT,&valide);
	stBrakePara.ucStartBit = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_BRAKE_BITNUM,&valide);
	stBrakePara.ucBitLth = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_BRAKE_NUMTYPE,&valide);
	stBrakePara.Type = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_BRAKE_LEFFECTIVE,&valide);
	stBrakePara.ucValid = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_BRAKE_SCALE_FACTOR_A,&valide);
	stBrakePara.ucCoeffA = (uint8_t)valide;
	FLASH_ReadWord(VEHICLE_BRAKE_SCALE_FACTOR_B,&valide);
	stBrakePara.ucCoeffB = (uint8_t)valide;
}

void Get_Vehicle_Parameter(void)
{
	uint32_t valide = 0;

	FLASH_ReadWord(VEHICLE_WITH_ADDRESS,&valide);
	stSysPara.ucVehicleWidth = valide;
	FLASH_ReadWord(VEHICLE_KEEP_DIS,&valide);
	stSysPara.ucDistaceKeep = valide;
	FLASH_ReadWord(VEHICLE_SENSITIVITY,&valide);
	stSysPara.ucBrakeSense = valide;
	FLASH_ReadWord(VEHICLE_TTC1,&valide);
	stSysPara.ttc1 = valide * 0.1;
	FLASH_ReadWord(VEHICLE_TTC2,&valide);
	stSysPara.ttc2 = valide * 0.1;
	FLASH_ReadWord(VEHICLE_COTROL_MODE,&valide);
	stSysPara.ucControlMode = valide;
	FLASH_ReadWord(VEHICLE_GEAR_NUM,&valide);
	stSysPara.GearNum = valide;

}

void Get_Can_Parameter(void)
{
	uint32_t valide = 0;

	FLASH_ReadWord(CAN0_RATE_ADDRESS,&valide);
	stCanPara.can0rate = (uint8_t)valide;
	FLASH_ReadWord(CAN1_RATE_ADDRESS,&valide);
	stCanPara.can1rate = (uint8_t)valide;
	FLASH_ReadWord(CAN2_RATE_ADDRESS,&valide);
	stCanPara.can2rate = (uint8_t)valide;
	FLASH_ReadWord(CAN3_RATE_ADDRESS,&valide);
	stCanPara.can3rate = (uint8_t)valide;
	FLASH_ReadWord(CAN4_RATE_ADDRESS,&valide);
	stCanPara.can4rate = (uint8_t)valide;
	FLASH_ReadWord(CAN5_RATE_ADDRESS,&valide);
	stCanPara.can5rate = (uint8_t)valide;
}

void Get_WLA_Parameter(void)
{
	uint32_t valide = 0;

	FLASH_ReadWord(FCW_LINKAGE_ADDRESS,&valide);
	stWarninglinkage.FCWlinkage = (uint8_t)valide;
	FLASH_ReadWord(FCW_LINKAGE_SPEED_ADDRESS,&valide);
	stWarninglinkage.FCWlinkageSpeed = (uint8_t)valide;
	FLASH_ReadWord(LDW_LINKAGE_ADDRESS,&valide);
	stWarninglinkage.LDWlinkage = (uint8_t)valide;
	FLASH_ReadWord(LDW_LINKAGE_SPEED_ADDRESS,&valide);
	stWarninglinkage.LDWlinkageSpeed = (uint8_t)valide;
	FLASH_ReadWord(HMW_LINKAGE_ADDRESS,&valide);
	stWarninglinkage.HMWlinkage = (uint8_t)valide;
	FLASH_ReadWord(HMW_LINKAGE_SPEED_ADDRESS,&valide);
	stWarninglinkage.HMWlinkageSpeed = valide;
	FLASH_ReadWord(AEB_LINKAGE_ADDRESS,&valide);
	stWarninglinkage.AEBlinkage = (uint8_t)valide;
	FLASH_ReadWord(AEB_LINKAGE_SPEED_ADDRESS,&valide);
	stWarninglinkage.AEBlinkageSpeed = valide;
}

void Get_Swa_Parameter(void)
{
	uint32_t valide = 0;

	FLASH_ReadWord(SWA_IID_ADDRESS,&valide);
	stSwaPara.lId = valide;
	FLASH_ReadWord(SWA_BYTENUM_ADDRESS,&valide);
	stSwaPara.ucByteNo = (uint8_t)valide;
	FLASH_ReadWord(SWA_STARTBIT_ADDRESS,&valide);
	stSwaPara.ucStartBit = (uint8_t)valide;
	FLASH_ReadWord(SWA_BITLEGH_ADDRESS,&valide);
	stSwaPara.ucBitLth = (uint8_t)valide;
}

void Get_Urader_Sys(void)
{
	uint32_t valide = 0;

	FLASH_ReadWord(URADER_SYS_MESSAGE_FIRST_DIMENSIONAL_COEFFICIENT,&valide);
	stUraderSysMessage.firststage = valide;
	FLASH_ReadWord(URADER_SYS_MESSAGE_SECOND_DIMENSIONAL_COEFFICIENT,&valide);
	stUraderSysMessage.secondstage = valide;
	FLASH_ReadWord(URADER_SYS_MESSAGE_0SPEED_THRESHOLD,&valide);
	stUraderSysMessage.speed0distance = valide;
	FLASH_ReadWord(URADER_SYS_MESSAGE_5SPEED_THRESHOLD,&valide);
	stUraderSysMessage.speed5distance = valide;
	FLASH_ReadWord(URADER_SYS_MESSAGE_10SPEED_THRESHOLD,&valide);
	stUraderSysMessage.speed10distance = valide;
	FLASH_ReadWord(URADER_SYS_MESSAGE_15SPEED_THRESHOLD,&valide);
	stUraderSysMessage.speed15distance = valide;
	FLASH_ReadWord(URADER_SYS_MESSAGE_20SPEED_THRESHOLD,&valide);
	stUraderSysMessage.speed20distance = valide;
	FLASH_ReadWord(URADER_SYS_MESSAGE_25SPEED_THRESHOLD,&valide);
	stUraderSysMessage.speed25distance = valide;
	FLASH_ReadWord(URADER_SYS_MESSAGE_LINKAGE_STATA,&valide);
	stUraderSysMessage.SSSlinkage = valide;
	FLASH_ReadWord(URADER_SYS_MESSAGE_LINKAGE_SPEED,&valide);
	stUraderSysMessage.SSSlinkageSpeed = valide;
}

void Get_Urader_Position(void)
{
	uint8_t i,j;

	for(i = 0;i < 2;i ++)
	{
		for(j = 0;j < 12;j ++)
			Urader_Company_Message[i].Uposition[j] = Bluetooth_Get_URadP_Position(i + 1,j + 1);
	}
}

void Get_Flash_Parameter(void)
{
	Get_Speed_Parameter();
	Get_Turn_Parameter();
	Get_Abs_Parameter();
	Get_Break_Parameter();
	Get_Vehicle_Parameter();
	//fprintf(BLUETOOTH_STREAM,"Get_Vehicle_Parameter\t\n");
	Get_Can_Parameter();
	//fprintf(BLUETOOTH_STREAM,"Get_Can_Parameter\t\n");
	Get_WLA_Parameter();
	//fprintf(BLUETOOTH_STREAM,"Get_WLA_Parameter\t\n");
	Get_Swa_Parameter();
	//fprintf(BLUETOOTH_STREAM,"Get_Swa_Parameter\t\n");
	Get_Urader_Sys();
	//fprintf(BLUETOOTH_STREAM,"Get_Urader_Sys\t\n");
	Get_Urader_Position();
	//fprintf(BLUETOOTH_STREAM,"Get_Urader_Position\t\n");
}
