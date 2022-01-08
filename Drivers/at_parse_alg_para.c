/*
 * at_parse_alg_para.c
 *
 *  Created on: 2021-12-30
 *      Author: Administrator
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <malloc.h>
#include "cat.h"
#include "usart.h"
#include "upgrade_common.h"
#include "at_parse_alg_para.h"
#include "common.h"
#include "w25qxx.h"
/**************************** 宏/枚举 ****************************************************/
//#define	PROCESS_AT_CMD_5HZ		200
//#define	PROCESS_AT_CMD_10HZ		100
//#define	PROCESS_AT_CMD_20HZ		50
//#define	PROCESS_AT_CMD_40HZ		25

/**************************** 全局变量 ****************************************************/
struct cat_object at 				= {0};
KunLun_AEB_Para_Cfg rParms 			= {0};  	// read Copy
KunLun_AEB_Para_Cfg wParms 			= {0};  	// write Copy
uint8_t usart1_At_Buf[AT_BUFFER_SIZE] = {0};	// AT cmd buff

static char write_results[256] 		= {0};
static char ack_results[256] 		= {0};
static uint8_t Id_t, rw_t,para_ti 	= 0;
static uint8_t Id = 0, rw 			= 0;
static uint8_t para_tf[10] 			= {0};
static char const *input_text		= NULL;
static size_t input_index			= 0;
static struct cat_command cmds[];
uint8_t para_i 						= 0;
float para_f 						= 0.0f;
FunctionalState error_t 			= FALSE;	// parameter error flag,out of range
uint8_t valid_range[20] 			= {0};		// valid range warning string
uint8_t algConfigway				= 0;		// alg 参数配置方式：0：USART1串口配置；1：通过USART4蓝牙配置
/*
 * 写AEB算法参数
 */
void Write_AEB_ALG_Para(KunLun_AEB_Para_Cfg aebAlgPara)
{
	uint8_t *data = (uint8_t*)malloc(sizeof(aebAlgPara));

	memcpy(data,&aebAlgPara,sizeof(aebAlgPara));
	Set_AEB_Alg_Parameter(data,aebAlgPara.id);

	free(data);
}
/*
 * 读AEB算法参数
 */
KunLun_AEB_Para_Cfg Read_AEB_ALG_Para()
{
	KunLun_AEB_Para_Cfg algPara = {0};

	uint8_t *data = (uint8_t*)malloc(1024);
	memset(data,0,sizeof(data));
	Get_AEB_Alg_Parameter(data,wParms.id);
	memcpy(&algPara,data,sizeof(algPara));

	free(data);
	return algPara;
}
/*
 * 打印
 */
void Print_AEB_ALG_Para(STREAM* USART_Print)
{
	KunLun_AEB_Para_Cfg algP = Read_AEB_ALG_Para();

	fprintf(USART_Print,"-------------------------------------\r\n");
	fprintf(USART_Print,
			"Global_Switch:%d\r\n"
			"Brake_Cooling_Time:%2.2f\r\n"
			"Driver_Brake_Cooling_Time:%2.2f\r\n"
			"Max_Brake_Keep_Time:%2.2f\r\n"
			"Air_Brake_Delay_Time:%2.2f\r\n"
			"Max_Percent_Decelerate:%2.2f\r\n"
			"Min_Enable_Speed:%2.2f\r\n"
			"Ratio_Force_To_Deceleration:%2.2f\r\n"
			"CMS_HMW_Brake_Time_Thr:%2.2f\r\n"
			"CMS_HMW_Brake_Force_Feel_Para:%2.2f\r\n"
			"CMS_HMW_Warning_Time_Thr:%2.2f\r\n"
			"CMS_HMW_Dynamic_Offset_Speed_L:%2.2f\r\n"
			"CMS_HMW_Dynamic_Offset_Value_L:%2.2f\r\n"
			"CMS_HMW_Dynamic_Offset_Speed_H:%.2f\r\n"
			"CMS_HMW_Dynamic_Offset_Value_H:%2.2f\r\n"
			"CMS_TTC_Brake_Time_Thr:%2.2f\r\n"
			"CMS_TTC_Brake_Force_Feel_Para:%2.2f\r\n"
			"CMS_TTC_Warning_Time_Level_First:%2.2f\r\n"
			"CMS_TTC_Warning_Time_Level_Second:%2.2f\r\n"
			"AEB_Decelerate_Set:%2.2f\r\n"
			"AEB_Stop_Distance:%2.2f\r\n"
			"AEB_TTC_Warning_Time_Level_First:%2.2f\r\n"
			"AEB_TTC_Warning_Time_Level_Second:%2.2f\r\n"
			"SSS_Brake_Force:%2.2f\r\n"
			"SSS_Break_Enable_Distance:%2.2f\r\n"
			"SSS_Warning_Enable_Distance:%2.2f\r\n"
			"SSS_Max_Enable_Speed:%2.2f\r\n"
			"SSS_FR_FL_Install_Distance_To_Side:%2.2f\r\n"
			"SSS_Stop_Distance:%2.2f\r\n"
			"SSS_Default_Turn_Angle:%2.2f\r\n"
			"WheelSpeed_Coefficient:%2.2f\r\n"
			,algP.Global_Switch,algP.Brake_Cooling_Time,algP.Driver_Brake_Cooling_Time,algP.Max_Brake_Keep_Time,algP.Air_Brake_Delay_Time
			,algP.Max_Percent_Decelerate,algP.Min_Enable_Speed,algP.Ratio_Force_To_Deceleration,algP.CMS_HMW_Brake_Time_Thr,algP.CMS_HMW_Brake_Force_Feel_Para,
			algP.CMS_HMW_Warning_Time_Thr,algP.CMS_HMW_Dynamic_Offset_Speed_L,algP.CMS_HMW_Dynamic_Offset_Value_L,algP.CMS_HMW_Dynamic_Offset_Speed_H,
			algP.CMS_HMW_Dynamic_Offset_Value_H,algP.CMS_TTC_Brake_Time_Thr,algP.CMS_TTC_Brake_Force_Feel_Para,algP.CMS_TTC_Warning_Time_Level_First,
			algP.CMS_TTC_Warning_Time_Level_Second,algP.AEB_Decelerate_Set,algP.AEB_Stop_Distance,algP.AEB_TTC_Warning_Time_Level_First,algP.AEB_TTC_Warning_Time_Level_Second,
			algP.SSS_Brake_Force,algP.SSS_Break_Enable_Distance,algP.SSS_Warning_Enable_Distance,algP.SSS_Max_Enable_Speed,algP.SSS_FR_FL_Install_Distance_To_Side,
			algP.SSS_Stop_Distance,algP.SSS_Default_Turn_Angle,algP.WheelSpeed_Coefficient);
	fprintf(USART_Print,"-------------------------------------\r\n");
}

/*
 * AEB_CMS全系统使能--小端模式
 * 8bit 0：ACMS、AEB_TTC功能关闭；1：CMS、AEB_TTC功能开启；
 * 7bit 0：CMS_HMW 缓刹功能关闭； 1： CMS_HMW缓刹功能开启；
 * 6bit 0：CMS_TTC 缓刹功能关闭； 1： CMS_TTC缓刹功能开启；
 * 5bit 0：AEB_TTC 紧急制动功能关闭； 1： AEB_TTC紧急制动功能开启；
 * 4bit 0: SSS起步阻止功能关闭 ；1： SSS起步阻止功能开启
 * 3bit 0：动态HMW刹车阈值关系； 1： 动态HMW刹车阈值功能开启；
 * 2bit 0: 底盘安全策略关闭 ；1： 底盘安全策略开启；
 * 1bit 0：车道偏离预警关闭 ； 1：车道偏离预警开启；
 */
static int Global_Switch(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	wParms.Global_Switch = para_i;
	return 0;
}
/*
 * AEB/CMS 配置参数
 * 刹车冷却时间
 */
static int Brake_Cooling_Time(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=60.0){
		error_t = FALSE;
		wParms.Brake_Cooling_Time = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.0-60.0]");
	}
	return 0;
}
/*
 * AEB/CMS 配置参数
 * 司机介入后冷却时间
 */
static int Driver_Brake_Cooling_Time(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=60.0){
		error_t = FALSE;
		wParms.Driver_Brake_Cooling_Time = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.0-60.0]");
	}
	return 0;
}
/*
 * AEB/CMS 配置参数
 * 最长刹车时长
 */
static int Max_Brake_Keep_Time(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=30.0){
		error_t = FALSE;
		wParms.Max_Brake_Keep_Time = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.0-30.0]");
	}
	return 0;
}
/*
 * AEB/CMS 配置参数
 * 气刹延迟时间
 */
static int Air_Brake_Delay_Time(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=1.0){
		error_t = FALSE;
		wParms.Air_Brake_Delay_Time = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.0-1.0]");
	}
	return 0;
}
/*
 * AEB/CMS 配置参数
 * 最大减速量
 */
static int Max_Percent_Decelerate(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=1.0){
		error_t = FALSE;
		wParms.Max_Percent_Decelerate = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-1.00]");
	}
	return 0;
}
/*
 * AEB/CMS 配置参数
 * AEB_CMS最低启用车速
 */
static int Min_Enable_Speed(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=120.0){
		error_t = FALSE;
		wParms.Min_Enable_Speed = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.0-120.0]");
	}
	return 0;
}
/*
 * AEB/CMS 配置参数
 * 刹车力度系数
 */
static int Ratio_Force_To_Deceleration(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=10.0){
		error_t = FALSE;
		wParms.Ratio_Force_To_Deceleration = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-10.00]");
	}
	return 0;
}
/*
 * CMS/HMW
 * HMW刹车时间阈值
 */
static int CMS_HMW_Brake_Time_Thr(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=2.0){
		error_t = FALSE;
		wParms.CMS_HMW_Brake_Time_Thr = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-2.00]");
	}
	return 0;
}
/*
 * CMS/HMW
 * HMW刹车力度参数（体感）
 */
static int CMS_HMW_Brake_Force_Feel_Para(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=10.0){
		error_t = FALSE;
		wParms.CMS_HMW_Brake_Force_Feel_Para = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-10.00]");
	}
	return 0;
}
/*
 * CMS/HMW
 * HMW预警时间阈值
 */
static int CMS_HMW_Warning_Time_Thr(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=3.0){
		error_t = FALSE;
		wParms.CMS_HMW_Warning_Time_Thr = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-3.00]");
	}
	return 0;
}
/*
 * CMS/HMW
 * HMW刹车时间动态阈值参数1(低速车速)
 */
static int CMS_HMW_Dynamic_Offset_Speed_L(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=10.0 && para_f<=60.0){
		error_t = FALSE;
		wParms.CMS_HMW_Dynamic_Offset_Speed_L = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[10.0-60.0]");
	}
	return 0;
}
/*
 * CMS/HMW
 * HMW刹车时间动态阈值参数2（时间开度）
 */
static int CMS_HMW_Dynamic_Offset_Value_L(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=2.0){
		error_t = FALSE;
		wParms.CMS_HMW_Dynamic_Offset_Value_L = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-2.00]");
	}
	return 0;
}
/*
 * CMS/HMW
 * HMW刹车时间动态阈值参数3（高速车速）
 */
static int CMS_HMW_Dynamic_Offset_Speed_H(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=50.0 && para_f<=120.0){
		error_t = FALSE;
		wParms.CMS_HMW_Dynamic_Offset_Speed_H = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[50.0-120.0]");
	}
	return 0;
}
/*
 * CMS/HMW
 * HMW刹车时间动态阈值参数4（时间开度）
 */
static int CMS_HMW_Dynamic_Offset_Value_H(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=2.0){
		error_t = FALSE;
		wParms.CMS_HMW_Dynamic_Offset_Value_H = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-2.00]");
	}
	return 0;
}
/*
 * CMS/TTC
 * CMS_TTC缓刹刹车时间阈值
 */
static int CMS_TTC_Brake_Time_Thr(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=5.0){
		error_t = FALSE;
		wParms.CMS_TTC_Brake_Time_Thr = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-5.00]");
	}
	return 0;
}
/*
 * CMS/TTC
 * CMS_TTC缓刹固定刹车力度值（体感）
 */
static int CMS_TTC_Brake_Force_Feel_Para(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=10.0){
		error_t = FALSE;
		wParms.CMS_TTC_Brake_Force_Feel_Para = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-10.00]");
	}
	return 0;
}
/*
 * CMS/TTC
 * CMS_TTC缓刹一级预警时间
 */
static int CMS_TTC_Warning_Time_Level_First(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=6.0){
		error_t = FALSE;
		wParms.CMS_TTC_Warning_Time_Level_First = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-6.00]");
	}
	return 0;
}
/*
 * CMS/TTC
 * CMS_TTC缓刹二级预警时间
 */
static int CMS_TTC_Warning_Time_Level_Second(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=6.0){
		error_t = FALSE;
		wParms.CMS_TTC_Warning_Time_Level_Second = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-6.00]");
	}
	return 0;
}
/*
 * AEB
 * 预设刹车减速度
 */
static int AEB_Decelerate_Set(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=10.0){
		error_t = FALSE;
		wParms.AEB_Decelerate_Set = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-10.00]");
	}
	return 0;
}
/*
 * AEB
 * 刹停距离
 */
static int AEB_Stop_Distance(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=10.0){
		error_t = FALSE;
		wParms.AEB_Stop_Distance = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.0-10.00]");
	}
	return 0;
}
/*
 * AEB
 * 紧急制动TTC一级预警时间
 */
static int AEB_TTC_Warning_Time_Level_First(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=5.0){
		error_t = FALSE;
		wParms.AEB_TTC_Warning_Time_Level_First = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-5.00]");
	}
	return 0;
}
/*
 * AEB
 * 紧急制动TTC二级预警时间
 */
static int AEB_TTC_Warning_Time_Level_Second(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=5.0){
		error_t = FALSE;
		wParms.AEB_TTC_Warning_Time_Level_Second = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-5.00]");
	}
	return 0;
}
/*
 * SSS
 * 刹车力度
 */
static int SSS_Brake_Force(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=10.0){
		error_t = FALSE;
		wParms.SSS_Brake_Force = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.0-10.0]");
	}
	return 0;
}
/*
 * SSS
 * 刹车使能距离阈值
 */
static int SSS_Break_Enable_Distance(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=2.5){
		error_t = FALSE;
		wParms.SSS_Break_Enable_Distance = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-2.50]");
	}
	return 0;
}
/*
 * SSS
 * 预警使能距离阈值
 */
static int SSS_Warning_Enable_Distance(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=5.0){
		error_t = FALSE;
		wParms.SSS_Warning_Enable_Distance = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-5.00]");
	}
	return 0;
}
/*
 * SSS
 * 最大有效车速
 */
static int SSS_Max_Enable_Speed(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=40.0){
		error_t = FALSE;
		wParms.SSS_Max_Enable_Speed = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.0-40.0]");
	}
	return 0;
}
/*
 * SSS
 * 前向雷达距离侧面安装距离
 */
static int SSS_FR_FL_Install_Distance_To_Side(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=1.0){
		error_t = FALSE;
		wParms.SSS_FR_FL_Install_Distance_To_Side = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-1.00]");
	}
	return 0;
}
/*
 * SSS
 * 刹停后距离障碍物距离
 */
static int SSS_Stop_Distance(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=1.0){
		error_t = FALSE;
		wParms.SSS_Stop_Distance = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-1.00]");
	}
	return 0;
}
/*
 * SSS
 * 默认转向角度
 */
static int SSS_Default_Turn_Angle(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=60.0){
		error_t = FALSE;
		wParms.SSS_Default_Turn_Angle = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-60.00]");
	}
	return 0;
}
/*
 * Vehicle Speed
 * 轮速调整系数
 */
static int WheelSpeed_Coefficient(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
	if(para_f>=0.0 && para_f<=10000.0){
		error_t = FALSE;
		wParms.WheelSpeed_Coefficient = para_f;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"[0.00-10000.00]");
	}
	return 0;
}
/*
 * 设置所有参数
 */
static int SETALL(const struct cat_command *cmd, const uint8_t *data, const size_t data_size, const size_t args_num)
{
//	fprintf(USART1_STREAM,"para:%d\r\n",para_i);
	// write all parameter
	if(para_i >= 1){
		fprintf(USART1_STREAM,"write parameter.\r\n");
		rParms = wParms;
		Write_AEB_ALG_Para(rParms);
		Set_AEB_Alg_Group_ID(rParms.id);
	}
	return 0;
}

static int Param_Float_Write(const struct cat_variable *var, size_t write_size)
{
	para_f = atof(var->data);
//	fprintf(USART1_STREAM,"float:%2.2f\r\n",para_f);
	return 0;
}

static int Param_Int_Write(const struct cat_variable *var, size_t write_size)
{
	para_i = *(uint8_t*)(var->data);
//	para_i = atoi(var->data);
//	fprintf(USART1_STREAM,"int:%d,data:%s\r\n",para_i,var->data);
	return 0;
}

static int Id_write(const struct cat_variable *var, size_t write_size)
{
	uint8_t id = *(uint8_t*)(var->data);
	if(id>0 && id <=100){
		error_t = FALSE;
		wParms.id = id;
	}else{
		error_t = TRUE;
		strcpy(valid_range,"id [1-100]");
	}
//	fprintf(USART1_STREAM,"Id:%d\r\n",wParms.id);
	return 0;
}

static int Rw_Write(const struct cat_variable *var, size_t write_size)
{
//	wParms.rw_status = *(int8_t*)(var->data);
//	fprintf(USART1_STREAM,"Re:%d\r\n",wParms.rw_status);
	return 0;
}

static struct cat_variable vars_string[] = {
	{
		.type = CAT_VAR_UINT_DEC,
		.data = &Id_t,
		.data_size = sizeof(Id_t),
		.write = Id_write
	},
	{
		.type = CAT_VAR_BUF_STRING,
		.data = para_tf,
		.data_size = sizeof(para_tf),
		.write = Param_Float_Write
	},
	{
		.type = CAT_VAR_UINT_DEC,
		.data = &rw_t,
		.data_size = sizeof(rw_t),
		.write = Rw_Write
	}
};

static struct cat_variable vars_int[] = {
	{
		.type = CAT_VAR_UINT_DEC,
		.data = &Id_t,
		.data_size = sizeof(Id_t),
		.write = Id_write
	},
	{
		.type = CAT_VAR_UINT_DEC,
		.data = &para_ti,
		.data_size = sizeof(para_ti),
		.write = Param_Int_Write
	},
	{
		.type = CAT_VAR_UINT_DEC,
		.data = &rw_t,
		.data_size = sizeof(rw_t),
		.write = Rw_Write
	}
};
static struct cat_command cmds[] = {
	{
		.name = "+Global_Switch",
		.write = Global_Switch,
		.var = vars_int,
		.var_num = sizeof(vars_int) / sizeof(vars_int[0]),
	},
	{
		.name = "+Brake_Cooling_Time",
		.write = Brake_Cooling_Time,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+Driver_Brake_Cooling_Time",
		.write = Driver_Brake_Cooling_Time,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+Max_Brake_Keep_Time",
		.write = Max_Brake_Keep_Time,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+Air_Brake_Delay_Time",
		.write = Air_Brake_Delay_Time,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+Max_Percent_Decelerate",
		.write = Max_Percent_Decelerate,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+Min_Enable_Speed",
		.write = Min_Enable_Speed,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+Ratio_Force_To_Deceleration",
		.write = Ratio_Force_To_Deceleration,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+CMS_HMW_Brake_Time_Thr",
		.write = CMS_HMW_Brake_Time_Thr,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+CMS_HMW_Brake_Force_Feel_Para",
		.write = CMS_HMW_Brake_Force_Feel_Para,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+CMS_HMW_Warning_Time_Thr",
		.write = CMS_HMW_Warning_Time_Thr,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+CMS_HMW_Dynamic_Offset_Speed_L",
		.write = CMS_HMW_Dynamic_Offset_Speed_L,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+CMS_HMW_Dynamic_Offset_Value_L",
		.write = CMS_HMW_Dynamic_Offset_Value_L,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+CMS_HMW_Dynamic_Offset_Speed_H",
		.write = CMS_HMW_Dynamic_Offset_Speed_H,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+CMS_HMW_Dynamic_Offset_Value_H",
		.write = CMS_HMW_Dynamic_Offset_Value_H,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+CMS_TTC_Brake_Time_Thr",
		.write = CMS_TTC_Brake_Time_Thr,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+CMS_TTC_Brake_Force_Feel_Para",
		.write = CMS_TTC_Brake_Force_Feel_Para,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+CMS_TTC_Warning_Time_Level_First",
		.write = CMS_TTC_Warning_Time_Level_First,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+CMS_TTC_Warning_Time_Level_Second",
		.write = CMS_TTC_Warning_Time_Level_Second,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+AEB_Decelerate_Set",
		.write = AEB_Decelerate_Set,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+AEB_Stop_Distance",
		.write = AEB_Stop_Distance,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+AEB_TTC_Warning_Time_Level_First",
		.write = AEB_TTC_Warning_Time_Level_First,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+AEB_TTC_Warning_Time_Level_Second",
		.write = AEB_TTC_Warning_Time_Level_Second,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+SSS_Brake_Force",
		.write = SSS_Brake_Force,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+SSS_Break_Enable_Distance",
		.write = SSS_Break_Enable_Distance,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+SSS_Warning_Enable_Distance",
		.write = SSS_Warning_Enable_Distance,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+SSS_Max_Enable_Speed",
		.write = SSS_Max_Enable_Speed,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+SSS_FR_FL_Install_Distance_To_Side",
		.write = SSS_FR_FL_Install_Distance_To_Side,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+SSS_Stop_Distance",
		.write = SSS_Stop_Distance,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+SSS_Default_Turn_Angle",
		.write = SSS_Default_Turn_Angle,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+WheelSpeed_Coefficient",
		.write = WheelSpeed_Coefficient,
		.var = vars_string,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	},
	{
		.name = "+SETALL",
		.write = SETALL,
		.var = vars_int,
		.var_num = sizeof(vars_string) / sizeof(vars_string[0]),
	}

};

static char buf[AT_BUFFER_SIZE];

static struct cat_command_group cmd_group = {
        .cmd = cmds,
        .cmd_num = sizeof(cmds) / sizeof(cmds[0]),
};

static struct cat_command_group *cmd_desc[] = {
        &cmd_group
};

static struct cat_descriptor desc = {
        .cmd_group = cmd_desc,
        .cmd_group_num = sizeof(cmd_desc) / sizeof(cmd_desc[0]),

        .buf = buf,
        .buf_size = sizeof(buf)
};

static int write_char(char ch)
{
	char str[2];
	str[0] = ch;
	str[1] = 0;
	strcat(ack_results, str);
	return 1;
}

static int read_char(char *ch)
{
	if (input_index >= strlen(input_text))
			return 0;

	*ch = input_text[input_index];
	input_index++;
	return 1;
}

static struct cat_io_interface iface = {
	.read = read_char,
	.write = write_char
};

static void prepare_input(const char *text)
{
	input_text = text;
	input_index = 0;
	Id = 0;
	rw = 0;
	memset(ack_results, 0, sizeof(ack_results));
	memset(write_results, 0, sizeof(write_results));
}

//static const char test_case_1[] = "AT+Brake_Cooling_Time=1,\"1.23\",1\r\n";
//static const char test_case_2[] = "AT+Global_Switch=1,66,2\n";
void Init_AEB_Alg_Para()
{
	rParms.id = 1;
	rParms.Global_Switch = 0.0;
	rParms.Brake_Cooling_Time = 0.0;
	rParms.Driver_Brake_Cooling_Time = 0.0;
	rParms.Max_Brake_Keep_Time = 0.0;
	rParms.Air_Brake_Delay_Time = 0.0;
	rParms.Max_Percent_Decelerate = 0.0;
	rParms.Min_Enable_Speed = 0.0;
	rParms.Ratio_Force_To_Deceleration = 0.0;
	rParms.CMS_HMW_Brake_Time_Thr = 0.0;
	rParms.CMS_HMW_Brake_Force_Feel_Para = 0.0;
	rParms.CMS_HMW_Warning_Time_Thr = 0.0;
	rParms.CMS_HMW_Dynamic_Offset_Speed_L = 0.0;
	rParms.CMS_HMW_Dynamic_Offset_Value_L = 0.0;
	rParms.CMS_HMW_Dynamic_Offset_Speed_H = 0.0;
	rParms.CMS_HMW_Dynamic_Offset_Value_H = 0.0;
	rParms.CMS_TTC_Brake_Time_Thr = 0.0;
	rParms.CMS_TTC_Brake_Force_Feel_Para = 0.0;
	rParms.CMS_TTC_Warning_Time_Level_First = 0.0;
	rParms.CMS_TTC_Warning_Time_Level_Second = 0.0;
	rParms.AEB_Decelerate_Set = 0.0;
	rParms.AEB_Stop_Distance = 0.0;
	rParms.AEB_TTC_Warning_Time_Level_First = 0.0;
	rParms.AEB_TTC_Warning_Time_Level_Second = 0.0;
	rParms.SSS_Brake_Force = 0.0;
	rParms.SSS_Break_Enable_Distance = 0.0;
	rParms.SSS_Warning_Enable_Distance = 0.0;
	rParms.SSS_Max_Enable_Speed = 0.0;
	rParms.SSS_FR_FL_Install_Distance_To_Side = 0.0;
	rParms.SSS_Stop_Distance = 0.0;
	rParms.SSS_Default_Turn_Angle = 0.0;
	rParms.WheelSpeed_Coefficient = 0.0;

	wParms = rParms;
}
void AT_Init()
{
	cat_init(&at, &desc, &iface, NULL);
	Init_AEB_Alg_Para();
	rParms.id = Get_AEB_Alg_Group_ID();
	if(rParms.id == 0xFF) rParms.id = 1;
	rParms = Read_AEB_ALG_Para();		// start read AEB ALG parameter
	wParms = rParms;
}

uint32_t at_clk = 0;
void Analysis_AT_CMD_InMainLoop()
{
	if(SystemtimeClock - at_clk < 120) return ;
	at_clk = SystemtimeClock;

    if(strlen(usart1_At_Buf) > 10){	// valid data
    	// use USART1/BT Print info
    	STREAM *USART_Print = NULL;
    	switch(algConfigway){
    	case 1:USART_Print = USART4_STREAM;break;	// BT
    	default:USART_Print = USART1_STREAM;break;
    	}

    	if(strstr((const char*)usart1_At_Buf,(const char*)"ZKHYCHK*RALGPARA")){
    		Print_AEB_ALG_Para(USART_Print);
    	}else{
			// analysis
			prepare_input(usart1_At_Buf);
			while (cat_service(&at) != 0) {};
			// feedback
			if(error_t){
				error_t = FALSE;
				fprintf(USART_Print,"ERROR,valid range:%s\r\n",valid_range);
				algConfigway = 0;
				memset(valid_range,0,sizeof(valid_range));
			}else if(strstr(ack_results,"OK")){
				fprintf(USART_Print,"OK\r\n");
				algConfigway = 0;
			}
    	}
        // clear buffer
    	memset(usart1_At_Buf,0,AT_BUFFER_SIZE);
    }
}


uint8_t btBuf[AT_BUFFER_SIZE] = {0};
uint8_t btCnt	= 0;
void HC02_BT_Analysis_Alg_Para(uint8_t data)
{
	btBuf[btCnt] = data;
	char *strx = NULL;
	if(btBuf[btCnt-1]==',' && (btBuf[btCnt]=='0'|| btBuf[btCnt]=='1') || btBuf[btCnt]=='>'){//
		if((strx = strstr(btBuf,"AT+")) || (strx = strstr(btBuf,"ZKHYCHK*RALGPARA"))){	// 分出AT指令
			algConfigway = 1;
			sprintf(usart1_At_Buf,"%s\r\n",strx);
//			fprintf(USART4_STREAM,"test:%s",usart1_At_Buf);
			memset(btBuf,0,sizeof(btBuf));
			btCnt = 0;
			return ;
		}
	}else{
		btCnt ++;
	}
}
