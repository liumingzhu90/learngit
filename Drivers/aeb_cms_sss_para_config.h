/*
 * aeb_cms_sss_para_config.h
 *
 *  Created on: 2021-12-29
 *      Author: zkhy
 */

#ifndef AEB_CMS_SSS_PARA_CONFIG_H_
#define AEB_CMS_SSS_PARA_CONFIG_H_
#include "common.h"

typedef struct {
	// Version Number
	uint8_t id;
	// Status of writing or reading
//	uint8_t rw_status; // 0:reading ,can not writing; 1:read over,can write;2:writing, can not read; 3: write over can read
	// Global Function Switch
	uint8_t Global_Switch;
	// AEB¡¢CMS common parameters
	float Brake_Cooling_Time;
	float Driver_Brake_Cooling_Time;
	float Max_Brake_Keep_Time;
	float Air_Brake_Delay_Time;
	float Max_Percent_Decelerate;
	float Min_Enable_Speed;
	float Ratio_Force_To_Deceleration;
	// CMS parameters
	// CMS parameters - HMW
	float CMS_HMW_Brake_Time_Thr;
	float CMS_HMW_Brake_Force_Feel_Para;
	float CMS_HMW_Warning_Time_Thr;
	float CMS_HMW_Dynamic_Offset_Speed_L;
	float CMS_HMW_Dynamic_Offset_Value_L;
	float CMS_HMW_Dynamic_Offset_Speed_H;
	float CMS_HMW_Dynamic_Offset_Value_H;
	// CMS parameters - TTC
	float CMS_TTC_Brake_Time_Thr;
	float CMS_TTC_Brake_Force_Feel_Para;
	float CMS_TTC_Warning_Time_Level_First;
	float CMS_TTC_Warning_Time_Level_Second;
	// AEB
	float AEB_Decelerate_Set;
	float AEB_Stop_Distance;
	float AEB_TTC_Warning_Time_Level_First;
	float AEB_TTC_Warning_Time_Level_Second;
	// SSS
	float SSS_Brake_Force;
	float SSS_Break_Enable_Distance;
	float SSS_Warning_Enable_Distance;
	float SSS_Max_Enable_Speed;
	float SSS_FR_FL_Install_Distance_To_Side;
	float SSS_Stop_Distance;
	float SSS_Default_Turn_Angle;
	// Vehicle Speed
	float WheelSpeed_Coefficient;
}KunLun_AEB_Para_Cfg;

extern KunLun_AEB_Para_Cfg wParams; // "write Copy"
extern KunLun_AEB_Para_Cfg rParms;  // "read Copy"

#endif /* AEB_CMS_SSS_PARA_CONFIG_H_ */
