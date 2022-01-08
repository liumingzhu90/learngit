/*
 * AEB_CMS_alg.h
 *
 *  Created on: 2021-11-25
 *  Author: xujie
 */

#ifndef AEB_CMS_ALG_H_
#define AEB_CMS_ALG_H_
#include "proportional_valve.h"
#include "aebs_task.h"
#include "vehicle_can.h"
#include "common.h"
#include "gpio.h"
#include "aeb_cms_sss_para_config.h"
// define struct of CMS config parameter
typedef struct{
	float cms_uradar_dec_out_prev;
	float cms_uradar_dec_out;
	float cms_hmw_dec_out;
	float cms_hmw_dec_out_prev;
	float cms_ttc_dec_out_prev;
	float cms_ttc_dec_out;
	float cam_distance;
	float uradar_distance;

	uint8_t cms_is_warning_flag;
	uint8_t cms_is_break_flag;
	uint8_t cms_cam_keep_count;
	uint8_t cms_cam_delay_count;
	uint32_t cms_cam_over_timestamp;
	uint8_t cms_uradar_keep_count;
	uint8_t cms_uradar_delay_count;

	uint8_t cms_ttc_is_warning_flag;
	uint8_t cms_ttc_is_break_flag;
	uint8_t cms_ttc_cam_keep_count;
	uint8_t cms_ttc_cam_delay_count;
	uint8_t cms_ttc_uradar_keep_count;
	uint8_t cms_ttc_uradar_delay_count;

}CMS_Break_State;
typedef struct{
	float TTC_AEB;
	float TTC_AEB_dec_out;
	uint8_t aeb_TTC_is_break_flag;

}AEB_Break_State;
typedef struct{
	////////////////////////////////////////////////////
	// TOthers
	float max_enable_speed;    //km/h
	// recal HWM and TTC warning Level
	float Cam_HMW_warning_time; //too close to lead car
	float Cam_TTC_warning_1_time; //
	float Cam_TTC_warning_2_time; //
	// Vehicle parameters
	float ttc_max_dec_output;
	float Vehicle_max_react_dec;
	float Vehicle_min_react_dec;
	// parameters of CIPV
	float cipv_max_distance;
	uint8_t min_track_number;
	// Parameters of Uradar
	float Uradar_max_enable_speed;
}AEB_CMS_Control_Parameters;

enum LOG_LEVEL
{
    LOG_NONE  = 0x00,
    LOG_ERROR  = 0x01,
    LOG_WARNING  = 0x02,
    LOG_DEBUG  = 0x03,
    LOG_IN_OUT = 0x04
};

void Got_Vehicle_Para_form_GPIO(_VEHICLE_PARA* stVehicleParas);


float AEB_CMS_Break_control(uint8_t got_cipv,
							Obstacle_Basic_Data* obs_basic_data,
							Obstacle_Information* cipv,
							_VEHICLE_PARA* stVehicleParas,
							Camera_Essential_Data* cam_essiential,
							URADER_MESSAGE	Urader_Company_Message[]);
float AEB_TTC_Break(Obstacle_Information* cipv,
					Obstacle_Basic_Data* obs_basic_data,
					_VEHICLE_PARA* stVehicleParas);
float CMS_TTC_Break(Obstacle_Information* cipv,
					Obstacle_Basic_Data* obs_basic_data,
					_VEHICLE_PARA* stVehicleParas,
					Camera_Essential_Data* cam_essiential,
					URADER_MESSAGE Urader_Company_Message[]);
float CMS_HMW_Break(Obstacle_Information* cipv,
					Obstacle_Basic_Data* obs_basic_data,
					_VEHICLE_PARA* stVehicleParas,
					Camera_Essential_Data* cam_essiential,
					URADER_MESSAGE Urader_Company_Message[]);
float CMS_URadar_Break(_VEHICLE_PARA* stVehicleParas,
					URADER_MESSAGE Urader_Company_Message[]);
float Cal_TTC(Obstacle_Information* cipv);
uint8_t Approaching_to_lead_car(Obstacle_Information* cipv, uint8_t size_of_history);
uint8_t Approaching_to_lead_car_uradar(float diatance, uint8_t size_of_history);
uint8_t Approaching_to_target(uint8_t size_of_history, float distance, float distance_resived[], uint32_t distance_timestamp[],uint8_t* num_count);

uint8_t Break_Trigger(Obstacle_Information* cipv, _VEHICLE_PARA* stVehicleParas, Camera_Essential_Data* cam_essiential,Obstacle_Basic_Data* obs_basic_data);
uint8_t Break_Cancel(Obstacle_Information* cipv, _VEHICLE_PARA* stVehicleParas,Camera_Essential_Data* cam_essiential,Obstacle_Basic_Data* obs_basic_data);
uint8_t Driver_control(Obstacle_Information* cipv, _VEHICLE_PARA* stVehicleParas,Camera_Essential_Data* cam_essiential,Obstacle_Basic_Data* obs_basic_data);
void AEB_State_reset(void);
void CMS_State_reset(void);
void CMS_HMW_State_reset(void);
void CMS_TTC_State_reset(void);
void AEB_CMS_alg_init(void);
float Get_Uradar_distance(URADER_MESSAGE Urader_Company_Message[],_VEHICLE_PARA* stVehicleParas);
void Filter_noise(float *dec_out, float dec_out_k,uint8_t *is_break_flag, uint8_t *count_delay, uint8_t *count_keep);
void Log_print(enum LOG_LEVEL level, uint8_t cancel_n, uint8_t func_n, const char* description, const char* function_name, Obstacle_Information* cipv,_VEHICLE_PARA* stVehicleParas,Camera_Essential_Data* Cam_Essi_data,Obstacle_Basic_Data* obs_basic_data);
//test

float  Ret_test(void);


#endif /* AEB_CMS_ALG_H_ */
