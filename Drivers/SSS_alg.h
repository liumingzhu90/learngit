/*
 * SSS_alg.h
 *
 *  Created on: 2021-11-26
 *      Author: zkhy
 */

#ifndef SSS_ALG_H_
#define SSS_ALG_H_
#include "proportional_valve.h"
#include "aebs_task.h"
#include "vehicle_can.h"
#include "common.h"
#include "gpio.h"
#define sss_uradar_delay_count 5

float sss_dec_output;
/* SSS */
typedef struct{
	float break_ttc_thr;/*1.5s?*/
	float break_distance_thr;/*0.8m*/
	float warning_distance_thr;/*2.0m*/
	float dec_set;/*big force*/
	float break_delay_time;/*0.6s*/
	float max_effective_speed;/*20km/h*/

	float FL_FR_install_distance_to_side;
	float stop_distance;
	float uradar_delay_time;
	float turn_angle;

}SSS_parameters;
typedef struct{
	uint8_t is_break_flag;/*0.8m*/
	uint8_t send_break_delay_count;/*2.0m*/

	uint8_t dec_output;
	uint8_t warning_output;

}SSS_state;
typedef struct{
	float sss_uradar_history_distance[2][12][5];
	uint32_t sss_uradar_history_distance_timestamp[2][12][5];
	uint8_t sss_uradar_history_distance_count[2][12];
	uint8_t sss_uradar_location[2][12];
	uint8_t sss_uradar_available_num;
}SSS_uradar_history;

void sss_init(void);

float SSS_Break_control(_VEHICLE_PARA stVehicleParas,
						URADER_MESSAGE	Urader_Company_Message[]);


uint8_t SSS_Break_Trigger(_VEHICLE_PARA stVehicleParas);
uint8_t SSS_driver_control(_VEHICLE_PARA stVehicleParas);
float sss_cal_ttc(uint8_t size_of_history, float distance_resived[], uint32_t distance_timestamp[], uint8_t num_count);
uint8_t SSS_Approaching_to_target(uint8_t size_of_history, float distance, float distance_resived[], uint32_t distance_timestamp[],uint8_t* num_count);
float Get_Uradar_break_pressure_ttc(URADER_MESSAGE urader_company_message[], _VEHICLE_PARA stVehicleParas);
float Get_Uradar_break_pressure_distance(URADER_MESSAGE urader_company_message[], _VEHICLE_PARA stVehicleParas);
void Uradar_test(URADER_MESSAGE urader_company_message[]);
#endif /* SSS_ALG_H_ */
