/*
 * SSS_alg.c
 *
 *  Created on: 2021-11-26
 *      Author: zkhy
 */
#include "sss_alg.h"

#define speed_meter_p_s(v)            (v / 3.6)
#define sss_break_calculate_cancel      0x00
#define sss_break_calculate_continue    0x01
#define sss_history_size   3
#define sss_warning   0x01


SSS_parameters sss_param;
SSS_state sss_state;
SSS_uradar_history uradar_history;

float dec_out_sss;

extern URADER_MESSAGE	Urader_Company_Message[2];



void sss_init(void){
	sss_param.break_delay_time = 0.5;/*s*/
	sss_param.break_distance_thr = 0.8;/*s*/
	sss_param.dec_set = 5.0;/**/
	sss_param.max_effective_speed = 30;/*km/h*/
	sss_param.warning_distance_thr = 2.0;/**/
	sss_param.break_ttc_thr = 1.5;
	sss_param.FL_FR_install_distance_to_side = 0.3;
	sss_param.stop_distance = 0.3;
	sss_param.uradar_delay_time = 0.1;
	sss_param.turn_angle = 30.0;

}
float calculate_cos(float angle){
	return -0.0001391*angle*angle - 0.0003124*angle + 1.001;
}
float calculate_sin(float angle){
	return angle > 0.0? 0.0159*angle : 0.0;
}

uint8_t Approaching_all_location(uint8_t i, uint8_t j, float distance){

	return SSS_Approaching_to_target(sss_history_size, distance, uradar_history.sss_uradar_history_distance[i][j], uradar_history.sss_uradar_history_distance_timestamp[i][j],&(uradar_history.sss_uradar_history_distance_count[i][j]));

}

float SSS_Break_control(_VEHICLE_PARA stVehicleParas,
						URADER_MESSAGE	Urader_Company_Message[]){
	uint8_t break_or_not = SSS_Break_Trigger(stVehicleParas);
	if(break_or_not == sss_break_calculate_cancel){
		return 0.0;
	}
	// calculate decoutput
	dec_out_sss = Get_Uradar_break_pressure_distance(Urader_Company_Message, stVehicleParas);
	// break_cancel
	break_or_not = SSS_driver_control(stVehicleParas);
	if(break_or_not == sss_break_calculate_cancel){
		dec_out_sss = 0.0;
	}
	sss_state.dec_output = dec_out_sss;
	return sss_state.dec_output;
}
uint8_t SSS_Break_Trigger(_VEHICLE_PARA stVehicleParas){
	if((stVehicleParas.fVehicleSpeed <= 0)|(stVehicleParas.fVehicleSpeed > sss_param.max_effective_speed)){
		return sss_break_calculate_cancel;
	}
	if(stVehicleParas.ReverseGear == 1 ){
		return sss_break_calculate_cancel;
	}
	return sss_break_calculate_continue;
}
uint8_t SSS_driver_control(_VEHICLE_PARA stVehicleParas){
	if(stVehicleParas.BrakeFlag == 1 ){
		//return sss_break_calculate_cancel;
	}
	return sss_break_calculate_continue;
}
uint8_t SSS_Approaching_to_target(uint8_t size_of_history, float distance, float distance_resived[], uint32_t distance_timestamp[],uint8_t* num_count){
	uint8_t double_size = (size_of_history*2 - 1);
	// recored the history data
	if(*num_count < size_of_history){
		distance_resived[*num_count] = distance;
		if(distance_timestamp != NULL)
			distance_timestamp[*num_count] = SystemtimeClock;
		*num_count = *num_count + 1;
	}
	else{
		distance_resived[*num_count - size_of_history] = distance;
		if(distance_timestamp != NULL)
			distance_timestamp[*num_count] = SystemtimeClock;
		*num_count = *num_count + 1;
		if(*num_count > double_size){
			*num_count = size_of_history;
		}
	}
	// judge approaching or not
	if(*num_count >= size_of_history){
		uint8_t now_pointer = *num_count - 1;
		if(now_pointer < size_of_history)
			now_pointer = double_size;

		uint8_t middle_pointer = now_pointer - 1;
		if(middle_pointer < size_of_history)
			middle_pointer = double_size;
		middle_pointer = middle_pointer - 1;
		if(middle_pointer < size_of_history)
			middle_pointer = double_size;

		uint8_t old_pointer = *num_count;
		if((distance_resived[now_pointer - size_of_history] < distance_resived[old_pointer - size_of_history])){
			return 0x01;// approaching
		}
		else{
			return 0x01;//not approaching
		}

		/*if((distance_resived[now_pointer - size_of_history] < distance_resived[middle_pointer - size_of_history])&(distance_resived[middle_pointer - size_of_history] < distance_resived[old_pointer - size_of_history])){
			return 0x01;// approaching
		}
		else if((distance_resived[now_pointer - size_of_history] > distance_resived[old_pointer - size_of_history])&(distance_resived[middle_pointer - size_of_history] > distance_resived[old_pointer - size_of_history])){
			return 0x00;// not approaching
		}
		else {
			if((distance_resived[now_pointer - size_of_history] < distance_resived[middle_pointer - size_of_history])){
				return 0x01;// approaching
			}
			else if((distance_resived[now_pointer - size_of_history] > distance_resived[old_pointer - size_of_history])){
				return 0x00;// not approaching
			}
			else{
				return 0x02; // can not judge
			}
		}*/
	}
	else{
		return 0x02; // can not judge
	}
}

float sss_cal_ttc(uint8_t size_of_history, float distance_resived[], uint32_t distance_timestamp[], uint8_t num_count){
	uint8_t double_size = size_of_history * 2 - 1;
	float invalid_value = 10.0;
	if(num_count >= size_of_history){
		uint8_t now_pointer = num_count - 1;
		if(now_pointer < size_of_history)
			now_pointer = double_size;

		uint8_t old_pointer = num_count;
		float deta_d = distance_resived[old_pointer - size_of_history] - distance_resived[now_pointer - size_of_history];
		if(deta_d > 0.0){
			float deta_t = (distance_timestamp[now_pointer - size_of_history] - distance_timestamp[old_pointer - size_of_history])/1000;
			if(deta_t > 0.0){
				float deta_v = deta_d / deta_t;
				float ttc = distance_resived[now_pointer - size_of_history] / deta_v;
				if(ttc > invalid_value)
					ttc = invalid_value;
				return ttc;
			}
			else{
				return invalid_value;
			}
		}
		else{
			return invalid_value;
		}
	}
	else{
		return invalid_value; // can not judge
	}
}
float Get_Uradar_break_pressure_ttc(URADER_MESSAGE urader_company_message[], _VEHICLE_PARA stVehicleParas)
{
	float udistance = 0;
	uint8_t double_size = sss_history_size*2 - 1;
	for(uint8_t i = 0;i < 2;i ++)
	{
		for(uint8_t j = 0;j < 12;j ++)
		{
			if(urader_company_message[i].Urader_work_stata[j] == URWORK)
			{
				udistance = urader_company_message[i].distance[j];
				if(udistance > sss_param.warning_distance_thr){
					sss_state.warning_output = 0x00;
					uradar_history.sss_uradar_history_distance_count[i][j] = 0;
					continue;
				}
				else{
					if(udistance > sss_param.break_distance_thr){
						sss_state.warning_output = 0x00;
						continue;
					}
					//warning and jude if break
					sss_state.warning_output = 0x01;

					uint8_t p = uradar_history.sss_uradar_history_distance_count[i][j];
					if(p > sss_history_size){
						p = p - 1;
						if(p < sss_history_size)
							p = double_size;
						p = p - sss_history_size;
					}
					 if(uradar_history.sss_uradar_history_distance[i][j][p] == udistance){
						 continue;
					 }
					 else{
						 if(uradar_history.sss_uradar_location[i][j] != urader_company_message[i].Uposition[j]){
							 uradar_history.sss_uradar_location[i][j] = urader_company_message[i].Uposition[j];
						 }
						 uint8_t get_back = SSS_Approaching_to_target(sss_history_size, udistance, uradar_history.sss_uradar_history_distance[i][j], uradar_history.sss_uradar_history_distance_timestamp[i][j], &uradar_history.sss_uradar_history_distance_count[i][j]);
						 if(get_back == 1){
							 float ttc = sss_cal_ttc(sss_history_size, uradar_history.sss_uradar_history_distance[i][j], uradar_history.sss_uradar_history_distance_timestamp[i][j], uradar_history.sss_uradar_history_distance_count[i][j]);
							 if((ttc > 0.0)&(ttc <= sss_param.break_ttc_thr)){
								 float dec_out = udistance / ttc;
								 return  dec_out;
							 }
						 }
					 }
				}
			}
		}
	}

	return 0.0;
}
uint8_t approaching_or_not  = 0x00;
float distance_fixed 		= 0.0;
float speed_ego      		= 0.0;
float break_distance 		= 0.0;
float Get_Uradar_break_pressure_distance(URADER_MESSAGE urader_company_message[], _VEHICLE_PARA stVehicleParas){
	distance_fixed;
	speed_ego = speed_meter_p_s(stVehicleParas.fVehicleSpeed);
	break_distance = 0.5*speed_ego*speed_ego/sss_param.dec_set;
	break_distance = break_distance + speed_ego * sss_param.break_delay_time;
	break_distance = break_distance + sss_param.stop_distance;

	for(uint8_t i = 0;i < 2;i ++)
	{
		for(uint8_t j = 0;j < 12;j ++)
		{
			if(urader_company_message[i].Urader_work_stata[j] == URWORK)
			{
				//1. go stright
				if((stVehicleParas.LeftFlagTemp == 0)&(stVehicleParas.RightFlagTemp == 0)){
					if(FRONT == urader_company_message[i].Uposition[j] ||
					   FRMIDE == urader_company_message[i].Uposition[j] ||
					   FLMIDE == urader_company_message[i].Uposition[j] ||
					   FRSIDE == urader_company_message[i].Uposition[j]){
						distance_fixed = urader_company_message[i].distance[j] - speed_ego * sss_param.uradar_delay_time;
						approaching_or_not = Approaching_all_location(i, j, urader_company_message[i].distance[j]);
						if(distance_fixed <= sss_param.break_distance_thr){
							fprintf(USART1_STREAM,"sss_param.break_distance_thr:%d,%0.2f\r\n",approaching_or_not,speed_ego*3.6);
							if(approaching_or_not == 0x01){
								if((distance_fixed < break_distance)||(distance_fixed < sss_param.stop_distance)){
									return sss_param.dec_set;
								}
							}
						}
					}
					if(FLEFT == urader_company_message[i].Uposition[j] ||
					   FRIGHT == urader_company_message[i].Uposition[j])
					{
						distance_fixed = urader_company_message[i].distance[j] - speed_ego * sss_param.uradar_delay_time;
						approaching_or_not = Approaching_all_location(i, j, urader_company_message[i].distance[j]);
						if(distance_fixed <= sss_param.break_distance_thr){
							if(approaching_or_not == 0x01){
								if((distance_fixed < sss_param.FL_FR_install_distance_to_side)||(distance_fixed < sss_param.stop_distance)){
									return sss_param.dec_set;
								}
							}
						}
					}
				}//2. turn right
				else if((stVehicleParas.LeftFlagTemp == 0)&(stVehicleParas.RightFlagTemp == 1))
				{
					float component_cos_turn = calculate_cos(sss_param.turn_angle);
					if(FRONT == urader_company_message[i].Uposition[j] ||
					   FRIGHT == urader_company_message[i].Uposition[j] ||
					   FRMIDE == urader_company_message[i].Uposition[j] ||
					   FLMIDE == urader_company_message[i].Uposition[j] )
					{
						distance_fixed = urader_company_message[i].distance[j] - speed_ego * sss_param.uradar_delay_time;
						approaching_or_not = Approaching_all_location(i, j, urader_company_message[i].distance[j]);

						if(distance_fixed <= sss_param.break_distance_thr){
							if(approaching_or_not == 0x01){
								if((distance_fixed < break_distance * component_cos_turn)||(distance_fixed < sss_param.stop_distance)){
									return sss_param.dec_set;
								}
							}
						}

					}
					if(FRSIDE == urader_company_message[i].Uposition[j])
					{
						distance_fixed = urader_company_message[i].distance[j] - speed_ego * sss_param.uradar_delay_time;
						approaching_or_not = Approaching_all_location(i, j, urader_company_message[i].distance[j]);

						if(distance_fixed <= sss_param.break_distance_thr){
							if(approaching_or_not == 0x01){
								if((distance_fixed < break_distance)||(distance_fixed < sss_param.stop_distance)){
									return sss_param.dec_set;
								}
							}
						}
					}
					if(RIGHT == urader_company_message[i].Uposition[j])
					{
						distance_fixed = urader_company_message[i].distance[j] - speed_ego * sss_param.uradar_delay_time;
						approaching_or_not = Approaching_all_location(i, j, urader_company_message[i].distance[j]);

						if(distance_fixed <= sss_param.break_distance_thr){
							if(approaching_or_not == 0x01){
								if((distance_fixed < break_distance)||(distance_fixed < sss_param.stop_distance)){
									return sss_param.dec_set;
								}
							};
						}


					}
					if(FLEFT == urader_company_message[i].Uposition[j])
					{
						distance_fixed = urader_company_message[i].distance[j] - speed_ego * sss_param.uradar_delay_time;
						approaching_or_not = Approaching_all_location(i, j, urader_company_message[i].distance[j]);

						if(distance_fixed <= sss_param.break_distance_thr){
							if(approaching_or_not == 0x01){
								if((distance_fixed < sss_param.FL_FR_install_distance_to_side)||(distance_fixed < sss_param.stop_distance)){
									return sss_param.dec_set;
								}
							}
						}


					}

				}//3. turn left
				else if((stVehicleParas.LeftFlagTemp == 1)&(stVehicleParas.RightFlagTemp == 0)){
					float component_cos_turn = calculate_cos(sss_param.turn_angle);
					if(FRONT == urader_company_message[i].Uposition[j] ||
					   FLEFT == urader_company_message[i].Uposition[j] ||
					   FRMIDE == urader_company_message[i].Uposition[j] ||
					   FLMIDE == urader_company_message[i].Uposition[j] )
					{
						distance_fixed = urader_company_message[i].distance[j] - speed_ego * sss_param.uradar_delay_time;
						approaching_or_not = Approaching_all_location(i, j, urader_company_message[i].distance[j]);

						if(distance_fixed <= sss_param.break_distance_thr){
							if(approaching_or_not == 0x01){
								if((distance_fixed < break_distance * component_cos_turn)||(distance_fixed < sss_param.stop_distance)){
									return sss_param.dec_set;
								}
							}
						}

					}
					if(FLSIDE == urader_company_message[i].Uposition[j])
					{
						distance_fixed = urader_company_message[i].distance[j] - speed_ego * sss_param.uradar_delay_time;
						approaching_or_not = Approaching_all_location(i, j, urader_company_message[i].distance[j]);

						if(distance_fixed <= sss_param.break_distance_thr){
							if(approaching_or_not == 0x01){
								if((distance_fixed < break_distance)||(distance_fixed < sss_param.stop_distance)){
									return sss_param.dec_set;
								}
							}
						}


					}
					if(LEFT == urader_company_message[i].Uposition[j])
					{
						distance_fixed = urader_company_message[i].distance[j] - speed_ego * sss_param.uradar_delay_time;
						approaching_or_not = Approaching_all_location(i, j, urader_company_message[i].distance[j]);

						if(distance_fixed <= sss_param.break_distance_thr){
							if(approaching_or_not == 0x01){
								if((distance_fixed < break_distance)||(distance_fixed < sss_param.stop_distance)){
									return sss_param.dec_set;
								}
							}
						}


					}
					if(FRIGHT == urader_company_message[i].Uposition[j])
					{
						distance_fixed = urader_company_message[i].distance[j] - speed_ego * sss_param.uradar_delay_time;
						approaching_or_not = Approaching_all_location(i, j, urader_company_message[i].distance[j]);

						if(distance_fixed <= sss_param.break_distance_thr){
							if(approaching_or_not == 0x01){
								if((distance_fixed < sss_param.FL_FR_install_distance_to_side)||(distance_fixed < sss_param.stop_distance)){
									return sss_param.dec_set;
								}
							}
						}
					}
				}
			}
		}
	}

}


static float distance_FRONT;
static float distance_FRIGHT;
static float distance_FLEFT;
static float distance_FRMIDE;
static float distance_FLMIDE;
static float distance_FRSIDE;
static float distance_FLSIDE;
static float distance_RIGHT;
static float distance_LEFT;
void Uradar_test(URADER_MESSAGE urader_company_message[]){
	for(uint8_t i = 0;i < 2;i ++)
	{
		for(uint8_t j = 0;j < 12;j ++)
		{
			if(urader_company_message[i].Urader_work_stata[j] == URWORK)
			{
				enum _URADER_POSITION Now_position = urader_company_message[i].Uposition[j];
				switch(Now_position){
				case FRONT:
					distance_FRONT  = urader_company_message[i].distance[j];
									break;
				case FRIGHT:
					distance_FRIGHT = urader_company_message[i].distance[j];
									break;
				case FLEFT:
					distance_FLEFT  = urader_company_message[i].distance[j];
									break;
				case FRMIDE:
					distance_FRMIDE = urader_company_message[i].distance[j];
									break;
				case FLMIDE:
					distance_FLMIDE = urader_company_message[i].distance[j];
									break;
				case FRSIDE:
					distance_FRSIDE = urader_company_message[i].distance[j];
									break;
				case FLSIDE:
					distance_FLSIDE = urader_company_message[i].distance[j];
									break;
				case LEFT:
					distance_LEFT   = urader_company_message[i].distance[j];
									break;
				case RIGHT:
					distance_RIGHT  = urader_company_message[i].distance[j];
									break;
				}

			}

		}
	}
	fprintf(USART1_STREAM,"URADAR:F=%0.2f,FR=%0.2f,FL=%0.2f,FRM=%0.2f,FLM=%0.2f,FRS=%0.2f,FLS=%0.2f,R=%0.2f,L=%0.2f \r\n", \
				distance_FRONT,distance_FRIGHT,distance_FLEFT,distance_FRMIDE,distance_FLMIDE,distance_FRSIDE,distance_FLSIDE,distance_RIGHT,distance_LEFT);
}
