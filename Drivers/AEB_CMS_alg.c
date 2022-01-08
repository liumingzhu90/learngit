/*
 * AEB_CMS_alg.c
 *
 *  Created on: 2021-11-25
 *  Author: xujie
 */
#include "aeb_cms_alg.h"
#include "stereo_camera.h"
#include "wheelSpeed.h"

/**********************************************************
 * Public:
 * Share with Outside Function
 * ********************************************************
 */
/////////////////////////////////////////////////////////
//System Config Set From GPIO
/////////////////////////////////////////////////////////
uint8_t g_AEB_ON_OFF_GPIO = 0x01;

/////////////////////////////////////////////////////////
//System Config Set From Displayer
/////////////////////////////////////////////////////////
uint8_t g_AEB_ON_OFF_Displayer = 0x01;
uint8_t g_LDW_ON_OFF_Displayer = 0x01;// Just use in Displayer

/////////////////////////////////////////////////////////
//System Config Parameters, Set by FAE
/////////////////////////////////////////////////////////
uint8_t g_AEB_ON_OFF_BT = 0x01;
uint8_t g_LDW_ON_OFF_BT = 0x01;       // Just use in Displayer
uint8_t g_SSS_ON_OFF_BT = 0x01;

/////////////////////////////////////////////////////////
//System Config Parameters, Set by User by Cam Displayer
/////////////////////////////////////////////////////////
float g_HMW_Warning_Time_From_Cam_Displayer;

/////////////////////////////////////////////////////////
//Parameters for outside function reading(just reading)
/////////////////////////////////////////////////////////
//-----Brake Flag
float g_AEB_CMS_outside_dec_output       = 0.0;
uint8_t g_AEB_CMS_outside_BrakeBy_CMSHMW = 0x00;
uint8_t g_AEB_CMS_outside_BrakeBy_CMSTTC = 0x00;
uint8_t g_AEB_CMS_outside_BrakeBy_AEBTTC = 0x00;
//-----Middle Status
uint8_t g_AEB_CMS_approcahing = 0x00;
uint8_t g_AEB_CMS_hmw_warning = 0x00;
uint8_t g_AEB_CMS_cipv_age    = 0x00;
//-----Cam info
uint8_t g_is_hmw_warning      = 0x00;

/////////////////////////////////////////////////////////
//Log use ChipOn Debug Oscilloscop(just int)
/////////////////////////////////////////////////////////
uint8_t OSC_Cam_d_z;
uint8_t OSC_V_speed;

uint8_t OSC_Appro_to_Lead_Car;
uint8_t OSC_Cam_TTC;
uint8_t OSC_Cam_HMW;

uint8_t OSC_Dec_out;
uint8_t OSC_Dec_out_cms_hmw;
uint8_t OSC_Dec_out_cms_ttc;
uint8_t OSC_Dec_out_aeb_ttc;
uint8_t OSC_Dec_out_uradar;


/***********************************************************
 * Private:
 * Just use inside
 * *********************************************************
 */

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#ifndef speed_meter_p_s
#define speed_meter_p_s(v)            ((v) / (3.6))
#endif

#define TURN_ON   0x01
#define TURN_OFF  0x00

#define MIN_TIME_FORCE_FEEL  0.2


#define delay_count     5
#define keep_count      2

#define break_calculate_cancel      0x00
#define break_calculate_continue    0x01
#define break_cancel_NONE       0x00
#define break_cancel_TTC     	0x02
#define break_cancel_HMW     	0x03
#define break_cancel_TTCHMW     0x04

#define effictive_distance_hmw    3

#define LOGLEVEL    LOG_NONE

#define cal_ttc_history_n 4
#define approaching_history_n 5
#define time_gap_cam_to_radar 5

#define USE_CAM_Warning_info 1

#define ReCal_HMW_TTC 1

#define USE_ReCal_HMW_TTC 0

#define MIN_BREAK_COOLING_TIME 0.5




///////////////////////////////////////////////////////
// main_control
////////////////////////////////////////////////////////
uint8_t g_AEB_CMS_Enable 				= TURN_ON;
uint8_t g_CMS_HMW_Enable 				= TURN_ON;
uint8_t g_CMS_TTC_Enable 				= TURN_ON;
uint8_t g_AEB_TTC_Enable 				= TURN_OFF;
uint8_t g_SSS_Enable 					= TURN_ON;
uint8_t g_HMW_Dynamic_Thr_Enable 		= TURN_ON;
uint8_t g_Chassis_Safe_Strategy_Enable = TURN_ON;
uint8_t g_LDW_Enable 					= TURN_ON;

static CMS_Break_State cms_break_state;
static AEB_Break_State aeb_break_state;
static AEB_CMS_Control_Parameters AEB_CMS_parameters;
static volatile float dec_output = 0.0;
static volatile float ego_speed = 0.0;

static Obstacle_Information  cipv_history;
static Camera_Essential_Data cam_essiential_history;
static Obstacle_Basic_Data obs_basic_data_history;
static uint8_t got_cipv_history = 0;

static uint32_t driver_control_gap_time_stamp 	= 0;
static uint32_t aeb_break_cooling_time_stamp    = 0;
static uint32_t aeb_break_keep_time_stamp   	= 0;
static uint32_t cal_ttc_history_time_stamp[4];
float cal_ttc_history_distance_z[4];
static uint8_t cal_ttc_count = 0;
static uint8_t cal_ttc_history_cipv_id = 0;
static uint8_t cal_ttc_history_ID = 0;
static float ttc_r_cal;
static float hmw_r_cal;
static uint8_t warning_name;  //0: 无 1:
static uint8_t warning_function;
static float Uradar_distance;
static float ttc_aeb_dec;
static float ttc_cms_dec;
static float hmw_cms_dec;
static float uradar_cms_dec;
static float dynamic_hmw_thr = 0.0;
static float dynamic_ttc_thr = 0.0;


static float history_speed;
static float history_distance_z;
static float history_distance_x;

static float judge_approaching_cam_distance[5];
static uint8_t judge_approaching_cam_distance_count = 0;
static uint8_t judge_approaching_cam_distance_ID;

static uint32_t judge_approaching_uradar_timeatamp[5];
static float judge_approaching_uradar_distance[5];
static uint8_t judge_approaching_uradar_distance_count = 0;


static float hmw_queue[3];
static uint8_t hmw_queue_size = 3;
static uint8_t hmw_queue_count = 0;
static uint8_t hmw_queue_pointer = 0;



static uint8_t cipv_count = 0;
static uint8_t have_processed = 0;
static uint8_t have_save_to_history = 0;
static uint8_t have_processed_nocipv = 0;
static uint8_t approaching_ornot = 0;

static uint8_t  Singal_Stop_Sta;
static uint8_t  Singal_TurnLeft_Sta;
static uint8_t  Singal_TurnRight_Sta;
static uint8_t  Singal_Back_Sta;
static uint8_t  AEB_SWITCH_Sta;
static uint16_t wheel_speed;
//

static uint8_t s_Old_AEB_ON_OFF_GPIO        = 0x01;
static uint8_t s_Old_AEB_ON_OFF_Displayer   = 0x01;
static uint8_t s_Old_LDW_ON_OFF_Displayer   = 0x01;
static uint8_t s_Old_AEB_ON_OFF_BT		    = 0x01;
static uint8_t s_Old_LDW_ON_OFF_BT			= 0x01;
static uint8_t s_Old_SSS_ON_OFF_BT			= 0x01;

///////////////////////////////////////////////////////////////////////
//Global Switch Control
//////////////////////////////////////////////////////////////////////

void AEB_System_Switch(void){
	//  priority level : on_off_GPIO = on_off_dispalyer = on_off_BT
	if(s_Old_AEB_ON_OFF_GPIO != g_AEB_ON_OFF_GPIO){
		s_Old_AEB_ON_OFF_GPIO = g_AEB_ON_OFF_GPIO; // sync
		g_AEB_CMS_Enable = g_AEB_ON_OFF_GPIO;
		g_SSS_Enable	   = g_AEB_ON_OFF_GPIO;
	}
	if(s_Old_AEB_ON_OFF_Displayer != g_AEB_ON_OFF_Displayer){
		s_Old_AEB_ON_OFF_Displayer = g_AEB_ON_OFF_Displayer; // sync
		g_AEB_CMS_Enable = g_AEB_ON_OFF_Displayer;
		g_SSS_Enable	   = g_AEB_ON_OFF_Displayer;
	}
	if(s_Old_AEB_ON_OFF_BT != g_AEB_ON_OFF_BT){
		s_Old_AEB_ON_OFF_BT = g_AEB_ON_OFF_BT; // sync
		g_AEB_CMS_Enable = g_AEB_ON_OFF_BT;
		g_SSS_Enable	   = g_AEB_ON_OFF_BT;
	}
}

void LDW_Switch(void){
	if(s_Old_LDW_ON_OFF_BT != g_LDW_ON_OFF_BT){
		s_Old_LDW_ON_OFF_BT = g_LDW_ON_OFF_BT; // sync
		g_LDW_Enable = g_LDW_ON_OFF_BT;
	}
	if(s_Old_LDW_ON_OFF_Displayer != g_LDW_ON_OFF_Displayer){
		s_Old_LDW_ON_OFF_Displayer = g_LDW_ON_OFF_Displayer; // sync
		g_LDW_Enable = g_LDW_ON_OFF_Displayer;
	}
}

void Dynamic_hmw_ttc_thr(float speed){
	if((rParms.CMS_HMW_Dynamic_Offset_Speed_L < rParms.Min_Enable_Speed)|(rParms.CMS_HMW_Dynamic_Offset_Speed_L > 50.0)|(rParms.CMS_HMW_Dynamic_Offset_Speed_H > 90.0)|(rParms.CMS_HMW_Dynamic_Offset_Speed_H > AEB_CMS_parameters.max_enable_speed)){
		dynamic_hmw_thr = rParms.CMS_HMW_Brake_Time_Thr;
		return ;
	}
	if((speed > 0)&(speed < rParms.CMS_HMW_Dynamic_Offset_Speed_L)){
		dynamic_hmw_thr = rParms.CMS_HMW_Dynamic_Offset_Value_L * (rParms.CMS_HMW_Dynamic_Offset_Speed_L - speed)/ rParms.CMS_HMW_Dynamic_Offset_Speed_L;
		dynamic_hmw_thr += rParms.CMS_HMW_Brake_Time_Thr;
		if(dynamic_hmw_thr > 0.5){
			dynamic_hmw_thr = 0.5;
		}
	}
	else if((speed > rParms.CMS_HMW_Dynamic_Offset_Speed_H )){
		dynamic_hmw_thr = rParms.CMS_HMW_Dynamic_Offset_Value_H * (speed - rParms.CMS_HMW_Dynamic_Offset_Speed_H )/ (AEB_CMS_parameters.max_enable_speed - rParms.CMS_HMW_Dynamic_Offset_Speed_H );
		//
		dynamic_hmw_thr = rParms.CMS_HMW_Brake_Time_Thr - dynamic_hmw_thr;
		if(dynamic_hmw_thr < 0.3){
			dynamic_hmw_thr = 0.3;
		}
	}
}

void CMS_HMW_warning(float hmw_thr, float hmw, uint8_t warning_grade){
	if((warning_grade == 3)|(warning_grade == 0)){
		g_AEB_CMS_hmw_warning = 0;
		hmw_queue_count = 0;
		return;
	}
	if(hmw_queue_count >= 0){
		if((hmw_queue_count < hmw_queue_size)){
			hmw_queue[hmw_queue_count] = hmw;
			hmw_queue_count += 1;
		}else {
			hmw_queue[hmw_queue_count - hmw_queue_size] = hmw;
			hmw_queue_count += 1;
			if(hmw_queue_count > (hmw_queue_size*2 - 1)){
				hmw_queue_count = hmw_queue_size;
			}
		}
	}
	else{
		hmw_queue_count = 0;
	}

	if(hmw_queue_count >= hmw_queue_size){
		g_AEB_CMS_hmw_warning = 1;
		for(uint8_t l_i = 0;l_i < hmw_queue_size; l_i++){
			if(hmw_queue[l_i] > hmw_thr){
				g_AEB_CMS_hmw_warning = 0;
				return ;
			}
		}
		if(cipv_history.TrackNumber < AEB_CMS_parameters.min_track_number){
			g_AEB_CMS_hmw_warning = 0;
			return ;
		}
	}
	else{
		g_AEB_CMS_hmw_warning = 0;
	}
	return;

}

void Update_OSC_Data(void){
	OSC_Cam_d_z = (uint8_t)cipv_history.DistanceZ;
	OSC_Cam_TTC = (uint8_t)cipv_history.TTC;
	OSC_Cam_HMW = (uint8_t)cipv_history.HMW;
	OSC_V_speed = (uint8_t)stVehicleParas.fVehicleSpeed;
	OSC_Appro_to_Lead_Car = approaching_ornot;

	OSC_Dec_out = (uint8_t)dec_output;
	OSC_Dec_out_cms_hmw = (uint8_t)hmw_cms_dec;
	OSC_Dec_out_cms_ttc = (uint8_t)ttc_cms_dec;
	OSC_Dec_out_aeb_ttc = (uint8_t)ttc_aeb_dec;
	OSC_Dec_out_uradar = (uint8_t)uradar_cms_dec;
}


void Got_Vehicle_Para_form_GPIO(_VEHICLE_PARA* stVehicleParas){
	Singal_Stop_Sta=GPIO_Read_Input_Data_Bit(GPIOG_SFR,GPIO_PIN_MASK_6);//SingalStop
	Singal_TurnLeft_Sta = GPIO_Read_Input_Data_Bit(GPIOG_SFR,GPIO_PIN_MASK_7);//Singal_TurnLeft
	Singal_TurnRight_Sta = GPIO_Read_Input_Data_Bit(GPIOH_SFR,GPIO_PIN_MASK_5);//Singal_TurnRight
	Singal_Back_Sta = GPIO_Read_Input_Data_Bit(GPIOH_SFR,GPIO_PIN_MASK_6);//BACK
	AEB_SWITCH_Sta = GPIO_Read_Input_Data_Bit(GPIOH_SFR,GPIO_PIN_MASK_12);//AEB

	stVehicleParas->BrakeFlag = Singal_Stop_Sta;
	stVehicleParas->LeftFlagTemp = Singal_TurnLeft_Sta;
	stVehicleParas->RightFlagTemp = Singal_TurnRight_Sta;
	stVehicleParas->ReverseGear = Singal_Back_Sta;

	wheel_speed = Get_Wheel_Speed();
	stVehicleParas->fVehicleSpeed = wheel_speed /10.0;
	//fprintf(USART1_STREAM,"wheel_speed = %0.2f\r\n",stVehicleParas->fVehicleSpeed);
	if(LOG_DEBUG == LOGLEVEL){
		if(Singal_Stop_Sta == 1){
			fprintf(USART1_STREAM,"Singal_Stop:%d\r\n",Singal_Stop_Sta);
		}
		if(Singal_TurnLeft_Sta == 1){
			fprintf(USART1_STREAM,"Singal_TurnLeft_Sta:%d\r\n",Singal_TurnLeft_Sta);
		}
		if(Singal_TurnRight_Sta == 1){
			fprintf(USART1_STREAM,"Singal_TurnRight_Sta:%d\r\n",Singal_TurnRight_Sta);
		}
		if(Singal_Back_Sta == 1){
			fprintf(USART1_STREAM,"Singal_Back_Sta:%d\r\n",Singal_Back_Sta);
		}
	}

}
void AEB_System_Safe_Tactics(void){
	// keep enough time and then make sure that the system have finish an effective break
	if((((SystemtimeClock - aeb_break_cooling_time_stamp)/1000.0) < rParms.Brake_Cooling_Time)&(((SystemtimeClock - aeb_break_cooling_time_stamp)/1000.0) >= MIN_BREAK_COOLING_TIME)){
		dec_output = 0.0;
	}
	if(((SystemtimeClock - aeb_break_keep_time_stamp)/1000.0) > rParms.Max_Brake_Keep_Time){
		dec_output = 0.0;
	}
	if(dec_output > 0.000001){
		aeb_break_cooling_time_stamp = SystemtimeClock;
	}
	if(dec_output < 0.000001){
		aeb_break_keep_time_stamp = SystemtimeClock;
	}
}

float AEB_CMS_Break_control(uint8_t got_cipv,
							Obstacle_Basic_Data* obs_basic_data,
							Obstacle_Information* cipv,
							_VEHICLE_PARA* stVehicleParas,
							Camera_Essential_Data* cam_essiential,
							URADER_MESSAGE	Urader_Company_Message[]){
	// calculate the effictive distance every time
	///////////////
	//Switch Check
	///////////////
	AEB_System_Switch();
	LDW_Switch();
	///////////////////////
	//Switch Status judge
	///////////////////////
	if(0x00 == g_AEB_CMS_Enable){
		dec_output = 0.0;
		g_AEB_CMS_outside_dec_output = dec_output;
		AEB_State_reset();
		CMS_State_reset();
		g_AEB_CMS_hmw_warning = 0x00;
		return dec_output;
	}

	//////////////////////////////
	Uradar_distance = Get_Uradar_distance(Urader_Company_Message, stVehicleParas);
	g_is_hmw_warning = cam_essiential->HMWWarning;

	if((*obs_basic_data).TimeID == obs_basic_data_history.TimeID){// if the frame target no cipv, then keep the last one dec
		if(got_cipv_history == 0){// have no cipv so we need continue to judge "got_cipv"
			if(got_cipv == 1){
				got_cipv_history = 1;
				cipv_history = (*cipv);
				have_processed = 1;
			}
			else{
				return dec_output;// got_cipv_history == 0:keep
			}

		}else if(got_cipv_history == 1){
			// OK，got the cipv，do once time
			if(have_processed == 0){
				// got_cipv_history == 1: just do once
				have_processed = 1;
			}else if(have_processed == 1){
				return dec_output;// got_cipv_history == 1:keep
			}
		}
	}else if((*obs_basic_data).TimeID != obs_basic_data_history.TimeID){
		//
		if((got_cipv_history == 0)){
			// do not have cipv : history frame
			//cipv_history = (*cipv); // init cipv
			obs_basic_data_history = (*obs_basic_data);
			cam_essiential_history = (*cam_essiential);
			have_save_to_history = 1;
			return dec_output;

		}else if(got_cipv_history == 1){
			if(have_processed == 1){
				//cipv_history = (*cipv);
				obs_basic_data_history = (*obs_basic_data);
				cam_essiential_history = (*cam_essiential);
				have_processed = 0;
				got_cipv_history = 0;
				return dec_output;
			}else if(have_processed == 0){
				// just do once
				have_processed = 1;
			}
		}
	}
//
// IF HAVE CIPV, THEN CALCULATE DEC OUTPUT:warng
//IF Got One fram data,then process once

	// reCal_HMW
	dynamic_hmw_thr = rParms.CMS_HMW_Brake_Time_Thr;
	//Dynamic_hmw_ttc_thr(stVehicleParas->fVehicleSpeed);
	// Cal warning signal
	CMS_HMW_warning(max(rParms.CMS_HMW_Brake_Time_Thr, dynamic_hmw_thr) , cipv_history.HMW, cam_essiential->HMWGrade);
    // Cal break
#ifdef ReCal_HMW_TTC
	if(ReCal_HMW_TTC == 1){
		// recalculate hmw
		hmw_r_cal = cipv_history.DistanceZ / speed_meter_p_s((*stVehicleParas).fVehicleSpeed);
		if((hmw_r_cal > 10.0))
			hmw_r_cal = 10.0;
		// recalculate ttc
		ttc_r_cal = Cal_TTC(&cipv_history);
		history_speed = (*stVehicleParas).fVehicleSpeed;
		history_distance_z = cipv_history.DistanceZ;
		history_distance_x = cipv_history.DistanceX;
		if((ttc_r_cal > 10.0)|(ttc_r_cal < 0.0))
			ttc_r_cal = 10.0;
		// log
		if(LOGLEVEL == LOG_IN_OUT){
			//Log_print(LOG_IN_OUT, 0x00,0x00," ","main", cipv_history,stVehicleParas, cam_essiential_history,obs_basic_data_history);
		}
	}
#endif
#ifdef USE_ReCal_HMW_TTC
	if(USE_ReCal_HMW_TTC == 1){
		cipv_history.HMW = hmw_r_cal;
		cipv_history.TTC = ttc_r_cal;
		AEB_CMS_parameters.Cam_HMW_warning_time = obs_basic_data_history.HMWAlarmThreshold;
		if(cipv_history.HMW > 6.0)
			cam_essiential_history.HMWGrade = 0;
		if(cipv_history.HMW > 2.7)
			cam_essiential_history.HMWGrade = 3;
		else if(cipv_history.HMW > obs_basic_data_history.HMWAlarmThreshold)
			cam_essiential_history.HMWGrade = 2;
		else if(cipv_history.HMW < obs_basic_data_history.HMWAlarmThreshold)
			cam_essiential_history.HMWGrade = 1;
	}
#endif
	ego_speed = speed_meter_p_s((*stVehicleParas).fVehicleSpeed);
	//AEB_CMS_parameters.cipv_max_distance = ego_speed * effictive_distance_hmw;
	uint8_t return_back = Break_Trigger(&cipv_history, stVehicleParas, &cam_essiential_history,&obs_basic_data_history);
	//uint8_t return_back = 1;
	if(return_back == break_calculate_cancel){
		//不满足设置的阈值条件，直接返回，不进行计算
		//Log_print(LOG_DEBUG,0x00,0x00, "debug","debug", cipv,stVehicleParas, cam_essiential);
		dec_output = 0.0;
		if((*stVehicleParas).fVehicleSpeed > rParms.Min_Enable_Speed){
			if(LOGLEVEL == LOG_IN_OUT){
				ttc_r_cal = Cal_TTC(&cipv_history);
				hmw_r_cal = cipv_history.DistanceZ / ego_speed;
				Log_print(LOG_IN_OUT, 0x00,0x00," ","main", &cipv_history, stVehicleParas, &cam_essiential_history, &obs_basic_data_history);
			}
		}
		g_AEB_CMS_outside_dec_output = dec_output;
		return dec_output;
	}


	ttc_aeb_dec = AEB_TTC_Break(&cipv_history, &obs_basic_data_history, stVehicleParas);
	ttc_cms_dec = CMS_TTC_Break(&cipv_history, &obs_basic_data_history, stVehicleParas, &cam_essiential_history, Urader_Company_Message);
	hmw_cms_dec = CMS_HMW_Break(&cipv_history, &obs_basic_data_history, stVehicleParas, &cam_essiential_history, Urader_Company_Message);

	return_back = Break_Cancel(&cipv_history, stVehicleParas,&cam_essiential_history,&obs_basic_data_history);
	if(return_back == break_cancel_TTC){
		ttc_aeb_dec = 0.0;
		ttc_cms_dec = 0.0;
	}
	if(return_back == break_cancel_HMW){
		hmw_cms_dec = 0.0;
	}
	if(break_cancel_TTCHMW == return_back){
		ttc_aeb_dec = 0.0;
		ttc_cms_dec = 0.0;
		hmw_cms_dec = 0.0;
	}

	dec_output = max(max(ttc_aeb_dec, ttc_cms_dec), hmw_cms_dec);

	uradar_cms_dec = CMS_URadar_Break(stVehicleParas, Urader_Company_Message);
	if(uradar_cms_dec > 0.0){
		dec_output = max(dec_output,uradar_cms_dec);
	}


	// Driver Take over the BUS
	return_back = Driver_control(&cipv_history, stVehicleParas, &cam_essiential_history, &obs_basic_data_history);
	if(return_back == break_calculate_cancel){
		dec_output = 0.0;
	}
	if(got_cipv_history == 0){
		dec_output = 0.0;
	}
	//Log_print(LOG_DEBUG, " ","Break_Trigger", cipv);
	if(LOGLEVEL == LOG_IN_OUT){
		ttc_r_cal = Cal_TTC(&cipv_history);
		hmw_r_cal = cipv_history.DistanceZ / ego_speed;

		Log_print(LOG_IN_OUT, 0x00,0x00," ","main", &cipv_history, stVehicleParas, &cam_essiential_history, &obs_basic_data_history);
	}
	// cooling time and max keep time
	//AEB_System_Safe_Tactics(); // Safe tactics
	//Update_OSC_Data();
	g_AEB_CMS_outside_dec_output = dec_output;
	return dec_output;
}
float AEB_TTC_Break(Obstacle_Information* cipv,
					Obstacle_Basic_Data* obs_basic_data,
					_VEHICLE_PARA* stVehicleParas){
	return 0.0;
}
float CMS_TTC_Break(Obstacle_Information* cipv,
					Obstacle_Basic_Data* obs_basic_data,
					_VEHICLE_PARA* stVehicleParas,
					Camera_Essential_Data* cam_essiential,
					URADER_MESSAGE Urader_Company_Message[]){
	//TTC_CMS
	//float speed_ego = speed_meter_p_s(stVehicleParas.fVehicleSpeed);
	cms_break_state.cam_distance = (*cipv).DistanceZ;

	if(((rParms.CMS_TTC_Brake_Time_Thr + rParms.Air_Brake_Delay_Time) > (*cipv).TTC)&((*cipv).TTC > 0)&((*cipv).TTC < 6.0)){
		if(rParms.CMS_TTC_Brake_Force_Feel_Para > MIN_TIME_FORCE_FEEL){
			cms_break_state.cms_ttc_dec_out = (1.0/rParms.CMS_TTC_Brake_Force_Feel_Para) * ((*cipv).RelativeSpeedZ - (cms_break_state.cam_distance / rParms.CMS_TTC_Brake_Time_Thr));
		}
		else{
			cms_break_state.cms_ttc_dec_out = 0.0;
		}
		//cms_break_state.cms_ttc_dec_out = cms_break_state.cms_ttc_dec_out / AEB_CMS_parameters.ttc_thr_set;
		if(cms_break_state.cms_ttc_dec_out > AEB_CMS_parameters.ttc_max_dec_output){
			cms_break_state.cms_ttc_dec_out = AEB_CMS_parameters.ttc_max_dec_output;
			if(LOGLEVEL == LOG_WARNING){
				Log_print(LOG_WARNING, 0x08,0x02,"TTC_CMS too big dec", "CMS_TTC_Break", cipv, stVehicleParas, cam_essiential,obs_basic_data);
			}
		}else if(cms_break_state.cms_ttc_dec_out < 0.0){
			cms_break_state.cms_ttc_dec_out = 0.0;
		}
		Filter_noise(&cms_break_state.cms_ttc_dec_out,cms_break_state.cms_ttc_dec_out_prev, &cms_break_state.cms_is_break_flag,&cms_break_state.cms_cam_delay_count, &cms_break_state.cms_cam_keep_count);
	}
	else{//相机有输出，但是，无需刹车
			cms_break_state.cms_cam_delay_count = 0;
			cms_break_state.cms_ttc_dec_out = 0.0;
		}

	if(cms_break_state.cms_ttc_dec_out > 0.0){
		cms_break_state.cms_is_break_flag = 0x01;
		cms_break_state.cms_ttc_dec_out_prev = cms_break_state.cms_ttc_dec_out;
	}else{
		cms_break_state.cms_is_break_flag = 0x00;
	}

	return cms_break_state.cms_ttc_dec_out;

}
float CMS_HMW_Break(Obstacle_Information* cipv,
					Obstacle_Basic_Data* obs_basic_data,
					_VEHICLE_PARA* stVehicleParas,
					Camera_Essential_Data* cam_essiential,
					URADER_MESSAGE Urader_Company_Message[]){
	//HMW_CMS
	cms_break_state.cam_distance = (*cipv).DistanceZ;

	//if((((*cipv).HMW < 6.0))&((*cipv).HMW > 0)&((*cipv).HMW < (AEB_CMS_parameters.hmw_thr_set + AEB_CMS_parameters.Air_Break_delay_time))&(AEB_CMS_parameters.hmw_thr_set > 0)){
	if((((*cipv).HMW < 6.0))&((*cipv).HMW > 0)&((*cipv).HMW < (dynamic_hmw_thr + rParms.Air_Brake_Delay_Time))&(dynamic_hmw_thr > 0)){
		//cms_break_state.cms_hmw_dec_out = AEB_CMS_parameters.hmw_force_set * (ego_speed - (cms_break_state.cam_distance / dynamic_hmw_thr));// NO effective about break delay time
		if( rParms.CMS_HMW_Brake_Force_Feel_Para > MIN_TIME_FORCE_FEEL){
			cms_break_state.cms_hmw_dec_out = (1.0/rParms.CMS_HMW_Brake_Force_Feel_Para) * (ego_speed - (cms_break_state.cam_distance / (dynamic_hmw_thr + rParms.Air_Brake_Delay_Time)));// pre cal dec ,make sure that the driver can fell the break first time;
			//cms_break_state.cms_hmw_dec_out = cms_break_state.cms_hmw_dec_out / AEB_CMS_parameters.hmw_thr_set;
		}
		else {
			cms_break_state.cms_hmw_dec_out = 0.0;
		}

		if(cms_break_state.cms_hmw_dec_out > AEB_CMS_parameters.Vehicle_max_react_dec){
			cms_break_state.cms_hmw_dec_out = AEB_CMS_parameters.Vehicle_max_react_dec;
			if(LOGLEVEL == LOG_WARNING){
				Log_print(LOG_WARNING,0x06,0x01, "hmw_CMS too big dec", "CMS_HMW_Break", cipv,stVehicleParas,cam_essiential,obs_basic_data);
			}
		}else if(cms_break_state.cms_hmw_dec_out < 0.0){
			cms_break_state.cms_hmw_dec_out = 0.0;
			//cms_break_state.cms_break_flag = 0;
		}else if((*cipv).HMW > dynamic_hmw_thr){
			cms_break_state.cms_hmw_dec_out = 0.0;
		}
		Filter_noise(&cms_break_state.cms_hmw_dec_out,cms_break_state.cms_hmw_dec_out_prev,&cms_break_state.cms_is_break_flag,&cms_break_state.cms_cam_delay_count, &cms_break_state.cms_cam_keep_count);
		cms_break_state.cms_cam_over_timestamp = SystemtimeClock;// if the hmw is ok(samll than thr and small than 6.0, improve that the target was detected, if lost,this time stamp will recored the last timestamp)
	}else if((*cipv).HMW > 6.0){//相机输出无效0x3f 6.3
		cms_break_state.cms_cam_delay_count = 0;
		//先判断超声波有没有数据
		cms_break_state.uradar_distance = Get_Uradar_distance(Urader_Company_Message, stVehicleParas);
		//有数据,使用雷达数据
		// before use the uradar data , must confirmed that the camera lost the target just now
		if(((SystemtimeClock - cms_break_state.cms_cam_over_timestamp)/1000) < time_gap_cam_to_radar){
			if((cms_break_state.uradar_distance < 4.5)&((cms_break_state.uradar_distance > 0.0))){
				if( rParms.CMS_HMW_Brake_Force_Feel_Para > MIN_TIME_FORCE_FEEL){
					cms_break_state.cms_hmw_dec_out = (1.0/rParms.CMS_HMW_Brake_Force_Feel_Para) * (ego_speed - (cms_break_state.uradar_distance / dynamic_hmw_thr));
					//cms_break_state.cms_hmw_dec_out = cms_break_state.cms_hmw_dec_out / dynamic_hmw_thr;
				}
				else {
					cms_break_state.cms_hmw_dec_out = 0.0;
				}

				if(cms_break_state.cms_hmw_dec_out > AEB_CMS_parameters.Vehicle_max_react_dec){
					cms_break_state.cms_hmw_dec_out = AEB_CMS_parameters.Vehicle_max_react_dec;
					if(LOGLEVEL == LOG_WARNING){
						Log_print(LOG_WARNING, 0x07,0x01,"hmw_CMS use Uradar ", "CMS_HMW_Break", cipv, stVehicleParas, cam_essiential,obs_basic_data);
					}
				}else if(cms_break_state.cms_hmw_dec_out < 0.0){
					cms_break_state.cms_hmw_dec_out = 0.0;
					//cms_break_state.cms_break_flag = 0;
				}
				Filter_noise(&cms_break_state.cms_hmw_dec_out,cms_break_state.cms_hmw_dec_out_prev, &cms_break_state.cms_is_break_flag,&cms_break_state.cms_uradar_delay_count, &cms_break_state.cms_uradar_keep_count);

			}else{//雷达无数据，则判断是否需要保持
				cms_break_state.cms_uradar_delay_count = 0;
				cms_break_state.cms_hmw_dec_out = 0.0;
			}
		}else{
			cms_break_state.cms_hmw_dec_out = 0.0;
			Filter_noise(&cms_break_state.cms_hmw_dec_out,cms_break_state.cms_hmw_dec_out_prev, &cms_break_state.cms_is_break_flag,&cms_break_state.cms_uradar_delay_count, &cms_break_state.cms_uradar_keep_count);
		}

	}else{//相机有输出，但是，无需刹车
		cms_break_state.cms_hmw_dec_out = 0.0;
		Filter_noise(&cms_break_state.cms_hmw_dec_out,cms_break_state.cms_hmw_dec_out_prev, &cms_break_state.cms_is_break_flag,&cms_break_state.cms_cam_delay_count, &cms_break_state.cms_cam_keep_count);
		cms_break_state.cms_cam_over_timestamp = SystemtimeClock;// if the hmw is ok(samll than thr and small than 6.0, improve that the target was detected, if lost,this time stamp will recored the last timestamp)
	}

	if(cms_break_state.cms_hmw_dec_out > 0.0){
		cms_break_state.cms_is_break_flag = 0x01;
		cms_break_state.cms_hmw_dec_out_prev = cms_break_state.cms_hmw_dec_out;
	}else{
		cms_break_state.cms_is_break_flag = 0x00;
	}

	return cms_break_state.cms_hmw_dec_out;
}

float CMS_URadar_Break(_VEHICLE_PARA* stVehicleParas,
					URADER_MESSAGE Urader_Company_Message[]){
	// when the camera lost the target
	//先判断超声波有没有数据
	cms_break_state.uradar_distance = Get_Uradar_distance(Urader_Company_Message, stVehicleParas);
	//有数据,使用雷达数据
	// before use the uradar data , must confirmed that the camera lost the target just now
	if(((SystemtimeClock - cms_break_state.cms_cam_over_timestamp)/1000) < time_gap_cam_to_radar){
		if((cms_break_state.uradar_distance < 4.5)&((cms_break_state.uradar_distance > 0.0))){
			uint8_t approaching_ornot = Approaching_to_lead_car_uradar(cms_break_state.uradar_distance, approaching_history_n);
			if(approaching_ornot == 1){
				if( rParms.CMS_HMW_Brake_Force_Feel_Para > MIN_TIME_FORCE_FEEL){
					cms_break_state.cms_uradar_dec_out = (1.0/rParms.CMS_HMW_Brake_Force_Feel_Para) * (ego_speed - (cms_break_state.uradar_distance / dynamic_hmw_thr));
					cms_break_state.cms_uradar_dec_out = cms_break_state.cms_uradar_dec_out / dynamic_hmw_thr;
				}
				else {
					cms_break_state.cms_uradar_dec_out = 0.0;
				}

				if(cms_break_state.cms_uradar_dec_out > AEB_CMS_parameters.Vehicle_max_react_dec){
					cms_break_state.cms_uradar_dec_out = AEB_CMS_parameters.Vehicle_max_react_dec;

				}else if(cms_break_state.cms_uradar_dec_out < 0.0){
					cms_break_state.cms_uradar_dec_out = 0.0;
					//cms_break_state.cms_break_flag = 0;
				}
			}
			else{
				cms_break_state.cms_uradar_delay_count = 0;
				cms_break_state.cms_uradar_dec_out = 0.0;
			}

		}else{//雷达无数据，则判断是否需要保持
			cms_break_state.cms_uradar_delay_count = 0;
			cms_break_state.cms_uradar_dec_out = 0.0;
		}
	}else{
		cms_break_state.cms_uradar_dec_out = 0.0;
	}

	if(cms_break_state.cms_uradar_dec_out > 0.0){
		cms_break_state.cms_is_break_flag = 0x01;
		cms_break_state.cms_uradar_dec_out_prev = cms_break_state.cms_uradar_dec_out;
	}else{
		cms_break_state.cms_is_break_flag = 0x00;
	}

	return cms_break_state.cms_uradar_dec_out;
}

float Cal_TTC(Obstacle_Information* cipv){
	if(cal_ttc_history_cipv_id == (*cipv).ObstacleID){

		if((cal_ttc_count <cal_ttc_history_n)&(cal_ttc_count > 0)){
			cal_ttc_history_time_stamp[cal_ttc_count -1] = SystemtimeClock;
			cal_ttc_history_distance_z[cal_ttc_count -1] = (*cipv).DistanceZ;
			cal_ttc_count = cal_ttc_count + 1;
		}
		if(cal_ttc_count >= cal_ttc_history_n){
			cal_ttc_history_time_stamp[cal_ttc_count -cal_ttc_history_n] = SystemtimeClock;
			cal_ttc_history_distance_z[cal_ttc_count -cal_ttc_history_n] = (*cipv).DistanceZ;
			cal_ttc_count = cal_ttc_count + 1;
			if(cal_ttc_count > (cal_ttc_history_n*2 - 1)){
				cal_ttc_count = cal_ttc_history_n;
			}
			//
			uint8_t pointer_now = cal_ttc_count - 1;
			if(pointer_now < cal_ttc_history_n){
				pointer_now = (cal_ttc_history_n*2 - 1);
			}

			uint8_t pointer_old = cal_ttc_count;

			float d_d = cal_ttc_history_distance_z[pointer_old] - (*cipv).DistanceZ;
			if(d_d <= 0){
				//return -1.0;
				return 6.3;
			}
			float d_t = (SystemtimeClock - cal_ttc_history_time_stamp[pointer_old])/1000.0;
			if(d_t > 0){
				float d_v = d_d/d_t;
				float ttc = (*cipv).DistanceZ / d_v;
				if(ttc > 6.0){
					ttc = 6.3;
				}
				return ttc;
			}
		}
	}else{
		cal_ttc_history_cipv_id = (*cipv).ObstacleID;
		cal_ttc_count = 0;
	}
	return 6.3;//invalied
}

uint8_t Approaching_to_target(uint8_t size_of_history, float distance, float distance_resived[], uint32_t distance_timestamp[],uint8_t* num_count){
	uint8_t double_size = (size_of_history * 2 - 1);
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
			distance_timestamp[*num_count - size_of_history] = SystemtimeClock;
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
		if((distance_resived[now_pointer - size_of_history] < distance_resived[middle_pointer - size_of_history])|(distance_resived[now_pointer - size_of_history] < distance_resived[old_pointer - size_of_history])){
			return 0x01;// approaching
		}
		else{
			return 0x00;// not approaching
		}
	}
	else{
		return 0x00; // can not judge
	}
}

uint8_t Approaching_to_lead_car(Obstacle_Information* cipv, uint8_t size_of_history){

	// use cam distance,to judge the ego car approaching the lead car(1), or not(0);
	// 1: approching
	//    hmw working
	// 0: go away
	//    hmw not working
	// when ttc > 6.0 ,just use for hmw breaking
	// loop recored the history data
	if(judge_approaching_cam_distance_ID != (*cipv).ObstacleID){
		judge_approaching_cam_distance_count = 0;

		judge_approaching_cam_distance_ID = (*cipv).ObstacleID;
		//return 2;
	}
	return Approaching_to_target(approaching_history_n, (*cipv).DistanceZ, judge_approaching_cam_distance, NULL, &judge_approaching_cam_distance_count);
}
uint8_t Approaching_to_lead_car_uradar(float diatance, uint8_t size_of_history){
	uint8_t double_size = (size_of_history*2 - 1);
	if(diatance == 5.04){
		judge_approaching_uradar_distance_count = 0;
		return 2;
	}
	return Approaching_to_target(approaching_history_n, diatance, judge_approaching_uradar_distance, judge_approaching_uradar_timeatamp, &judge_approaching_uradar_distance_count);
}
uint8_t Break_Trigger(Obstacle_Information* cipv, _VEHICLE_PARA* stVehicleParas, Camera_Essential_Data* cam_essiential,Obstacle_Basic_Data* obs_basic_data){

	if(((*stVehicleParas)).fVehicleSpeed < rParms.Min_Enable_Speed){
		// some times speed become to 0.0, and then come back  immediately;so we should use filter to iden
		AEB_State_reset();
		CMS_State_reset();
		return break_calculate_cancel;
	}
	// calculate max_distance every time outside the function
	if((*cipv).DistanceZ > AEB_CMS_parameters.cipv_max_distance){/*target too far :link with ego car speed*/
		AEB_State_reset();
		CMS_State_reset();
		if(LOGLEVEL == LOG_WARNING){
			Log_print(LOG_WARNING,0x01,0x00, "cipv_distance","Break_Trigger", cipv, stVehicleParas, cam_essiential, obs_basic_data);
		}
		return break_calculate_cancel;
	}
	if((*stVehicleParas).ReverseGear == 1){/*  reverse gear*/
		if(LOGLEVEL == LOG_WARNING){
			Log_print(LOG_WARNING,0x03,0x00,"ReverseGear","Break_Trigger", cipv, stVehicleParas, cam_essiential, obs_basic_data);
		}
		AEB_State_reset();
		CMS_State_reset();
		return break_calculate_cancel;
	}
	if((*cipv).RelativeSpeedX > 4){ /* process turning and fast crossing car*/
		if(LOGLEVEL == LOG_WARNING){
			Log_print(LOG_WARNING,0x05,0x00, "crossing target ","Break_Trigger", cipv, stVehicleParas, cam_essiential, obs_basic_data);
		}
		//AEB_State_reset();
		//CMS_State_reset();
		//return break_calculate_cancel;
	}

	return break_calculate_continue;
}
uint8_t Break_Cancel(Obstacle_Information* cipv, _VEHICLE_PARA* stVehicleParas, Camera_Essential_Data* cam_essiential, Obstacle_Basic_Data* obs_basic_data){
	uint8_t return_value = break_cancel_NONE;
#ifdef USE_CAM_Warning_info
	if(USE_CAM_Warning_info == 1){
		approaching_ornot = Approaching_to_lead_car(cipv, approaching_history_n);
		if(approaching_ornot != 1){
			CMS_HMW_State_reset();// when the lead car go away ,shouldnt break HMW too;
			//return break_cancel_TTCHMW;
			return_value = break_cancel_HMW;
		}
		if(((*cipv).TTC < 0.0)|((*cipv).TTC > 6.0)){//|((cipv.RelativeSpeedZ < 0)&((abs(cipv.RelativeSpeedZ + stVehicleParas.fVehicleSpeed / 3.6) > 0.5)))){ /* lead car go away*/
			AEB_State_reset();
			CMS_TTC_State_reset();

			if(LOGLEVEL == LOG_WARNING){
				//Log_print(LOG_WARNING, 0x02,0x00,"TTC < 0","Break_Trigger", cipv);
			}
			//return break_cancel_TTC;
			if(return_value == break_cancel_NONE)
				return_value = break_cancel_TTC;
			if(return_value == break_cancel_HMW)
				return_value = break_cancel_TTCHMW;
		}
		if((*cipv).TrackNumber < AEB_CMS_parameters.min_track_number){//|((cipv.RelativeSpeedZ < 0)&((abs(cipv.RelativeSpeedZ + stVehicleParas.fVehicleSpeed / 3.6) > 0.5)))){ /* lead car go away*/
			AEB_State_reset();
			CMS_TTC_State_reset();
			CMS_HMW_State_reset();// when the lead car go away ,shouldnt break HMW too;
			if(LOGLEVEL == LOG_WARNING){
				//Log_print(LOG_WARNING, 0x02,0x00,"TTC < 0","Break_Trigger", cipv);
			}
			//return break_cancel_TTCHMW;
			return_value = break_cancel_TTCHMW;

		}
		if(((*cam_essiential).HMWGrade == 0)|((*cam_essiential).HMWGrade == 3)){// need count
			if(LOGLEVEL == LOG_WARNING){
				Log_print(LOG_WARNING,0x05,0x00, "HMW_Grade ","Break_Cancel", cipv, stVehicleParas, cam_essiential,obs_basic_data);
			}
			CMS_HMW_State_reset();
			//return break_cancel_HMW;
			if(return_value == break_cancel_NONE)
				return_value = break_cancel_HMW;
			else if(return_value == break_cancel_TTC)
				return_value = break_cancel_TTCHMW;

		}
		if((*cam_essiential).FCWStatus == 0){
			if(LOGLEVEL == LOG_WARNING){
				Log_print(LOG_WARNING,0x05,0x00, "FCW_Status == 0 ","Break_Cancel", cipv, stVehicleParas, cam_essiential,obs_basic_data);
			}
			AEB_State_reset();
			CMS_TTC_State_reset();
			//return break_cancel_TTC;
			if(return_value == break_cancel_NONE)
				return_value = break_cancel_TTC;
			else if(return_value == break_cancel_HMW)
				return_value = break_cancel_TTCHMW;
		}
	}
	else{
		if((hmw_r_cal > AEB_CMS_parameters.Cam_HMW_warning_time)|(hmw_r_cal < 0)){
			if(LOGLEVEL == LOG_WARNING){
				Log_print(LOG_WARNING,0x05,0x00, "hmw ","Break_Cancel", cipv, stVehicleParas, cam_essiential,obs_basic_data);
			}
			CMS_HMW_State_reset();
			//return break_cancel_HMW;
			if(return_value == break_cancel_NONE)
				return_value = break_cancel_HMW;
			else if(return_value == break_cancel_TTC)
				return_value = break_cancel_TTCHMW;
		}
	}
#endif

	return return_value;
}
uint8_t Driver_control(Obstacle_Information* cipv, _VEHICLE_PARA* stVehicleParas, Camera_Essential_Data* cam_essiential, Obstacle_Basic_Data* obs_basic_data){
	if((*stVehicleParas).LeftFlagTemp ^ (*stVehicleParas).RightFlagTemp){/* turning left or right*/
		AEB_State_reset();
		CMS_State_reset();
		driver_control_gap_time_stamp = SystemtimeClock;
		return break_calculate_cancel;
	}
	if((*stVehicleParas).BrakeFlag == 1){
		AEB_State_reset();
		CMS_State_reset();
		driver_control_gap_time_stamp = SystemtimeClock;
		if(LOGLEVEL == LOG_WARNING){
			Log_print(LOG_WARNING,0x04,0x00, "Driver BrakeFlag","Driver_control", cipv, stVehicleParas, cam_essiential, obs_basic_data);
		}
		return break_calculate_cancel;
	}else if((*stVehicleParas).BrakeFlag == 0){
		if(((SystemtimeClock - driver_control_gap_time_stamp)/1000.0) < rParms.Brake_Cooling_Time){
			AEB_State_reset();
			CMS_State_reset();
			return break_calculate_cancel;
		}
	}
	return break_calculate_continue;
}

void AEB_State_reset(void){
	aeb_break_state.TTC_AEB = 0.0;
	aeb_break_state.aeb_TTC_is_break_flag = 0x00;
}
void CMS_State_reset(void){
	cms_break_state.cms_uradar_dec_out_prev = 0.0;
	cms_break_state.cms_uradar_dec_out = 0.0;
	cms_break_state.cms_hmw_dec_out_prev = 0.0;
	cms_break_state.cms_hmw_dec_out = 0.0;
	cms_break_state.cms_ttc_dec_out_prev = 0.0;
	cms_break_state.cms_ttc_dec_out = 0.0;
	cms_break_state.cam_distance = 0.0;
	cms_break_state.uradar_distance = 0.0;


	cms_break_state.cms_is_warning_flag = 0x00;
	cms_break_state.cms_is_break_flag = 0x00;
	cms_break_state.cms_cam_keep_count = 0x00;
	cms_break_state.cms_cam_delay_count = 0x00;
	cms_break_state.cms_cam_over_timestamp = 0x00000000;
	cms_break_state.cms_uradar_keep_count = 0x00;
	cms_break_state.cms_uradar_delay_count = 0x00;

	cms_break_state.cms_ttc_is_warning_flag = 0x00;
	cms_break_state.cms_ttc_is_break_flag = 0x00;
	cms_break_state.cms_ttc_cam_keep_count = 0x00;
	cms_break_state.cms_ttc_cam_delay_count = 0x00;
	cms_break_state.cms_ttc_uradar_keep_count = 0x00;
	cms_break_state.cms_ttc_uradar_delay_count = 0x00;
}
void CMS_HMW_State_reset(void){
	cms_break_state.cms_uradar_dec_out_prev = 0.0;
	cms_break_state.cms_uradar_dec_out = 0.0;
	cms_break_state.cms_hmw_dec_out_prev = 0.0;
	cms_break_state.cms_hmw_dec_out = 0.0;

	cms_break_state.cms_is_warning_flag = 0x00;
	cms_break_state.cms_is_break_flag = 0x00;
	cms_break_state.cms_cam_keep_count = 0x00;
	cms_break_state.cms_cam_delay_count = 0x00;
	cms_break_state.cms_cam_over_timestamp = 0x00000000;
	cms_break_state.cms_uradar_keep_count = 0x00;
	cms_break_state.cms_uradar_delay_count = 0x00;
}
void CMS_TTC_State_reset(void){
	cms_break_state.cms_ttc_dec_out_prev = 0.0;
	cms_break_state.cms_ttc_dec_out = 0.0;
	cms_break_state.cam_distance = 0.0;
	cms_break_state.uradar_distance = 0.0;

	cms_break_state.cms_ttc_is_warning_flag = 0x00;
	cms_break_state.cms_ttc_is_break_flag = 0x00;
	cms_break_state.cms_ttc_cam_keep_count = 0x00;
	cms_break_state.cms_ttc_cam_delay_count = 0x00;
	cms_break_state.cms_ttc_uradar_keep_count = 0x00;
	cms_break_state.cms_ttc_uradar_delay_count = 0x00;
	cms_break_state.cms_uradar_delay_count = 0x00;
}
void AEB_CMS_alg_init(void){
#if 0
	rParms.id = 0x01;
	// Status of writing or reading
//	rParms.rw_status = 0x00; // 0:reading ,can not writing; 1:read over,can write;2:writing, can not read; 3: write over can read
	// Global Function Switch
	rParms.Global_Switch == 235;
	/*******************************
	AEB_CMS_Enable 				 1
	CMS_HMW_Enable 				 1
	CMS_TTC_Enable 				 1
	AEB_TTC_Enable 				 0
	SSS_Enable     				 0
	HMW_Dynamic_Thr_Enable 		 0
	Chassis_Safe_Strategy_Enable 1
	LDW_Enable                   1
	 *******************************/
	// AEB、CMS common parameters
	rParms.Brake_Cooling_Time        	= 5.0;
	rParms.Driver_Brake_Cooling_Time 	= 5.0;
	rParms.Max_Brake_Keep_Time		 	= 5.0;
	rParms.Air_Brake_Delay_Time		 	= 0.2;
	rParms.Max_Percent_Decelerate	 	= 0.5;
	rParms.Min_Enable_Speed			 	= 20.0;
	rParms.Ratio_Force_To_Deceleration  = 1.0;
	// CMS parameters
	// CMS parameters - HMW
	rParms.CMS_HMW_Brake_Time_Thr			= 1.0;
	rParms.CMS_HMW_Brake_Force_Feel_Para	= 3.20;
	rParms.CMS_HMW_Warning_Time_Thr			= 1.0;
	rParms.CMS_HMW_Dynamic_Offset_Speed_L 	= 30.0;
	rParms.CMS_HMW_Dynamic_Offset_Value_L	= 0.5;
	rParms.CMS_HMW_Dynamic_Offset_Speed_H	= 60;
	rParms.CMS_HMW_Dynamic_Offset_Value_H	= 0.5;
	// CMS parameters - TTC
	rParms.CMS_TTC_Brake_Time_Thr			= 1.5;
	rParms.CMS_TTC_Brake_Force_Feel_Para	= 0.67;
	rParms.CMS_TTC_Warning_Time_Level_First = 1.4;
	rParms.CMS_TTC_Warning_Time_Level_Second= 0.8;
	// AEB
	rParms.AEB_Decelerate_Set				= 5.0;
	rParms.AEB_Stop_Distance				= 0.5;
	rParms.AEB_TTC_Warning_Time_Level_First = 1.4;
	rParms.AEB_TTC_Warning_Time_Level_Second= 0.8;
	// SSS
	rParms.SSS_Brake_Force					= 5.0;
	rParms.SSS_Break_Enable_Distance		= 0.8;
	rParms.SSS_Warning_Enable_Distance		= 2.0;
	rParms.SSS_Max_Enable_Speed				= 20.0;
	rParms.SSS_FR_FL_Install_Distance_To_Side = 0.3;
	rParms.SSS_Stop_Distance				= 0.3;
	rParms.SSS_Default_Turn_Angle			= 30.0;
	// Vehicle Speed
	rParms.WheelSpeed_Coefficient			= 0.0;
#endif
	////////////////////////////////////////////////////
	// TOthers
	AEB_CMS_parameters.max_enable_speed        = 120.0;    //km/h
	// recal HWM and TTC warning Level
	AEB_CMS_parameters.Cam_HMW_warning_time    = 1.0; //too close to lead car
	AEB_CMS_parameters.Cam_TTC_warning_1_time  = 1.5; //
	AEB_CMS_parameters.Cam_TTC_warning_2_time  = 1.0; //
	// Vehicle parameters
	AEB_CMS_parameters.ttc_max_dec_output	   = 5.0;
	AEB_CMS_parameters.Vehicle_max_react_dec   = 10.0;
	AEB_CMS_parameters.Vehicle_min_react_dec   = 0.5;
	// parameters of CIPV
	AEB_CMS_parameters.cipv_max_distance       = 50.0;
	AEB_CMS_parameters.min_track_number        = 10.0;
	// Parameters of Uradar
	AEB_CMS_parameters.Uradar_max_enable_speed = 20.0;
}

float Get_Uradar_distance(URADER_MESSAGE urader_company_message[], _VEHICLE_PARA* stVehicleParas)
{
	//fprintf(USART1_STREAM,"Get_Uradar_distance \r\n");
	if(((*stVehicleParas).fVehicleSpeed > AEB_CMS_parameters.Uradar_max_enable_speed)|((*stVehicleParas).fVehicleSpeed <= 0)){
		return 5.04;
	}
	float position = 0;
	for(uint8_t i = 0;i < 2;i ++)
	{
		for(uint8_t j = 0;j < 12;j ++)
		{
			if(urader_company_message[i].Urader_work_stata[j] == URWORK)
			{

				if((urader_company_message[i].Uposition[j] == FRONT )|
				   (urader_company_message[i].Uposition[j] == FRMIDE)|
				   (urader_company_message[i].Uposition[j] == FLMIDE))
				{
					position = min(position, urader_company_message[i].distance[j]);
				}
			}
		}
	}
	if((position >0)&(position < 5.0)){
		return position;
	}
	return 5.04;
}
void Filter_noise(float *dec_out, float dec_out_k,uint8_t *is_break_flag, uint8_t *count_delay, uint8_t *count_keep){
	return;
	if((*dec_out > 0.0)&(*is_break_flag == 0x00)){
		*count_delay = *count_delay + 1;
		if(*count_delay < delay_count){
			*dec_out = 0.0;
		}
		else{
			*count_delay = 0;//delay_count + 1;
			*is_break_flag = 0x01;
		}
	}
	else if((*dec_out == 0.0)&(*is_break_flag == 0x00)){
		*count_delay = 0x00;
	}

	// 过滤高频消失噪声
	if((*dec_out == 0.0)&(*is_break_flag == 0x01)){
		*count_keep = *count_keep + 1;
		if(*count_keep < keep_count){
			*dec_out = dec_out_k;
			//cms_break_state.cms_break_flag = 0;
		}else{
			*count_keep = 0;//keep_count + 1;
			*is_break_flag = 0x00;
			//cms_break_state.cms_break_flag = 1;
		}
	}else if((*dec_out > 0.0)&(*is_break_flag == 0x01)){
		*count_keep = 0;
	}
}

void Log_print(enum LOG_LEVEL level, uint8_t cancel_n, uint8_t func_n, const char* description, const char* function_name, Obstacle_Information* cipv,_VEHICLE_PARA* stVehicleParas,Camera_Essential_Data* Cam_Essi_data,Obstacle_Basic_Data* obs_basic_data){
	//fprintf(USART1_STREAM,"DEBUG:");
	switch (level){
		case LOG_NONE:
			break;
		case LOG_ERROR:
			fprintf(USART1_STREAM,"ERROR: ego_speed: ,%.6f,  decp_out: ,%6.6f,  function: %s \r\n",ego_speed, dec_output,function_name);
			break;
		case LOG_WARNING:
			//fprintf(USART1_STREAM,"WARNING:ego_speed: %.6f,  decp_out: ,%6.6f, description: %s  function: %s \r\n",ego_speed, dec_output,description,function_name);
			fprintf(USART1_STREAM,"WARNING_speed_dec_cancleNum_functionNum:%.6f,%6.6f,%d,%d,%d,%s,%s \r\n",ego_speed, dec_output,cancel_n,func_n,0,description,function_name);
			break;
		case LOG_DEBUG:
			/*fprintf(USART1_STREAM,"DEBUG: ,%.6f, function: ,%s ",dec_output,function_name);
			fprintf(USART1_STREAM,"Ego_Speed: ,%6.6f ,Dec_out: ,%6.6f, TTC: ,%6.6f HMW: ,%6.6f CAM_distance: %6.6f CMS_HMW_FLAG: %d cms_hmw_decout: %6.6f  CMS_TTC_FLAG: %d cms_ttc_decout: %6.6f AEB_TTC_FLAG: %d aeb_ttc_decout: %6.6f", \
									ego_speed, dec_output,cipv.TTC,cipv.HMW,cipv.DistanceZ, cms_break_state.cms_is_break_flag, cms_break_state.cms_hmw_dec_out, cms_break_state.cms_ttc_is_break_flag, cms_break_state.cms_ttc_dec_out, aeb_break_state.aeb_TTC_is_break_flag, aeb_break_state.TTC_AEB_dec_out);
			fprintf(USART1_STREAM,"Relative_Speed_Z: ,%6.6f,  Relative_Speed_X: ,%6.6f \r\n",cipv.RelativeSpeedZ, cipv.RelativeSpeedX);
			fprintf(USART1_STREAM,"Urada_distance: %6.6f \r\n",cms_break_state.uradar_distance);*/
			fprintf(USART1_STREAM,"speed_dec_TTC_HMW_life:,%6.6f,%6.6f,%6.6f,%6.6f,%d \r\n",ego_speed, dec_output,(*cipv).TTC,(*cipv).HMW,(*cipv).TrackNumber);
			break;
		case LOG_IN_OUT:
			//speed+decout+CIPV+TTC+FCWStatus+HMW+HMW_Grade+
			/*fprintf(USART1_STREAM,"speed=%6.6f,dec_out=%6.6f,CIPV=%d,TTC=%6.6f,FCW_Status=%d,HMW=%6.6f,HMW_Grade=%d,Break_flag=%d,R_X_speed=%6.6f,R_Z_speed=%6.6f,Cam_distance=%6.6f,TrackNumber=%d,Urard_distance=%6.6f,ttc_aeb_dec=%6.6f,ttc_cms_dec=%6.6f,hmw_cms_dec=%6.6f\r\n",\
					ego_speed, dec_output,cipv.CIPV,cipv.TTC,Cam_Essi_data.FCWStatus,cipv.HMW,Cam_Essi_data.HMWGrade,stVehicleParas.BrakeFlag,cipv.RelativeSpeedX,cipv.RelativeSpeedZ,cipv.DistanceZ,cipv.TrackNumber,Uradar_distance,ttc_aeb_dec,ttc_cms_dec,hmw_cms_dec);*/
			//fprintf(USART1_STREAM,"t=%6.2f,v=%6.2f,dec=%6.2f,cipvn=%d,cipv=%d,TTC=%6.2f,ttc=%6.2f,FCW=%d,HMWn=%6.2f,HMW=%6.2f,HMWG=%d,Bf=%d,RXv=%6.2f,RZv=%6.2f,Camd=%6.2f,age=%d,Ud=%6.2f,ttcaeb=%6.2f,ttccms=%6.2f,hmwcms=%6.2f,TL=%d,TR=%d,R=%d\r\n",\
			//		SystemtimeClock/6000,ego_speed, dec_output,cipv_now.CIPV,cipv.CIPV,cipv.TTC,ttc_r_cal,Cam_Essi_data.FCWStatus,cipv_now.HMW,cipv.HMW,Cam_Essi_data.HMWGrade,stVehicleParas.BrakeFlag,cipv.RelativeSpeedX,cipv.RelativeSpeedZ,cipv.DistanceZ,cipv.TrackNumber,Uradar_distance,ttc_aeb_dec,ttc_cms_dec,hmw_cms_dec,stVehicleParas.LeftFlagTemp,stVehicleParas.RightFlagTemp,stVehicleParas.ReverseGear);
			fprintf(USART1_STREAM,"\r\n %ld,Tid=%d,Tid=%d,wv=%0.2f,v=%0.2f,dec=%0.2f,appro=%d,ID=%d,type=%d,cipvgot=%d,cipv=%d,TTC=%0.2f,ttc=%0.2f,FCW=%d,HMW=%0.2f,hmw=%0.2f",\
								SystemtimeClock, obs_basic_data_history.TimeID,(*obs_basic_data).TimeID,Get_Wheel_Speed()/10.0,ego_speed*3.6,dec_output,approaching_ornot,cipv_history.ObstacleID,cipv_history.ObstacleType,got_cipv_history,cipv_history.CIPV,cipv_history.TTC,ttc_r_cal,cam_essiential_history.FCWStatus,cipv_history.HMW,hmw_r_cal);
			fprintf(USART1_STREAM,",HMWG=%d,Bf=%d,W=%0.2f,H=%0.2f,RXv=%0.2f,RZv=%0.2f,Camdz=%0.2f,Camdx=%0.2f,age=%d,Ud=%0.2f,ttcaeb=%0.2f,ttccms=%0.2f,hmwcms=%0.2f,TL=%d,TR=%d,R=%d\r\n",\
					cam_essiential_history.HMWGrade,(*stVehicleParas).BrakeFlag,cipv_history.ObstacleWidth,cipv_history.ObstacleHeight,cipv_history.RelativeSpeedZ,cipv_history.RelativeSpeedX,cipv_history.DistanceZ,cipv_history.DistanceX,cipv_history.TrackNumber,Uradar_distance,ttc_aeb_dec,ttc_cms_dec,hmw_cms_dec,(*stVehicleParas).LeftFlagTemp,(*stVehicleParas).RightFlagTemp,(*stVehicleParas).ReverseGear);
			/*fprintf(USART1_STREAM,"\r\n wv=%0.2f,v=%0.2f",\
											Get_Wheel_Speed()/10.0,ego_speed*3.6);
											*/
			//fprintf(USART1_STREAM,"dec=%0.2f,Camdz_can=%0.2f Camdz_history=%0.2f\r\n",dec_output, obstacle_cipv_data.DistanceZ,cipv_history.DistanceZ);

			break;
		default:
			break;
	}
	return;
}

//for test
uint16_t m_pressure_xj;
static uint32_t Ret_test_time_tamp = 0;
float  Ret_test(void){
	if((SystemtimeClock - Ret_test_time_tamp) > 30){
		if(m_pressure_xj <= 50){
			m_pressure_xj = 500;//m_pressure_xj + 50;//ret_pressure + 3;
		}
		else if(m_pressure_xj > 50){
			m_pressure_xj = m_pressure_xj - 10;
		}
		else{
			m_pressure_xj = 0;
		}
		Ret_test_time_tamp = SystemtimeClock;
	}
	//m_pressure_xj = 0;

	//m_pressure_xj  = (uint16_t)(ret_pressure * 100);
	/*tx_valve_control.data[0] = (uint8_t)(m_pressure_xj & 0x00FF);
	tx_valve_control.data[1] = (uint8_t)((m_pressure_xj & 0xFF00) >> 8);
	*tx_frame = tx_valve_control;*/
	//fprintf(USART1_STREAM,"SystemtimeClock: %d Set_Pressure: %d  tx_valve_control: %d ",SystemtimeClock,m_pressure_xj,tx_valve_control);

	return (float)m_pressure_xj;
}

