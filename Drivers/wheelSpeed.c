/*
 * wheelSpeed.c
 *
 *  Created on: 2021-11-30
 *      Author: Administrator
 */
#include "wheelSpeed.h"
#include "usart.h"
#include "stdio.h"
#include "common.h"

/* -----------------------ȫ�ֱ���------------------------------- */
volatile uint16_t pulse			= 0;
volatile uint16_t oldpulse_count= 0;
uint16_t wheelSpeed 			= 0;
uint32_t clock_old 				= 0;

static uint8_t cir_num                 = 0;
static uint8_t min_plus                = 5;
static uint32_t save_plus_num          = 0;

static uint8_t pointer_first           = 0;
static float cache_first[9];

static uint8_t pointer_second          = 0;
static float cache_second[9];

static uint8_t loop_time_gap           = 10;
static uint8_t filter_size             = 5;
static uint8_t double_size             = 0;
static uint8_t min_time_count          = 10;//���0.1s����Ҫ����һ�γ���
static uint8_t max_time_count          = 40;//�0.4s����Ҫ����һ�γ���

/* -----------------------�ֲ���������------------------------------- */
void Calc_Wheel_Speed(uint16_t pulse);
/*
 * ���ٳ�ʼ����
 */
void WheelSpeed_Init()
{
	//���ܽ���ӳ��PC4ΪT2CK
	GPIO_Write_Mode_Bits (GPIOC_SFR,GPIO_PIN_MASK_4, GPIO_MODE_RMP);  	//������ӳ��
	GPIO_Pin_RMP_Config(GPIOC_SFR,GPIO_Pin_Num_4,GPIO_RMP_AF1_T2);     	//��ӳ�书��

	TIM_Reset(T2_SFR);         								 //��ʱ�����踴λ��ʹ������ʱ��
	GPTIM_InitTypeDef gptimInitStruct_2;
	GPTIM_Struct_Init (&gptimInitStruct_2);

	gptimInitStruct_2.m_Counter		= 0;    				//д��cnt
	gptimInitStruct_2.m_CounterMode	= GPTIM_COUNT_UP_OF;
	gptimInitStruct_2.m_Period		= 0xFFFF;
	gptimInitStruct_2.m_WorkMode	= GPTIM_COUNTER_MODE;	//����ģʽ
	gptimInitStruct_2.m_Prescaler	= 0;    				//��Ƶ
//	gptimInitStruct_2.m_Clock		= GPTIM_SCLK;
//	gptimInitStruct_2.m_SlaveMode	= GPTIM_SLAVE_COUNTER_MODE;
//	gptimInitStruct_2.m_EXPulseSync	= GPTIM_NO_SYNC_MODE;

	GPTIM_Updata_Immediately_Config(T2_SFR,TRUE);    		//�������¿���
	GPTIM_Updata_Enable(T2_SFR,TRUE);       				//���ø���ʹ��
	GPTIM_Configuration(T2_SFR,&gptimInitStruct_2);
	GPTIM_Cmd(T2_SFR,TRUE);	  								//ʹ��t2������
	// filter init
	double_size = filter_size * 2 - 1;
}
/*
 * ��ȡ�󳵱�������������
 */
uint16_t Calc_Bus_Encoder_Velocity()
{
	if(SystemtimeClock - clock_old > loop_time_gap){
		clock_old = SystemtimeClock;
		uint16_t pulse_count= (uint16_t) (T2_SFR->CNT);

		if(pulse_count - oldpulse_count >= 0)
			pulse = pulse_count - oldpulse_count;
		else if(pulse_count - oldpulse_count < 0)
			pulse = 65535 - oldpulse_count + pulse_count;
		else{
			oldpulse_count 	= pulse_count;
			pulse_count		= 0;
			return pulse;
		}

		if(((pulse < min_plus)&(pulse >= 0))|(cir_num < min_time_count)){
			cir_num += 1;
			//Calc_Wheel_Speed(save_plus_num);
			if(cir_num >= max_time_count){
				cir_num -=1;
				Calc_Wheel_Speed(pulse);
				//fprintf(USART1_STREAM,"low_:p=%d,n=%d,wv=%0.2f\r\n",pulse,cir_num,wheelSpeed*0.1);
				cir_num = 0;
				oldpulse_count 	= pulse_count;
				pulse_count		= 0;

			}
		}
		else{
			//stVehicleParas.fVehicleSpeed = pulse*1.0;
			Calc_Wheel_Speed(pulse);
			//fprintf(USART1_STREAM,"p=%d,n=%d,wv=%0.2f\r\n",pulse,cir_num,wheelSpeed*0.1);
			cir_num = 0;
			oldpulse_count 	= pulse_count;
			pulse_count		= 0;

			// ���ñ������������������ٶ����߷���
			//fprintf(USART1_STREAM,"%d,%0.2f,%d\r\n",pulse,stVehicleParas.fVehicleSpeed*10,wheelSpeed);
		}


/*lop:
	oldpulse_count 	= pulse_count;
	pulse_count		= 0;*/
	}


	return pulse;
}
/***
 * �����˲�,�����˲�
 */
float Filter_Average_first(float data_input, uint8_t filter_size){
	if(pointer_first < filter_size){
		cache_first[pointer_first] = data_input;
		pointer_first += 1;
		return data_input;
	}
	else{
		cache_first[pointer_first - filter_size] = data_input;
		pointer_first += 1;
		if(pointer_first > double_size)
			pointer_first = filter_size;
		float output;
		output = 0.0;
		for(uint8_t i = 0; i < filter_size; i++){
			output += cache_first[i];
		}
		return output/(float)(filter_size);
	}
}
/***
 * �����˲�,�����˲�
 */
float Filter_Average_second(float data_input, uint8_t filter_size){
	if(pointer_second < filter_size){
		cache_second[pointer_second] = data_input;
		pointer_second += 1;
		return data_input;
	}
	else{
		cache_second[pointer_second - filter_size] = data_input;
		pointer_second += 1;
		if(pointer_second > double_size)
			pointer_second = filter_size;
		float output;
		output = 0.0;
		for(uint8_t i = 0; i < filter_size; i++){
			output += cache_second[i];
		}
		return output/(float)(filter_size);
	}
}
/*
 * ��������
 */
void Calc_Wheel_Speed(uint16_t pulse)
{
//	float speed = 0.4847 * pulse + 5.549;	// ��ȡֵ ƫ��
//	float speed = 0.4841 * pulse + 5.2912;	// ��ȡֵ
//	float speed = 0.4844 * pulse + 5.4188;	// ƽ��ֵ
	//float speed = 1.0674 * pulse - 1.255;	// ����
	float speed = 680.0 * (float)(pulse) / (float)((cir_num + 1)*loop_time_gap);
	if(speed > 5){
		speed = Filter_Average_first(speed , filter_size);
		speed = Filter_Average_second(speed , filter_size);
	}
	//speed = Filter_Average(speed , filter_size);.
	wheelSpeed = (uint16_t)(speed * 10);
}

/*
 * ��ȡ���ٽӿ�
 * ��������
 * ���أ���������
 * ˵����1�����ٷŴ�10����2����λkm/h
 */
uint16_t Get_Wheel_Speed()
{
	return wheelSpeed;
}
