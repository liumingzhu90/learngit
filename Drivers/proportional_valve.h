/*
 * proportional_valve.h
 *
 *  Created on: 2021-6-18
 *      Author: shuai
 */

#ifndef PROPORTIONAL_VALVE_H_
#define PROPORTIONAL_VALVE_H_
#include "system_init.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stdio.h"
#include "canhl.h"

#define VALVE_CAN_BAUDRATE CAN_BAUDRATE_250K

#define VALVE_CONTROL_ID 0x08FF8100
#define VALVE_CONTROL_REFRESH_RATE 20

#define VALVE_STATE_ID 0x08FF1081
#define VALVE_STATE_REFRESH_RATE 20

#define VALVE_FAULT_CODE_ID 0x18FF0A81
#define VALVE_FAULT_CODE_REFRESH_RATE 100

#define VALVE_PRESSURE_LIMIT_VALUE 7.5;

typedef enum
{
	No_Fault = 0,
	Internal_Fault = 1, //Action�� Enter failsafe mode. Recover�� Power off and restart.
	Low_Voltage = 2, //Lower than 7.5V.Action�� Enter failsafe mode.Recover�� voltage higher than 7.5V.
	High_Voltage = 3, //Higher than 37V.Action�� hardware power cut off to protect.Recover�� voltage lower than 36V.
	Big_Pressure_Delta = 4, //pressure delta bigger than 1bar.Action�� Continue 50s then enter failsafe mode.Recover�� Power off and restart.
	Never_Receive_Order = 5, //Never receive pressure control order:not receive pressure order after turn on 50s.
	Control_Order_Lost = 6, //Pressure control order lost�� after receive order 1s, no order received again.
	CAN_Bus_Off = 7, //CAN bus Off: just keep it, no display
	Sensor_Open_Circuit = 8, //Pressure sensor open circuit.Action�� Enter failsafe mode.Recover�� Power off and restart.
	Sensor_Short_Circuit = 9, //Pressure sensor short circuit.Action�� Enter failsafe mode.Recover�� Power off and restar.
	Valve_Open_Circuit = 10, //Proportional valve open circuit.Action�� Enter failsafe mode.Recover�� Power off and restart.
	Valve_Short_Circuit = 11 //Proportional valve short circuit.Action�� Enter failsafe mode.Recover�� Power off and restart.
}Valve_Fault_Code;

typedef struct
{
	uint8_t ValveState; //0x00 Not used 0x01 Pressure Control 0x02 Fail Safe 0x03 Other
	int8_t InternalTemp; //measurement limited from -40��C to 150��C 0xFB reserved 0xFC reserved 0xFD reserved 0xFE measurement error 0xFF not available
	float ActualPressure; //Range Used: 0 - 960
	float TargetPressure; //Range Used: 0 - 960
	uint8_t FaultCode; //See Description of the fault codes
	uint8_t Free; //0xFF

}Valve_State;

typedef enum
{
	No_Working = 0, //δִ��
	In_The_Climbing = 1, //ѹ��������
	In_The_Keep = 2, //ѹ��������
	In_The_Downhill = 3, //ѹ�������
	In_Cooling_Time = 4 //��������ȴ��
}Control_Model_State;

typedef struct
{
	int operating_state; //ģ�͹���״̬��0δִ�У�1ѹ�������У�2ѹ�������У�3ѹ�������
	float minimum_pressure; //��ʼ��Сѹǿ��Ĭ��Ϊ0.0KP
	float maxinum_pressure; //���ѹǿ��������֧�����7.5KP��
	int maxinum_pressure_time; //���ѹǿ����ʱ�䣬��λ���롣
	int climbing_time; //�������߳���ʱ�䣬��λ���롣
	int total_braking_time; //�����ƶ����̳���ʱ�䣬��λ���롣
	int cooling_time; //�ƶ�������һ��ʱ�����ȴ�ڡ���λ���롣
}Pressure_Control_Model;

extern Pressure_Control_Model valve_control_model;
extern SemaphoreHandle_t valveMutex;

void Valve_Init();
BaseType_t Valve_Pressure_Get(struct can_frame *tx_frame);
BaseType_t Valve_Target_Deceleration_Get(struct can_frame *tx_frame);
BaseType_t Valve_Deceleration_Request_Get(struct can_frame *tx_frame);
void Valve_CAN_Analysis(struct can_frame *rx_frame);
int Valve_Braking_Force_Cal(float ttc_value); //ɲ���ƶ������㣬����ѹ������ģ�͡�
static float Valve_Pressure_Cal(); //����ѹ������ģ�ͼ����������Ҫ�������ѹ��ֵ��
float Valve_break_Pressure(void);
float Valve_break_Target_Deceleration(void);
float Valve_break_Deceleration_Request(void);
#endif /* PROPORTIONAL_VALVE_H_ */
