/*
 * proportional_valve.c
 *
 *  Created on: 2021-6-18
 *      Author: shuai
 */
#include "proportional_valve.h"
#include "aebs_task.h"
#include "vehicle_can.h"
#include "common.h"
#include "gpio.h"

struct can_frame tx_valve_control;
struct can_frame can_recv_valve_control;
float pressure_set = 2.0;
struct can_frame rx_valve_state;
struct can_frame rx_valve_fault_code;
extern Decision_Control_Information aebsControlInfo;
extern Vehicle_Info veh_info;
Valve_State valve_state_message;
Pressure_Control_Model valve_control_model;
SemaphoreHandle_t valveMutex = NULL;

void Valve_Init()
{
	can_recv_valve_control.TargetID = 0x0;
	can_recv_valve_control.lenth = 8;
	can_recv_valve_control.MsgType = CAN_DATA_FRAME;
	can_recv_valve_control.RmtFrm = CAN_FRAME_FORMAT_SFF;
	can_recv_valve_control.RefreshRate = VALVE_CONTROL_REFRESH_RATE;

	tx_valve_control.TargetID = VALVE_CONTROL_ID;
	tx_valve_control.lenth = 8;
	tx_valve_control.MsgType = CAN_DATA_FRAME;
	tx_valve_control.RmtFrm = CAN_FRAME_FORMAT_EFF;
	tx_valve_control.RefreshRate = VALVE_CONTROL_REFRESH_RATE;

	rx_valve_state.TargetID = VALVE_STATE_ID;
	rx_valve_state.lenth = 8;
	rx_valve_state.MsgType = CAN_DATA_FRAME;
	rx_valve_state.RmtFrm = CAN_FRAME_FORMAT_EFF;
	rx_valve_state.RefreshRate = VALVE_STATE_REFRESH_RATE;

	rx_valve_fault_code.TargetID = VALVE_FAULT_CODE_ID;
	rx_valve_fault_code.lenth = 8;
	rx_valve_fault_code.MsgType = CAN_DATA_FRAME;
	rx_valve_fault_code.RmtFrm = CAN_FRAME_FORMAT_EFF;
	rx_valve_fault_code.RefreshRate = VALVE_FAULT_CODE_REFRESH_RATE;

	for(int i=0; i<8; i++)
	{
		tx_valve_control.data[i] = 0xFF;
		rx_valve_state.data[i] = 0xFF;
		rx_valve_fault_code.data[i] = 0xFF;
		can_recv_valve_control.data[i] = 0xFF;
	}

	valve_state_message.ValveState = 0;
	valve_state_message.InternalTemp = 0;
	valve_state_message.ActualPressure = 0.0;
	valve_state_message.TargetPressure = 0.0;
	valve_state_message.FaultCode = 0;
	valve_state_message.Free = 0xFF;

	valve_control_model.minimum_pressure = 0;
	valve_control_model.operating_state = 0;
	valve_control_model.maxinum_pressure = 7.5;
	valve_control_model.maxinum_pressure_time = 0;
	valve_control_model.climbing_time = 0;
	valve_control_model.total_braking_time = 0;
	valve_control_model.cooling_time = 200;

	//valveMutex = xSemaphoreCreateRecursiveMutex();
}


float Valve_Pressure_Out_Set(float pressure)
{
	uint8_t turnleft = 0;
	uint8_t turnright = 0;

	turnleft = stVehicleParas.LeftFlagTemp;
	turnright = stVehicleParas.LeftFlagTemp;

	if(turnleft | turnright)
		pressure = 0;
	if(stVehicleParas.BrakeFlag == 1)
		pressure = 0;
	if(warning_status.AEBstatus != 0)
		pressure = 0;
	return pressure;
}

void Set_Break_Light_Stata(float pressure)
{
	if(pressure > 0)
		GPIO_Set_Output_Data_Bits(GPIOH_SFR,GPIO_PIN_MASK_14, 1);
	else
		GPIO_Set_Output_Data_Bits(GPIOH_SFR,GPIO_PIN_MASK_14, 0);
}

void Valve_TTC_break(float *pressure)
{
	uint8_t turnleft = 0;
	uint8_t turnright = 0;

	*pressure = Valve_break_Pressure();

	turnleft = stVehicleParas.LeftFlagTemp;
	turnright = stVehicleParas.LeftFlagTemp;

	if(turnleft | turnright)
		*pressure = 0;
	if(stVehicleParas.BrakeFlag == 1)
		*pressure = 0;
	if(warning_status.AEBstatus != 0)
		*pressure = 0;

}

/*
void Pressure_Type(uint8_t *type)
{
	if(CameraMessage.ttc < stSysPara.ttc1)
		*type = 1;
	else if()
}
*/

float distance = 0;
float position2 = 0;

uint8_t Urader_Bbreak_Speed(uint8_t speed,float position)
{
	float k;
	uint8_t upressure = 0;

	position2 = position;
	if(speed == 0)
	{
		distance = (float)stUraderSysMessage.speed0distance * 0.1;
	}
	if((speed > 0) && (speed < 5))
	{
		k = (float)(((float)(stUraderSysMessage.speed5distance - stUraderSysMessage.speed0distance) * 0.1)/ (5 - 0));
		distance = k * speed + (float)stUraderSysMessage.speed0distance * 0.1;
	}
	if(speed == 5)
	{
		distance = stUraderSysMessage.speed5distance * 0.1;
	}
	if((speed > 5) && (speed < 10))
	{
		k = (float)(((float)(stUraderSysMessage.speed10distance - stUraderSysMessage.speed5distance) * 0.1)/ (10 - 5));
		distance = k * speed + (float)stUraderSysMessage.speed0distance * 0.1;
	}
	if(speed == 10)
	{
		distance = (float)stUraderSysMessage.speed10distance * 0.1;
	}
	if((speed > 10) && (speed < 15))
	{
		k = (float)(((float)(stUraderSysMessage.speed15distance - stUraderSysMessage.speed10distance) * 0.1)/ (15 - 10));
		distance = k * speed + (float)stUraderSysMessage.speed0distance * 0.1;
	}
	if(speed == 15)
	{
		distance = (float)stUraderSysMessage.speed15distance * 0.1;
	}
	if((speed > 15) && (speed < 20))
	{
		k = (float)(((float)(stUraderSysMessage.speed20distance - stUraderSysMessage.speed15distance) * 0.1)/ (20 - 15));
		distance = k * speed + (float)stUraderSysMessage.speed0distance * 0.1;
	}
	if(speed == 20)
	{
		distance = (float)stUraderSysMessage.speed20distance * 0.1;
	}
	if((speed > 20) && (speed < 25))
	{
		k = (float)(((float)(stUraderSysMessage.speed25distance - stUraderSysMessage.speed20distance) * 0.1)/ (25 - 20));
		distance = k * speed + (float)stUraderSysMessage.speed0distance * 0.1;
	}
	if(speed == 25)
	{
		distance = (float)stUraderSysMessage.speed25distance * 0.1;
	}
	if(speed > 25)
	{
		distance = (float)stUraderSysMessage.speed25distance * 0.1;
	}
	distance = distance + (speed / 2 + 1)* 0.5;
	//fprintf(USART1_STREAM,"%.2f  %.2f  %.2f\r\n",k,distance,position2);
	if(distance > position2)
	{
		upressure = upressure | 0x01;
	}
	return upressure;
}



//0无转向  1左转  2右转
uint8_t Urader_Break_Pressure(void)
{
	uint8_t upressure = 0;
	uint8_t i = 0,j = 0;
	uint8_t distance = 0;
	uint8_t speed = 0;
	float position = 0;
	uint32_t disgree = 0;
	uint8_t turnleft = 0;
	uint8_t turnright = 0;
	uint8_t dis = 0;

	turnleft = stVehicleParas.LeftFlagTemp;
	turnright = stVehicleParas.RightFlagTemp;

	if((turnleft == 0) && (turnright == 0))
		dis = 0;
	else if((turnleft == 1) && (turnright == 0))
		dis = 1;
	else if((turnleft == 0) && (turnright == 1))
		dis = 2;
	else
		dis = 0;

	disgree = stSWAParas.SWADegree;

	if(dis == 0)			//当没有转向或双闪
	{
		i = 0;
		j = 0;
		if(Urader_Company_Message[i].Urader_work_stata[j] == URWORK)
		{
			position = Urader_Company_Message[i].distance[j];
			speed = stVehicleParas.fVehicleSpeed;
			if((Urader_Company_Message[i].Uposition[j] == FRONT)
				| (Urader_Company_Message[i].Uposition[j] == FLMIDE)
				| (Urader_Company_Message[i].Uposition[j] == FRMIDE))
			{
						//fprintf(USART1_STREAM,"%d %d %d %.2f\r\n",i,j,Urader_Company_Message[i].Uposition[j],position);
						upressure = upressure | Urader_Bbreak_Speed(speed,position);
			}
		}
		i = 0;
		j = 1;
		if(Urader_Company_Message[i].Urader_work_stata[j] == URWORK)
		{
			position = Urader_Company_Message[i].distance[j];
			speed = stVehicleParas.fVehicleSpeed;
			if((Urader_Company_Message[i].Uposition[j] == FRONT)
				| (Urader_Company_Message[i].Uposition[j] == FLMIDE)
				| (Urader_Company_Message[i].Uposition[j] == FRMIDE))
			{
						//fprintf(USART1_STREAM,"%d %d %d %.2f\r\n",i,j,Urader_Company_Message[i].Uposition[j],position);
						upressure = upressure | Urader_Bbreak_Speed(speed,position);
			}
		}
		i = 0;
		j = 2;
		if(Urader_Company_Message[i].Urader_work_stata[j] == URWORK)
		{
			position = Urader_Company_Message[i].distance[j];
			speed = stVehicleParas.fVehicleSpeed;
			if((Urader_Company_Message[i].Uposition[j] == FRONT)
				| (Urader_Company_Message[i].Uposition[j] == FLMIDE)
				| (Urader_Company_Message[i].Uposition[j] == FRMIDE))
			{
						//fprintf(USART1_STREAM,"%d %d %d %.2f\r\n",i,j,Urader_Company_Message[i].Uposition[j],position);
						upressure = upressure | Urader_Bbreak_Speed(speed,position);
			}
		}

		//for(i = 0;i < 2;i ++)
		//{
		//	for(j = 0;j < 12;j ++)
		//	{
		//		if(Urader_Company_Message[i].Urader_work_stata[j] == URWORK)
		//		{
		//			position = Urader_Company_Message[i].distance[j];
		//			speed = stVehicleParas.fVehicleSpeed;
		//			if((Urader_Company_Message[i].Uposition[j] == FRONT)
		//					| (Urader_Company_Message[i].Uposition[j] == FLMIDE)
		//					| (Urader_Company_Message[i].Uposition[j] == FRMIDE))
		//			{
						//fprintf(USART1_STREAM,"%d %d %d %.2f\r\n",i,j,Urader_Company_Message[i].Uposition[j],position);
		//				upressure = upressure | Urader_Bbreak_Speed(speed,position);
		//			}
		//		}
		//	}
		//}
	}
	else if(dis == 2)
	{
		i = 0;
		j = 3;
		if(Urader_Company_Message[i].Urader_work_stata[j] == URWORK)
		{
			position = Urader_Company_Message[i].distance[j];
			speed = stVehicleParas.fVehicleSpeed;
			//fprintf(USART1_STREAM,"%d %d = %.2f  %.2f\r\n",i,j,position,speed);
			if((Urader_Company_Message[i].Uposition[j] == LEFT)
					| (Urader_Company_Message[i].Uposition[j] == FLEFT)
					| (Urader_Company_Message[i].Uposition[j] == FLSIDE)
					| (Urader_Company_Message[i].Uposition[j] == FLMIDE))
			{

				upressure = upressure | Urader_Bbreak_Speed(speed,position);
			}
		}
		i = 0;
		j = 4;
		if(Urader_Company_Message[i].Urader_work_stata[j] == URWORK)
		{
			position = Urader_Company_Message[i].distance[j];
			speed = stVehicleParas.fVehicleSpeed;
			//fprintf(USART1_STREAM,"%d %d = %.2f  %.2f\r\n",i,j,position,speed);
			if((Urader_Company_Message[i].Uposition[j] == LEFT)
					| (Urader_Company_Message[i].Uposition[j] == FLEFT)
					| (Urader_Company_Message[i].Uposition[j] == FLSIDE)
					| (Urader_Company_Message[i].Uposition[j] == FLMIDE))
			{

				upressure = upressure | Urader_Bbreak_Speed(speed,position);
			}
		}
		i = 0;
		j = 5;
		if(Urader_Company_Message[i].Urader_work_stata[j] == URWORK)
		{
			position = Urader_Company_Message[i].distance[j];
			speed = stVehicleParas.fVehicleSpeed;
			//fprintf(USART1_STREAM,"%d %d = %.2f  %.2f\r\n",i,j,position,speed);
			if((Urader_Company_Message[i].Uposition[j] == LEFT)
					| (Urader_Company_Message[i].Uposition[j] == FLEFT)
					| (Urader_Company_Message[i].Uposition[j] == FLSIDE)
					| (Urader_Company_Message[i].Uposition[j] == FLMIDE))
			{

				upressure = upressure | Urader_Bbreak_Speed(speed,position);
			}
		}
	}

		/*
		for(i = 0;i < 2;i ++)
		{
			for(j = 0;j < 12;j ++)
			{
				if(Urader_Company_Message[i].Urader_work_stata[j] == URWORK)
				{
					position = Urader_Company_Message[i].distance[j];
					speed = stVehicleParas.fVehicleSpeed;
					//fprintf(USART1_STREAM,"%d %d = %.2f  %.2f\r\n",i,j,position,speed);
					if((Urader_Company_Message[i].Uposition[j] == LEFT)
							| (Urader_Company_Message[i].Uposition[j] == FLEFT)
							| (Urader_Company_Message[i].Uposition[j] == FLSIDE)
							| (Urader_Company_Message[i].Uposition[j] == FLMIDE))
					{

						upressure = upressure | Urader_Bbreak_Speed(speed,position);
					}
				}
			}
		}
		*/
	/*
	else
	{
		if(((disgree > 0) | (disgree == 0)) &&
				(disgree < 60))
		{
			for(i = 0;i < 2;i ++)
			{
				for(j = 0;j < 12;j ++)
				{
					if(Urader_Company_Message[i].Urader_work_stata[j] == URWORK)
					{
						position = Urader_Company_Message[i].distance[j] * stUraderSysMessage.secondstage;
						speed = stVehicleParas.fVehicleSpeed;
						if((Urader_Company_Message[i].Uposition[j] == FRONT)
								| (Urader_Company_Message[i].Uposition[j] == FLMIDE)
								| (Urader_Company_Message[i].Uposition[j] == FRMIDE))
						{
							upressure = upressure | Urader_Bbreak_Speed(position,speed);
						}
					}
				}
			}
		}
		if(((disgree > 60) | (disgree == 60)) &&
				(disgree < 200))
		{
			for(i = 0;i < 2;i ++)
			{
				for(j = 0;j < 12;j ++)
				{
					if(Urader_Company_Message[i].Urader_work_stata[j] == URWORK)
					{
						if(dis == 1)
						{
							position = Urader_Company_Message[i].distance[j];
							speed = stVehicleParas.fVehicleSpeed;
							if((Urader_Company_Message[i].Uposition[j] == FRONT) |
									(Urader_Company_Message[i].Uposition[j] == FRMIDE) |
									(Urader_Company_Message[i].Uposition[j] == LEFT) |
									(Urader_Company_Message[i].Uposition[j] == FLEFT))
							{
								position = position * stUraderSysMessage.secondstage;
								upressure = upressure | Urader_Bbreak_Speed(position,speed);
							}
							if(Urader_Company_Message[i].Uposition[j] == FLSIDE)
							{
								position = position * stUraderSysMessage.firststage;
								upressure = upressure | Urader_Bbreak_Speed(position,speed);
							}
							if(Urader_Company_Message[i].Uposition[j] == FLMIDE)
							{
								upressure = upressure | Urader_Bbreak_Speed(position,speed);
							}
						}
						if(dis == 2)
						{
							position = Urader_Company_Message[i].distance[j];
							speed = stVehicleParas.fVehicleSpeed;
							if((Urader_Company_Message[i].Uposition[j] == FRONT) |
									(Urader_Company_Message[i].Uposition[j] == FLMIDE) |
									(Urader_Company_Message[i].Uposition[j] == RIGHT) |
									(Urader_Company_Message[i].Uposition[j] == FRIGHT))
							{
								position = position * stUraderSysMessage.secondstage;
								upressure = upressure | Urader_Bbreak_Speed(position,speed);
							}
							if(Urader_Company_Message[i].Uposition[j] == FRSIDE)
							{
								position = position * stUraderSysMessage.firststage;
								upressure = upressure | Urader_Bbreak_Speed(position,speed);
							}
							if(Urader_Company_Message[i].Uposition[j] == FRMIDE)
							{
								upressure = upressure | Urader_Bbreak_Speed(position,speed);
							}
						}
					}
				}
			}
		}
		if(((disgree > 200) | (disgree == 200)) &&
				(disgree < 480))
		{

			for(i = 0;i < 2;i ++)
			{
				for(j = 0;j < 12;j ++)
				{
					if(Urader_Company_Message[i].Urader_work_stata[j] == URWORK)
					{
						if(dis == 1)
						{
							position = Urader_Company_Message[i].distance[j];
							speed = stVehicleParas.fVehicleSpeed;
							if((Urader_Company_Message[i].Uposition[j] == FLMIDE) |
									(Urader_Company_Message[i].Uposition[j] == FLSIDE))
							{
								upressure = upressure | Urader_Bbreak_Speed(position,speed);
							}
						}
						if(dis == 2)
						{
							position = Urader_Company_Message[i].distance[j];
							speed = stVehicleParas.fVehicleSpeed;
							if((Urader_Company_Message[i].Uposition[j] == FRMIDE) |
									(Urader_Company_Message[i].Uposition[j] == FRSIDE))
							{
								upressure = upressure | Urader_Bbreak_Speed(position,speed);
							}
						}
					}
				}
			}
		}
		if(disgree > 480)
		{
			for(i = 0;i < 2;i ++)
			{
				for(j = 0;j < 12;j ++)
				{
					if(Urader_Company_Message[i].Urader_work_stata[j] == URWORK)
					{
						if(dis == 1)
						{
							position = Urader_Company_Message[i].distance[j];
							speed = stVehicleParas.fVehicleSpeed;
							if((Urader_Company_Message[i].Uposition[j] == FLMIDE) |
									(Urader_Company_Message[i].Uposition[j] == FLEFT) |
									(Urader_Company_Message[i].Uposition[j] == LEFT) |
									(Urader_Company_Message[i].Uposition[j] == FLSIDE))
							{
								upressure = upressure | Urader_Bbreak_Speed(position,speed);
							}
						}
						if(dis == 2)
						{
							position = Urader_Company_Message[i].distance[j];
							speed = stVehicleParas.fVehicleSpeed;
							if((Urader_Company_Message[i].Uposition[j] == FRMIDE) |
									(Urader_Company_Message[i].Uposition[j] == FRIGHT) |
									(Urader_Company_Message[i].Uposition[j] == RIGHT) |
									(Urader_Company_Message[i].Uposition[j] == FRSIDE))
							{
								upressure = upressure | Urader_Bbreak_Speed(position,speed);
							}
						}
					}
				}
			}
		}
	}
	*/

	if(upressure != 0)
		stVehicleParas.BreakState = stVehicleParas.BreakState | 0x04;
	else
		stVehicleParas.BreakState = stVehicleParas.BreakState & 0xFB;

	return  upressure;
}

BaseType_t Valve_Deceleration_Request_Get(struct can_frame *tx_frame)
{
	uint8_t upressure = 0;
	float ret_pressure;
	struct can_frame frame;
	static uint8_t Rollong_Counter = 0;
	uint8_t sum = 0;
	uint8_t i = 0,j = 0;

	uint8_t m_pressure;
	uint8_t turnleft = 0;
	uint8_t turnright = 0;

	ret_pressure = Valve_break_Deceleration_Request();

	turnleft = stVehicleParas.LeftFlagTemp;
	turnright = stVehicleParas.RightFlagTemp;

	if(turnleft | turnright)
		ret_pressure = 16;
/*
	if((turnleft == 0) && (turnright == 0))
		upressure = Urader_Break_Pressure(0);
	else if((turnleft == 1) && (turnright == 0))
		upressure = Urader_Break_Pressure(1);
	else if((turnleft == 0) && (turnright == 1))
		upressure = Urader_Break_Pressure(2);
	else
		upressure = Urader_Break_Pressure(0);
	if(upressure == 1)									//当有超声波雷达刹车时，最大刹车力度
		ret_pressure = 0;
*/
	if(stVehicleParas.BrakeFlag == 1)
		ret_pressure = 16;
	if(warning_status.AEBstatus != 0)
		ret_pressure = 16;
	if(stVehicleParas.fVehicleSpeed == 0)
		ret_pressure = 16;								//实际值-7 ~ 5.75	offset -7
	m_pressure  = (uint8_t)(ret_pressure * 0x7ff);		//Factor ： 0.05
	frame.TargetID = 0x111;
	frame.lenth = 8;
	frame.MsgType = CAN_DATA_FRAME;
	frame.RmtFrm = CAN_FRAME_FORMAT_SFF;
	frame.RefreshRate = VALVE_CONTROL_REFRESH_RATE;//帧刷新时间
	frame.data[0] = (uint8_t)(m_pressure >> 8);
	frame.data[1] = (uint8_t)(m_pressure & 0xff);
	frame.data[6] = Rollong_Counter;
	for(i = 0;i < 8;i ++)
		sum = sum + frame.data[i];
	frame.data[7] = sum ^ 0xff;
	if(Rollong_Counter > 15)
		Rollong_Counter = 0;
	else
		Rollong_Counter ++;

	*tx_frame = frame;
	return pdTRUE;

}


BaseType_t Valve_Target_Deceleration_Get(struct can_frame *tx_frame)
{
	float ret_pressure;
	uint8_t upressure = 0;
	struct can_frame frame;
	uint8_t m_pressure;
	uint8_t turnleft = 0;
	uint8_t turnright = 0;

	ret_pressure = Valve_break_Target_Deceleration();

	turnleft = stVehicleParas.LeftFlagTemp;
	turnright = stVehicleParas.RightFlagTemp;

	if(turnleft | turnright)
		ret_pressure = 7;
/*
	if((turnleft == 0) && (turnright == 0))
		upressure = Urader_Break_Pressure(0);
	else if((turnleft == 1) && (turnright == 0))
		upressure = Urader_Break_Pressure(1);
	else if((turnleft == 0) && (turnright == 1))
		upressure = Urader_Break_Pressure(2);
	else
		upressure = Urader_Break_Pressure(0);
	if(upressure == 1)									//当有超声波雷达刹车时，最大刹车力度
		ret_pressure = 0;
*/
	if(stVehicleParas.BrakeFlag == 1)
		ret_pressure = 7;
	if(warning_status.AEBstatus != 0)
		ret_pressure = 7;
	if(stVehicleParas.fVehicleSpeed == 0)
		ret_pressure = 7;								//实际值-7 ~ 5.75	offset -7

	m_pressure  = (uint8_t)(ret_pressure / 0.05);		//Factor ： 0.05
	frame.TargetID = 0x111;
	frame.lenth = 8;
	frame.MsgType = CAN_DATA_FRAME;
	frame.RmtFrm = CAN_FRAME_FORMAT_SFF;
	frame.RefreshRate = VALVE_CONTROL_REFRESH_RATE;//帧刷新时间
	frame.data[1] = (uint8_t)m_pressure;

	*tx_frame = frame;
	return pdTRUE;

}

BaseType_t Valve_AEBdecActive_Get(struct can_frame *tx_frame)
{
	static uint8_t Rollong_Counter = 0;
	uint8_t sum = 0;
	uint8_t i = 0;

	tx_valve_control.data[3] = (0x03 << 6) & 0xc0;
	tx_valve_control.data[6] = Rollong_Counter;
	for(i = 0;i < 8;i ++)
		sum = sum + tx_valve_control.data[i];
	tx_valve_control.data[7] = sum ^ 0xff;

	*tx_frame = tx_valve_control;
	if(Rollong_Counter > 15)
		Rollong_Counter = 0;
	else
		Rollong_Counter ++;

	return pdTRUE;

}

BaseType_t Valve_Pressure_Get(struct can_frame *tx_frame)
{
	float ret_pressure = 0; //= Valve_break_Pressure();    //= Valve_Pressure_Cal();
	uint8_t upressure = 0;
	uint16_t m_pressure; //= (uint16_t)(ret_pressure * 128.0);
//	uint8_t turnleft = 0;
//	uint8_t turnright = 0;

	ret_pressure = Valve_break_Pressure();
	/*
	if(ret_pressure == 0)
	{
		upressure = Urader_Break_Pressure();
		if(upressure == 0)
			ret_pressure = 0;
		else
			ret_pressure = 3.0;
	}*/
//	turnleft = stVehicleParas.LeftFlagTemp;
//	turnright = stVehicleParas.RightFlagTemp;

//	if(turnleft | turnright)
//		ret_pressure = 0;
/*
	if((turnleft == 0) && (turnright == 0))
		upressure = Urader_Break_Pressure(0);
	else if((turnleft == 1) && (turnright == 0))
		upressure = Urader_Break_Pressure(1);
	else if((turnleft == 0) && (turnright == 1))
		upressure = Urader_Break_Pressure(2);
	else
		upressure = Urader_Break_Pressure(0);
	if(upressure == 1)									//当有超声波雷达刹车时，最大刹车力度
		ret_pressure = 750;
*/
//	if(stVehicleParas.BrakeFlag == 1)
//		ret_pressure = 0;
//	if(warning_status.AEBstatus != 0)
//		ret_pressure = 0;
//	if(stVehicleParas.fVehicleSpeed == 0)
//		ret_pressure = 0;
	if((0 == stVehicleParas.fVehicleSpeed) | ((0 < stVehicleParas.fVehicleSpeed) && (stVehicleParas.fVehicleSpeed < 25)))
	{
		upressure = Urader_Break_Pressure();
		fprintf(USART1_STREAM,"%d\r\n",upressure);
	}

	if(upressure != 0)
	{
		ret_pressure = 0.5;
	}

	Set_Break_Light_Stata(ret_pressure);
	m_pressure  = (uint16_t)(ret_pressure * 128.0);
	tx_valve_control.data[1] = (uint8_t)(m_pressure & 0x00FF);
	tx_valve_control.data[0] = (uint8_t)((m_pressure & 0xFF00) >> 8);
	*tx_frame = tx_valve_control;
	return pdTRUE;

}

void Valve_CAN_Analysis(struct can_frame *rx_frame)
{
	if(rx_frame->lenth < 8)
		return;
	if(rx_frame->TargetID == rx_valve_state.TargetID)
	{
		for(uint8_t i=0; i<rx_frame->lenth; i++)
		{
			rx_valve_state.data[i] = rx_frame->data[i];
		}
		valve_state_message.ValveState = rx_valve_state.data[0];
		valve_state_message.InternalTemp = (int8_t)rx_valve_state.data[1];
		valve_state_message.ActualPressure = (float)(rx_valve_state.data[2]<<4 + rx_valve_state.data[3])/128.0;
		valve_state_message.TargetPressure = (float)(rx_valve_state.data[4]<<4 + rx_valve_state.data[5])/128.0;
		valve_state_message.FaultCode = rx_valve_state.data[6];
		valve_state_message.Free = rx_valve_state.data[7];
	}else if(rx_frame->TargetID == rx_valve_fault_code.TargetID)
	{
		for(uint8_t i=0; i<rx_frame->lenth; i++)
		{
			rx_valve_fault_code.data[i] = rx_frame->data[i];
		}

//		fprintf(USART2_STREAM, "ValveState %d, InternalTemp %d, ActualPressure %0.2f, TargetPressure %0.2f, FaultCode %d  \r\n",
//				valve_state_message.ValveState, valve_state_message.InternalTemp, valve_state_message.ActualPressure,
//				valve_state_message.TargetPressure, valve_state_message.FaultCode);
	}
}

int Valve_Braking_Force_Cal(float ttc_value)
{
	float pressure_coefficient = 10.0;
	if( xSemaphoreTakeRecursive( valveMutex, ( TickType_t ) 10 ) == pdTRUE )
	{
		switch(valve_control_model.operating_state)
		{
		case No_Working:
			if (ttc_value < 2.7)
			{
				valve_control_model.operating_state = In_The_Climbing;
				valve_control_model.minimum_pressure = (1.0 - ttc_value/2.7)*pressure_coefficient;
				valve_control_model.maxinum_pressure = VALVE_PRESSURE_LIMIT_VALUE;
				valve_control_model.maxinum_pressure_time = 1000;
				valve_control_model.climbing_time = 1000;
				valve_control_model.total_braking_time = 3000;
			}
			break;
		case In_The_Climbing:
			valve_control_model.maxinum_pressure = (1.0 - ttc_value/2.7)*pressure_coefficient;
			break;
		case In_The_Keep:
			valve_control_model.maxinum_pressure = (1.0 - ttc_value/2.7)*pressure_coefficient;
			break;
		case In_The_Downhill:
			valve_control_model.maxinum_pressure = (1.0 - ttc_value/2.7)*pressure_coefficient;
			break;
		case In_Cooling_Time:
			valve_control_model.maxinum_pressure = 0.0;
			break;
		default:
			break;
		}
		//xSemaphoreGiveRecursive( valveMutex );
	}

	return 1;
}

float Valve_break_Pressure(void)
{
	static float ret_pressure = 0;
	static uint8_t CameraBreakState = 0;
	uint8_t turnleft = 0;
	uint8_t turnright = 0;
	static uint8_t hwmstate = 0;
	static uint8_t fcwstate = 0;

	if((0 < CameraMessage.hmw) && (CameraMessage.hmw < stSysPara.hmw1))
	//if((0 < camera_data.HMW) && (camera_data.HMW < stSysPara.hmw1))
	{
		CameraBreakState = 0x01;
	}
	else
	{
		CameraBreakState = CameraBreakState & 0xFE;
	}

	if((0 < CameraMessage.ttc) && (CameraMessage.ttc < stSysPara.ttc1))
	{
		CameraBreakState = 0x02;
	}
	else
	{
		CameraBreakState = CameraBreakState & 0xFD;
	}

	if(CameraBreakState & 0x01)				//HMW制动
	{
		ret_pressure = 0.5;
	}
	else if(CameraBreakState & 0x02)		//FCW制动
	{
		ret_pressure = 3.0;
	}
	else
	{
		ret_pressure = 0;
	}

	turnleft = stVehicleParas.LeftFlagTemp;
	turnright = stVehicleParas.RightFlagTemp;

	if(turnleft | turnright)
		ret_pressure = 0;
	if(stVehicleParas.BrakeFlag == 1)
		ret_pressure = 0;
	if(warning_status.AEBstatus != 0)
		ret_pressure = 0;
	if(stVehicleParas.fVehicleSpeed == 0)
		ret_pressure = 0;

	if(ret_pressure > 0)
		stVehicleParas.BreakState = stVehicleParas.BreakState | 0x01;
	else
		stVehicleParas.BreakState = stVehicleParas.BreakState & 0xFE;

	return ret_pressure;
}
/*
float Valve_break_Pressure(void)
{
	static float ret_pressure = 0;
	static float Camerattc = 0;

	if(CameraMessage.ttc < CameraMessage.hmw)
	{
		Camerattc = 1;
	}
	else //if(CameraMessage.ttc > CameraMessage.hmw)
	{
		Camerattc = 2;
	}
	//else
	//	Camerattc = 3;
	if(Camerattc == 2)
	{
		if((0 < CameraMessage.ttc) && (CameraMessage.ttc < stSysPara.ttc1))
		{
			ret_pressure = 3.0;
		}
		else
		{
			ret_pressure = 0;
		}
	}
	else //if(Camerattc == 1)
	{
		if((0 < CameraMessage.hmw) && (CameraMessage.hmw < stSysPara.hmw1))
		{
			ret_pressure = 1.5;
		}
		else
		{
			ret_pressure = 0;
		}
	}
	//else if(Camerattc == 3)
	//{}
	return ret_pressure;
}
*/

float Valve_break_Target_Deceleration(void)
{
	float ret_pressure = -2.0;

	if((stSysPara.ttc2< CameraMessage.ttc)	&
			(CameraMessage.ttc < stSysPara.ttc1))
	{
		ret_pressure = -2.6;
	}
	else if((0 < CameraMessage.ttc)	&
			(CameraMessage.ttc < stSysPara.ttc2))
	{
		ret_pressure = -7.0;
	}
	else
	{
		ret_pressure = 0;
	}
	ret_pressure = ret_pressure + 7;
	return ret_pressure;
}

float Valve_break_Deceleration_Request(void)
{
	float ret_pressure = -3;

	if((stSysPara.ttc2< CameraMessage.ttc)	&
			(CameraMessage.ttc < stSysPara.ttc1))
	{
		ret_pressure = -8;
	}
	else if((0 < CameraMessage.ttc)	&
			(CameraMessage.ttc < stSysPara.ttc2))
	{
		ret_pressure = -16;
	}
	else
	{
		ret_pressure = 0;
	}
	ret_pressure = ret_pressure + 16;
	return ret_pressure;
}


static float Valve_Pressure_Cal()
{
	float ret_pressure = 0.0;
	float add_pressure = 0.0;

	//if( xSemaphoreTakeRecursive( valveMutex, ( TickType_t ) 10 ) == pdTRUE )
	{
		switch(valve_control_model.operating_state)
		{
		case No_Working:
			valve_control_model.operating_state = 0;
			valve_control_model.minimum_pressure = 0.0;
			valve_control_model.maxinum_pressure = 0.0;
			valve_control_model.maxinum_pressure_time = 0;
			valve_control_model.climbing_time = 0;
			valve_control_model.total_braking_time = 0;
			break;
		case In_The_Climbing:
			if(valve_control_model.climbing_time > 0)
			{
				add_pressure = 20 * (valve_control_model.maxinum_pressure - valve_control_model.minimum_pressure) / valve_control_model.climbing_time;
				valve_control_model.minimum_pressure += add_pressure;

				valve_control_model.climbing_time -= 20;//压力爬坡时间减20ms
				valve_control_model.total_braking_time -= 20;//制动总时间减20ms

				//判断是否更新状态
				if((valve_control_model.climbing_time <= 0) ||
						(valve_control_model.minimum_pressure > valve_control_model.maxinum_pressure))
				{
					valve_control_model.operating_state = In_The_Keep;
					valve_control_model.minimum_pressure = valve_control_model.maxinum_pressure;
					valve_control_model.climbing_time = 0;
				}

				ret_pressure = valve_control_model.minimum_pressure;
			}
			break;
		case In_The_Keep:
			if(valve_control_model.maxinum_pressure_time > 0)
			{
				valve_control_model.maxinum_pressure_time -= 20;//最大压力保持时间减20ms
				valve_control_model.total_braking_time -= 20;//制动总时间减20ms

				//判断是否更新状态
				if(valve_control_model.maxinum_pressure_time <= 0)
				{
					valve_control_model.operating_state = In_The_Downhill;
					valve_control_model.maxinum_pressure_time = 0;
				}

				ret_pressure = valve_control_model.maxinum_pressure;
			}
			break;
		case In_The_Downhill:
			if(valve_control_model.total_braking_time > 0)
			{
				add_pressure = 20 * valve_control_model.maxinum_pressure / valve_control_model.total_braking_time;
				valve_control_model.maxinum_pressure -= add_pressure;

				valve_control_model.total_braking_time -= 20;//制动总时间减20ms

				//判断是否更新状态
				if(valve_control_model.total_braking_time <= 0 || valve_control_model.maxinum_pressure < 0)
				{
					valve_control_model.operating_state = In_Cooling_Time;
					valve_control_model.maxinum_pressure = 0.0;
					valve_control_model.total_braking_time = 0;
				}

				ret_pressure = valve_control_model.maxinum_pressure;
			} else{
				valve_control_model.operating_state = In_Cooling_Time;
				valve_control_model.maxinum_pressure = 0.0;
				valve_control_model.total_braking_time = 0;
			}
			break;
		case In_Cooling_Time:
			if(valve_control_model.cooling_time > 0)
			{
				valve_control_model.maxinum_pressure = 0.0;
				valve_control_model.cooling_time -= 20;//冷却期时间减20ms

				//判断是否更新状态
				if(valve_control_model.cooling_time <= 0)
				{
					valve_control_model.operating_state = No_Working;
					valve_control_model.maxinum_pressure = 0.0;
					valve_control_model.total_braking_time = 0;
				}

				ret_pressure = valve_control_model.maxinum_pressure;
			}else{
				valve_control_model.operating_state = No_Working;
				valve_control_model.maxinum_pressure = 0.0;
				valve_control_model.total_braking_time = 0;
			}
			break;
		default:
			break;
		}
		//xSemaphoreGiveRecursive( valveMutex );
	}

	if(ret_pressure < 0)
		ret_pressure = 0;
	return ret_pressure;
}
