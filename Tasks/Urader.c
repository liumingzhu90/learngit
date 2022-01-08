/*
 * Urader.c
 *
 *  Created on: 2021-9-1
 *      Author: wangzhenbao
 */
#include "can_task.h"
#include "stdio.h"
#include "canhl.h"
#include "usart.h"
#include "task_manager.h"
#include "vehicle_can.h"
#include "common.h"
#include "gpio.h"

URADER_MESSAGE	Urader_Company_Message[2];

void Urader_RxValve_Data_Analysis(uint32_t ulSysTime)
{
	uint8_t i;
	float k;

	if(stUraderCanData[0].flgBoxRxEnd == 1)
	{
		stUraderCanData[0].flgBoxRxEnd = 0;
		//第一路雷达主机
			if(((stUraderCanData[0].data[1] >> 6) & 0x3) == 0)
			{
				Urader_Company_Message[0].Company = 1;
			}
			else if(((stUraderCanData[0].data[1] >> 6) & 0x3) == 1)
			{
				Urader_Company_Message[0].Company = 2;
			}
			//0xfc未检测到障碍物
			//0xfd未插入雷达
			//0xfe雷达不需要要工作
			for(i = 0;i < 4;i ++)
			{
				if(stUraderCanData[0].data[i + 4] == 0xfc)
				{
					Urader_Company_Message[0].Urader_work_stata[i] = ACCESSIBILITY;
				}
				else if(stUraderCanData[0].data[i + 4] == 0xfd)
				{
					Urader_Company_Message[0].Urader_work_stata[i] = NOURADER;
				}
				else if(stUraderCanData[0].data[i + 4] == 0xfe)
				{
					Urader_Company_Message[0].Urader_work_stata[i] = NOWORK;
				}
				else
				{
					Urader_Company_Message[0].Urader_work_stata[i] = URWORK;
				}
				if(Urader_Company_Message[0].Urader_work_stata[i] == URWORK)
				{
					//if(Urader_Company_Message[0].Company == 1)
					//{
					//	Urader_Company_Message[0].distance[i] = (float)stUraderCanData[0].data[i + 4] * 0.01;
					//}
					//else
					{
						Urader_Company_Message[0].distance[i] = ((float)stUraderCanData[0].data[i + 4] * 0.02);
					}
					//fprintf(USART1_STREAM,"%d = %.2f\r\n",i,Urader_Company_Message[0].distance[i]);
				}

			}
			//fprintf(USART1_STREAM,"11111111");
	}
	if(stUraderCanData[1].flgBoxRxEnd == 1)
	{
		stUraderCanData[1].flgBoxRxEnd = 0;
		//if(stUraderCanData[1].TargetID = 0x701)
		{
			for(i = 0;i < 8;i ++)
			{
				if(stUraderCanData[1].data[i] == 0xfc)
				{
					Urader_Company_Message[0].Urader_work_stata[i + 4] = ACCESSIBILITY;
				}
				else if(stUraderCanData[1].data[i] == 0xfd)
				{
					Urader_Company_Message[0].Urader_work_stata[i + 4] = NOURADER;
				}
				else if(stUraderCanData[1].data[i] == 0xfe)
				{
					Urader_Company_Message[0].Urader_work_stata[i + 4] = NOWORK;
				}
				else
				{
					Urader_Company_Message[0].Urader_work_stata[i + 4] = URWORK;
				}
				if(Urader_Company_Message[0].Urader_work_stata[i + 4] == URWORK)
				{
					//if(Urader_Company_Message[0].Company == 1)
					//{
					//	Urader_Company_Message[0].distance[i + 4] = (float)stUraderCanData[1].data[i] * 0.01;
					//}
					//else
					{
						Urader_Company_Message[0].distance[i + 4] = ((float)stUraderCanData[1].data[i] * 0.02);
					}
					//fprintf(USART1_STREAM,"%d = %.2f\r\n",i + 4,Urader_Company_Message[0].distance[i + 4]);
				}
			}
		}
	}

	if(stUraderCanData[2].flgBoxRxEnd == 1)
	{
		stUraderCanData[2].flgBoxRxEnd = 0;

		//第二路雷达主机
		if(((stUraderCanData[2].data[1] >> 6) & 0x3) == 0)
		{
			Urader_Company_Message[1].Company = 1;
		}
		else if(((stUraderCanData[2].data[1] >> 6) & 0x3) == 1)
		{
			Urader_Company_Message[1].Company = 2;
		}
		for(i = 0;i < 4;i ++)
		{
			if(stUraderCanData[2].data[i + 4] == 0xfc)
			{
				Urader_Company_Message[1].Urader_work_stata[i] = ACCESSIBILITY;
			}
			else if(stUraderCanData[2].data[i + 4] == 0xfd)
			{
				Urader_Company_Message[1].Urader_work_stata[i] = NOURADER;
			}
			else if(stUraderCanData[2].data[i + 4] == 0xfe)
			{
				Urader_Company_Message[1].Urader_work_stata[i] = NOWORK;
			}
			else
			{
				Urader_Company_Message[1].Urader_work_stata[i] = URWORK;
			}
			if(Urader_Company_Message[0].Urader_work_stata[i] == URWORK)
			{
				if(Urader_Company_Message[1].Company == 1)
				{
					Urader_Company_Message[1].distance[i] = ((float)stUraderCanData[2].data[i + 4] * 0.01);
				}
				else
				{
					Urader_Company_Message[1].distance[i] = (float)stUraderCanData[2].data[i + 4] * 0.02;
				}
			}
		}
	}
	if(stUraderCanData[3].flgBoxRxEnd == 1)
	{
		stUraderCanData[3].flgBoxRxEnd = 0;

		//if(stUraderCanData[3].TargetID = 0x703)
		{
			for(i = 0;i < 8;i ++)
			{
				if(stUraderCanData[3].data[i] == 0xfc)
				{
					Urader_Company_Message[1].Urader_work_stata[i + 4] = ACCESSIBILITY;
				}
				else if(stUraderCanData[3].data[i] == 0xfd)
				{
					Urader_Company_Message[1].Urader_work_stata[i + 4] = NOURADER;
				}
				else if(stUraderCanData[3].data[i] == 0xfe)
				{
					Urader_Company_Message[1].Urader_work_stata[i + 4] = NOWORK;
				}
				else
				{
					Urader_Company_Message[1].Urader_work_stata[i + 4] = URWORK;
				}
				if(Urader_Company_Message[0].Urader_work_stata[i + 4] == URWORK)
				{
					if(Urader_Company_Message[1].Company == 1)
					{
						Urader_Company_Message[1].distance[i + 4] = (float)stUraderCanData[3].data[i] * 0.01;
					}
					else
					{
						Urader_Company_Message[1].distance[i + 4] = (float)stUraderCanData[3].data[i] * 0.02;
					}
				}
			}
		}

	}
}
