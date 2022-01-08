/*
 * utc_time.c
 *
 *  Created on: 2021-9-15
 *      Author: Administrator
 */

#include "utc_time.h"
//ƽ���ۻ��·�������
static const uint16_t NonleapYearMonth[12] = \
	{ 31,31 + 28, 31 + 28 + 31, 31 + 28 + 31 + 30, 31 + 28 + 31 + 30 + 31,\
	31 + 28 + 31 + 30 + 31 + 30, 31 + 28 + 31 + 30 + 31 + 30 + 31, 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31, \
	31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30, 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31, \
	31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30, 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + \
	30 + 31 + 30 + 31};
//�����ۻ��·�������
static const uint16_t LeapYearMonth[12] = \
	{ 31,31 + 29, 31 + 29 + 31, 31 + 29 + 31 + 30, 31 + 29 + 31 + 30 + 31, \
	31 + 29 + 31 + 30 + 31 + 30, 31 + 29 + 31 + 30 + 31 + 30 + 31, \
	31 + 29 + 31 + 30 + 31 + 30 + 31 + 31, 31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30, \
	31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31, 31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30, \
	31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30 + 31};


uint8_t alg_IsLeapYear(uint32_t year)
{
	if((year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0))) //�ܱ�4����,���ܱ�������,�ܱ�400������
	{
		return 1;				//����
	}
	else
	{
		return 0;				//ƽ��
	}
}

TimeType alg_Utc2LocalTime(uint32_t UtcVal, int8_t TimeZone)
{
	uint32_t i = 0;
	TimeType LocalTime;
	uint32_t Hour,Days,Year;

	LocalTime.tm_sec	=	UtcVal%60;						//�õ�������
	LocalTime.tm_min	=	(UtcVal/60)%60;				//�õ�����������

	Hour	=	(UtcVal/60)/60;										//�õ�����Сʱ��
	LocalTime.tm_hour	=	Hour%24+TimeZone;			//�õ�Сʱ����+ʱ��
	if(LocalTime.tm_hour>23)
	{
		LocalTime.tm_hour-=24;
		Days=Hour/24+1;
	}
	else
	{
		Days=Hour/24;
	}
	LocalTime.tm_wday=(Days+4)%7;							//��������,0-��ʾ������		ע��1970-1-1 ������4

													//ע��400��=146097��,100��=36524��,4��=1461��
	Year = 1970;															//utcʱ���1970��ʼ
	Year += (Days/146097)*400;

	Days %= 146097;														//����400���ڵ�ʣ������
	Year += (Days/36525)*100;

	Days %= 36525;
	Year += (Days/1461)*4;

	Days %= 1461;															//����4����ʣ������,1970ƽ1972����
	while( Days > 365)
	{
		if(alg_IsLeapYear(Year))
		{
			Days--;
		}
		Days -= 365;
		Year++;
	}
	if (!alg_IsLeapYear(Year) && (Days == 365) )
	{
		Year++;
		LocalTime.tm_mday	=1;
		LocalTime.tm_mon	=1;
		LocalTime.tm_year	=Year;
		return LocalTime;
	}
	LocalTime.tm_year	=Year;
	LocalTime.tm_mon=0;
	LocalTime.tm_mday=0;
	if (alg_IsLeapYear(Year))									//����������
	{
		for (i = 0; i < 12; i++)
		{
			if (Days < LeapYearMonth[i])
			{
				LocalTime.tm_mon = i + 1;
				if (i == 0)
				{
					LocalTime.tm_mday = Days;
				}
				else
				{
					LocalTime.tm_mday = Days - LeapYearMonth[i - 1];
				}
				LocalTime.tm_mday++;
				return LocalTime;
			}
		}
	}
	else																			//������ƽ��
	{
		for (i = 0; i < 12; i++)
		{
			if (Days < NonleapYearMonth[i])
			{
				LocalTime.tm_mon  = i + 1;
				if (i == 0)
				{
					LocalTime.tm_mday = Days;
				}
				else
				{
					LocalTime.tm_mday = Days - NonleapYearMonth[i - 1];
				}
				LocalTime.tm_mday++;
				return LocalTime;
			}
		}
	}
	return LocalTime;
}


uint32_t  alg_LocalTime2Utc(TimeType LocalTime, int8_t TimeZone)
{
	uint32_t y = LocalTime.tm_year -1970;							//��һ���м���400��,����100��,����4��
	uint32_t dy = (y / 400);
	uint32_t days = dy *  (400 * 365 + 97);						//400�������

	dy = (y % 400) / 100;
	days += dy * (100 * 365 + 25);										//100�������

	dy = (y % 100) / 4;
	days += dy * (4 * 365 + 1);												//4�������

	dy = y % 4;																				//ע��:����1972������,��1970ֻ��2��
	days += dy * 365 ;

	if(dy == 3)																				//���4����,��û������Ͳ�1��
	{
		days++;	//ֻ�������Ҫ�ֶ���������,��Ϊ1973�����ʱǰ���������365����,1972������һ��
	}

	if (LocalTime.tm_mon != 1)
	{
		if(alg_IsLeapYear(LocalTime.tm_year))												//�������������껹��ƽ��
		{
			days += LeapYearMonth[(LocalTime.tm_mon - 1) - 1];
		}
		else
		{
			days += NonleapYearMonth[(LocalTime.tm_mon  - 1) - 1]; 		//����������·���Ϊx��,ֻ��x-1��������
		}
	}
	days += LocalTime.tm_mday - 1;

	return (days * 24 * 3600 + ((uint32_t)LocalTime.tm_hour - TimeZone)* 3600 + (uint32_t)LocalTime.tm_min * 60 + (uint32_t)LocalTime.tm_sec);
}
