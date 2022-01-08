/*
 * utc_time.c
 *
 *  Created on: 2021-9-15
 *      Author: Administrator
 */

#include "utc_time.h"
//平年累积月分天数表
static const uint16_t NonleapYearMonth[12] = \
	{ 31,31 + 28, 31 + 28 + 31, 31 + 28 + 31 + 30, 31 + 28 + 31 + 30 + 31,\
	31 + 28 + 31 + 30 + 31 + 30, 31 + 28 + 31 + 30 + 31 + 30 + 31, 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31, \
	31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30, 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31, \
	31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30, 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + \
	30 + 31 + 30 + 31};
//闰年累积月分天数表
static const uint16_t LeapYearMonth[12] = \
	{ 31,31 + 29, 31 + 29 + 31, 31 + 29 + 31 + 30, 31 + 29 + 31 + 30 + 31, \
	31 + 29 + 31 + 30 + 31 + 30, 31 + 29 + 31 + 30 + 31 + 30 + 31, \
	31 + 29 + 31 + 30 + 31 + 30 + 31 + 31, 31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30, \
	31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31, 31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30, \
	31 + 29 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30 + 31};


uint8_t alg_IsLeapYear(uint32_t year)
{
	if((year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0))) //能被4整除,不能被百整除,能被400整除。
	{
		return 1;				//闰年
	}
	else
	{
		return 0;				//平年
	}
}

TimeType alg_Utc2LocalTime(uint32_t UtcVal, int8_t TimeZone)
{
	uint32_t i = 0;
	TimeType LocalTime;
	uint32_t Hour,Days,Year;

	LocalTime.tm_sec	=	UtcVal%60;						//得到秒余数
	LocalTime.tm_min	=	(UtcVal/60)%60;				//得到整数分钟数

	Hour	=	(UtcVal/60)/60;										//得到整数小时数
	LocalTime.tm_hour	=	Hour%24+TimeZone;			//得到小时余数+时区
	if(LocalTime.tm_hour>23)
	{
		LocalTime.tm_hour-=24;
		Days=Hour/24+1;
	}
	else
	{
		Days=Hour/24;
	}
	LocalTime.tm_wday=(Days+4)%7;							//计算星期,0-表示星期天		注：1970-1-1 是星期4

													//注：400年=146097天,100年=36524天,4年=1461天
	Year = 1970;															//utc时间从1970开始
	Year += (Days/146097)*400;

	Days %= 146097;														//计算400年内的剩余天数
	Year += (Days/36525)*100;

	Days %= 36525;
	Year += (Days/1461)*4;

	Days %= 1461;															//计算4年内剩余天数,1970平1972闰年
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
	if (alg_IsLeapYear(Year))									//本年是闰年
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
	else																			//本年是平年
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
	uint32_t y = LocalTime.tm_year -1970;							//看一下有几个400年,几个100年,几个4年
	uint32_t dy = (y / 400);
	uint32_t days = dy *  (400 * 365 + 97);						//400年的天数

	dy = (y % 400) / 100;
	days += dy * (100 * 365 + 25);										//100年的天数

	dy = (y % 100) / 4;
	days += dy * (4 * 365 + 1);												//4年的天数

	dy = y % 4;																				//注意:这里1972是闰年,与1970只差2年
	days += dy * 365 ;

	if(dy == 3)																				//这个4年里,有没有闰年就差1天
	{
		days++;	//只有这个是要手动加天数的,因为1973年计算时前面的天数按365天算,1972少算了一天
	}

	if (LocalTime.tm_mon != 1)
	{
		if(alg_IsLeapYear(LocalTime.tm_year))												//看看今年是闰年还是平年
		{
			days += LeapYearMonth[(LocalTime.tm_mon - 1) - 1];
		}
		else
		{
			days += NonleapYearMonth[(LocalTime.tm_mon  - 1) - 1]; 		//如果给定的月份数为x则,只有x-1个整数月
		}
	}
	days += LocalTime.tm_mday - 1;

	return (days * 24 * 3600 + ((uint32_t)LocalTime.tm_hour - TimeZone)* 3600 + (uint32_t)LocalTime.tm_min * 60 + (uint32_t)LocalTime.tm_sec);
}
