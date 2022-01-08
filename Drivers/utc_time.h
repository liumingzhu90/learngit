/*
 * utc_time.h
 *
 *  Created on: 2021-9-15
 *      Author: Administrator
 */

#ifndef UTC_TIME_H_
#define UTC_TIME_H_
#include "system_init.h"

typedef struct
{
	uint8_t 	tm_sec;
	uint8_t 	tm_min;
	uint8_t 	tm_hour;
	uint8_t 	tm_mday;
	uint8_t 	tm_mon;
	uint8_t 	tm_wday;
	uint16_t 	tm_year;
  //uint16_t 	tm_yday;
}TimeType;


TimeType  alg_Utc2LocalTime(uint32_t UtcVal, int8_t TimeZone);
uint32_t  alg_LocalTime2Utc(TimeType LocalTime, int8_t TimeZone);

#endif /* UTC_TIME_H_ */
