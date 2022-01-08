/*
 * watchdog.h
 *
 *  Created on: 2021-7-22
 *      Author: chenq
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include "system_init.h"

void IWDT_Config(uint32_t Overflow,uint32_t Prescaler);

#endif /* WATCHDOG_H_ */
