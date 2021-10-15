/*
 * screen_task.h
 *
 *  Created on: 2021-6-21
 *      Author: chenq
 */

#ifndef SCREEN_TASK_H_
#define SCREEN_TASK_H_

#include "system_init.h"
#include "canhl.h"
#include "usart.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

void screen_task(void *pvParameters);

#endif /* SCREEN_TASK_H_ */
