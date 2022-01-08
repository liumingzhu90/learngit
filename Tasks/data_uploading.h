/*
 * data_uploading.h
 *
 *  Created on: 2021-10-10
 *      Author: Administrator
 */

#ifndef DATA_UPLOADING_H_
#define DATA_UPLOADING_H_
#include "system_init.h"
/****************************宏、枚举定义**************************************/

/****************************结构体定义**************************************/

/****************************全局函数**************************************/
void UpLoad_Car_Body_Info();
void Data_Uploading_In_KFIT(uint8_t data);
//void Data_Uploading_In_SysTick();
void Data_Uploading_In_MainLoop();

#endif /* DATA_UPLOADING_H_ */
