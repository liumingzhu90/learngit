/*
 * displayer.h
 *
 *  Created on: 2021-7-7
 *      Author: shuai
 */

#ifndef DISPLAYER_H_
#define DISPLAYER_H_
#include "system_init.h"
#include "canhl.h"

#define DISPLAYER_CAN_BAUDRATE CAN_BAUDRATE_250K

#define DISPLAYER_SHOW_ID 0x18FFED80
#define DISPLAYER_SHOW_REFRESH_RATE 100

#define DISPLAYER_FEEDBACK_ID 0x18FFED81
#define DISPLAYER_FEEDBACK_REFRESH_RATE 100

typedef struct
{
	uint8_t FrontCameraStatus; //前视相机状态【0：未就绪】、【1： 就绪】
	uint8_t MillimeterWaveRadarStatus; //毫米波雷达状态【0：未就绪】、【1： 就绪】
	uint8_t UltrasonicRadarStatus; //超声波雷达状态【0：未就绪】、【1： 就绪】
	uint8_t ValveStatus; //比例阀状态【0：未就绪】、【1： 就绪】
	uint8_t VehicleCANBusStatus; //整车CAN总线状态【0：未就绪】、【1： 就绪】
	uint8_t VehicleSpeedStatus; //车速状态【0：未就绪】、【1： 就绪】
	uint8_t Module4GStatus; //4G模块状态【0：未就绪】、【1： 就绪】
	uint8_t GPSStatus; //GPS模块状态【0：未就绪】、【1： 就绪】
	uint8_t AEBSTaskStatus; //AEBS任务状态【0：故障】、【1： 自检】、【2：正常】、【3：其他】
	uint8_t FCWTaskStatus; //FCW任务状态【0：故障】、【1： 自检】、【2：正常】、【3：其他】
	uint8_t LDWTaskStatus; //LDW任务状态【0：故障】、【1： 自检】、【2：正常】、【3：其他】
	uint8_t SSSTaskStatus; //SSS任务状态【0：故障】、【1： 自检】、【2：正常】、【3：其他】
	uint8_t CMSTaskStatus; //CMS任务状态【0：故障】、【1： 自检】、【2：正常】、【3：其他】
	uint8_t FCWLevel; //前向碰撞预警等级【0：否】、【1：一级预警】、【2：二级预警】、【3：其他】
	uint8_t LDW; //车道偏离预警【0：未识别车道线】、【1：左车道偏离预警】、【2：右车道偏离预警】、【3：无偏离预警】
	uint8_t HMWGrade; //车距监测预警等级【0：无效值】、【1：阈值＜HMW≤2.7S]、【2： 0≤HMW≤阈值】、【3：HMW>2.7S】
	uint8_t DigitalDisplay; //数码显示内容【0：否】、【1：TTC】、【2：DTC】、【3：HMW】
	float HMWTime; //车距时间，范围: 0.0～6.0[秒]、 Factor： 0.1、 Offset： 0、 无效值：0x3F
	float LengthwaysRelativeSpeed; //纵向相对速度，范围: -127.93～127.93[米/秒]、 Factor： 0.25、 Offset：-127.9375、 无效值： 0xFFF
	float TTC; //本车与障碍物间的碰撞时间，范围: 0.0～6.0[秒]、 Factor： 0.1、 Offset： 0、 无效值：0x3F
	float LengthwaysDistance; //DTC纵向相对距离，范围: 0.0～300.0[米]、 Factor： 0.1、 Offset： 0、 无效值： 0x7FF
	uint8_t ObstacleType; //障碍物类型【0：无效值】、【1：车辆】、【2：人】、【3：其他】、【4：骑行者】、【5-11：其他】
}Displayer_Show;

extern Displayer_Show displayer_show_info;
extern BaseType_t isAEBTaskClose;
extern BaseType_t isLDWTaskClose;

void Displayer_Init();
BaseType_t Displayer_CANTx_Get(struct can_frame *tx_frame);
void Displayer_CANRx_Analysis(struct can_frame *rx_frame);

#endif /* DISPLAYER_H_ */
