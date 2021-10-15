/*
 * stereo_camera.h
 *
 *  Created on: 2021-6-22
 *      Author: shuai
 */

#ifndef STEREO_CAMERA_H_
#define STEREO_CAMERA_H_
#include "system_init.h"
#include "stdio.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "canhl.h"
#include "uart_task.h"

#define CAMERA_CAN_BAUDRATE CAN_BAUDRATE_500K

#define CAMERA_ESSENTIAL_DATA_ID 0x79F
#define CAMERA_ESSENTIAL_REFRESH_RATE 100

#define CAMERA_OBSTACLE_INFO_ID 0x7A0
#define CAMERA_OBSTACLE_INFO_REFRESH_RATE 100

#define CAMERA_OBSTACLE_INFO_A_ID 0x7A1
#define CAMERA_OBSTACLE_INFO_A_REFRESH_RATE 100

#define CAMERA_OBSTACLE_INFO_B_ID 0x7A2
#define CAMERA_OBSTACLE_INFO_B_REFRESH_RATE 100

#define CAMERA_INPUT_DATA_ID 0x7B0
#define CAMERA_INPUT_DATA_REFRESH_RATE 100

typedef enum{
	No_Error = 0, //没有错误
	Camera_Error = 2, //相机异常错误
	Watchdog_Error = 4, //相机看门狗错误
	FPGA_Error = 5, //FPGA 端的内存出错
	No_Remap = 6, //没有 remap 数据
	ISP_Error = 7, //ISP检查返回错误
	Camera_Unknown_Error = 8, //相机未知错误
	No_CriticalData = 11, //处于非标定模式的设备启动时“adasCriticalData” 路径下标定文件丢失。
	Soft_Unknown_Error = 19, //软件未知错误
	CAN_Network_Error = 31, //无法通过指定的 CAN 协议获取汽车信号
	CAN_Unknown_Error = 39 //CAN未知错误
}Camera_Fault_Code;

typedef struct{
	uint8_t LeftLDW; //参数说明： 【0：否]、【1：是】左车道线偏离
	uint8_t RightLDW; //参数说明： 【0：否]、【1：是】右车道线偏离
	uint8_t LDWSentivity; //车道偏离预警等级,参数说明： 【0：关闭】、【1：正常】、【2：高】、【3：低】
	uint8_t Invalid; //参数说明： 【0： 未失效】、【1：光线弱】、【2：图像模糊】、【3：场景复杂】、【4：高温保护]、【5： 图像异常】、【6-7:保留】
	uint8_t OffSound; //设备是否抑制声音,参数说明：【0： 不抑制】、【1： 抑制】
	uint8_t HMWGrade; //参数说明： 【0：无效值】、【1：阈值＜HMW≤2.7S】、【2： 0≤HMW≤阈值】、【3：车距时间>2.7S】
	float HMW; //预警区域内离本车最近障碍物的车距时间，范围: 0～6[秒]Factor： 0.1、 Offset： 0 无效值: 0x3F
	uint8_t ErrorCode; //参数说明：【 0：表示无错误】、【 1-9：相机类错误】、【 11-19：应用软件类错误】、【 31-39：CAN 通讯类故障】
	uint8_t HMWEnable; //参数说明： 【 0:否】、【1:是】车距监测预警功能是否启用
	uint8_t FCWLevel; //参数说明：【0：否】、【1：一级预警】、【2：二级预警】
	uint8_t AmbientLuminance; //参数说明： 【0： 保留】、【1： 白天】、【2：黄昏】、【3：夜晚】、【4： 遮挡】
	uint8_t FCWStatus; //参数说明： 【0：否】、【1： 是】大于前向碰撞预警功能启用车速后，满足预警条件时才会输出 1，默认输出： 0
	/*以下数据暂时不进行解析*/
	//uint8_t VersionSegment; //参数说明： 【0-2】 0:版本号第一节； 1: 版本号第二节；2: 版本号第三节；
	//uint8_t SWVersion[3]; //软件版本号，参数说明： 三帧一个周期，每次代表一节，例如 0.0.0
	//uint8_t HWVersion[3]; //硬件版本号，参数说明： 三帧一个周期，每次代表一节，例如 0.0.0
	//uint8_t CanProtocoIVersion[3]; //CAN协议版本号，参数说明： 三帧一个周期，每次代表一节， 例如 0.0.0
}Camera_Essential_Data;

typedef struct{
//	uint8_t reserved;
	uint8_t LeftLaneStatus;	// 参数说明：【0：未压线】、【1：压线】、【2：骑线】、【3：保留】
	uint8_t LeftLaneStyle;	// 参数说明：【0：无】、【1：预测】、【2：虚线】、【3：实线】、【4：双虚线】、【5：双实线】、【6：三线】、【7-15：保留】
//	uint8_t reserved1;
	uint8_t RightLaneStatus;// 参数说明： 【0：未压线】、【1：压线】、【2：骑线】、【3：保留】
	uint8_t RightLaneStyle;	// 参数说明： 【0：无】、【1：预测】、【2：虚线】、【3：实线】、【4：双虚线】、【5：双实线】、【6：三线】、【7-15：保留】
	uint8_t CarDepth;		// 参数说明： 类型: unsigned 8 bits、范围: 0～2.50[米]、Factor：0.01、Offset：0、无效值：0xFF
	uint16_t SpeedOfRushLane;// 参数说明： 类型: unsigned 10 bits、范围: -50.0～50.0[米/秒]、Factor：0.1、Offset：-51.1、无效值：0x3FF
//	uint8_t resered;
}Camera_LDW_data;	// 0x7A3 车道线解析

typedef struct{
	uint8_t ObstacleNumber; //
	float HMWAlarmThreshold; //参数说明： 类型: unsigned 5 bits、 范围:0.0～3.0[秒]、 Factor： 0.1、 Offset： 0、 无效值: 0x1F
	/*以下数据暂时不进行解析*/
	//uint8_t TimeID; //参数说明： 单位毫秒， 标准时间戳低 8 位
	//float YawAngle; //参数说明： 类型: unsigned 14 bits、 范围: -0.8～0.8[弧度]、 Factor： 0.0001、 Offset： -0.8191、无效值: 0x3FFF
	//float PithAngle; //参数说明： 类型: unsigned 12 bits、 范围: -0.2～0.2[弧度]、 Factor： 0.0001、 Offset： -0.2047、无效值: 0x7FF
	//float RotationAngle; //参数说明： 类型: unsigned 12 bits、 范围: -0.2～0.2[弧度]、 Factor： 0.0001、 Offset： -0.2047、无效值: 0x7FF
}Obstacle_Basic_Data;

typedef struct{
	uint8_t ObstacleID; //障碍物ID,参数说明： 【0～255】
	uint8_t TrackNumber; //障碍物连续跟踪帧数，范围: 1～63、 第二次检测到障碍物后从 1 开始； 最大值为 63，每帧增加 1，增长到 63 后保持为 63
	float ObstacleWidth; //障碍物宽度，参数说明：范围: 0.0～40.0[米]、 Factor： 0.01、 Offset： 0、 无效值： 0xFFF
	float ObstacleHeight; //障碍物高度，参数说明：范围: 0.0～40.0[米]、 Factor： 0.01、 Offset： 0、 无效值： 0xFFF
	float RelativeSpeedZ; //纵向相对速度，参数说明：范围: -127.93～127.93[米/秒]、 Factor： 0.0625、 Offset：-127.9375、 无效值： 0xFFF
	float DistanceY; //竖向相对距离，参数说明： 类型: 范围: -20.0～20.0[米]、 Factor： 0.01、 Offset： -20.47、无效值： 0xFFF
	float DistanceX; //横向相对距离，参数说明： 范围: -150.0～150.0[米]、 Factor： 0.01、 Offset： -163.83、无效值： 0x7FFF
	float DistanceZ; //纵向相对距离，参数说明： 范围: 0.0～300.0[米]、 Factor： 0.01、 Offset： 0、 无效值： 0x7FFF
	float TTC; //本车与障碍物间的碰撞时间，参数说明： 范围: 0.0～6.0[秒]、 Factor： 0.1、 Offset： 0、 无效值：0x3F
	float HMW; //车距时间，参数说明：  范围: 0.0～6.0[秒]、 Factor： 0.1、 Offset： 0、 无效值：0x3F、说明： CIPV 为 1 的障碍物才输出该数据。
	float RelativeSpeedX; //横向相对速度，参数说明： 类型: 范围: -50.0～50.0[米/秒]、 Factor： 0.1、 Offset： -51.1、无效值： 0x3FF
	uint8_t CIPV; //预警区域内离本车最近障碍物的标识，参数说明： 【0：否】、【1：是预警区域内离本车最近的障碍物】
	uint8_t ObstacleType; //障碍物类型，参数说明：【0：无效值】、【1： 车辆】、【2：人】、【3： 其他】、【4：骑行者】、【5-11：其他】
}Obstacle_Information;

typedef struct{
	//Camera_Essential_Data CammeraEssentialData;
	//Obstacle_Basic_Data ObsBasicData;
	//Obstacle_Information ObsInormation;
	float hmw;
	float ttc;
}Camera_All_Info;

extern QueueHandle_t CAN1_Rxqueue_Handler;

void Camera_CIPV_Init();
void Camera_Init();
void Camera_CAN_Transmition(struct can_frame *tx_frame,float speed);
void Urader_CAN_Transmition(struct can_frame *tx_frame,float speed);
void Camera_CAN_Analysis(struct can_frame *rx_frame);

#endif /* STEREO_CAMERA_H_ */
