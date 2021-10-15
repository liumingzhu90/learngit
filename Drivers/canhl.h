/**
  ******************************************************************************
  * 文件名 canhl.h
  * 作  者  ChipON_AE/FAE_Group
  * 日  期  2019-10-19
  * 描  述  该文件提供了CAN例程使用的变量与函数
  ******************************************************************************/

#ifndef _CANHL_H_
#define _CANHL_H_
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//波特率
typedef enum
{
    CAN_BAUDRATE_125K = 0,   //!<baudrate 125K
    CAN_BAUDRATE_250K = 1,   //!<baudrate 250K
    CAN_BAUDRATE_500K = 2,   //!<baudrate 500K
    CAN_BAUDRATE_1M   = 3    //!<baudrate 1M
} CAN_BaudRateT;

//CAN发送函数返回值
typedef enum
{
    CAN_ERROR_NOERROR = 0,			 //!<return no error
    CAN_ERROR_BUFFERFULL=1,
    CAN_ERROR_NOINIT=2,
    CAN_ERROR_EMPTY=3,
} CAN_ErrorT;

struct can_frame
{
	uint32_t  TargetID;		 //ID
	uint8_t   data[8];   	 //数据指针
	uint8_t   lenth;   		 //长度
	uint32_t  MsgType; 		 //帧类型        CAN_DATA_FRAME数据帧                     CAN_REMOTE_FRAME远程帧
	uint32_t  RmtFrm;  		 //帧格式        CAN_FRAME_FORMAT_SFF标准帧     CAN_FRAME_FORMAT_EFF扩展帧
	uint32_t  RefreshRate;   //帧间隔    单位毫秒
	uint8_t		flgBoxRxEnd;	//结束数据标志
};


struct CAN_Filter
{
	uint32_t ID;
	uint32_t Mask;
	uint32_t FrameFormat;// CAN_FRAME_FORMAT_SFF or CAN_FRAME_FORMAT_EFF
};

//全局变量声明
#define CAN_BUFFER_MAX 20
extern QueueHandle_t xQueue_can1;
extern struct can_frame can1_rx_frame;
extern uint8_t CAN1_RX_COUNT;

extern QueueHandle_t xQueue_can2;
extern struct can_frame can2_rx_frame;
extern uint8_t CAN2_RX_COUNT;

extern QueueHandle_t xQueue_can0;
extern struct can_frame can0_rx_frame;
extern uint8_t CAN0_RX_COUNT;

extern QueueHandle_t xQueue_can3;
extern struct can_frame can3_rx_frame;
extern uint8_t CAN3_RX_COUNT;

extern QueueHandle_t xQueue_can4;
extern struct can_frame can4_rx_frame;
extern uint8_t CAN4_RX_COUNT;

extern QueueHandle_t xQueue_can5;
extern struct can_frame can5_rx_frame;
extern uint8_t CAN5_RX_COUNT;
//函数声明
void xGPIO_CAN1();
void xINT_CAN(CAN_SFRmap* CANx);
void xInit_CAN(CAN_SFRmap* CANx,uint8_t Bdrt,uint32_t MODE,struct CAN_Filter *Filter);//Filter可以为NULL
CAN_ErrorT CAN_Transmit_DATA(CAN_SFRmap* CANx, struct can_frame tx_frame);
CAN_ErrorT CAN_Receive_DATA(CAN_SFRmap* CANx, struct can_frame *rx_frame);
void CAN_check_busoff(CAN_SFRmap* CANx);
void CAN0_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter);//Filter可以为NULL
void CAN1_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter);//Filter可以为NULL
void CAN2_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter);//Filter可以为NULL
void CAN3_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter);//Filter可以为NULL
void CAN4_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter);//Filter可以为NULL
void CAN5_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter);//Filter可以为NULL
#endif /* _CANHL_H_ */
