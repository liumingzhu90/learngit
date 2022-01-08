/**
  ******************************************************************************
  * �ļ��� canhl.h
  * ��  ��  ChipON_AE/FAE_Group
  * ��  ��  2019-10-19
  * ��  ��  ���ļ��ṩ��CAN����ʹ�õı����뺯��
  ******************************************************************************/

#ifndef _CANHL_H_
#define _CANHL_H_
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//������
typedef enum
{
    CAN_BAUDRATE_125K = 0,   //!<baudrate 125K
    CAN_BAUDRATE_250K = 1,   //!<baudrate 250K
    CAN_BAUDRATE_500K = 2,   //!<baudrate 500K
    CAN_BAUDRATE_1M   = 3    //!<baudrate 1M
} CAN_BaudRateT;

//CAN���ͺ�������ֵ
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
	uint8_t   data[8];   	 //����ָ��
	uint8_t   lenth;   		 //����
	uint32_t  MsgType; 		 //֡����        CAN_DATA_FRAME����֡                     CAN_REMOTE_FRAMEԶ��֡
	uint32_t  RmtFrm;  		 //֡��ʽ        CAN_FRAME_FORMAT_SFF��׼֡     CAN_FRAME_FORMAT_EFF��չ֡
	uint32_t  RefreshRate;   //֡���    ��λ����
	uint8_t		flgBoxRxEnd;	//�������ݱ�־
};


struct CAN_Filter
{
	uint32_t ID;
	uint32_t Mask;
	uint32_t FrameFormat;// CAN_FRAME_FORMAT_SFF or CAN_FRAME_FORMAT_EFF
};

//ȫ�ֱ�������
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
//��������
void xGPIO_CAN1();
void xINT_CAN(CAN_SFRmap* CANx);
void xInit_CAN(CAN_SFRmap* CANx,uint8_t Bdrt,uint32_t MODE,struct CAN_Filter *Filter);//Filter����ΪNULL
CAN_ErrorT CAN_Transmit_DATA(CAN_SFRmap* CANx, struct can_frame tx_frame);
CAN_ErrorT CAN_Receive_DATA(CAN_SFRmap* CANx, struct can_frame *rx_frame);
void CAN_check_busoff(CAN_SFRmap* CANx);
void CAN0_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter);//Filter����ΪNULL
void CAN1_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter);//Filter����ΪNULL
void CAN2_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter);//Filter����ΪNULL
void CAN3_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter);//Filter����ΪNULL
void CAN4_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter);//Filter����ΪNULL
void CAN5_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter);//Filter����ΪNULL
#endif /* _CANHL_H_ */
