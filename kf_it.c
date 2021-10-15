/****************************************************************************************
 *
 * File Name: kf_it.c
 * Project Name: kungfu_aebs
 * Version: v1.0
 * Date: 2021-06-16- 16:51:48
 * Author: shuai
 * 
 ****************************************************************************************/
//#include<KF32A151MQV.h>
#include "system_init.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "canhl.h"
#include "usart.h"
#include "bluetooth.h"
#include "EC200U.h"
#include "uart_task.h"
#include "common.h"
#include "data_uploading.h"
uint8_t GPS_Step = 0;
uint8_t speed_level = 0;
uint32_t first_time = 0;
//double doubulespeedsensor = 0;
uint32_t second_time = 0;
extern uint8_t Receive_flag_Uart_4;
extern void can0_receive(struct can_frame rx_frame);
extern void can1_receive(struct can_frame rx_frame);
extern void can2_receive(struct can_frame rx_frame);
extern void can3_receive(struct can_frame rx_frame);
extern void can4_receive(struct can_frame rx_frame);
extern void can5_receive(struct can_frame rx_frame);
extern void can0_can2_receive(struct can_frame rx_frame);
extern uint8_t User_Rxbuf[1024];
extern uint32_t User_Rxc;
uint32_t Time14_CNT;
uint32_t Time15_CNT;
//asm(".include		\"KF32A151MQV.inc\"	");	 

//Note:
//*****************************************************************************************
//                                 NMI Interrupt Course
//*****************************************************************************************	
void __attribute__((interrupt)) _NMI_exception (void)
{	

}

//*****************************************************************************************
//                               HardFault Interrupt Course
//*****************************************************************************************	

void __attribute__((interrupt)) _HardFault_exception (void)
{

}

//*****************************************************************************************
//                               StackFault Interrupt Course
//*****************************************************************************************	
void __attribute__((interrupt)) _StackFault_exception (void)
{

}

//*****************************************************************************************
//                               SVC Interrupt Course
//*****************************************************************************************	
//void __attribute__((interrupt)) _SVC_exception (void)
//{

//}

//*****************************************************************************************
//                              SoftSV Interrupt Course
//*****************************************************************************************	
//void __attribute__((interrupt)) _SoftSV_exception (void)
//{

//}

//*****************************************************************************************
//                              SysTick Interrupt Course
//*****************************************************************************************	
//void __attribute__((interrupt)) _SysTick_exception (void)
//{
	
//}

//*****************************************************************************************
//                              CAN0 摄像头can
//*****************************************************************************************	//

void __attribute__((interrupt))_CAN0_exception (void)
{
	uint8_t i;
	static uint8_t Receice_addr=0x00;//接收RAM偏移地址
	CAN_MessageTypeDef CAN_MessageStructure;//接收报文结构体
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//struct can_frame rx_frame;
	/* 清CAN中断标志 */
	INT_Clear_Interrupt_Flag(INT_CAN0);

	/* 判断是否为CAN发送标志 */
	if(CAN_Get_INT_Flag(CAN0_SFR,CAN_INT_TRANSMIT) != RESET)
	{
		CAN_Clear_INT_Flag(CAN0_SFR,CAN_INT_TRANSMIT);
		CAN0_SFR->CTLR &= ~0x300;//清除发送使能位

	}

	/* 判断总线错误标志 */
	if(CAN_Get_INT_Flag(CAN0_SFR,CAN_INT_BUS_ERROR) != RESET)
	{
		CAN_Clear_INT_Flag(CAN0_SFR,CAN_INT_BUS_ERROR);
		CAN0_SFR->CTLR &= ~0x300; //总线错误不重发
	}

	/* 判断是否为CAN接收标志 */
	if(CAN_Get_INT_Flag(CAN0_SFR,CAN_INT_RECEIVE) != RESET)
	{
//		GPIO_Toggle_Output_Data_Config (GPIOB_SFR,GPIO_PIN_MASK_8);
		/* 接收RAM数据 */
		CAN_Receive_Message_Configuration(CAN0_SFR,Receice_addr,&CAN_MessageStructure);
		/* RAM地址自跳 */
		Receice_addr+=0x10;
		/* 释放一次计数器 */
		CAN_Release_Receive_Buffer(CAN0_SFR,1);

		if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_SFF)//标准帧
		{
			can0_rx_frame.TargetID = CAN_MessageStructure.m_StandardID;			 //标准帧ID
		}
		else if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_EFF)//扩展帧
		{
			can0_rx_frame.TargetID = CAN_MessageStructure.m_ExtendedID;			 //扩展帧ID
		}

		if(CAN_MessageStructure.m_RemoteTransmit != CAN_DATA_FRAME)//远程帧
		{
			can0_rx_frame. MsgType = CAN_MessageStructure.m_RemoteTransmit;
			can0_rx_frame.data[0] =0xAA;
			//用户代码
			//USART_Send(USART2_SFR,ReceiveData,1);

		}
		else  //数据帧
		{

			can0_rx_frame.lenth = CAN_MessageStructure.m_DataLength;			    	 //长度
			can0_rx_frame.RmtFrm = CAN_MessageStructure.m_FrameFormat;             	 //帧格式
			can0_rx_frame. MsgType= CAN_MessageStructure.m_RemoteTransmit;		 	 //帧类型

			for(i=0;i<CAN_MessageStructure.m_DataLength;i++)
			{
				can0_rx_frame.data[i] = CAN_MessageStructure.m_Data[i];
			}
			/*
			if(CAN0_RX_COUNT<(CAN_BUFFER_MAX-1))
				CAN0_RX_COUNT++;
			else
				CAN0_RX_COUNT=0;
			*/
			can0_receive(can0_rx_frame);

		}
	}

}

//*****************************************************************************************
//                              CAN1 超声波雷达can
//*****************************************************************************************	//

void __attribute__((interrupt))_CAN1_exception (void)
{
	uint8_t i;
	static uint8_t Receice_addr=0x00;//接收RAM偏移地址
	CAN_MessageTypeDef CAN_MessageStructure;//接收报文结构体
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//struct can_frame rx_frame;
	/* 清CAN中断标志 */
	INT_Clear_Interrupt_Flag(INT_CAN1);

	/* 判断是否为CAN发送标志 */
	if(CAN_Get_INT_Flag(CAN1_SFR,CAN_INT_TRANSMIT) != RESET)
	{
		CAN_Clear_INT_Flag(CAN1_SFR,CAN_INT_TRANSMIT);
		CAN1_SFR->CTLR &= ~0x300;//清除发送使能位

	}

	/* 判断总线错误标志 */
	if(CAN_Get_INT_Flag(CAN1_SFR,CAN_INT_BUS_ERROR) != RESET)
	{
		CAN_Clear_INT_Flag(CAN1_SFR,CAN_INT_BUS_ERROR);
		CAN1_SFR->CTLR &= ~0x300; //总线错误不重发
	}

	/* 判断是否为CAN接收标志 */
	if(CAN_Get_INT_Flag(CAN1_SFR,CAN_INT_RECEIVE) != RESET)
	{
//		GPIO_Toggle_Output_Data_Config (GPIOB_SFR,GPIO_PIN_MASK_8);
		/* 接收RAM数据 */
		CAN_Receive_Message_Configuration(CAN1_SFR,Receice_addr,&CAN_MessageStructure);
		/* RAM地址自跳 */
		Receice_addr+=0x10;
		/* 释放一次计数器 */
		CAN_Release_Receive_Buffer(CAN1_SFR,1);

		if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_SFF)//标准帧
		{
			can1_rx_frame.TargetID = CAN_MessageStructure.m_StandardID;			 //标准帧ID
		}
		else if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_EFF)//扩展帧
		{
			can1_rx_frame.TargetID = CAN_MessageStructure.m_ExtendedID;			 //扩展帧ID
		}

		if(CAN_MessageStructure.m_RemoteTransmit != CAN_DATA_FRAME)//远程帧
		{
			can1_rx_frame. MsgType = CAN_MessageStructure.m_RemoteTransmit;
			can1_rx_frame.data[0] =0xAA;
			//用户代码
			//USART_Send(USART2_SFR,ReceiveData,1);

		}
		else  //数据帧
		{
			can1_rx_frame.lenth = CAN_MessageStructure.m_DataLength;			    	 //长度
			can1_rx_frame.RmtFrm = CAN_MessageStructure.m_FrameFormat;             	 //帧格式
			can1_rx_frame. MsgType= CAN_MessageStructure.m_RemoteTransmit;		 	 //帧类型
			/* 获取数据 */
			for(i=0;i<CAN_MessageStructure.m_DataLength;i++)
			{
				can1_rx_frame.data[i] = CAN_MessageStructure.m_Data[i];
			}
			//if(CAN1_RX_COUNT<(CAN_BUFFER_MAX-1))
			//	CAN1_RX_COUNT++;
			//else
			//	CAN1_RX_COUNT=0;
			//can3_receive(can1_rx_frame);
			//can1_receive(can1_rx_frame);
			Ureader_receive(can1_rx_frame);
			//
		}
	}

}

//*****************************************************************************************
//                              CAN2整车can
//*****************************************************************************************	//

void __attribute__((interrupt))_CAN2_exception (void)
{
	uint8_t i;
	static uint8_t Receice_addr=0x00;//接收RAM偏移地址
	CAN_MessageTypeDef CAN_MessageStructure;//接收报文结构体
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//struct can_frame rx_frame;
	/* 清CAN中断标志 */
	INT_Clear_Interrupt_Flag(INT_CAN2);

	/* 判断是否为CAN发送标志 */
	if(CAN_Get_INT_Flag(CAN2_SFR,CAN_INT_TRANSMIT) != RESET)
	{
		CAN_Clear_INT_Flag(CAN2_SFR,CAN_INT_TRANSMIT);
		CAN2_SFR->CTLR &= ~0x300;//清除发送使能位
	}

	/* 判断总线错误标志 */
	if(CAN_Get_INT_Flag(CAN2_SFR,CAN_INT_BUS_ERROR) != RESET)
	{
		CAN_Clear_INT_Flag(CAN2_SFR,CAN_INT_BUS_ERROR);
		CAN2_SFR->CTLR &= ~0x300; //总线错误不重发
	}

	/* 判断是否为CAN接收标志 */
	if(CAN_Get_INT_Flag(CAN2_SFR,CAN_INT_RECEIVE) != RESET)
	{
//		GPIO_Toggle_Output_Data_Config (GPIOB_SFR,GPIO_PIN_MASK_8);
		/* 接收RAM数据 */
		CAN_Receive_Message_Configuration(CAN2_SFR,Receice_addr,&CAN_MessageStructure);
		/* RAM地址自跳 */
		Receice_addr+=0x10;
		/* 释放一次计数器 */
		CAN_Release_Receive_Buffer(CAN2_SFR,1);

		if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_SFF)//标准帧
		{
			can2_rx_frame.TargetID = CAN_MessageStructure.m_StandardID;			 //标准帧ID
		}
		else if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_EFF)//扩展帧
		{
			can2_rx_frame.TargetID = CAN_MessageStructure.m_ExtendedID;			 //扩展帧ID
		}

		if(CAN_MessageStructure.m_RemoteTransmit != CAN_DATA_FRAME)//远程帧
		{
			can2_rx_frame. MsgType = CAN_MessageStructure.m_RemoteTransmit;
			can2_rx_frame.data[0] =0xAA;
			//用户代码
			//USART_Send(USART2_SFR,ReceiveData,1);

		}
		else  //数据帧
		{
			can2_rx_frame.lenth = CAN_MessageStructure.m_DataLength;			    	 //长度
			can2_rx_frame.RmtFrm = CAN_MessageStructure.m_FrameFormat;             	 //帧格式
			can2_rx_frame. MsgType= CAN_MessageStructure.m_RemoteTransmit;		 	 //帧类型
			/* 获取数据 */
			for(i=0;i<CAN_MessageStructure.m_DataLength;i++)
			{
				can2_rx_frame.data[i] = CAN_MessageStructure.m_Data[i];
			}
			can2_receive(can2_rx_frame);
		}
	}

}

//***********************************************************************************
//								CAN3比例阀can
//**********************************************************************************
void __attribute__((interrupt))_CAN3_exception (void)
{
	uint8_t i;
	static uint8_t Receice_addr=0x00;//接收RAM偏移地址
	CAN_MessageTypeDef CAN_MessageStructure;//接收报文结构体
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//struct can_frame rx_frame;
	/* 清CAN中断标志 */
	INT_Clear_Interrupt_Flag(INT_CAN3);

	/* 判断是否为CAN发送标志 */
	if(CAN_Get_INT_Flag(CAN3_SFR,CAN_INT_TRANSMIT) != RESET)
	{
		CAN_Clear_INT_Flag(CAN3_SFR,CAN_INT_TRANSMIT);
		CAN3_SFR->CTLR &= ~0x300;//清除发送使能位
	}

	/* 判断总线错误标志 */
	if(CAN_Get_INT_Flag(CAN3_SFR,CAN_INT_BUS_ERROR) != RESET)
	{
		CAN_Clear_INT_Flag(CAN3_SFR,CAN_INT_BUS_ERROR);
		CAN3_SFR->CTLR &= ~0x300; //总线错误不重发
	}

	/* 判断是否为CAN接收标志 */
	if(CAN_Get_INT_Flag(CAN3_SFR,CAN_INT_RECEIVE) != RESET)
	{
//		GPIO_Toggle_Output_Data_Config (GPIOB_SFR,GPIO_PIN_MASK_8);
		/* 接收RAM数据 */
		CAN_Receive_Message_Configuration(CAN3_SFR,Receice_addr,&CAN_MessageStructure);
		/* RAM地址自跳 */
		Receice_addr+=0x10;
		/* 释放一次计数器 */
		CAN_Release_Receive_Buffer(CAN3_SFR,1);

		if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_SFF)//标准帧
		{
			can3_rx_frame.TargetID = CAN_MessageStructure.m_StandardID;			 //标准帧ID
		}
		else if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_EFF)//扩展帧
		{
			can3_rx_frame.TargetID = CAN_MessageStructure.m_ExtendedID;			 //扩展帧ID
		}

		if(CAN_MessageStructure.m_RemoteTransmit != CAN_DATA_FRAME)//远程帧
		{
			can3_rx_frame. MsgType = CAN_MessageStructure.m_RemoteTransmit;
			can3_rx_frame.data[0] =0xAA;
			//用户代码
			//USART_Send(USART2_SFR,ReceiveData,1);

		}
		else  //数据帧
		{
			can3_rx_frame.lenth = CAN_MessageStructure.m_DataLength;			    	 //长度
			can3_rx_frame.RmtFrm = CAN_MessageStructure.m_FrameFormat;             	 //帧格式
			can3_rx_frame. MsgType= CAN_MessageStructure.m_RemoteTransmit;		 	 //帧类型
			/* 获取数据 */
			for(i=0;i<CAN_MessageStructure.m_DataLength;i++)
			{
				can3_rx_frame.data[i] = CAN_MessageStructure.m_Data[i];
			}
			//if(CAN3_RX_COUNT<(CAN_BUFFER_MAX-1))
			//	CAN3_RX_COUNT++;
			//else
			//	CAN3_RX_COUNT=0;
			can3_receive(can3_rx_frame);
		}
	}

}

//*****************************************************************************************
//                              CAN4毫米波雷达can
//*****************************************************************************************	//

void __attribute__((interrupt))_CAN4_exception (void)
{
	uint8_t i;
	static uint8_t Receice_addr=0x00;//接收RAM偏移地址
	CAN_MessageTypeDef CAN_MessageStructure;//接收报文结构体
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//struct can_frame rx_frame;
	// 清CAN中断标志
	INT_Clear_Interrupt_Flag(INT_CAN4);

	// 判断是否为CAN发送标志
	if(CAN_Get_INT_Flag(CAN4_SFR,CAN_INT_TRANSMIT) != RESET)
	{
		CAN_Clear_INT_Flag(CAN4_SFR,CAN_INT_TRANSMIT);
		CAN4_SFR->CTLR &= ~0x300;//清除发送使能位
	}

	// 判断总线错误标志
	if(CAN_Get_INT_Flag(CAN4_SFR,CAN_INT_BUS_ERROR) != RESET)
	{
		CAN_Clear_INT_Flag(CAN4_SFR,CAN_INT_BUS_ERROR);
		CAN4_SFR->CTLR &= ~0x300; //总线错误不重发
	}

	// 判断是否为CAN接收标志
	if(CAN_Get_INT_Flag(CAN4_SFR,CAN_INT_RECEIVE) != RESET)
	{
//		GPIO_Toggle_Output_Data_Config (GPIOB_SFR,GPIO_PIN_MASK_8);
		// 接收RAM数据
		CAN_Receive_Message_Configuration(CAN4_SFR,Receice_addr,&CAN_MessageStructure);
		// RAM地址自跳
		Receice_addr+=0x10;
		// 释放一次计数器
		CAN_Release_Receive_Buffer(CAN4_SFR,1);

		if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_SFF)//标准帧
		{
			can4_rx_frame.TargetID = CAN_MessageStructure.m_StandardID;			 //标准帧ID
		}
		else if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_EFF)//扩展帧
		{
			can4_rx_frame.TargetID = CAN_MessageStructure.m_ExtendedID;			 //扩展帧ID
		}

		if(CAN_MessageStructure.m_RemoteTransmit != CAN_DATA_FRAME)//远程帧
		{
			can4_rx_frame. MsgType = CAN_MessageStructure.m_RemoteTransmit;
			can4_rx_frame.data[0] =0xAA;
			//用户代码
			//USART_Send(USART2_SFR,ReceiveData,1);

		}
		else  //数据帧
		{
			can4_rx_frame.lenth = CAN_MessageStructure.m_DataLength;			    	 //长度
			can4_rx_frame.RmtFrm = CAN_MessageStructure.m_FrameFormat;             	 //帧格式
			can4_rx_frame. MsgType= CAN_MessageStructure.m_RemoteTransmit;		 	 //帧类型
			// 获取数据
			for(i=0;i<CAN_MessageStructure.m_DataLength;i++)
			{
				can4_rx_frame.data[i] = CAN_MessageStructure.m_Data[i];
			}
			//if(CAN4_RX_COUNT<(CAN_BUFFER_MAX-1))
			//	CAN4_RX_COUNT++;
			//else
			//	CAN4_RX_COUNT=0;
			//fprintf(BLUETOOTH_STREAM,"can4_receive");
			can4_receive(can4_rx_frame);
			//Ureader_receive(can1_rx_frame);
		}
	}

}


void __attribute__((interrupt))_CAN5_exception (void)
{
	uint8_t i;
	static uint8_t Receice_addr=0x00;//接收RAM偏移地址
	CAN_MessageTypeDef CAN_MessageStructure;//接收报文结构体
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//struct can_frame rx_frame;
	/* 清CAN中断标志 */
	INT_Clear_Interrupt_Flag(INT_CAN5);

	/* 判断是否为CAN发送标志 */
	if(CAN_Get_INT_Flag(CAN5_SFR,CAN_INT_TRANSMIT) != RESET)
	{
		CAN_Clear_INT_Flag(CAN5_SFR,CAN_INT_TRANSMIT);
		CAN5_SFR->CTLR &= ~0x300;//清除发送使能位
	}

	/* 判断总线错误标志 */
	if(CAN_Get_INT_Flag(CAN5_SFR,CAN_INT_BUS_ERROR) != RESET)
	{
		CAN_Clear_INT_Flag(CAN5_SFR,CAN_INT_BUS_ERROR);
		CAN5_SFR->CTLR &= ~0x300; //总线错误不重发
	}

	/* 判断是否为CAN接收标志 */
	if(CAN_Get_INT_Flag(CAN5_SFR,CAN_INT_RECEIVE) != RESET)
	{
//		GPIO_Toggle_Output_Data_Config (GPIOB_SFR,GPIO_PIN_MASK_8);
		/* 接收RAM数据 */
		CAN_Receive_Message_Configuration(CAN5_SFR,Receice_addr,&CAN_MessageStructure);
		/* RAM地址自跳 */
		Receice_addr+=0x10;
		/* 释放一次计数器 */
		CAN_Release_Receive_Buffer(CAN5_SFR,1);

		if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_SFF)//标准帧
		{
			can5_rx_frame.TargetID = CAN_MessageStructure.m_StandardID;			 //标准帧ID
		}
		else if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_EFF)//扩展帧
		{
			can5_rx_frame.TargetID = CAN_MessageStructure.m_ExtendedID;			 //扩展帧ID
		}

		if(CAN_MessageStructure.m_RemoteTransmit != CAN_DATA_FRAME)//远程帧
		{
			can5_rx_frame. MsgType = CAN_MessageStructure.m_RemoteTransmit;
			can5_rx_frame.data[0] =0xAA;
			//用户代码
			//USART_Send(USART2_SFR,ReceiveData,1);

		}
		else  //数据帧
		{
			can5_rx_frame.lenth = CAN_MessageStructure.m_DataLength;			    	 //长度
			can5_rx_frame.RmtFrm = CAN_MessageStructure.m_FrameFormat;             	 //帧格式
			can5_rx_frame. MsgType= CAN_MessageStructure.m_RemoteTransmit;		 	 //帧类型
			/* 获取数据 */
			for(i=0;i<CAN_MessageStructure.m_DataLength;i++)
			{
				can5_rx_frame.data[i] = CAN_MessageStructure.m_Data[i];
			}
			//if(CAN4_RX_COUNT<(CAN_BUFFER_MAX-1))
			//	CAN4_RX_COUNT++;
			//else
			//	CAN4_RX_COUNT=0;
			can5_receive(can5_rx_frame);
		}
	}

}

//*****************************************************************************************
//                              UART1中断函数
//*****************************************************************************************	//
uint8_t startflag = 0;
uint8_t stopflag = 1;

void __attribute__((interrupt))_USART1_exception (void)
{
	uint8_t data;
	uint8_t i = 0;
	//uint8
	if(USART_Get_Receive_BUFR_Ready_Flag(USART1_SFR))
	{
		data=USART_ReceiveData(USART1_SFR);
		if(User_Rxcount	<	1024){
			User_Rxbuffer[User_Rxcount ]=data;
			if(User_Rxbuffer[0] == 0x02)
			{
				if(User_Rxcount < 2)
					User_Rxcount++;
				else
				{
					if(User_Rxbuffer[1] == 0x03)
					{
						if(User_Rxcount < 10)
						{
							User_Rxcount++;
						}
						else if((User_Rxbuffer[User_Rxcount - 1] == 0x02)	&&
								(User_Rxbuffer[User_Rxcount] == 0x03) && User_Rxcount == 11)
						{
							User_Rxcount = 0;
							for(i = 0;i < 8;i ++)
								User_Rxbuf[i] = User_Rxbuffer[i + 2];
							DisPlay_Message_Analy(User_Rxbuf);
						}
						else if(User_Rxcount < 11)
							User_Rxcount++;
						else
							User_Rxcount = 0;
					}
					else
						User_Rxcount = 0;
				}
				/*
				if(User_Rxcount < 10)
				{
					User_Rxcount++;
				}
				else if((User_Rxbuffer[User_Rxcount - 1] == 0x02)	&&
						(User_Rxbuffer[User_Rxcount] == 0x03))
				{
					User_Rxcount = 0;
					for(i = 0;i < 8;i ++)
						User_Rxbuf[i] = User_Rxbuffer[i + 2];
					DisPlay_Message_Analy(User_Rxbuf);
				}
				else
					User_Rxcount++;
					*/
			}
			else
				User_Rxcount = 0;
		}

			/*
			if((User_Rxbuffer[0] == 0x02) && (startflag == 0) && (stopflag == 1))
			{
				User_Rxcount ++;
				//stopflag = 0;
			}

			if((startflag == 1) && (stopflag == 1))
			{
				if((User_Rxc < 8))
				{
					User_Rxbuf[User_Rxc ++] = User_Rxbuffer[User_Rxcount];
					User_Rxcount ++;
				}
				else if(User_Rxc == 8)
				{
					//User_Rxcount = 0;
					//startflag = 0;
					User_Rxc = 0;
					stopflag = 0;
					DisPlayMessageAnaly(User_Rxbuf);
					User_Rxcount ++;
				}
			}

			if((User_Rxbuffer[0] == 0x02) && (User_Rxbuffer[1] == 0x03) && (startflag == 0) && (stopflag == 1))
			{
				User_Rxcount ++;
				startflag = 1;
				User_Rxc = 0;
				stopflag == 0;
			}

			if((startflag == 1)&& (User_Rxbuffer[11] == 0x03)&& (User_Rxbuffer[10] == 0x02))
			{
				startflag == 0;
				stopflag = 1;
			}*/
	}

}

//*****************************************************************************************
//                              UART2中断函数
//*****************************************************************************************	//
//void __attribute__((interrupt))_USART2_exception (void)
//{
//	uint8_t data;
//	if(USART_Get_Receive_BUFR_Ready_Flag(USART2_SFR))
//	{
//		data=USART_ReceiveData(USART2_SFR);
//		USART_Send(USART2_SFR,&data,1);
//		if(User_Rxcount<256&&User_Receive_flag==0){
//			User_Rxbuffer[User_Rxcount]=data;
//			if(User_Rxbuffer[User_Rxcount]=='\r'||User_Rxbuffer[User_Rxcount]=='\n')
//				User_Receive_flag=1;
//			User_Rxcount++;
//		}
//		else
//			User_Receive_flag=1;
//
//	}
//
//}

//*****************************************************************************************
//                              蓝牙中断函数
//*****************************************************************************************	//

void __attribute__((interrupt))_USART4_exception (void)
{
	uint8_t data;
	if(USART_Get_Receive_BUFR_Ready_Flag(USART4_SFR))
	{
		data=USART_ReceiveData(USART4_SFR);
		//USART_Send(USART1_SFR,&data,1);
		if(User_Rxcount	<	1024){
			User_Rxbuffer[User_Rxcount]=data;
			//if(User_Rxbuffer[User_Rxcount]=='\r'||User_Rxbuffer[User_Rxcount]=='\n')
			//	User_Receive_flag=1;
			if(User_Rxbuffer[0] == 'U')
			{
				if(User_Rxcount < 10)
				{
					User_Rxcount++;
				}
				else if((User_Rxbuffer[User_Rxcount - 4] == 'U')	&&
						(User_Rxbuffer[User_Rxcount - 3] == 'S')	&&
						(User_Rxbuffer[User_Rxcount - 2] == 'T')	&&
						(User_Rxbuffer[User_Rxcount - 1] == 'O')	&&
						(User_Rxbuffer[User_Rxcount] == 'P'))
				{
					User_Rxcount = 0;
					Bluetooth_Data_Annalyse(User_Rxbuffer);
				}
				else
					User_Rxcount++;
			}
		}
	}
}

//*****************************************************************************************
//                              预留超声波雷达串口、byd上报串口
//*****************************************************************************************	//
uint8_t data;
void __attribute__((interrupt))_USART2_exception (void)
{

	if(USART_Get_Receive_BUFR_Ready_Flag(USART2_SFR))
	{
		data=USART_ReceiveData(USART2_SFR);
		Data_Uploading_In_KFIT(data);			// 数据上传(配合才库GPS 4G模块) add lmz 20211012
		//USART_Send(USART1_SFR,&data,1);
#if 0
		if(Uart2_Rxcount	<	1024){
			Uart2_Rxbuffer[Uart2_Rxcount]=data;
			//if(User_Rxbuffer[User_Rxcount]=='\r'||User_Rxbuffer[User_Rxcount]=='\n')
			//	User_Receive_flag=1;
			if(Uart2_Rxbuffer[0] == 'U')
			{
				if(Uart2_Rxcount < 10)
				{
					Uart2_Rxcount++;
				}
				else if((Uart2_Rxbuffer[Uart2_Rxcount - 4] == 'U')	&&
						(Uart2_Rxbuffer[Uart2_Rxcount - 3] == 'S')	&&
						(Uart2_Rxbuffer[Uart2_Rxcount - 2] == 'T')	&&
						(Uart2_Rxbuffer[Uart2_Rxcount - 1] == 'O')	&&
						(Uart2_Rxbuffer[Uart2_Rxcount] == 'P'))
				{
					Uart2_Rxcount = 0;
					Bluetooth_Data_Annalyse(Uart2_Rxbuffer);
				}
				else
					Uart2_Rxcount++;
			}
		}
#endif
	}
}
/*
void __attribute__((interrupt))_USART4_exception (void)
{
	uint8_t data;
	if(USART_Get_Receive_BUFR_Ready_Flag(USART4_SFR))
	{
		if(USART_ReceiveData(USART4_SFR)==0x5A)  //读接收缓存，清0标志位
		{
			Receive_flag_Uart_4=1;
		}
	}
	//if(USART_Get_Receive_BUFR_Ready_Flag(USART4_SFR))
		{
			data=USART_ReceiveData(USART4_SFR);
			Usart4_recv_buff[Usart4_recv_len] = data;
			if(Usart4_recv_len < 5)
			{

				Usart4_recv_len ++;
			}
			else
			{
				if((Usart4_recv_buff[Usart4_recv_len] == 'U')&&
						(Usart4_recv_buff[Usart4_recv_len] == 'S')&&
						(Usart4_recv_buff[Usart4_recv_len] == 'T')&&
						(Usart4_recv_buff[Usart4_recv_len] == 'O')&&
						(Usart4_recv_buff[Usart4_recv_len] == 'P'))
				{

					Usart4_recv_len = 0;
					Receive_flag_Uart_4 = 1;
				}
				else
				{
					Usart4_recv_len ++;
				}
			}

			//Usart4_Receive_data(data);


			if(Bluetooth_Rxcount<Bluetooth_Rxbuffer_MAX){
				Bluetooth_Rxbuffer[Bluetooth_Rxcount]=data;
				if(Bluetooth_Rxbuffer[Bluetooth_Rxcount]=='\r'||Bluetooth_Rxbuffer[Bluetooth_Rxcount]=='\n')
					Bluetooth_Receive_flag=1;
				Bluetooth_Rxcount++;
			}
			else
				Bluetooth_Receive_flag=1;


		}

}
*/
//*****************************************************************************************
//                              UART0中断函数
//*****************************************************************************************	//
void __attribute__((interrupt))_USART0_exception (void)
{
	uint8_t data;
	if(USART_Get_Receive_BUFR_Ready_Flag(USART0_SFR))
	{
		data=USART_ReceiveData(USART0_SFR);
		//USART_Send(USART0_SFR,&data,1);
		if(EC200U_Rxcount < EC200U_Rxbuffer_MAX){
			EC200U_Rxbuffer[EC200U_Rxcount]=data;
			if(EC200U_Rxbuffer[EC200U_Rxcount - 1]=='O'||EC200U_Rxbuffer[EC200U_Rxcount]=='K')
			{
				GPS_Data_Analysis(EC200U_Rxbuffer);
				EC200U_Receive_flag=1;
				EC200U_Rxcount = 0;
			}
			else
				EC200U_Rxcount++;
		}
		else
			EC200U_Receive_flag=1;

	}

}

void __attribute__((interrupt))_T14_exception (void)
{

	BTIM_Clear_Updata_INT_Flag(T14_SFR);										//清更新时间标志位
	BTIM_Clear_Overflow_INT_Flag (T14_SFR);										//清T14溢出中断标志位

	Time14_CNT++;
	//if(Time14_CNT>=10)//100ms进行一次翻转
	//{
		//Time14_CNT =0;
		//GPIO_Toggle_Output_Data_Config(GPIOB_SFR,GPIO_PIN_MASK_13);               //PB13翻转 LED0闪烁
	//}
	if(GPS_Step == 0)
	{
		//EC200U_SendData("AT+QGPS?\r\n",10);
		//gps_oldtime = 0;
	}

}

//*****************************************************************************************
//                              T15中断函数
//*****************************************************************************************	//
uint32_t PLUSE_WIDTH=0;//捕捉到PWM周期变量
void __attribute__((interrupt))_T15_exception (void)
{

	BTIM_Clear_Updata_INT_Flag(T15_SFR);									 //清更新时间标志位
	BTIM_Clear_Overflow_INT_Flag (T15_SFR);									//清T15溢出中断标志位

	Time15_CNT++;
	if(Time15_CNT>=10)  //100ms进行一
	{
		Time15_CNT =0;
	}
}

//*****************************************************************************************
//                              SysTick中断函数
//*****************************************************************************************
void __attribute__((interrupt)) _SysTick_exception (void)
{
	static uint32_t send_time = 0;
	struct can_frame tx_frame_valve;
	//节拍定时器的中断无需软件清除标志位
	SystemtimeClock ++;

	if(stSpeedPara.uSingnalType == 1)
	{
		if((0 == speed_level) && (1 == GPIO_Read_Input_Data_Bit (GPIOC_SFR, GPIO_PIN_MASK_4)))
		{
			first_time = SystemtimeClock - second_time;
			second_time = SystemtimeClock;
		}
		speed_level = GPIO_Read_Input_Data_Bit (GPIOC_SFR, GPIO_PIN_MASK_4);
	}
	if(stSysPara.ucControlMode == 2)	//BYD公交需要发送AEB存在命令20ms一次
	{
		if((SystemtimeClock - send_time) > 20)
		{
			Valve_AEBdecActive_Get(&tx_frame_valve);
			tx_frame_valve.TargetID = 0x1C01F025;
			CAN_Transmit_DATA(CAN2_SFR,tx_frame_valve);
		}
	}
}
