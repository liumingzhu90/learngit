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
#include "usart_upgrade.h"
#include "_4g_upgrade.h"
#include "_4g_data_upload.h"
#include "at_parse_alg_para.h"
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

uint8_t test_ttt = 0;
//asm(".include		\"KF32A151MQV.inc\"	");	 

//Note:
//*****************************************************************************************
//                                 NMI Interrupt Course
//*****************************************************************************************	
void __attribute__((interrupt)) _NMI_exception (void)
{	
	test_ttt = 0;
}

//*****************************************************************************************
//                               HardFault Interrupt Course
//*****************************************************************************************	

void __attribute__((interrupt)) _HardFault_exception (void)
{
	test_ttt = 1;
}

//*****************************************************************************************
//                               StackFault Interrupt Course
//*****************************************************************************************	
void __attribute__((interrupt)) _StackFault_exception (void)
{
	test_ttt = 2;
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
//                              CAN0 ����ͷcan
//*****************************************************************************************	//

void __attribute__((interrupt))_CAN0_exception (void)
{
	uint8_t i;
	static uint8_t Receice_addr=0x00;//����RAMƫ�Ƶ�ַ
	CAN_MessageTypeDef CAN_MessageStructure;//���ձ��Ľṹ��
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//struct can_frame rx_frame;
	/* ��CAN�жϱ�־ */
	INT_Clear_Interrupt_Flag(INT_CAN0);

	/* �ж��Ƿ�ΪCAN���ͱ�־ */
	if(CAN_Get_INT_Flag(CAN0_SFR,CAN_INT_TRANSMIT) != RESET)
	{
		CAN_Clear_INT_Flag(CAN0_SFR,CAN_INT_TRANSMIT);
		CAN0_SFR->CTLR &= ~0x300;//�������ʹ��λ

	}

	/* �ж����ߴ����־ */
	if(CAN_Get_INT_Flag(CAN0_SFR,CAN_INT_BUS_ERROR) != RESET)
	{
		CAN_Clear_INT_Flag(CAN0_SFR,CAN_INT_BUS_ERROR);
		CAN0_SFR->CTLR &= ~0x300; //���ߴ����ط�
	}

	/* �ж��Ƿ�ΪCAN���ձ�־ */
	if(CAN_Get_INT_Flag(CAN0_SFR,CAN_INT_RECEIVE) != RESET)
	{
//		GPIO_Toggle_Output_Data_Config (GPIOB_SFR,GPIO_PIN_MASK_8);
		/* ����RAM���� */
		CAN_Receive_Message_Configuration(CAN0_SFR,Receice_addr,&CAN_MessageStructure);
		/* RAM��ַ���� */
		Receice_addr+=0x10;
		/* �ͷ�һ�μ����� */
		CAN_Release_Receive_Buffer(CAN0_SFR,1);

		if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_SFF)//��׼֡
		{
			can0_rx_frame.TargetID = CAN_MessageStructure.m_StandardID;			 //��׼֡ID
		}
		else if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_EFF)//��չ֡
		{
			can0_rx_frame.TargetID = CAN_MessageStructure.m_ExtendedID;			 //��չ֡ID
		}

		if(CAN_MessageStructure.m_RemoteTransmit != CAN_DATA_FRAME)//Զ��֡
		{
			can0_rx_frame. MsgType = CAN_MessageStructure.m_RemoteTransmit;
			can0_rx_frame.data[0] =0xAA;
			//�û�����
			//USART_Send(USART2_SFR,ReceiveData,1);

		}
		else  //����֡
		{

			can0_rx_frame.lenth = CAN_MessageStructure.m_DataLength;			    	 //����
			can0_rx_frame.RmtFrm = CAN_MessageStructure.m_FrameFormat;             	 //֡��ʽ
			can0_rx_frame. MsgType= CAN_MessageStructure.m_RemoteTransmit;		 	 //֡����

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
//                              CAN1 �������״�can
//*****************************************************************************************	//

void __attribute__((interrupt))_CAN1_exception (void)
{
	uint8_t i;
	static uint8_t Receice_addr=0x00;//����RAMƫ�Ƶ�ַ
	CAN_MessageTypeDef CAN_MessageStructure;//���ձ��Ľṹ��
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//struct can_frame rx_frame;
	/* ��CAN�жϱ�־ */
	INT_Clear_Interrupt_Flag(INT_CAN1);

	/* �ж��Ƿ�ΪCAN���ͱ�־ */
	if(CAN_Get_INT_Flag(CAN1_SFR,CAN_INT_TRANSMIT) != RESET)
	{
		CAN_Clear_INT_Flag(CAN1_SFR,CAN_INT_TRANSMIT);
		CAN1_SFR->CTLR &= ~0x300;//�������ʹ��λ

	}

	/* �ж����ߴ����־ */
	if(CAN_Get_INT_Flag(CAN1_SFR,CAN_INT_BUS_ERROR) != RESET)
	{
		CAN_Clear_INT_Flag(CAN1_SFR,CAN_INT_BUS_ERROR);
		CAN1_SFR->CTLR &= ~0x300; //���ߴ����ط�
	}

	/* �ж��Ƿ�ΪCAN���ձ�־ */
	if(CAN_Get_INT_Flag(CAN1_SFR,CAN_INT_RECEIVE) != RESET)
	{
//		GPIO_Toggle_Output_Data_Config (GPIOB_SFR,GPIO_PIN_MASK_8);
		/* ����RAM���� */
		CAN_Receive_Message_Configuration(CAN1_SFR,Receice_addr,&CAN_MessageStructure);
		/* RAM��ַ���� */
		Receice_addr+=0x10;
		/* �ͷ�һ�μ����� */
		CAN_Release_Receive_Buffer(CAN1_SFR,1);

		if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_SFF)//��׼֡
		{
			can1_rx_frame.TargetID = CAN_MessageStructure.m_StandardID;			 //��׼֡ID
		}
		else if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_EFF)//��չ֡
		{
			can1_rx_frame.TargetID = CAN_MessageStructure.m_ExtendedID;			 //��չ֡ID
		}

		if(CAN_MessageStructure.m_RemoteTransmit != CAN_DATA_FRAME)//Զ��֡
		{
			can1_rx_frame. MsgType = CAN_MessageStructure.m_RemoteTransmit;
			can1_rx_frame.data[0] =0xAA;
			//�û�����
			//USART_Send(USART2_SFR,ReceiveData,1);

		}
		else  //����֡
		{
			can1_rx_frame.lenth = CAN_MessageStructure.m_DataLength;			    	 //����
			can1_rx_frame.RmtFrm = CAN_MessageStructure.m_FrameFormat;             	 //֡��ʽ
			can1_rx_frame. MsgType= CAN_MessageStructure.m_RemoteTransmit;		 	 //֡����
			/* ��ȡ���� */
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
//                              CAN2����can
//*****************************************************************************************	//

void __attribute__((interrupt))_CAN2_exception (void)
{
	uint8_t i;
	static uint8_t Receice_addr=0x00;//����RAMƫ�Ƶ�ַ
	CAN_MessageTypeDef CAN_MessageStructure;//���ձ��Ľṹ��
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//struct can_frame rx_frame;
	/* ��CAN�жϱ�־ */
	INT_Clear_Interrupt_Flag(INT_CAN2);

	/* �ж��Ƿ�ΪCAN���ͱ�־ */
	if(CAN_Get_INT_Flag(CAN2_SFR,CAN_INT_TRANSMIT) != RESET)
	{
		CAN_Clear_INT_Flag(CAN2_SFR,CAN_INT_TRANSMIT);
		CAN2_SFR->CTLR &= ~0x300;//�������ʹ��λ
	}

	/* �ж����ߴ����־ */
	if(CAN_Get_INT_Flag(CAN2_SFR,CAN_INT_BUS_ERROR) != RESET)
	{
		CAN_Clear_INT_Flag(CAN2_SFR,CAN_INT_BUS_ERROR);
		CAN2_SFR->CTLR &= ~0x300; //���ߴ����ط�
	}

	/* �ж��Ƿ�ΪCAN���ձ�־ */
	if(CAN_Get_INT_Flag(CAN2_SFR,CAN_INT_RECEIVE) != RESET)
	{
//		GPIO_Toggle_Output_Data_Config (GPIOB_SFR,GPIO_PIN_MASK_8);
		/* ����RAM���� */
		CAN_Receive_Message_Configuration(CAN2_SFR,Receice_addr,&CAN_MessageStructure);
		/* RAM��ַ���� */
		Receice_addr+=0x10;
		/* �ͷ�һ�μ����� */
		CAN_Release_Receive_Buffer(CAN2_SFR,1);

		if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_SFF)//��׼֡
		{
			can2_rx_frame.TargetID = CAN_MessageStructure.m_StandardID;			 //��׼֡ID
		}
		else if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_EFF)//��չ֡
		{
			can2_rx_frame.TargetID = CAN_MessageStructure.m_ExtendedID;			 //��չ֡ID
		}

		if(CAN_MessageStructure.m_RemoteTransmit != CAN_DATA_FRAME)//Զ��֡
		{
			can2_rx_frame. MsgType = CAN_MessageStructure.m_RemoteTransmit;
			can2_rx_frame.data[0] =0xAA;
			//�û�����
			//USART_Send(USART2_SFR,ReceiveData,1);

		}
		else  //����֡
		{
			can2_rx_frame.lenth = CAN_MessageStructure.m_DataLength;			    	 //����
			can2_rx_frame.RmtFrm = CAN_MessageStructure.m_FrameFormat;             	 //֡��ʽ
			can2_rx_frame. MsgType= CAN_MessageStructure.m_RemoteTransmit;		 	 //֡����
			/* ��ȡ���� */
			for(i=0;i<CAN_MessageStructure.m_DataLength;i++)
			{
				can2_rx_frame.data[i] = CAN_MessageStructure.m_Data[i];
			}
			can2_receive(can2_rx_frame);
		}
	}

}

//***********************************************************************************
//								CAN3������can
//**********************************************************************************
void __attribute__((interrupt))_CAN3_exception (void)
{
	uint8_t i;
	static uint8_t Receice_addr=0x00;//����RAMƫ�Ƶ�ַ
	CAN_MessageTypeDef CAN_MessageStructure;//���ձ��Ľṹ��
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//struct can_frame rx_frame;
	/* ��CAN�жϱ�־ */
	INT_Clear_Interrupt_Flag(INT_CAN3);

	/* �ж��Ƿ�ΪCAN���ͱ�־ */
	if(CAN_Get_INT_Flag(CAN3_SFR,CAN_INT_TRANSMIT) != RESET)
	{
		CAN_Clear_INT_Flag(CAN3_SFR,CAN_INT_TRANSMIT);
		CAN3_SFR->CTLR &= ~0x300;//�������ʹ��λ
	}

	/* �ж����ߴ����־ */
	if(CAN_Get_INT_Flag(CAN3_SFR,CAN_INT_BUS_ERROR) != RESET)
	{
		CAN_Clear_INT_Flag(CAN3_SFR,CAN_INT_BUS_ERROR);
		CAN3_SFR->CTLR &= ~0x300; //���ߴ����ط�
	}

	/* �ж��Ƿ�ΪCAN���ձ�־ */
	if(CAN_Get_INT_Flag(CAN3_SFR,CAN_INT_RECEIVE) != RESET)
	{
//		GPIO_Toggle_Output_Data_Config (GPIOB_SFR,GPIO_PIN_MASK_8);
		/* ����RAM���� */
		CAN_Receive_Message_Configuration(CAN3_SFR,Receice_addr,&CAN_MessageStructure);
		/* RAM��ַ���� */
		Receice_addr+=0x10;
		/* �ͷ�һ�μ����� */
		CAN_Release_Receive_Buffer(CAN3_SFR,1);

		if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_SFF)//��׼֡
		{
			can3_rx_frame.TargetID = CAN_MessageStructure.m_StandardID;			 //��׼֡ID
		}
		else if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_EFF)//��չ֡
		{
			can3_rx_frame.TargetID = CAN_MessageStructure.m_ExtendedID;			 //��չ֡ID
		}

		if(CAN_MessageStructure.m_RemoteTransmit != CAN_DATA_FRAME)//Զ��֡
		{
			can3_rx_frame. MsgType = CAN_MessageStructure.m_RemoteTransmit;
			can3_rx_frame.data[0] =0xAA;
			//�û�����
			//USART_Send(USART2_SFR,ReceiveData,1);

		}
		else  //����֡
		{
			can3_rx_frame.lenth = CAN_MessageStructure.m_DataLength;			    	 //����
			can3_rx_frame.RmtFrm = CAN_MessageStructure.m_FrameFormat;             	 //֡��ʽ
			can3_rx_frame. MsgType= CAN_MessageStructure.m_RemoteTransmit;		 	 //֡����
			/* ��ȡ���� */
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
//                              CAN4���ײ��״�can
//*****************************************************************************************	//

void __attribute__((interrupt))_CAN4_exception (void)
{
	uint8_t i;
	static uint8_t Receice_addr=0x00;//����RAMƫ�Ƶ�ַ
	CAN_MessageTypeDef CAN_MessageStructure;//���ձ��Ľṹ��
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//struct can_frame rx_frame;
	// ��CAN�жϱ�־
	INT_Clear_Interrupt_Flag(INT_CAN4);

	// �ж��Ƿ�ΪCAN���ͱ�־
	if(CAN_Get_INT_Flag(CAN4_SFR,CAN_INT_TRANSMIT) != RESET)
	{
		CAN_Clear_INT_Flag(CAN4_SFR,CAN_INT_TRANSMIT);
		CAN4_SFR->CTLR &= ~0x300;//�������ʹ��λ
	}

	// �ж����ߴ����־
	if(CAN_Get_INT_Flag(CAN4_SFR,CAN_INT_BUS_ERROR) != RESET)
	{
		CAN_Clear_INT_Flag(CAN4_SFR,CAN_INT_BUS_ERROR);
		CAN4_SFR->CTLR &= ~0x300; //���ߴ����ط�
	}

	// �ж��Ƿ�ΪCAN���ձ�־
	if(CAN_Get_INT_Flag(CAN4_SFR,CAN_INT_RECEIVE) != RESET)
	{
//		GPIO_Toggle_Output_Data_Config (GPIOB_SFR,GPIO_PIN_MASK_8);
		// ����RAM����
		CAN_Receive_Message_Configuration(CAN4_SFR,Receice_addr,&CAN_MessageStructure);
		// RAM��ַ����
		Receice_addr+=0x10;
		// �ͷ�һ�μ�����
		CAN_Release_Receive_Buffer(CAN4_SFR,1);

		if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_SFF)//��׼֡
		{
			can4_rx_frame.TargetID = CAN_MessageStructure.m_StandardID;			 //��׼֡ID
		}
		else if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_EFF)//��չ֡
		{
			can4_rx_frame.TargetID = CAN_MessageStructure.m_ExtendedID;			 //��չ֡ID
		}

		if(CAN_MessageStructure.m_RemoteTransmit != CAN_DATA_FRAME)//Զ��֡
		{
			can4_rx_frame. MsgType = CAN_MessageStructure.m_RemoteTransmit;
			can4_rx_frame.data[0] =0xAA;
			//�û�����
			//USART_Send(USART2_SFR,ReceiveData,1);

		}
		else  //����֡
		{
			can4_rx_frame.lenth = CAN_MessageStructure.m_DataLength;			    	 //����
			can4_rx_frame.RmtFrm = CAN_MessageStructure.m_FrameFormat;             	 //֡��ʽ
			can4_rx_frame. MsgType= CAN_MessageStructure.m_RemoteTransmit;		 	 //֡����
			// ��ȡ����
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
	static uint8_t Receice_addr=0x00;//����RAMƫ�Ƶ�ַ
	CAN_MessageTypeDef CAN_MessageStructure;//���ձ��Ľṹ��
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//struct can_frame rx_frame;
	/* ��CAN�жϱ�־ */
	INT_Clear_Interrupt_Flag(INT_CAN5);

	/* �ж��Ƿ�ΪCAN���ͱ�־ */
	if(CAN_Get_INT_Flag(CAN5_SFR,CAN_INT_TRANSMIT) != RESET)
	{
		CAN_Clear_INT_Flag(CAN5_SFR,CAN_INT_TRANSMIT);
		CAN5_SFR->CTLR &= ~0x300;//�������ʹ��λ
	}

	/* �ж����ߴ����־ */
	if(CAN_Get_INT_Flag(CAN5_SFR,CAN_INT_BUS_ERROR) != RESET)
	{
		CAN_Clear_INT_Flag(CAN5_SFR,CAN_INT_BUS_ERROR);
		CAN5_SFR->CTLR &= ~0x300; //���ߴ����ط�
	}

	/* �ж��Ƿ�ΪCAN���ձ�־ */
	if(CAN_Get_INT_Flag(CAN5_SFR,CAN_INT_RECEIVE) != RESET)
	{
//		GPIO_Toggle_Output_Data_Config (GPIOB_SFR,GPIO_PIN_MASK_8);
		/* ����RAM���� */
		CAN_Receive_Message_Configuration(CAN5_SFR,Receice_addr,&CAN_MessageStructure);
		/* RAM��ַ���� */
		Receice_addr+=0x10;
		/* �ͷ�һ�μ����� */
		CAN_Release_Receive_Buffer(CAN5_SFR,1);

		if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_SFF)//��׼֡
		{
			can5_rx_frame.TargetID = CAN_MessageStructure.m_StandardID;			 //��׼֡ID
		}
		else if(CAN_MessageStructure.m_FrameFormat==CAN_FRAME_FORMAT_EFF)//��չ֡
		{
			can5_rx_frame.TargetID = CAN_MessageStructure.m_ExtendedID;			 //��չ֡ID
		}

		if(CAN_MessageStructure.m_RemoteTransmit != CAN_DATA_FRAME)//Զ��֡
		{
			can5_rx_frame. MsgType = CAN_MessageStructure.m_RemoteTransmit;
			can5_rx_frame.data[0] =0xAA;
			//�û�����
			//USART_Send(USART2_SFR,ReceiveData,1);

		}
		else  //����֡
		{
			can5_rx_frame.lenth = CAN_MessageStructure.m_DataLength;			    	 //����
			can5_rx_frame.RmtFrm = CAN_MessageStructure.m_FrameFormat;             	 //֡��ʽ
			can5_rx_frame. MsgType= CAN_MessageStructure.m_RemoteTransmit;		 	 //֡����
			/* ��ȡ���� */
			for(i=0;i<CAN_MessageStructure.m_DataLength;i++)
			{
				can5_rx_frame.data[i] = CAN_MessageStructure.m_Data[i];
			}
			//if(CAN4_RX_COUNT<(CAN_BUFFER_MAX-1))
			//	CAN4_RX_COUNT++;
			//else
			//	CAN4_RX_COUNT=0;
			//can5_receive(can5_rx_frame);
			if((can5_rx_frame.TargetID == 0x700) |
				(can5_rx_frame.TargetID == 0x701) |
				(can5_rx_frame.TargetID == 0x702) |
				(can5_rx_frame.TargetID == 0x703))
			{
				//fprintf(USART1_STREAM,"CAN5_Uradar\r\n");
				Ureader_receive(can5_rx_frame);
			}

		}
	}

}

//*****************************************************************************************
//                              UART1�жϺ���
//*****************************************************************************************	//
uint8_t startflag = 0;
uint8_t stopflag = 1;
char *btStrx = NULL;
void __attribute__((interrupt))_USART1_exception (void)
{
	uint8_t data;

	if(USART_Get_Receive_BUFR_Ready_Flag(USART1_SFR))
	{
		data=USART_ReceiveData(USART1_SFR);
		if(upgrade_p.rx_mode == RECV_BIN){
			upgrade_p.pkg.data_i[upgrade_p.pkg.cnt] = data;
			upgrade_p.pkg.cnt ++;
		}else{
			User_Rxbuffer[User_Rxcount] = data;
			if(User_Rxbuffer[User_Rxcount-1]=='\r' && User_Rxbuffer[User_Rxcount]=='\n'){
				if((btStrx = strstr(User_Rxbuffer,"AT+")) || (btStrx = strstr(User_Rxbuffer,"ZKHYCHK*RALGPARA"))){	// �ֳ�ATָ��
					sprintf(usart1_At_Buf,"%s",btStrx);
					Clear_Usart1_Buffer();
					return ;
				}
				Usart1_UserCmd_Analysis();
				OTA_UserCmd_Analysis_In_USART1IT(User_Rxbuffer);
				GPS_UserCmd_Analysis_In_USART1IT();
			}else
				User_Rxcount++;
		}
#if 0
		uint8_t i = 0;
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
#endif
	}

}

//*****************************************************************************************
//                              �����жϺ���
//*****************************************************************************************	//

void __attribute__((interrupt))_USART4_exception (void)
{
	uint8_t data;
	if(USART_Get_Receive_BUFR_Ready_Flag(USART4_SFR))
	{
		data=USART_ReceiveData(USART4_SFR);
//		USART_Send(USART4_SFR,&data,1);
		HC02_BT_Analysis_Alg_Para(data);
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
//                              Ԥ���������״ﴮ�ڡ�byd�ϱ�����
//*****************************************************************************************	//

void __attribute__((interrupt))_USART2_exception (void)
{
	uint8_t data;
	if(USART_Get_Receive_BUFR_Ready_Flag(USART2_SFR))
	{
		data=USART_ReceiveData(USART2_SFR);
		Data_Uploading_In_KFIT(data);			// �����ϴ�(��ϲſ�GPS 4Gģ��) add lmz 20211012
	}
}
//*****************************************************************************************
//                              UART0�жϺ���
//*****************************************************************************************	//
void __attribute__((interrupt))_USART0_exception (void)
{
	uint8_t data;
	if(USART_Get_Receive_BUFR_Ready_Flag(USART0_SFR))
	{
		data = USART_ReceiveData(USART0_SFR);
		if(upgrade_p.rx_mode == RECV_BIN){
			OTA_Upgrade_In_USART0IT(data);
		}else{
			EC200U_Rxbuffer[EC200U_Rxcount] = data;

			if(EC200U_Rxbuffer[EC200U_Rxcount]=='\n'){	//  && EC200U_Rxbuffer[EC200U_Rxcount-1]=='\n'
				upgrade_p.recv_ok = SUCCESS;
				// ���Դ�ӡ
//				if(upgrade_p.isFinish == TRUE){
//					if(strstr((const char*)EC200U_Rxbuffer,(const char*)"AT+"));
//					else fprintf(USART1_STREAM,">>%s\r\n",EC200U_Rxbuffer);
//				}
//				fprintf(USART1_STREAM,">>%s\r\n",EC200U_Rxbuffer);
				if(server_p.stage==3 && strstr((const char*)EC200U_Rxbuffer,(const char*)"CONNECT")){
					upgrade_p.rx_mode 	= RECV_BIN;
					server_p._4G_status = OTA_WAIT;
				}
			}else
				EC200U_Rxcount++;
		}
	}
}

void __attribute__((interrupt))_T14_exception (void)
{

	BTIM_Clear_Updata_INT_Flag(T14_SFR);										//�����ʱ���־λ
	BTIM_Clear_Overflow_INT_Flag (T14_SFR);										//��T14����жϱ�־λ

	Time14_CNT++;
	//if(Time14_CNT>=10)//100ms����һ�η�ת
	//{
		//Time14_CNT =0;
		//GPIO_Toggle_Output_Data_Config(GPIOB_SFR,GPIO_PIN_MASK_13);               //PB13��ת LED0��˸
	//}
	if(GPS_Step == 0)
	{
		//EC200U_SendData("AT+QGPS?\r\n",10);
		//gps_oldtime = 0;
	}

}

//*****************************************************************************************
//                              T15�жϺ���
//*****************************************************************************************	//
uint32_t PLUSE_WIDTH=0;//��׽��PWM���ڱ���
void __attribute__((interrupt))_T15_exception (void)
{

	BTIM_Clear_Updata_INT_Flag(T15_SFR);									 //�����ʱ���־λ
	BTIM_Clear_Overflow_INT_Flag (T15_SFR);									//��T15����жϱ�־λ

	Time15_CNT++;
	if(Time15_CNT>=10)  //100ms����һ
	{
		Time15_CNT =0;
	}
}

//*****************************************************************************************
//                              SysTick�жϺ���
//*****************************************************************************************
void __attribute__((interrupt)) _SysTick_exception (void)
{
	static uint32_t send_time = 0;
	struct can_frame tx_frame_valve;
	//���Ķ�ʱ�����ж�������������־λ
	SystemtimeClock ++;

	OTA_In_SysClick_IT(SystemtimeClock);

	if(stSpeedPara.uSingnalType == 1)
	{
		if((0 == speed_level) && (1 == GPIO_Read_Input_Data_Bit (GPIOC_SFR, GPIO_PIN_MASK_4)))
		{
			first_time = SystemtimeClock - second_time;
			second_time = SystemtimeClock;
		}
		speed_level = GPIO_Read_Input_Data_Bit (GPIOC_SFR, GPIO_PIN_MASK_4);
	}
	if(stSysPara.ucControlMode == 2)	//BYD������Ҫ����AEB��������20msһ��
	{
		if((SystemtimeClock - send_time) > 20)
		{
			Valve_AEBdecActive_Get(&tx_frame_valve);
			tx_frame_valve.TargetID = 0x1C01F025;
			CAN_Transmit_DATA(CAN2_SFR,tx_frame_valve);
		}
	}

}
