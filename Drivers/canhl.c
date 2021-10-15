/**
  ******************************************************************************
  * �ļ��� canhl.c
  * ��  ��  ChipON_AE/FAE_Group
  * ��  ��  2019-10-19
  * ��  ��  ���ļ��ṩ��CANģ����ص������뷢�͹��ܣ�����
  *          + CAN������ӳ��
  *          + CAN�ж�����
  *          + CAN����
  *          + CAN��ʼ��
  ******************************************************************************/
#include "system_init.h"
#include "canhl.h"

#define CAN_STB_HIG()        GPIO_Set_Output_Data_Bits(GPIOD_SFR,GPIO_PIN_MASK_10,Bit_SET)  //����Ϊ�ߵ�ƽ
#define CAN_STB_LOW()        GPIO_Set_Output_Data_Bits(GPIOD_SFR,GPIO_PIN_MASK_10,Bit_RESET)//����Ϊ�͵�ƽ

QueueHandle_t xQueue_can0 = NULL;
struct can_frame can0_rx_frame;
uint8_t CAN0_RX_COUNT=0;

QueueHandle_t xQueue_can1 = NULL;
struct can_frame can1_rx_frame;
uint8_t CAN1_RX_COUNT=0;

QueueHandle_t xQueue_can2 = NULL;
struct can_frame can2_rx_frame;
uint8_t CAN2_RX_COUNT=0;

QueueHandle_t xQueue_can3 = NULL;
struct can_frame can3_rx_frame;
uint8_t CAN3_RX_COUNT=0;

QueueHandle_t xQueue_can4 = NULL;
struct can_frame can4_rx_frame;
uint8_t CAN4_RX_COUNT=0;

QueueHandle_t xQueue_can5 = NULL;
struct can_frame can5_rx_frame;
uint8_t CAN5_RX_COUNT=0;

/*============================================================================
 *void xGPIO_CAN()
------------------------------------------------------------------------------
����		: can�����ض���
����		: ��
���		: ��
��ע		:
============================================================================*/
void xGPIO_CAN1()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*��ʼ����λGPIOH,ʹ��GPIOH����ʱ��*/
	//GPIO_Reset(GPIOD_SFR);
    //GPIO_Write_Mode_Bits(GPIOD_SFR,GPIO_PIN_MASK_9,GPIO_MODE_OUT);
    //CAN_STB_LOW();//CAN_STB ��ʼ��Ϊ��

	/*����CAN��������Ϊ����ģʽ��������Ӧ�˿�ʱ��*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_4|GPIO_PIN_MASK_5;
	GPIO_Configuration(GPIOA_SFR,&GPIO_InitStructure);
	/*��������ӳ�书��ΪCANģʽ*/
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOACLKEN,TRUE);
	//PCLK_CTL2_Peripheral_Clock_Enable(PCLK_CTL2_CAN1CLKEN,TRUE);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_4,GPIO_RMP_AF9_CAN1);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_5,GPIO_RMP_AF9_CAN1);

}

void xGPIO_CAN0()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*��ʼ����λGPIOH,ʹ��GPIOH����ʱ��*/
	//GPIO_Reset(GPIOD_SFR);
    //GPIO_Write_Mode_Bits(GPIOD_SFR,GPIO_PIN_MASK_9,GPIO_MODE_OUT);
    //CAN_STB_LOW();//CAN_STB ��ʼ��Ϊ��

	/*����CAN��������Ϊ����ģʽ��������Ӧ�˿�ʱ��*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_2|GPIO_PIN_MASK_3;
	GPIO_Configuration(GPIOA_SFR,&GPIO_InitStructure);
	/*��������ӳ�书��ΪCANģʽ*/
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOACLKEN,TRUE);
	//PCLK_CTL2_Peripheral_Clock_Enable(PCLK_CTL2_CAN0CLKEN,TRUE);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_2,GPIO_RMP_AF9_CAN0);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_3,GPIO_RMP_AF9_CAN0);
}

void xGPIO_CAN2()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*��ʼ����λGPIOH,ʹ��GPIOH����ʱ��*/
	//GPIO_Reset(GPIOD_SFR);
    //GPIO_Write_Mode_Bits(GPIOD_SFR,GPIO_PIN_MASK_9,GPIO_MODE_OUT);
    //CAN_STB_LOW();//CAN_STB ��ʼ��Ϊ��

	/*����CAN��������Ϊ����ģʽ��������Ӧ�˿�ʱ��*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_11|GPIO_PIN_MASK_12;
	GPIO_Configuration(GPIOA_SFR,&GPIO_InitStructure);
	/*��������ӳ�书��ΪCANģʽ*/
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOACLKEN,TRUE);
	//PCLK_CTL2_Peripheral_Clock_Enable(PCLK_CTL2_CAN2CLKEN,TRUE);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_11,GPIO_RMP_AF9_CAN2);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_12,GPIO_RMP_AF9_CAN2);
}

void xGPIO_CAN3()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*��ʼ����λGPIOH,ʹ��GPIOH����ʱ��*/
	//GPIO_Reset(GPIOD_SFR);
    //GPIO_Write_Mode_Bits(GPIOD_SFR,GPIO_PIN_MASK_9,GPIO_MODE_OUT);
    //CAN_STB_LOW();//CAN_STB ��ʼ��Ϊ��

	/*����CAN��������Ϊ����ģʽ��������Ӧ�˿�ʱ��*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_1|GPIO_PIN_MASK_2;
	GPIO_Configuration(GPIOE_SFR,&GPIO_InitStructure);
	/*��������ӳ�书��ΪCANģʽ*/
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOECLKEN,TRUE);
	//PCLK_CTL2_Peripheral_Clock_Enable(PCLK_CTL2_CAN3CLKEN,TRUE);
	GPIO_Pin_RMP_Config(GPIOE_SFR,GPIO_Pin_Num_1,GPIO_RMP_AF9_CAN3);
	GPIO_Pin_RMP_Config(GPIOE_SFR,GPIO_Pin_Num_2,GPIO_RMP_AF9_CAN3);
}

void xGPIO_CAN4()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*��ʼ����λGPIOH,ʹ��GPIOH����ʱ��*/
	//GPIO_Reset(GPIOD_SFR);
    //GPIO_Write_Mode_Bits(GPIOD_SFR,GPIO_PIN_MASK_9,GPIO_MODE_OUT);
    //CAN_STB_LOW();//CAN_STB ��ʼ��Ϊ��

	/*����CAN��������Ϊ����ģʽ��������Ӧ�˿�ʱ��*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_5|GPIO_PIN_MASK_6;
	GPIO_Configuration(GPIOD_SFR,&GPIO_InitStructure);
	/*��������ӳ�书��ΪCANģʽ*/
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIODCLKEN,TRUE);
	//PCLK_CTL3_Peripheral_Clock_Enable(PCLK_CTL3_CAN4CLKEN,TRUE);
	GPIO_Pin_RMP_Config(GPIOD_SFR,GPIO_Pin_Num_5,GPIO_RMP_AF9_CAN4);
	GPIO_Pin_RMP_Config(GPIOD_SFR,GPIO_Pin_Num_6,GPIO_RMP_AF9_CAN4);
}

void xGPIO_CAN5()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*��ʼ����λGPIOH,ʹ��GPIOH����ʱ��*/
	//GPIO_Reset(GPIOD_SFR);
    //GPIO_Write_Mode_Bits(GPIOD_SFR,GPIO_PIN_MASK_9,GPIO_MODE_OUT);
    //CAN_STB_LOW();//CAN_STB ��ʼ��Ϊ��

	/*����CAN��������Ϊ����ģʽ��������Ӧ�˿�ʱ��*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	//GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_13|GPIO_PIN_MASK_14;
	//GPIO_Configuration(GPIOB_SFR,&GPIO_InitStructure);
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_2|GPIO_PIN_MASK_3;
	GPIO_Configuration(GPIOF_SFR,&GPIO_InitStructure);

	/*��������ӳ�书��ΪCANģʽ*/
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOFCLKEN,TRUE);
	//PCLK_CTL3_Peripheral_Clock_Enable(PCLK_CTL3_CAN5CLKEN,TRUE);
	//GPIO_Pin_RMP_Config(GPIOB_SFR,GPIO_Pin_Num_13,GPIO_RMP_AF9_CAN5);
	//GPIO_Pin_RMP_Config(GPIOB_SFR,GPIO_Pin_Num_14,GPIO_RMP_AF9_CAN5);
	GPIO_Pin_RMP_Config(GPIOF_SFR,GPIO_Pin_Num_2,GPIO_RMP_AF9_CAN5);
	GPIO_Pin_RMP_Config(GPIOF_SFR,GPIO_Pin_Num_3,GPIO_RMP_AF9_CAN5);

}



/*============================================================================
 *void xInit_CAN(CAN_SFRmap* CANx,uint8_t Bdrt,uint32_t MODE)
------------------------------------------------------------------------------
����		: canģ���ʼ��
����		: 1.CANx: ָ��CAN�ڴ�ṹ��ָ�룬ȡֵΪCAN0_SFR~CAN3_SFR
		  2.Bdrt  �����ʣ���canhl.h����
		  3.MODE  ����ģʽ��CAN_MODE_NORMAL CAN_MODE_SILENT CAN_MODE_LOOPBACK CAN_MODE_SILENT_LOOPBACK
���		: ��
��ע		:
============================================================================*/
void xInit_CAN(CAN_SFRmap* CANx,uint8_t Bdrt,uint32_t MODE,struct CAN_Filter *Filter)
{
	CAN_InitTypeDef CAN_InitStructure;
	/* CANʱ��ʹ�ܣ���λ���� */
	uint32_t ACRR,MSKR;
	CAN_Reset(CANx);
	if(Filter==NULL)
	{
		ACRR=0x00000000;
		MSKR=0xffffffff;
	}
	else
		if(Filter->FrameFormat==CAN_FRAME_FORMAT_SFF)
		{
			ACRR=Filter->ID<<21;
			MSKR=~(Filter->Mask<<21);
		}
		else
		{
			ACRR=Filter->ID<<3;
			MSKR=~(Filter->Mask<<3);
		}

	CAN_InitStructure.m_Acceptance = ACRR;                    //�����˲�
	CAN_InitStructure.m_AcceptanceMask = MSKR;		          //��������
	CAN_InitStructure.m_WorkSource = CAN_SOURCE_HFCLK_DIV_2;      //CANʱ���ڲ���Ƶ

	if(Bdrt==CAN_BAUDRATE_125K)
	{
		CAN_InitStructure.m_BaudRate = 7;					  //ʱ�ӷ�Ƶ  1M
	}
	else if(Bdrt==CAN_BAUDRATE_250K)
	{
		CAN_InitStructure.m_BaudRate = 3;					  //ʱ�ӷ�Ƶ  2M
	}
	else if(Bdrt==CAN_BAUDRATE_500K)
	{
		CAN_InitStructure.m_BaudRate = 1;					  //ʱ�ӷ�Ƶ  4M
	}
	else if(Bdrt==CAN_BAUDRATE_1M)
	{
		CAN_InitStructure.m_BaudRate = 0;					  //ʱ�ӷ�Ƶ 8M
	}
	//TSEG1������TSEG2���ñ�ֵһ��Ϊ70-80%���CAN�Ĳ���Ҫ��
	CAN_InitStructure.m_TimeSeg1 = 4;						   //TSEG1
	CAN_InitStructure.m_TimeSeg2 = 1;						   //TSEG2
	CAN_InitStructure.m_BusSample = CAN_BUS_SAMPLE_3_TIMES;	   //���������
	CAN_InitStructure.m_SyncJumpWidth = 1;					   //ͬ����ת���
	CAN_InitStructure.m_Enable = TRUE;						   //ʹ��
	CAN_InitStructure.m_Mode = MODE;				           //ģʽ
	CAN_Configuration(CANx,&CAN_InitStructure);			   //д������

}
/*============================================================================
 *void xINT_CAN(CAN_SFRmap* CANx)
------------------------------------------------------------------------------
����		: can�ж�����
����		: 1.CANx: ָ��CAN�ڴ�ṹ��ָ�룬ȡֵΪCAN0_SFR~CAN3_SFR
���		: ��
��ע		:
============================================================================*/
void xINT_CAN(CAN_SFRmap* CANx)
{

	/* ���������ж� */
	CAN_Set_INT_Enable(CANx,CAN_INT_TRANSMIT,TRUE);
	/* ���������ж� */
	CAN_Set_INT_Enable(CANx,CAN_INT_RECEIVE,TRUE);
	/* ����������ж� */
//	CAN_Set_INT_Enable(CANx,CAN_INT_DATA_OVERFLOW,TRUE);
	/* CAN�ж����� */

	if(CANx=CAN1_SFR)
	{
		INT_Interrupt_Priority_Config(INT_CAN1,4,1);						//CAN��ռ���ȼ�4,�����ȼ�0
		INT_Clear_Interrupt_Flag(INT_CAN1);									//CAN���жϱ�־
		INT_Interrupt_Enable(INT_CAN1,TRUE);								//CAN�ж�ʹ��
	}
	if(CANx=CAN0_SFR)
	{
		INT_Interrupt_Priority_Config(INT_CAN0,3,3);						//CAN��ռ���ȼ�4,�����ȼ�0
		INT_Clear_Interrupt_Flag(INT_CAN0);									//CAN���жϱ�־
		INT_Interrupt_Enable(INT_CAN0,TRUE);								//CAN�ж�ʹ��
	}
	if(CANx=CAN2_SFR)
	{
		INT_Interrupt_Priority_Config(INT_CAN2,4,2);						//CAN��ռ���ȼ�4,�����ȼ�0
		INT_Clear_Interrupt_Flag(INT_CAN2);									//CAN���жϱ�־
		INT_Interrupt_Enable(INT_CAN2,TRUE);								//CAN�ж�ʹ��
	}
	if(CANx=CAN3_SFR)
	{
		INT_Interrupt_Priority_Config(INT_CAN3,4,4);						//CAN��ռ���ȼ�4,�����ȼ�0
		INT_Clear_Interrupt_Flag(INT_CAN3);									//CAN���жϱ�־
		INT_Interrupt_Enable(INT_CAN3,TRUE);								//CAN�ж�ʹ��
	}
	if(CANx=CAN4_SFR)
	{
		INT_Interrupt_Priority_Config(INT_CAN4,3,1);						//CAN��ռ���ȼ�4,�����ȼ�0
		INT_Clear_Interrupt_Flag(INT_CAN4);									//CAN���жϱ�־
		INT_Interrupt_Enable(INT_CAN4,TRUE);								//CAN�ж�ʹ��
	}
	if(CANx=CAN5_SFR)
	{
		INT_Interrupt_Priority_Config(INT_CAN5,3,2);						//CAN��ռ���ȼ�4,�����ȼ�0
		INT_Clear_Interrupt_Flag(INT_CAN5);									//CAN���жϱ�־
		INT_Interrupt_Enable(INT_CAN5,TRUE);								//CAN�ж�ʹ��
	}

	INT_All_Enable (TRUE);
}

/*============================================================================
 *void CAN_Transmit_DATA(CAN_SFRmap* CANx, uint8_t *data , uint8_t lenth)
------------------------------------------------------------------------------
����		: ���ݷ��ͼ���
����		: 1.Ҫ��ʼ����CAN���	CAN0_SFR��CAN1_SFR��CAN2_SFR
		  2.ID
		  2.�������������ַ
		  3.�������ݳ���
		  4.֡����        CAN_DATA_FRAME����֡                     CAN_REMOTE_FRAMEԶ��֡
		  5.֡��ʽ        CAN_FRAME_FORMAT_SFF��׼֡     CAN_FRAME_FORMAT_EFF��չ֡
���		: CAN_ERROR_BUFFERFULL  CAN_ERROR_NOERROR
��ע		:
============================================================================*/
CAN_ErrorT CAN_Transmit_DATA(CAN_SFRmap* CANx, struct can_frame tx_frame)
{
	CAN_ErrorT x;
	uint8_t i;
	CAN_MessageTypeDef	CAN_MessageStructure;

	CAN_MessageStructure.m_FrameFormat = tx_frame.RmtFrm;             	 //֡��ʽ
	CAN_MessageStructure.m_RemoteTransmit = tx_frame.MsgType;		 	 //֡����
	if(tx_frame.RmtFrm==CAN_FRAME_FORMAT_SFF)//��׼֡
	{
		CAN_MessageStructure.m_StandardID = tx_frame.TargetID;			 //��׼֡ID
		CAN_MessageStructure.m_ExtendedID = 0;			         //��չ֡ID
	}
	else if(tx_frame.RmtFrm==CAN_FRAME_FORMAT_EFF)//��չ֡
	{
		CAN_MessageStructure.m_StandardID = 0;					 //��׼֡ID
		CAN_MessageStructure.m_ExtendedID = tx_frame.TargetID;			 //��չ֡ID
	}
	if(tx_frame.lenth>8)
	{
		tx_frame.lenth=8;
	}
	CAN_MessageStructure.m_DataLength =tx_frame.lenth;			    	 //����
	for(i=0;i<tx_frame.lenth;i++)
	{
		CAN_MessageStructure.m_Data[i] = tx_frame.data[i];				 //����
	}
	/* ���ͻ������� */
//	if((!CAN_Get_Transmit_Status(CANx,CAN_TX_BUFFER_STATUS)))
//	{
//		x=CAN_ERROR_BUFFERFULL;
//		return x;
//	}
	int count = 0;
	while(count < 100)
	{
		if(CAN_Get_Transmit_Status(CANx,CAN_TX_BUFFER_STATUS))
		{
			break;
		}
		count ++;
	}
	if(count == 100)
	{
		x=CAN_ERROR_BUFFERFULL;
		return x;
	}

	/* ת�����ݵ����ͻ����� */
	CAN_Transmit_Message_Configuration(CANx,&CAN_MessageStructure);
	/* �ظ�����ģʽ */
	//CAN_Transmit_Repeat(CANx);
	CANx->CTLR = CANx->CTLR | 0x300;

	x=CAN_ERROR_NOERROR;
	return x;
}

CAN_ErrorT CAN_Receive_DATA(CAN_SFRmap* CANx, struct can_frame *rx_frame)
{
	QueueHandle_t xQueue_can;
	if(CANx==CAN1_SFR)
		xQueue_can=xQueue_can1;
	else
		xQueue_can=NULL;

	if(xQueue_can==NULL)
		return CAN_ERROR_NOINIT;
	else if(xQueueReceive(xQueue_can,(void *)rx_frame,1 ) == pdPASS)
		return CAN_ERROR_NOERROR;
	else
		return CAN_ERROR_EMPTY;

}

/********************************************
 * ��������CAN0_INIT
 * �������ܣ�can0��ʼ��
 * �������룺֡�ʣ�֡����
 * �����������
********************************************/
void CAN0_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter)
{
	xGPIO_CAN0();//CAN1������ӳ��
	xInit_CAN(CAN0_SFR,BaudRate,CAN_MODE_NORMAL,Filter);//CANģ���ʼ����500K,����ģʽ
	xINT_CAN(CAN0_SFR);//ʹ��CAN�����ж�
	//xQueue_can0 = xQueueCreate( 10, sizeof( struct can_frame ) );
}
/********************************************
 * ��������CAN1_INIT
 * �������ܣ�can1��ʼ��
 * �������룺֡�ʣ�֡����
 * �����������
********************************************/
void CAN1_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter)
{
	xGPIO_CAN1();//CAN1������ӳ��
	xInit_CAN(CAN1_SFR,BaudRate,CAN_MODE_NORMAL,Filter);//CANģ���ʼ����500K,����ģʽ
	xINT_CAN(CAN1_SFR);//ʹ��CAN�����ж�
	//xQueue_can1 = xQueueCreate( 10, sizeof( struct can_frame ) );
}
/********************************************
 * ��������CAN2_INIT
 * �������ܣ�can2��ʼ��
 * �������룺֡�ʣ�֡����
 * �����������
********************************************/
void CAN2_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter)
{
	xGPIO_CAN2();//CAN1������ӳ��
	xInit_CAN(CAN2_SFR,BaudRate,CAN_MODE_NORMAL,Filter);//CANģ���ʼ����500K,����ģʽ
	xINT_CAN(CAN2_SFR);//ʹ��CAN�����ж�
	//xQueue_can2 = xQueueCreate( 10, sizeof( struct can_frame ) );
}
/********************************************
 * ��������CAN3_INIT
 * �������ܣ�can3��ʼ��
 * �������룺֡�ʣ�֡����
 * �����������
********************************************/
void CAN3_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter)
{
	xGPIO_CAN3();//CAN1������ӳ��
	xInit_CAN(CAN3_SFR,BaudRate,CAN_MODE_NORMAL,Filter);//CANģ���ʼ����500K,����ģʽ
	xINT_CAN(CAN3_SFR);//ʹ��CAN�����ж�
	//xQueue_can3 = xQueueCreate( 10, sizeof( struct can_frame ) );
}
/********************************************
 * ��������CAN1_INIT
 * �������ܣ�can1��ʼ��
 * �������룺֡�ʣ�֡����
 * �����������
********************************************/
void CAN4_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter)
{
	xGPIO_CAN4();//CAN1������ӳ��
	xInit_CAN(CAN4_SFR,BaudRate,CAN_MODE_NORMAL,Filter);//CANģ���ʼ����500K,����ģʽ
	xINT_CAN(CAN4_SFR);//ʹ��CAN�����ж�
	//xQueue_can4 = xQueueCreate( 10, sizeof( struct can_frame ) );
}

void CAN5_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter)
{
	xGPIO_CAN5();//CAN1������ӳ��
	xInit_CAN(CAN5_SFR,BaudRate,CAN_MODE_NORMAL,Filter);//CANģ���ʼ����500K,����ģʽ
	xINT_CAN(CAN5_SFR);//ʹ��CAN�����ж�
	//xQueue_can4 = xQueueCreate( 10, sizeof( struct can_frame ) );
}

void CAN_Filter_Config(CAN_SFRmap* CANx,uint32_t ID,uint32_t Mask,uint32_t FrameFormat)
{
	uint32_t ACRR,MSKR;

	if(FrameFormat==CAN_FRAME_FORMAT_SFF)
	{
		ACRR=ID<<21;
		MSKR=~(Mask<<21);
	}
	else if(FrameFormat==CAN_FRAME_FORMAT_EFF)
	{
		ACRR=ID<<3;
		MSKR=~(Mask<<3);
	}
	else
	{
		ACRR=0x00000000;
		MSKR=0xffffffff;
	}
	CAN_Acceptance_Config(CANx,ACRR);
	CAN_Acceptance_Mask_Config(CANx,MSKR);
}

void CAN_check_busoff(CAN_SFRmap* CANx)
{
	if(CANx->CTLR &(1<<23))//Boff��λ
	{
		CANx->CTLR &= (~0x01);//�����λģʽ��
	}
}
