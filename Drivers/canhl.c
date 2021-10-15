/**
  ******************************************************************************
  * 文件名 canhl.c
  * 作  者  ChipON_AE/FAE_Group
  * 日  期  2019-10-19
  * 描  述  该文件提供了CAN模块相关的配置与发送功能，包括
  *          + CAN引脚重映射
  *          + CAN中断配置
  *          + CAN发送
  *          + CAN初始化
  ******************************************************************************/
#include "system_init.h"
#include "canhl.h"

#define CAN_STB_HIG()        GPIO_Set_Output_Data_Bits(GPIOD_SFR,GPIO_PIN_MASK_10,Bit_SET)  //设置为高电平
#define CAN_STB_LOW()        GPIO_Set_Output_Data_Bits(GPIOD_SFR,GPIO_PIN_MASK_10,Bit_RESET)//设置为低电平

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
描述		: can引脚重定义
输入		: 无
输出		: 无
备注		:
============================================================================*/
void xGPIO_CAN1()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*初始化复位GPIOH,使能GPIOH外设时钟*/
	//GPIO_Reset(GPIOD_SFR);
    //GPIO_Write_Mode_Bits(GPIOD_SFR,GPIO_PIN_MASK_9,GPIO_MODE_OUT);
    //CAN_STB_LOW();//CAN_STB 初始化为低

	/*配置CAN引脚类型为复用模式，开启对应端口时钟*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_4|GPIO_PIN_MASK_5;
	GPIO_Configuration(GPIOA_SFR,&GPIO_InitStructure);
	/*配置引脚映射功能为CAN模式*/
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOACLKEN,TRUE);
	//PCLK_CTL2_Peripheral_Clock_Enable(PCLK_CTL2_CAN1CLKEN,TRUE);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_4,GPIO_RMP_AF9_CAN1);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_5,GPIO_RMP_AF9_CAN1);

}

void xGPIO_CAN0()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*初始化复位GPIOH,使能GPIOH外设时钟*/
	//GPIO_Reset(GPIOD_SFR);
    //GPIO_Write_Mode_Bits(GPIOD_SFR,GPIO_PIN_MASK_9,GPIO_MODE_OUT);
    //CAN_STB_LOW();//CAN_STB 初始化为低

	/*配置CAN引脚类型为复用模式，开启对应端口时钟*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_2|GPIO_PIN_MASK_3;
	GPIO_Configuration(GPIOA_SFR,&GPIO_InitStructure);
	/*配置引脚映射功能为CAN模式*/
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOACLKEN,TRUE);
	//PCLK_CTL2_Peripheral_Clock_Enable(PCLK_CTL2_CAN0CLKEN,TRUE);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_2,GPIO_RMP_AF9_CAN0);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_3,GPIO_RMP_AF9_CAN0);
}

void xGPIO_CAN2()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*初始化复位GPIOH,使能GPIOH外设时钟*/
	//GPIO_Reset(GPIOD_SFR);
    //GPIO_Write_Mode_Bits(GPIOD_SFR,GPIO_PIN_MASK_9,GPIO_MODE_OUT);
    //CAN_STB_LOW();//CAN_STB 初始化为低

	/*配置CAN引脚类型为复用模式，开启对应端口时钟*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_11|GPIO_PIN_MASK_12;
	GPIO_Configuration(GPIOA_SFR,&GPIO_InitStructure);
	/*配置引脚映射功能为CAN模式*/
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOACLKEN,TRUE);
	//PCLK_CTL2_Peripheral_Clock_Enable(PCLK_CTL2_CAN2CLKEN,TRUE);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_11,GPIO_RMP_AF9_CAN2);
	GPIO_Pin_RMP_Config(GPIOA_SFR,GPIO_Pin_Num_12,GPIO_RMP_AF9_CAN2);
}

void xGPIO_CAN3()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*初始化复位GPIOH,使能GPIOH外设时钟*/
	//GPIO_Reset(GPIOD_SFR);
    //GPIO_Write_Mode_Bits(GPIOD_SFR,GPIO_PIN_MASK_9,GPIO_MODE_OUT);
    //CAN_STB_LOW();//CAN_STB 初始化为低

	/*配置CAN引脚类型为复用模式，开启对应端口时钟*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_1|GPIO_PIN_MASK_2;
	GPIO_Configuration(GPIOE_SFR,&GPIO_InitStructure);
	/*配置引脚映射功能为CAN模式*/
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIOECLKEN,TRUE);
	//PCLK_CTL2_Peripheral_Clock_Enable(PCLK_CTL2_CAN3CLKEN,TRUE);
	GPIO_Pin_RMP_Config(GPIOE_SFR,GPIO_Pin_Num_1,GPIO_RMP_AF9_CAN3);
	GPIO_Pin_RMP_Config(GPIOE_SFR,GPIO_Pin_Num_2,GPIO_RMP_AF9_CAN3);
}

void xGPIO_CAN4()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*初始化复位GPIOH,使能GPIOH外设时钟*/
	//GPIO_Reset(GPIOD_SFR);
    //GPIO_Write_Mode_Bits(GPIOD_SFR,GPIO_PIN_MASK_9,GPIO_MODE_OUT);
    //CAN_STB_LOW();//CAN_STB 初始化为低

	/*配置CAN引脚类型为复用模式，开启对应端口时钟*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_5|GPIO_PIN_MASK_6;
	GPIO_Configuration(GPIOD_SFR,&GPIO_InitStructure);
	/*配置引脚映射功能为CAN模式*/
	PCLK_CTL0_Peripheral_Clock_Enable(PCLK_CTL0_GPIODCLKEN,TRUE);
	//PCLK_CTL3_Peripheral_Clock_Enable(PCLK_CTL3_CAN4CLKEN,TRUE);
	GPIO_Pin_RMP_Config(GPIOD_SFR,GPIO_Pin_Num_5,GPIO_RMP_AF9_CAN4);
	GPIO_Pin_RMP_Config(GPIOD_SFR,GPIO_Pin_Num_6,GPIO_RMP_AF9_CAN4);
}

void xGPIO_CAN5()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*初始化复位GPIOH,使能GPIOH外设时钟*/
	//GPIO_Reset(GPIOD_SFR);
    //GPIO_Write_Mode_Bits(GPIOD_SFR,GPIO_PIN_MASK_9,GPIO_MODE_OUT);
    //CAN_STB_LOW();//CAN_STB 初始化为低

	/*配置CAN引脚类型为复用模式，开启对应端口时钟*/
	GPIO_InitStructure.m_Mode = GPIO_MODE_RMP;
	GPIO_InitStructure.m_Speed = GPIO_HIGH_SPEED;
	GPIO_InitStructure.m_PullDown = GPIO_NOPULL;
	GPIO_InitStructure.m_PullUp = GPIO_NOPULL;
	GPIO_InitStructure.m_OpenDrain = GPIO_POD_PP;
	//GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_13|GPIO_PIN_MASK_14;
	//GPIO_Configuration(GPIOB_SFR,&GPIO_InitStructure);
	GPIO_InitStructure.m_Pin = GPIO_PIN_MASK_2|GPIO_PIN_MASK_3;
	GPIO_Configuration(GPIOF_SFR,&GPIO_InitStructure);

	/*配置引脚映射功能为CAN模式*/
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
描述		: can模块初始化
输入		: 1.CANx: 指向CAN内存结构的指针，取值为CAN0_SFR~CAN3_SFR
		  2.Bdrt  波特率，见canhl.h定义
		  3.MODE  工作模式：CAN_MODE_NORMAL CAN_MODE_SILENT CAN_MODE_LOOPBACK CAN_MODE_SILENT_LOOPBACK
输出		: 无
备注		:
============================================================================*/
void xInit_CAN(CAN_SFRmap* CANx,uint8_t Bdrt,uint32_t MODE,struct CAN_Filter *Filter)
{
	CAN_InitTypeDef CAN_InitStructure;
	/* CAN时钟使能，复位禁能 */
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

	CAN_InitStructure.m_Acceptance = ACRR;                    //验收滤波
	CAN_InitStructure.m_AcceptanceMask = MSKR;		          //验收屏蔽
	CAN_InitStructure.m_WorkSource = CAN_SOURCE_HFCLK_DIV_2;      //CAN时钟内部高频

	if(Bdrt==CAN_BAUDRATE_125K)
	{
		CAN_InitStructure.m_BaudRate = 7;					  //时钟分频  1M
	}
	else if(Bdrt==CAN_BAUDRATE_250K)
	{
		CAN_InitStructure.m_BaudRate = 3;					  //时钟分频  2M
	}
	else if(Bdrt==CAN_BAUDRATE_500K)
	{
		CAN_InitStructure.m_BaudRate = 1;					  //时钟分频  4M
	}
	else if(Bdrt==CAN_BAUDRATE_1M)
	{
		CAN_InitStructure.m_BaudRate = 0;					  //时钟分频 8M
	}
	//TSEG1设置与TSEG2设置比值一般为70-80%配合CAN的采样要求
	CAN_InitStructure.m_TimeSeg1 = 4;						   //TSEG1
	CAN_InitStructure.m_TimeSeg2 = 1;						   //TSEG2
	CAN_InitStructure.m_BusSample = CAN_BUS_SAMPLE_3_TIMES;	   //采样点次数
	CAN_InitStructure.m_SyncJumpWidth = 1;					   //同步跳转宽度
	CAN_InitStructure.m_Enable = TRUE;						   //使能
	CAN_InitStructure.m_Mode = MODE;				           //模式
	CAN_Configuration(CANx,&CAN_InitStructure);			   //写入配置

}
/*============================================================================
 *void xINT_CAN(CAN_SFRmap* CANx)
------------------------------------------------------------------------------
描述		: can中断配置
输入		: 1.CANx: 指向CAN内存结构的指针，取值为CAN0_SFR~CAN3_SFR
输出		: 无
备注		:
============================================================================*/
void xINT_CAN(CAN_SFRmap* CANx)
{

	/* 开启发送中断 */
	CAN_Set_INT_Enable(CANx,CAN_INT_TRANSMIT,TRUE);
	/* 开启接收中断 */
	CAN_Set_INT_Enable(CANx,CAN_INT_RECEIVE,TRUE);
	/* 开启包溢出中断 */
//	CAN_Set_INT_Enable(CANx,CAN_INT_DATA_OVERFLOW,TRUE);
	/* CAN中断配置 */

	if(CANx=CAN1_SFR)
	{
		INT_Interrupt_Priority_Config(INT_CAN1,4,1);						//CAN抢占优先级4,子优先级0
		INT_Clear_Interrupt_Flag(INT_CAN1);									//CAN清中断标志
		INT_Interrupt_Enable(INT_CAN1,TRUE);								//CAN中断使能
	}
	if(CANx=CAN0_SFR)
	{
		INT_Interrupt_Priority_Config(INT_CAN0,3,3);						//CAN抢占优先级4,子优先级0
		INT_Clear_Interrupt_Flag(INT_CAN0);									//CAN清中断标志
		INT_Interrupt_Enable(INT_CAN0,TRUE);								//CAN中断使能
	}
	if(CANx=CAN2_SFR)
	{
		INT_Interrupt_Priority_Config(INT_CAN2,4,2);						//CAN抢占优先级4,子优先级0
		INT_Clear_Interrupt_Flag(INT_CAN2);									//CAN清中断标志
		INT_Interrupt_Enable(INT_CAN2,TRUE);								//CAN中断使能
	}
	if(CANx=CAN3_SFR)
	{
		INT_Interrupt_Priority_Config(INT_CAN3,4,4);						//CAN抢占优先级4,子优先级0
		INT_Clear_Interrupt_Flag(INT_CAN3);									//CAN清中断标志
		INT_Interrupt_Enable(INT_CAN3,TRUE);								//CAN中断使能
	}
	if(CANx=CAN4_SFR)
	{
		INT_Interrupt_Priority_Config(INT_CAN4,3,1);						//CAN抢占优先级4,子优先级0
		INT_Clear_Interrupt_Flag(INT_CAN4);									//CAN清中断标志
		INT_Interrupt_Enable(INT_CAN4,TRUE);								//CAN中断使能
	}
	if(CANx=CAN5_SFR)
	{
		INT_Interrupt_Priority_Config(INT_CAN5,3,2);						//CAN抢占优先级4,子优先级0
		INT_Clear_Interrupt_Flag(INT_CAN5);									//CAN清中断标志
		INT_Interrupt_Enable(INT_CAN5,TRUE);								//CAN中断使能
	}

	INT_All_Enable (TRUE);
}

/*============================================================================
 *void CAN_Transmit_DATA(CAN_SFRmap* CANx, uint8_t *data , uint8_t lenth)
------------------------------------------------------------------------------
描述		: 数据发送加载
输入		: 1.要初始化的CAN标号	CAN0_SFR、CAN1_SFR、CAN2_SFR
		  2.ID
		  2.发送数据数组地址
		  3.发送数据长度
		  4.帧类型        CAN_DATA_FRAME数据帧                     CAN_REMOTE_FRAME远程帧
		  5.帧格式        CAN_FRAME_FORMAT_SFF标准帧     CAN_FRAME_FORMAT_EFF扩展帧
输出		: CAN_ERROR_BUFFERFULL  CAN_ERROR_NOERROR
备注		:
============================================================================*/
CAN_ErrorT CAN_Transmit_DATA(CAN_SFRmap* CANx, struct can_frame tx_frame)
{
	CAN_ErrorT x;
	uint8_t i;
	CAN_MessageTypeDef	CAN_MessageStructure;

	CAN_MessageStructure.m_FrameFormat = tx_frame.RmtFrm;             	 //帧格式
	CAN_MessageStructure.m_RemoteTransmit = tx_frame.MsgType;		 	 //帧类型
	if(tx_frame.RmtFrm==CAN_FRAME_FORMAT_SFF)//标准帧
	{
		CAN_MessageStructure.m_StandardID = tx_frame.TargetID;			 //标准帧ID
		CAN_MessageStructure.m_ExtendedID = 0;			         //扩展帧ID
	}
	else if(tx_frame.RmtFrm==CAN_FRAME_FORMAT_EFF)//扩展帧
	{
		CAN_MessageStructure.m_StandardID = 0;					 //标准帧ID
		CAN_MessageStructure.m_ExtendedID = tx_frame.TargetID;			 //扩展帧ID
	}
	if(tx_frame.lenth>8)
	{
		tx_frame.lenth=8;
	}
	CAN_MessageStructure.m_DataLength =tx_frame.lenth;			    	 //长度
	for(i=0;i<tx_frame.lenth;i++)
	{
		CAN_MessageStructure.m_Data[i] = tx_frame.data[i];				 //数据
	}
	/* 发送缓冲器空 */
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

	/* 转载数据到发送缓冲器 */
	CAN_Transmit_Message_Configuration(CANx,&CAN_MessageStructure);
	/* 重复发送模式 */
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
 * 函数名：CAN0_INIT
 * 函数功能：can0初始化
 * 函数输入：帧率，帧类型
 * 函数输出：无
********************************************/
void CAN0_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter)
{
	xGPIO_CAN0();//CAN1引脚重映射
	xInit_CAN(CAN0_SFR,BaudRate,CAN_MODE_NORMAL,Filter);//CAN模块初始化，500K,正常模式
	xINT_CAN(CAN0_SFR);//使能CAN接收中断
	//xQueue_can0 = xQueueCreate( 10, sizeof( struct can_frame ) );
}
/********************************************
 * 函数名：CAN1_INIT
 * 函数功能：can1初始化
 * 函数输入：帧率，帧类型
 * 函数输出：无
********************************************/
void CAN1_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter)
{
	xGPIO_CAN1();//CAN1引脚重映射
	xInit_CAN(CAN1_SFR,BaudRate,CAN_MODE_NORMAL,Filter);//CAN模块初始化，500K,正常模式
	xINT_CAN(CAN1_SFR);//使能CAN接收中断
	//xQueue_can1 = xQueueCreate( 10, sizeof( struct can_frame ) );
}
/********************************************
 * 函数名：CAN2_INIT
 * 函数功能：can2初始化
 * 函数输入：帧率，帧类型
 * 函数输出：无
********************************************/
void CAN2_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter)
{
	xGPIO_CAN2();//CAN1引脚重映射
	xInit_CAN(CAN2_SFR,BaudRate,CAN_MODE_NORMAL,Filter);//CAN模块初始化，500K,正常模式
	xINT_CAN(CAN2_SFR);//使能CAN接收中断
	//xQueue_can2 = xQueueCreate( 10, sizeof( struct can_frame ) );
}
/********************************************
 * 函数名：CAN3_INIT
 * 函数功能：can3初始化
 * 函数输入：帧率，帧类型
 * 函数输出：无
********************************************/
void CAN3_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter)
{
	xGPIO_CAN3();//CAN1引脚重映射
	xInit_CAN(CAN3_SFR,BaudRate,CAN_MODE_NORMAL,Filter);//CAN模块初始化，500K,正常模式
	xINT_CAN(CAN3_SFR);//使能CAN接收中断
	//xQueue_can3 = xQueueCreate( 10, sizeof( struct can_frame ) );
}
/********************************************
 * 函数名：CAN1_INIT
 * 函数功能：can1初始化
 * 函数输入：帧率，帧类型
 * 函数输出：无
********************************************/
void CAN4_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter)
{
	xGPIO_CAN4();//CAN1引脚重映射
	xInit_CAN(CAN4_SFR,BaudRate,CAN_MODE_NORMAL,Filter);//CAN模块初始化，500K,正常模式
	xINT_CAN(CAN4_SFR);//使能CAN接收中断
	//xQueue_can4 = xQueueCreate( 10, sizeof( struct can_frame ) );
}

void CAN5_INIT(CAN_BaudRateT BaudRate,struct CAN_Filter *Filter)
{
	xGPIO_CAN5();//CAN1引脚重映射
	xInit_CAN(CAN5_SFR,BaudRate,CAN_MODE_NORMAL,Filter);//CAN模块初始化，500K,正常模式
	xINT_CAN(CAN5_SFR);//使能CAN接收中断
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
	if(CANx->CTLR &(1<<23))//Boff置位
	{
		CANx->CTLR &= (~0x01);//清除复位模式后，
	}
}
