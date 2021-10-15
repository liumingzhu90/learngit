/****************************************************************************************
 *
 * File Name: main 
 * Project Name: kungfu_aebs
 * Version: v1.0
 * Date: 2021-06-16- 16:51:48
 * Author: shuai
 * 
 ****************************************************************************************/
/* MCU运行所需头文件 */
#include "system_init.h"
#include "task_manager.h"
#include "usart.h"
#include "canhl.h"
#include "config.h"
#include "bluetooth.h"
#include "watchdog.h"
#include "tim.h"
#include "common.h"
#include "flash.h"
#include "spi_flash.h"
#include "set_parameter.h"
#include "can_task.h"
#include "vehicle_can.h"
#include "gpio.h"
#include "EC200U.h"
#include "data_uploading.h"
uint32_t SystemtimeClock = 0;
uint32_t oldsystime = 0;
uint32_t Messageoldsystime = 0;
uint32_t GPSstarttime = 0;
uint32_t stype = 0;
//#include "ConfigParameter.h"
//#include "timer.h"
uint8_t temp_data_num[2][2];
//float UraderSpeed[5];
extern uint32_t Time14_CNT;
extern uint32_t Time15_CNT;
volatile uint8_t Receive_flag_Uart_1; //接收标志位
uint8_t Receive_flag_Uart_4; //接收标志位
//extern struct can_frame  stVehicleCanData;
extern struct can_frame	stCammeraCanData;

extern void Vehicle_Can_Analysis(struct can_frame* rx_frame);
extern void Camera_Can_Data(struct can_frame *rx_frame);
void Send_Break_Control(void);
void Sys_FcwBrake_Ctrl(void);
void GPIO_Init(void);
void Led_Control(void);

void Delay(volatile uint32_t a, volatile uint32_t b)
{
	volatile uint32_t i;
	volatile uint32_t j;
	for(i=a;i>1;i--)
	{
		j=b;
        while(j--);
	}

}

void BASIC_TIMER_Config_new(BTIM_SFRmap* BTIMx, InterruptIndex Peripheral)
{
	//定时器时钟源选用SCLK  设周期为50000 设预分频23+1=24分频 120M主频 定时10ms进一次中断

	TIM_Reset(BTIMx);												//定时器外设复位，使能外设时钟
	BTIM_Updata_Immediately_Config(BTIMx,TRUE);						//立即更新控制
	BTIM_Updata_Enable(BTIMx,TRUE);									//配置更新使能
	BTIM_Work_Mode_Config(BTIMx,BTIM_TIMER_MODE);					//定时模式选择
	BTIM_Set_Counter(BTIMx,0);										//定时器计数值
	BTIM_Set_Period(BTIMx,50000);									//定时器周期值50000
	BTIM_Set_Prescaler(BTIMx,23);//23);								    //定时器预分频值23+1=24
	//BTIM_Set_Period(BTIMx,50000);
	//BTIM_Set_Prescaler(BTIMx,23);
	BTIM_Counter_Mode_Config(BTIMx,BTIM_COUNT_UP_OF);				//向上计数,上溢产生中断标志
	BTIM_Clock_Config(BTIMx,BTIM_SCLK);								//选用SCLK时钟
	INT_Interrupt_Priority_Config(Peripheral,4,0);					//抢占优先级4,子优先级0
	BTIM_Overflow_INT_Enable(BTIMx,TRUE);							//计数溢出中断使能
	INT_Interrupt_Enable(Peripheral,TRUE);						    //外设中断使能
	INT_Clear_Interrupt_Flag(Peripheral);							//清中断标志
	BTIM_Cmd(BTIMx,TRUE);											//定时器启动控制使能
	INT_Stack_Align_Config(INT_STACK_SINGLE_ALIGN);					//中断自动堆栈使用单字对齐
	INT_All_Enable (TRUE);											//全局可屏蔽中断使能,该中断使能控制不包含复位/NMI/硬件错误中断

}

void Send_Speed(void)
{
	struct can_frame tx_frame;

	Camera_CAN_Transmition(&tx_frame,stVehicleParas.fVehicleSpeed);
	if(stVehicleParas.LeftFlagTemp == 1)
		tx_frame.data[0] = 0x01;
	else
		tx_frame.data[0] = 0x00;
	if(stVehicleParas.RightFlagTemp == 1)
		tx_frame.data[0] = tx_frame.data[0] | 0x02;
	else
		tx_frame.data[0] = tx_frame.data[0] & 0xfd;
	//tx_frame.TargetID = 0x700;
	CAN_Transmit_DATA(CAN1_SFR,tx_frame);
	CAN_Transmit_DATA(CAN2_SFR,tx_frame);
}

void Send_Speed_To_Cammera(void)
{
	struct can_frame tx_frame;
	if(SystemtimeClock - oldsystime > 100)
	{
		//stVehicleParas.LeftFlagTemp = 0;
		//stVehicleParas.RightFlagTemp = 1;
		//stVehicleParas.fVehicleSpeed = 13;
		oldsystime = SystemtimeClock;
		//Receive_flag_Uart_4 = 0;
		//打印当前函数所在文件名和行号
		Camera_CAN_Transmition(&tx_frame,stVehicleParas.fVehicleSpeed);			//发送车速到相机
		CAN_Transmit_DATA(CAN0_SFR,tx_frame);
		Urader_CAN_Transmition(&tx_frame,stVehicleParas.fVehicleSpeed);		//发送车速到超声波雷达
		tx_frame.TargetID = 0x3f5;
		CAN_Transmit_DATA(CAN1_SFR,tx_frame);


	}
}

void Send_Message_To_Pc(void)
{
	struct can_frame tx_frame;
	if(SystemtimeClock - Messageoldsystime > 100)
	{
		Messageoldsystime = SystemtimeClock;
		Delay(100,100);
		Send_Warning_Message();			//报警信息
		Delay(100,100);
		Send_Targt_Message();			//目标信息
		Delay(100,100);
		Send_Sys_Status();				//系统信息
		Delay(100,100);
		Send_Vehicle_Status();			//车辆信息
		//Delay(100,100);
		//Send_Soft_Version();			//软件信息
	}
}
//CCP_Get_Capture_Result
void CCPx_Capture_Mode_init(CCP_SFRmap* CCPx)
{
	/*设置定时器的预分频值 以及捕捉通道的模式*/
	TIM_Reset(CCPx);										//定时器外设复位，使能外设时钟
	CCP_PWM_Input_Measurement_Config(CCPx,TRUE);            //PWM输入测量模式使能
	GPTIM_Slave_Mode_Config(CCPx,GPTIM_SLAVE_RESET_MODE);   //设置从模式：复位模式
	//GPTIM_Trigger_Select_Config(CCPx,GPTIM_TRIGGER_CCPXCH1);  //选择触发源为CH1
	GPTIM_Trigger_Select_Config(CCPx,GPTIM_TRIGGER_CCPXCH2);
	CCP_Capture_Mode_Config(CCPx, CCP_CHANNEL_2,CCP_CAP_RISING_EDGE);      ///设置捕捉通道 模式:每个下降沿发生捕捉
	//CCP_Capture_Mode_Config(CCPx, CCP_CHANNEL_2,CCP_CAP_FALLING_EDGE);      ///设置捕捉通道 模式:每个下降沿发生捕捉


	GPTIM_Updata_Immediately_Config(CCPx,TRUE);				//立即更新控制
	GPTIM_Updata_Enable(CCPx,TRUE);							//配置更新使能
	GPTIM_Work_Mode_Config(CCPx,GPTIM_TIMER_MODE);			//定时模式选择
	GPTIM_Set_Counter(CCPx,0);								//定时器计数值

	GPTIM_Set_Prescaler(CCPx,119);							//定时器预分频值 预分频为119+1=120分频，主时钟120M,1us计数一次
	GPTIM_Counter_Mode_Config(CCPx,GPTIM_COUNT_UP_OF);		//向上,上溢产生中断标志
	GPTIM_Clock_Config(CCPx,GPTIM_SCLK);					//选用SCLK时钟为定时器时钟源
	GPTIM_Cmd(CCPx,TRUE);                                   //使能通用定时器

}

void Init_Can(void)
{
	CAN0_INIT(stCanPara.can0rate,NULL);//毫米波雷达
	CAN1_INIT(stCanPara.can1rate,NULL);//相机
	CAN2_INIT(stCanPara.can2rate,NULL);//整车CAN
	CAN3_INIT(stCanPara.can3rate,NULL);//比例阀
	CAN4_INIT(stCanPara.can4rate,NULL);//显示屏
	CAN5_INIT(stCanPara.can5rate,NULL);//预留
}

void SysTick_Configuration(uint32_t Reload)
{
	SYSTICK_Cmd (FALSE);
	SYSTICK_Reload_Config(Reload);
	SYSTICK_Counter_Updata();                           //向ST_CV寄存器写任意值，以清零当前值寄存器
	SYSTICK_Clock_Config(SYSTICK_SYS_CLOCK_DIV_1);      //系统节拍定时器时钟源选择，SCLK作为时钟源
	SYSTICK_Systick_INT_Enable(TRUE);
	SYSTICK_Cmd(TRUE);
    INT_Interrupt_Enable(INT_SysTick,TRUE);				//使能SYSTICK中断
	INT_All_Enable (TRUE);
}

//主函数
void main()
{
	struct can_frame tx_frame;

	uint32_t send_time = 0;
	uint32_t i = 0;
	uint8_t temp2=0;
	uint32_t first_weight;
	uint32_t second_weight;
	uint32_t addrtest;

	SystemInit();
	//232 uart1
	User_USART_Init(115200);
	USART_Async_config(USART1_SFR,115200);
	USART_ReceiveInt_config(USART1_SFR,INT_USART1);
	//蓝牙串口
	User_USART4_Init(9600);							//蓝牙串口
	USART_Async_config(USART4_SFR,9600);
	USART_ReceiveInt_config(USART4_SFR,INT_USART4);
	//4G串口
	User_USART0_Init(115200);
	USART_Async_config(USART0_SFR,115200);
	USART_ReceiveInt_config(USART0_SFR,INT_USART0);
	//备用超声波雷达串口    上传串口
	User_USART2_Init(115200);
	USART_Async_config(USART2_SFR,115200);
	USART_ReceiveInt_config(USART2_SFR,INT_USART2);

	BASIC_TIMER_Config_new(T14_SFR,INT_T14); //定时10ms进一次中断
	//配置使能定时器T15
	BASIC_TIMER_Config_new(T15_SFR,INT_T15); //定时10ms进一次中断
	Init_Config_Parameter();
	Init_Sys_Parameter();
	Init_Can();
	GPIO_Out_Config();			//输出口初始化  led1- 7   刹车灯正向输出和刹车灯负向输出
	GPIO_Singal_init();			//输入口初始化
	GPIO_Power_CANFA_Control();

	EC200U_INIT();
	EC200U_POWERKEY(Bit_RESET);
	Delay(1000,1000);
	EC200U_POWERKEY(Bit_SET);
	Delay(1000,1000);

	GPIO_Write_Mode_Bits(GPIOE_SFR, GPIO_PIN_MASK_6, GPIO_MODE_OUT);//LED1
	GPIO_Write_Mode_Bits(GPIOF_SFR, GPIO_PIN_MASK_0, GPIO_MODE_OUT);//LED2

	oldsystime = SystemtimeClock;
	Messageoldsystime = SystemtimeClock;
	GPSstarttime  = SystemtimeClock;
	//配置使能定时器T14
	Vehicle_CAN_Init();
	TaskInit();

	SysTick_Configuration(120000);			//滴答定时器1ms一次中断

	while(1)
	{
		//fprintf(USART1_STREAM,"11111111");
		Vehicle_Parameter_Analysis(SystemtimeClock);		//obd口报文解析及io端口输入解析
		Urader_RxValve_Data_Analysis(SystemtimeClock);
		Camera_Can_Data(&stCammeraCanData);
		Sys_Brake_Ctrl();							//报警及刹车
		Check_SysErr_Alarm();							//状态检测
		Led_Control();								//显示灯控制
		Proprot_RxValve_Data_Analysis(SystemtimeClock);		//阀can报文解析
		//Sys_Send_Break();
		Send_Display_Message();						//报警显示器显示信息
		Send_Speed_To_Cammera();
		//Report_Vehicle_Base_Info();
		//Report_Warning_Info();
		Data_Uploading_In_MainLoop();				// 数据上传(配合才库GPS 4G模块) add lmz 20211012
		Send_Message_To_Pc();
	}
}


//****************************************************************************
//						OS Hook
//****************************************************************************
//任务栈溢出钩子
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	RTOS_DEBUG_MSG("vApplicationStackOverflowHook: %s over: %d \r\n", pcTaskName, uxTaskGetStackHighWaterMark(xTask));
	while (1)
		;
}

//任务内存分配错误钩子
void vApplicationMallocFailedHook(void)
{
	RTOS_DEBUG_MSG("vApplicationMallocFailedHook\r\n");
	while (1)
		;
}
