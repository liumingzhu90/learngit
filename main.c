/****************************************************************************************
 *
 * File Name: main 
 * Project Name: kungfu_aebs
 * Version: v1.0
 * Date: 2021-06-16- 16:51:48
 * Author: shuai
 * 
 ****************************************************************************************/
/* MCU��������ͷ�ļ� */
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
//#include "spi_flash.h"
#include "set_parameter.h"
#include "can_task.h"
#include "vehicle_can.h"
#include "gpio.h"
#include "EC200U.h"
#include "data_uploading.h"
#include "wheelSpeed.h"
#include "usart_upgrade.h"
#include "_4g_upgrade.h"
#include "_4G_data_upload.h"
#include "AEB_CMS_alg.h"
#include "sss_alg.h"
#include "aeb_cms_sss_para_config.h"
#include "at_parse_alg_para.h"
#define _JIN_LV_BUS
//#define _JIE_FANG_TRUCK

uint32_t SystemtimeClock = 0;
uint32_t oldsystime = 0;
uint32_t Uoldsystime = 0;
uint32_t Messageoldsystime = 0;
uint32_t GPSstarttime = 0;
uint32_t stype = 0;
//#include "ConfigParameter.h"
//#include "timer.h"
uint8_t temp_data_num[2][2];
//float UraderSpeed[5];
extern uint32_t Time14_CNT;
extern uint32_t Time15_CNT;
volatile uint8_t Receive_flag_Uart_1; //���ձ�־λ
uint8_t Receive_flag_Uart_4; //���ձ�־λ
//extern struct can_frame  stVehicleCanData;
extern struct can_frame	stCammeraCanData;

extern void Vehicle_Can_Analysis(struct can_frame* rx_frame);
extern void Got_Vehicle_Para_form_GPIO(_VEHICLE_PARA* stVehicleParas);
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
	//��ʱ��ʱ��Դѡ��SCLK  ������Ϊ50000 ��Ԥ��Ƶ23+1=24��Ƶ 120M��Ƶ ��ʱ10ms��һ���ж�

	TIM_Reset(BTIMx);												//��ʱ�����踴λ��ʹ������ʱ��
	BTIM_Updata_Immediately_Config(BTIMx,TRUE);						//�������¿���
	BTIM_Updata_Enable(BTIMx,TRUE);									//���ø���ʹ��
	BTIM_Work_Mode_Config(BTIMx,BTIM_TIMER_MODE);					//��ʱģʽѡ��
	BTIM_Set_Counter(BTIMx,0);										//��ʱ������ֵ
	BTIM_Set_Period(BTIMx,50000);									//��ʱ������ֵ50000
	BTIM_Set_Prescaler(BTIMx,23);//23);								    //��ʱ��Ԥ��Ƶֵ23+1=24
	//BTIM_Set_Period(BTIMx,50000);
	//BTIM_Set_Prescaler(BTIMx,23);
	BTIM_Counter_Mode_Config(BTIMx,BTIM_COUNT_UP_OF);				//���ϼ���,��������жϱ�־
	BTIM_Clock_Config(BTIMx,BTIM_SCLK);								//ѡ��SCLKʱ��
	INT_Interrupt_Priority_Config(Peripheral,4,0);					//��ռ���ȼ�4,�����ȼ�0
	BTIM_Overflow_INT_Enable(BTIMx,TRUE);							//��������ж�ʹ��
	INT_Interrupt_Enable(Peripheral,TRUE);						    //�����ж�ʹ��
	INT_Clear_Interrupt_Flag(Peripheral);							//���жϱ�־
	BTIM_Cmd(BTIMx,TRUE);											//��ʱ����������ʹ��
	INT_Stack_Align_Config(INT_STACK_SINGLE_ALIGN);					//�ж��Զ���ջʹ�õ��ֶ���
	INT_All_Enable (TRUE);											//ȫ�ֿ������ж�ʹ��,���ж�ʹ�ܿ��Ʋ�������λ/NMI/Ӳ�������ж�

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
		//��ӡ��ǰ���������ļ������к�
		Camera_CAN_Transmition(&tx_frame,stVehicleParas.fVehicleSpeed);			//���ͳ��ٵ����
		CAN_Transmit_DATA(CAN0_SFR,tx_frame);
		//Urader_CAN_Transmition(&tx_frame,stVehicleParas.fVehicleSpeed);		//���ͳ��ٵ��������״�
		//tx_frame.TargetID = 0x3f5;
		//CAN_Transmit_DATA(CAN1_SFR,tx_frame);


	}
}
void Send_Message_To_Cammera(void)
{
	struct can_frame tx_frame;
	if(SystemtimeClock - oldsystime > 100)
	{
		//stVehicleParas.LeftFlagTemp = 0;
		//stVehicleParas.RightFlagTemp = 1;
		//stVehicleParas.fVehicleSpeed = 13;
		oldsystime = SystemtimeClock;
		//Receive_flag_Uart_4 = 0;
		//��ӡ��ǰ���������ļ������к�
		Camera_CAN_Transmition(&tx_frame,stVehicleParas.fVehicleSpeed);			//���ͳ��ٵ����
		if(stVehicleParas.LeftFlagTemp){
			tx_frame.data[0] |= 0x02;
		}
		if(stVehicleParas.RightFlagTemp){
			tx_frame.data[0] |= 0x01;
		}
		if(stVehicleParas.BrakeFlag){
			tx_frame.data[0] |= 0x10;
		}
		if(stVehicleParas.ReverseGear){
			tx_frame.data[0] |= 0x40;
		}
		CAN_Transmit_DATA(CAN0_SFR,tx_frame);
		//Urader_CAN_Transmition(&tx_frame,stVehicleParas.fVehicleSpeed);		//���ͳ��ٵ��������״�
		//tx_frame.TargetID = 0x3f5;
		//CAN_Transmit_DATA(CAN1_SFR,tx_frame);


	}
}

void Send_Speed_To_Urader(void)
{
	struct can_frame tx_frame;
	//if(stCanCommSta.stHRadar.rcvflag == 1)
	{
		if(SystemtimeClock - Uoldsystime > 100)
		{
			//stVehicleParas.LeftFlagTemp = 0;
			//stVehicleParas.RightFlagTemp = 1;
			//stVehicleParas.fVehicleSpeed = 13;
			Uoldsystime = SystemtimeClock;
			//Receive_flag_Uart_4 = 0;
			//��ӡ��ǰ���������ļ������к�
//			Delay(41,41);
			Urader_CAN_Transmition(&tx_frame,stVehicleParas.fVehicleSpeed);		//���ͳ��ٵ��������״�
			tx_frame.TargetID = 0x3f5;
			CAN_Transmit_DATA(CAN_UREADER,tx_frame);
//			Delay(41,41);
		}
	}
}
void Send_Message_To_Pc(void)
{
	struct can_frame tx_frame;
	if(SystemtimeClock - Messageoldsystime > 100)
	{
		Messageoldsystime = SystemtimeClock;
		Delay(100,100);
		Send_Warning_Message();			//������Ϣ
		Delay(100,100);
		Send_Targt_Message();			//Ŀ����Ϣ
		Delay(100,100);
		Send_Sys_Status();				//ϵͳ��Ϣ
		Delay(100,100);
		Send_Vehicle_Status();			//������Ϣ
		//Delay(100,100);
		//Send_Soft_Version();			//�����Ϣ
	}
}
//CCP_Get_Capture_Result
void CCPx_Capture_Mode_init(CCP_SFRmap* CCPx)
{
	/*���ö�ʱ����Ԥ��Ƶֵ �Լ���׽ͨ����ģʽ*/
	TIM_Reset(CCPx);										//��ʱ�����踴λ��ʹ������ʱ��
	CCP_PWM_Input_Measurement_Config(CCPx,TRUE);            //PWM�������ģʽʹ��
	GPTIM_Slave_Mode_Config(CCPx,GPTIM_SLAVE_RESET_MODE);   //���ô�ģʽ����λģʽ
	//GPTIM_Trigger_Select_Config(CCPx,GPTIM_TRIGGER_CCPXCH1);  //ѡ�񴥷�ԴΪCH1
	GPTIM_Trigger_Select_Config(CCPx,GPTIM_TRIGGER_CCPXCH2);
	CCP_Capture_Mode_Config(CCPx, CCP_CHANNEL_2,CCP_CAP_RISING_EDGE);      ///���ò�׽ͨ�� ģʽ:ÿ���½��ط�����׽
	//CCP_Capture_Mode_Config(CCPx, CCP_CHANNEL_2,CCP_CAP_FALLING_EDGE);      ///���ò�׽ͨ�� ģʽ:ÿ���½��ط�����׽


	GPTIM_Updata_Immediately_Config(CCPx,TRUE);				//�������¿���
	GPTIM_Updata_Enable(CCPx,TRUE);							//���ø���ʹ��
	GPTIM_Work_Mode_Config(CCPx,GPTIM_TIMER_MODE);			//��ʱģʽѡ��
	GPTIM_Set_Counter(CCPx,0);								//��ʱ������ֵ

	GPTIM_Set_Prescaler(CCPx,119);							//��ʱ��Ԥ��Ƶֵ Ԥ��ƵΪ119+1=120��Ƶ����ʱ��120M,1us����һ��
	GPTIM_Counter_Mode_Config(CCPx,GPTIM_COUNT_UP_OF);		//����,��������жϱ�־
	GPTIM_Clock_Config(CCPx,GPTIM_SCLK);					//ѡ��SCLKʱ��Ϊ��ʱ��ʱ��Դ
	GPTIM_Cmd(CCPx,TRUE);                                   //ʹ��ͨ�ö�ʱ��

}

void Init_Can(void)
{
	CAN0_INIT(stCanPara.can0rate,NULL);//���ײ��״�
	CAN1_INIT(stCanPara.can1rate,NULL);//���
	CAN2_INIT(stCanPara.can2rate,NULL);//����CAN
	CAN3_INIT(stCanPara.can3rate,NULL);//������
	CAN4_INIT(stCanPara.can4rate,NULL);//��ʾ��
	CAN5_INIT(stCanPara.can5rate,NULL);//Ԥ��
}

void SysTick_Configuration(uint32_t Reload)
{
	SYSTICK_Cmd (FALSE);
	SYSTICK_Reload_Config(Reload);
	SYSTICK_Counter_Updata();                           //��ST_CV�Ĵ���д����ֵ�������㵱ǰֵ�Ĵ���
	SYSTICK_Clock_Config(SYSTICK_SYS_CLOCK_DIV_1);      //ϵͳ���Ķ�ʱ��ʱ��Դѡ��SCLK��Ϊʱ��Դ
	SYSTICK_Systick_INT_Enable(TRUE);
	SYSTICK_Cmd(TRUE);
    INT_Interrupt_Enable(INT_SysTick,TRUE);				//ʹ��SYSTICK�ж�
	INT_All_Enable (TRUE);
}
extern uint8_t test_ttt;
//������
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
	INT_Set_Interrupt_Priority(INT_USART1,15);
	USART_Async_config(USART1_SFR,115200);
	USART_ReceiveInt_config(USART1_SFR,INT_USART1);
	//��������
	User_USART4_Init(9600);							//��������
	INT_Set_Interrupt_Priority(INT_USART4,15);
	USART_Async_config(USART4_SFR,9600);
	USART_ReceiveInt_config(USART4_SFR,INT_USART4);
	//4G����
	User_USART0_Init(115200);
	INT_Set_Interrupt_Priority(INT_USART0,10);
	USART_Async_config(USART0_SFR,115200);
	USART_ReceiveInt_config(USART0_SFR,INT_USART0);
	//���ó������״ﴮ��    �ϴ�����
	User_USART2_Init(115200);
	INT_Set_Interrupt_Priority(INT_USART2,11);
	USART_Async_config(USART2_SFR,115200);
	USART_ReceiveInt_config(USART2_SFR,INT_USART2);

	//BASIC_TIMER_Config_new(T14_SFR,INT_T14); //��ʱ10ms��һ���ж�
	//����ʹ�ܶ�ʱ��T15
	//BASIC_TIMER_Config_new(T15_SFR,INT_T15); //��ʱ10ms��һ���ж�
	Init_Config_Parameter();
	Init_Sys_Parameter();
	Init_Can();
	GPIO_Out_Config();			//����ڳ�ʼ��  led1- 7   ɲ�������������ɲ���Ƹ������
	GPIO_Singal_init();			//����ڳ�ʼ��
	GPIO_Power_CANFA_Control();

	W25QXX_Init();				// init external flash
	AT_4G_Module_Init();		// init 4G OTA ,will check 4G,use some time.
	WheelSpeed_Init();			// init Calc Wheel Speed
	_4G_Data_Uploading_Init();	// init data uploading
	AT_Init();

	oldsystime = SystemtimeClock;
	Messageoldsystime = SystemtimeClock;
	GPSstarttime  = SystemtimeClock;
	//����ʹ�ܶ�ʱ��T14
	Vehicle_CAN_Init();
	TaskInit();
	Init_Can();
	SysTick_Configuration(120000);			//�δ�ʱ��1msһ���ж�

	// Ӧ�ó������к����ھ�ʾ����������λ��
	fprintf(USART1_STREAM,"00000000application is runing....\r\n");	// ��ɾ��
	IWDT_Config(4000,IWDT_PRESCALER_64);//�������Ź����� 32KHZ 1��64��Ƶ������500�Σ� 1���Ӻ����ϵͳ��λ
	fprintf(USART1_STREAM,"IWD\r\n");

	uint32_t previousMillis = SystemtimeClock;
	uint32_t interval = 1;
	while(1)
	{
		if(upgrade_p.rx_mode == RECV_BIN){
			IWDT_Feed_The_Dog();
			Upgrade_Warning();							// �����о�ʾ 20211211
			if(upgrade_p.up_mode == UPGRADE_USART1)
				Usart1_UserUpgrade_Interactoin_In_MainLoop();// �û�����1���� add lmz 20211122
			else
				OTA_Upgrade_Write_In_MainLoop();		// OTA����д������ 20211207
		}else{
			if(SystemtimeClock - previousMillis >= interval){
				previousMillis = SystemtimeClock;
#ifdef _JIN_LV_BUS
				Vehicle_Parameter_Analysis(SystemtimeClock);		//obd�ڱ��Ľ�����io�˿��������
#endif

#ifdef _JIE_FANG_TRUCK
				Got_Vehicle_Para_form_GPIO(&stVehicleParas);
#endif			
				Analysis_AT_CMD_InMainLoop();
				OTA_Upgrade_Interaction_In_MainLoop();		// OTA ���� 20211207
				_4G_Data_Uploading_In_MainLoop();			// �����ϴ�(ֱ�ӶԽ�ƽ̨)
				Calc_Bus_Encoder_Velocity();				// ����
				Vehicle_Parameter_Analysis(SystemtimeClock);		//obd�ڱ��Ľ�����io�˿��������
				Urader_RxValve_Data_Analysis(SystemtimeClock);
				Camera_Can_Data(&stCammeraCanData);
				Sys_Brake_Ctrl();							//������ɲ��
				Check_SysErr_Alarm();							//״̬���
				Led_Control();								//��ʾ�ƿ���
				Proprot_RxValve_Data_Analysis(SystemtimeClock);		//��can���Ľ���
				//Sys_Send_Break();
				Send_Display_Message();						//������ʾ����ʾ��Ϣ
				//Send_Speed_To_Cammera();
				Send_Message_To_Cammera();
				Send_Speed_To_Urader();
				//Data_Uploading_In_MainLoop();
			}
			IWDT_Feed_The_Dog();
		}
	}
}


//****************************************************************************
//						OS Hook
//****************************************************************************
//����ջ�������
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	RTOS_DEBUG_MSG("vApplicationStackOverflowHook: %s over: %d \r\n", pcTaskName, uxTaskGetStackHighWaterMark(xTask));
	while (1)
		;
}

//�����ڴ���������
void vApplicationMallocFailedHook(void)
{
	RTOS_DEBUG_MSG("vApplicationMallocFailedHook\r\n");
	while (1)
		;
}
