/**
  ********************************************************************
  * �ļ���  flash.h
  * ��  ��   ChipON_AE/FAE_Group
  * ��  ��  V2.1
  * ��  ��  2019-11-16
  * ��  ��  ���ļ��ṩ��flash���ܵ���ض�д�������ܶ���
  *
  *********************************************************************
*/
#ifndef FLASH_H_
#define FLASH_H_
#include "system_init.h"

#define Flash_OK                 1
#define Flash_FAIL               0
#define			FLASH_BUFFER_MAX	    256         //��ҳ�����ݸ���
#define			FLASH_BUFFER_Halfpage	128         //��ҳ�����ݸ���
uint32_t     	FLASH_BUFFER[FLASH_BUFFER_MAX];     //��������ҳдʱ�õ�����
uint32_t     	FLASH_BUFFER_CFG[FLASH_BUFFER_MAX];  //��Ϣ����ҳ��дʱ�õ�����
uint32_t     	FLASH_BUFFER_HALFPAGE[FLASH_BUFFER_Halfpage];   //��ҳд�õ�������
uint32_t     	FLASH_BUFFER_Read[FLASH_BUFFER_MAX];            //��ҳ��ʱ�õ�����
uint32_t Read_Flash_or_CFR_RAM (uint32_t address,uint32_t ZoneSelect);
void FLASH_HALFPAGE_WRITECODE_fun(uint32_t address,uint32_t *p_FlashBuffer,uint32_t length);
void FLASH_PageWrite_fun(uint32_t address,uint32_t *p_FlashBuffer,uint8_t length);
void FLASH_WriteCODE_ONE(uint32_t address,uint32_t *p_FlashBuffer);
void FLASH_READCODE_fun(uint32_t address,uint32_t *p_FlashBuffer,uint32_t length);


void FLASH_WriteCFG_ONE(uint32_t address,uint32_t *p_FlashBuffer);
void FLASH_HALFPAGE_WRITECFG_fun(uint32_t address,uint32_t *p_FlashBuffer,uint32_t length);
void FLASH_PageWrite_CFG_fun(uint32_t address,uint32_t *p_FlashBuffer,uint8_t length);

//��ַ����Ϊ��8����
void FLASH_WriteByte(uint32_t address,uint8_t p_FlashBuffer);     //дbyte
void FLASH_WriteHalfWord(uint32_t address,uint16_t p_FlashBuffer);//дHalWord
void FLASH_WriteWord(uint32_t address,uint32_t p_FlashBuffer);    //дword
void FLASH_WriteNByte(uint32_t address,uint8_t *p_FlashBuffer,uint32_t leng);//д��Byte

uint32_t FLASH_ReadByte(uint32_t address,uint8_t *p_FlashBuffer);  //��byte
uint32_t FLASH_ReadHalWord(uint32_t address,uint16_t *p_FlashBuffer);//��HalWord
uint32_t FLASH_ReadWord(uint32_t address,uint32_t *p_FlashBuffer);//��Word
void FLASH_ReadNByte(uint32_t address,uint8_t *p_FlashBuffer,uint32_t leng);//����Byte
void FLASH_WriteHalfWord_Cfg(uint32_t address,uint16_t p_FlashBuffer);
void FLASH_WriteCFG_ONE(uint32_t address,uint32_t *p_FlashBuffer);
/*������ַ����*/
//��һ��
#define  CAN0_RATE_ADDRESS	0x7f000			//can0���������ò�����
#define  CAN1_RATE_ADDRESS	0x7f010			//can1���������ò�����
#define  CAN2_RATE_ADDRESS	0x7f020			//can2���������ò�����
#define  CAN3_RATE_ADDRESS	0x7f030			//can3���������ò�����
#define  CAN4_RATE_ADDRESS	0x7f040			//can4���������ò�����
#define  CAN5_RATE_ADDRESS	0x7f050			//can5���������ò�����

#define  VEHICLE_WITH_ADDRESS	0x7f060			//�������ò�����
#define  VEHICLE_KEEP_DIS	 	0x7f070			//���ౣ�����ò�����
#define  VEHICLE_TTC1	 		0x7f080			//ttc1���ò�����
#define  VEHICLE_TTC2	 		0x7f090			//ttc2���ò�����
#define  VEHICLE_SENSITIVITY	0x7f0A0			//ttc2���ò�����
#define	 VEHICLE_COTROL_MODE	0x7f0B0
#define	 VEHICLE_GEAR_NUM		0x7f0C0
#define	 VEHICLE_HMW1		0x7f0D0
#define	 VEHICLE_HMW2		0x7f0E0

//�ڶ���
#define  VEHICLE_SPEED_MODE				0x7f100			//����ģʽ
#define  VEHICLE_SPEED_ID	 			0x7f110			//����ID
#define  VEHICLE_SPEED_BYTETYPE	 		0x7f120			//�����ֽ�ģʽ
#define  VEHICLE_SPEED_BYTESTART	 	0x7f130			//���ٿ�ʼ�ֽ�
#define  VEHICLE_SPEED_BYTENUM			0x7f140			//�����ֽڸ���

#define	 VEHICLE_SPEED_SCALE_FACTOR_A	0x7f150	//�ٶ��������ϵ��
#define	 VEHICLE_SPEED_SCALE_FACTOR_B	0x7f160	//�ٶȸ������ϵ��

#define  VEHICLE_ABS_ID	 				0x7f170			//ABSID
#define  VEHICLE_ABS_TYPE	 			0x7f180			//ABS�ź���Դ
			#define  VEHICLE_ABS_BYTENUM	 			0x7f190			//ABS�ֽ����			//�����
#define  VEHICLE_ABS_STARTBIT	 		0x7f1A0			//ABS��ʼbit
#define  VEHICLE_ABS_BITNUM				0x7f1B0			//ABS bit����
#define	 VEHICLE_ABS_WRONGNUM			0x7f1C0			//ABS����ֵ
//������
#define  VEHICLE_TURN_ID	 				0x7f200			//ת��ID
#define  VEHICLE_TURN_BYPE	 			0x7f210			//ת���ź���Դ
#define  VEHICLE_TURN_STARTBYTE	 		0x7f220			//ת���ֽ�
#define  VEHICLE_TURN_LSTARTBIT				0x7f230			//��ת����ʼbit
#define	 VEHICLE_TURN_LBITNUM			0x7f240			//��ת��bit����
#define  VEHICLE_TURN_LEFFECTIVE	 			0x7f250			//��ת����Чֵ
//#define  VEHICLE_TURN_RSTARTBIT	 		0x7f260			//��ת����ʼbit
//#define  VEHICLE_TURN_RBITNUM				0x7f270			//��ת��bit����
//#define	 VEHICLE_TURN_REFFECTIVE			0x7f280			//��ת����Чֵ

#define  VEHICLE_TURN_ID1	 				0x7f260			//ת��ID
#define  VEHICLE_TURN_BYPE1	 			0x7f270			//ת���ź���Դ
#define  VEHICLE_TURN_STARTBYTE1	 		0x7f280			//ת���ֽ�
//#define  VEHICLE_TURN_LSTARTBIT				0x7f2c0			//��ת����ʼbit
//#define	 VEHICLE_TURN_LBITNUM			0x7f2d0			//��ת��bit����
//#define  VEHICLE_TURN_LEFFECTIVE	 			0x7f2e0			//��ת����Чֵ
#define  VEHICLE_TURN_RSTARTBIT	 		0x7f290			//��ת����ʼbit
#define  VEHICLE_TURN_RBITNUM				0x7f2a0			//��ת��bit����
#define	 VEHICLE_TURN_REFFECTIVE			0x7f2b0			//��ת����Чֵ

//���Ŀ�
#define  VEHICLE_BRAKE_ID	 				0x7f300			//ɲ��ID
#define  VEHICLE_BRAKE_BYPE	 			0x7f310			//ɲ���ź���Դ
#define  VEHICLE_BRAKE_STARTBYTE	 		0x7f320			//ɲ���ֽ�
#define  VEHICLE_BRAKE_STARTBIT				0x7f330			//ɲ����ʼbit
#define	 VEHICLE_BRAKE_BITNUM			0x7f340			//ɲ��bit����
#define  VEHICLE_BRAKE_NUMTYPE	 			0x7f350			//ɲ��ֵ����
#define  VEHICLE_BRAKE_LEFFECTIVE	 		0x7f360			//ɲ����Чֵ
#define  VEHICLE_BRAKE_SCALE_FACTOR_A			0x7f370			//ɲ������ϵ��A
#define	 VEHICLE_BRAKE_SCALE_FACTOR_B			0x7f380			//ɲ������ϵ��B
//�����
#define  FCW_LINKAGE_ADDRESS	0x7f400			//FCW���ٹ������ò�����
#define  FCW_LINKAGE_SPEED_ADDRESS	0x7f410			//FCW�����������ò�����
#define  LDW_LINKAGE_ADDRESS	0x7f420			//LDW���ٹ������ò�����
#define  LDW_LINKAGE_SPEED_ADDRESS	0x7f430			//LDW�����������ò�����
#define  HMW_LINKAGE_ADDRESS	0x7f440			//HMW���ٹ������ò�����
#define  HMW_LINKAGE_SPEED_ADDRESS	0x7f450			//HMW�����������ò�����
#define  AEB_LINKAGE_ADDRESS	0x7f460			//AEB���ٹ������ò�����
#define  AEB_LINKAGE_SPEED_ADDRESS	0x7f470			//AEB�����������ò�����

#define  SWA_IID_ADDRESS	0x7f480			//������ת��id���ò�����
#define  SWA_BYTENUM_ADDRESS	0x7f490			//������ת����ʼ�ֽ����ò�����
#define  SWA_STARTBIT_ADDRESS	0x7f4a0			//������ת�ǿ�ʼbit���ò�����
#define  SWA_BITLEGH_ADDRESS	0x7f4b0			//������ת��bit�������ò�����
//������
#define  URADER_HOST_ONE_ONE_ADDRESS	0x7f500			//�������״��һ������һ����״����ò�����
#define  URADER_HOST_ONE_TWO_ADDRESS	0x7f510			//�������״��һ�����ڶ�����״����ò�����
#define  URADER_HOST_ONE_THREE_ADDRESS	0x7f520			//�������״��һ������������״����ò�����
#define  URADER_HOST_ONE_FOUR_ADDRESS	0x7f530			//�������״��һ�������ı���״����ò�����

#define  URADER_HOST_ONE_FIVE_ADDRESS	0x7f540			//�������״��һ�����������״����ò�����
#define  URADER_HOST_ONE_SIX_ADDRESS	0x7f550			//�������״��һ������������״����ò�����
#define  URADER_HOST_ONE_SEVEN_ADDRESS	0x7f560			//�������״��һ�������߱���״����ò�����
#define  URADER_HOST_ONE_EIGHT_ADDRESS	0x7f570			//�������״��һ�����ڰ˱���״����ò�����

#define  URADER_HOST_ONE_NINE_ADDRESS	0x7f580			//�������״��һ�����ھű���״����ò�����
#define  URADER_HOST_ONE_TEN_ADDRESS	0x7f590			//�������״��һ������ʮ����״����ò�����
#define  URADER_HOST_ONE_ELEVEN_ADDRESS	0x7f5a0			//�������״��һ������ʮһ����״����ò�����
#define  URADER_HOST_ONE_TWELVE_ADDRESS	0x7f5b0			//�������״��һ������ʮ������״����ò�����
//���߿�
#define  URADER_HOST_TWO_ONE_ADDRESS	0x7f600			//�������״�ڶ�������һ����״����ò�����
#define  URADER_HOST_TWO_TWO_ADDRESS	0x7f610			//�������״�ڶ������ڶ�����״����ò�����
#define  URADER_HOST_TWO_THREE_ADDRESS	0x7f620			//�������״�ڶ�������������״����ò�����
#define  URADER_HOST_TWO_FOUR_ADDRESS	0x7f630			//�������״�ڶ��������ı���״����ò�����

#define  URADER_HOST_TWO_FIVE_ADDRESS	0x7f640			//�������״�ڶ������������״����ò�����
#define  URADER_HOST_TWO_SIX_ADDRESS	0x7f650			//�������״�ڶ�������������״����ò�����
#define  URADER_HOST_TWO_SEVEN_ADDRESS	0x7f660			//�������״�ڶ��������߱���״����ò�����
#define  URADER_HOST_TWO_EIGHT_ADDRESS	0x7f670			//�������״�ڶ������ڰ˱���״����ò�����

#define  URADER_HOST_TWO_NINE_ADDRESS	0x7f680			//�������״�ڶ������ھű���״����ò�����
#define  URADER_HOST_TWO_TEN_ADDRESS	0x7f690			//�������״�ڶ�������ʮ����״����ò�����
#define  URADER_HOST_TWO_ELEVEN_ADDRESS	0x7f6a0			//�������״�ڶ�������ʮһ����״����ò�����
#define  URADER_HOST_TWO_TWELVE_ADDRESS	0x7f6b0			//�������״�ڶ�������ʮ������״����ò�����

//�ڰ˿�
#define URADER_SYS_MESSAGE_FIRST_DIMENSIONAL_COEFFICIENT 0x7f700
#define URADER_SYS_MESSAGE_SECOND_DIMENSIONAL_COEFFICIENT 0x7f710
#define URADER_SYS_MESSAGE_0SPEED_THRESHOLD 0x7f720
#define URADER_SYS_MESSAGE_5SPEED_THRESHOLD 0x7f730
#define URADER_SYS_MESSAGE_10SPEED_THRESHOLD 0x7f740
#define URADER_SYS_MESSAGE_15SPEED_THRESHOLD 0x7f750
#define URADER_SYS_MESSAGE_20SPEED_THRESHOLD 0x7f760
#define URADER_SYS_MESSAGE_LINKAGE_STATA 0x7f770
#define URADER_SYS_MESSAGE_LINKAGE_SPEED 0x7f780
#define URADER_SYS_MESSAGE_25SPEED_THRESHOLD 0x7f790
#endif /* FLASH_H_ */
