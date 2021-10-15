/**
  ******************************************************************************
  *
  * 文件名  Usart.h
  * 作  者  ChipON_AE/FAE_Group
  * 日  期  2019-10-19
  * 描  述  该文件提供了串口例程相关配置函数
  *
  ******************************************************************************/

#ifndef USART_H_
#define USART_H_
#include "system_init.h"
#define KM_MASTER

extern uint8_t Uart2_Rxbuffer[1024];
extern uint32_t Uart2_Rxcount;


extern uint8_t User_Rxbuffer[1024];
extern uint32_t User_Rxcount;
extern uint8_t User_Receive_flag;
extern uint8_t User_RxString[];
extern uint8_t User_Rx2String[30][15];
void User_USART_Init(uint32_t baud);
extern void USART_Async_config(USART_SFRmap *USARTx,uint32_t baud);//串口异步全双工配置
extern void USART_Async_config_tx1(USART_SFRmap *USARTx,uint32_t baud);
extern void USART_Sync_config(USART_SFRmap* USARTx);//串口半双工同步配置
extern void USART_ReceiveInt_config(USART_SFRmap *USARTx,InterruptIndex Peripheral);//串口接收中断使能
extern void USART_Send(USART_SFRmap* USARTx, uint8_t* Databuf, uint32_t length);//串口发送函数
//void String_Cut(uint8_t src[],uint32_t len,uint8_t *drc[],uint32_t *first,uint32_t *second);
void Buffer_Length(uint8_t src[],uint32_t *len);
uint8_t Bluetooth_Commod_Type_Annalyse(uint8_t *data);
void Bluetooth_Data_Annalyse(uint8_t *data);
void USART_line_feed(USART_SFRmap *USARTx);
void User_USART4_Init(uint32_t baud);
void User_USART0_Init(uint32_t baud);
void User_USART2_Init(uint32_t baud);
void User_USART_Init(uint32_t baud);
#ifdef KM_MASTER
void xGPIO_USART0();
void xGPIO_USART1();
void xGPIO_USART2();
void xGPIO_USART3();
void xGPIO_USART4();
void xGPIO_USART5();
void xGPIO_USART6();
void xGPIO_USART7();
#endif

#endif /* USART_H_ */
