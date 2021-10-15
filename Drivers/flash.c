/**
  ********************************************************************
  * 文件名  flash.c
  * 作  者   ChipON_AE/FAE_Group
  * 版  本  V2.1
  * 日  期  2019-11-16
  * 描  述  该文件提供了flash应用读写函数。
  *
  *********************************************************************
*/
#include"flash.h"
#include "system_init.h"
FLASH_ProgramTypeDef flash_write;


/**
  * 描述  flash 程序区写单个双字节数据（写入两个32位数据）。
  * 输入  address： 指定flash  程序区地址
  *  p_FlashBuffer: 写入数据的指针
  *
  * 返回  无。
  * 写flash注意说明：页擦会把指定地址所在的一页128个双字地址的数据都擦除掉
  */
void FLASH_WriteCODE_ONE(uint32_t address,uint32_t *p_FlashBuffer)
{

	//	//--------------------------------------------------------------------------------------------
		FLASH_Wipe_Configuration_RAM(FLASH_WIPE_CODE_PAGE,address);//程序区页擦

		flash_write.m_Mode=FLASH_PROGRAM_WORD;//FLASH_PROGRAM_HALF_PAGE //FLASH_PROGRAM_WORD
		flash_write.m_Zone=FLASH_PROGRAM_CODE;//FLASH_PROGRAM_CFG  FLASH_PROGRAM_CODE
		flash_write.m_Addr=address;
		flash_write.m_WriteSize=1;//半页编程不得为0，如果写1（+1）实际写的是2个双字 单字编程无影响
		flash_write.m_Data=p_FlashBuffer;
		FLASH_Program_Configuration_RAM(&flash_write);
}

/*
*
* 描述：程序区半页编程，按半页或小于半页数据写入数据到FLASH，个数参数1~63，地址为页的首地址 如十六进制下结尾000 400 800 C00
* 			如果地址不是页的首地址，必须确定后续页结果为0xFFFF，或前面操作过页首写，使后续块值被0xFFFF，否则写结果异常。
* 输入参数：address 待写地址
*          p_FlashBuffer 待写地址的数据指针
*          length        待写数据长度取值：1~63
*  （一次写64位的双字节，如设最大63时，实际最大写入（63+1）*2=128个32位的数据，如没有补齐数据，系统会生成随机数一起写入flash
* 返回          ：无
* 写flash注意说明：页擦会把指定地址所在的那一页128个双字地址的数据都擦除掉，
*/
void FLASH_HALFPAGE_WRITECODE_fun(uint32_t address,uint32_t *p_FlashBuffer,uint32_t length)
{

		FLASH_Wipe_Configuration_RAM(FLASH_WIPE_CODE_PAGE,address);//程序区页擦

		flash_write.m_Mode=FLASH_PROGRAM_HALF_PAGE;//FLASH_PROGRAM_HALF_PAGE //FLASH_PROGRAM_WORD
		flash_write.m_Zone=FLASH_PROGRAM_CODE;//FLASH_PROGRAM_CFG  FLASH_PROGRAM_CODE
		flash_write.m_Addr=address;
		flash_write.m_WriteSize=length;//半页编程不得为0，如果写1（+1）实际写的是2个双字 单字编程无影响
		flash_write.m_Data=p_FlashBuffer;
		FLASH_Program_Configuration_RAM(&flash_write);
}

/*
*
* 描述：程序区按页写入数据到FLASH，个数参数只能为128 ，地址必须为页的首地址 如十六进制下结尾000 400 800 C00
* 			如果地址不是页的首地址，必须确定后续块结果为0xFFFF，或前面操作过块首写，使后续页值被0xFFFF，否则写结果异常。
* 输入参数：address 待写地址
*          p_FlashBuffer 待写地址的数据指针
*          length        待写数据长度(个数参数只能为128)
* 返回          ：无
* 写flash注意说明：页擦会把指定地址所在的一页128个双字地址的数据都擦除掉，半页编程只能64个双字进行写操作，页写需要进行两次半页写。
*/
void FLASH_PageWrite_fun(uint32_t address,uint32_t *p_FlashBuffer,uint8_t length)
{
	volatile uint8_t i;
	static uint32_t Flash_Buff1[128];
	static uint32_t Flash_Buff2[128];

	//	//--------------------------------------------------------------------------------------------
		FLASH_Wipe_Configuration_RAM(FLASH_WIPE_CODE_PAGE,address);//程序区页擦

		//--------------------------------------------------------------------------------------------
		for (i = 0; i < length; i++)
		{
			Flash_Buff1[i] =*p_FlashBuffer;
			p_FlashBuffer ++;
		}

		flash_write.m_Mode=FLASH_PROGRAM_HALF_PAGE;//FLASH_PROGRAM_HALF_PAGE //FLASH_PROGRAM_WORD
		flash_write.m_Zone=FLASH_PROGRAM_CODE;//FLASH_PROGRAM_CFG  FLASH_PROGRAM_CODE
		flash_write.m_Addr=address;
		flash_write.m_WriteSize=63;//半页编程不得为0，如果写1（+1）实际写的是2个双字 单字编程无影响
		flash_write.m_Data=Flash_Buff1;
		FLASH_Program_Configuration_RAM(&flash_write);


		for (i = 0; i < length; i++)
		{
			Flash_Buff2[i] = *p_FlashBuffer;
			p_FlashBuffer ++;
		}

		flash_write.m_Mode=FLASH_PROGRAM_HALF_PAGE;//FLASH_PROGRAM_HALF_PAGE //FLASH_PROGRAM_WORD
		flash_write.m_Zone=FLASH_PROGRAM_CODE;//FLASH_PROGRAM_CFG  FLASH_PROGRAM_CODE
		flash_write.m_Addr=address+0x200;
		flash_write.m_WriteSize=63;//半页编程不得为0，如果写1（+1）实际写的是2个双字 单字编程无影响
		flash_write.m_Data=Flash_Buff2;
		FLASH_Program_Configuration_RAM(&flash_write);


}


/**
  * 描述  flash 程序区读多字节数据
  * 输入  address： 指定flash地址
  *  p_FlashBuffer: 写入数据的指针
  *  length:写入数据的长度
  * 返回  无。
  */
void FLASH_READCODE_fun(uint32_t address,uint32_t *p_FlashBuffer,uint32_t length)
{

	volatile uint32_t read_num;

	for(read_num=0;read_num<length;read_num++)
	{
		*p_FlashBuffer=Read_Flash_or_CFR_RAM(address,FLASH_PROGRAM_CODE);
		p_FlashBuffer++;
		address=address+0x0004;
	}
}






/*******************************************信息用户区读写函数*********************************************/
/**
  * 描述  flash 信息用户区写单个双字节数据（写入两个32位数据）。
  * 输入  address： 指定flash用户区地址（0x1C00~0X1FF8）地址十六进制下结尾0x1C00 0x1C08  0x1E00 0x1E08
  * p_FlashBuffer: 写入数据的指针
  *
  * 返回  无。
  * 写flash注意说明：页擦会把指定地址所在的一页128个双字地址的数据都擦除掉
  */
void FLASH_WriteCFG_ONE(uint32_t address,uint32_t *p_FlashBuffer)
{

	//	//--------------------------------------------------------------------------------------------
		FLASH_Wipe_Configuration_RAM(FLASH_WIPE_CFG_PAGE,address);//程序区页擦

		flash_write.m_Mode=FLASH_PROGRAM_WORD;//FLASH_PROGRAM_HALF_PAGE //FLASH_PROGRAM_WORD
		flash_write.m_Zone=FLASH_PROGRAM_CFG;//FLASH_PROGRAM_CFG  FLASH_PROGRAM_CODE
		flash_write.m_Addr=address;
		flash_write.m_WriteSize=1;//半页编程不得为0，如果写1（+1）实际写的是2个双字 单字编程无影响
		flash_write.m_Data=p_FlashBuffer;
		FLASH_Program_Configuration_RAM(&flash_write);
}

/*
* 描述：用户区半页编程，按半页或小于半页数据写入数据到FLASH，个数参数1~63，地址为页的首地址 如十六进制下结尾000 400 800 C00
* 			如果地址不是页的首地址，必须确定后续页结果为0xFFFF，或前面操作过页首写，使后续块值被0xFFFF，否则写结果异常。
* 输入参数：address 指定用户区地址（0x1C00~0X1FF8）
*          p_FlashBuffer 待写地址的数据指针
*          length        待写数据长度取值：1~63
*  （一次写64位的双字节，如设最大63时，实际最大写入（63+1）*2=128个32位的数据，如没有补齐数据，系统会生成随机数一起写入flash）
* 返回          ：无
* 写flash注意说明：页擦会把指定地址所在的那一页128个双字地址的数据都擦除掉，
*/
void FLASH_HALFPAGE_WRITECFG_fun(uint32_t address,uint32_t *p_FlashBuffer,uint32_t length)
{

		FLASH_Wipe_Configuration_RAM(FLASH_WIPE_CFG_PAGE,address);//程序区页擦

		flash_write.m_Mode=FLASH_PROGRAM_HALF_PAGE;//FLASH_PROGRAM_HALF_PAGE //FLASH_PROGRAM_WORD
		flash_write.m_Zone=FLASH_PROGRAM_CFG;//FLASH_PROGRAM_CFG  FLASH_PROGRAM_CODE
		flash_write.m_Addr=address;
		flash_write.m_WriteSize=length;//半页编程不得为0，如果写1（+1）实际写的是2个双字 单字编程无影响
		flash_write.m_Data=p_FlashBuffer;
		FLASH_Program_Configuration_RAM(&flash_write);
}


/*
*
* 描述：用户区按页写入数据到FLASH，个数参数只能为128 ，地址必须为页的首地址 如十六进制下结尾000 400 800 C00
* 			如果地址不是页的首地址，必须确定后续块结果为0xFFFF，或前面操作过块首写，使后续页值被0xFFFF，否则写结果异常。
* 输入参数：address 待写地址因目前开放的只有最后一个PAGE 因此首地址只能从0x1C00开始
*          p_FlashBuffer 待写地址的数据指针
*          length        待写数据长度(个数参数只能为128)
* 返回          ：无
* 写flash注意说明：页擦会把指定地址所在的一页128个双字地址的数据都擦除掉，半页编程只能64个双字进行写操作，页写需要进行两次半页写。
*/
void FLASH_PageWrite_CFG_fun(uint32_t address,uint32_t *p_FlashBuffer,uint8_t length)
{
	volatile uint8_t i;
	static uint32_t Flash_Buff1[128];
	static uint32_t Flash_Buff2[128];

	//	//--------------------------------------------------------------------------------------------
		//FLASH_Wipe_Configuration_RAM(FLASH_WIPE_CFG_PAGE,address);//信息用户区页擦

		//--------------------------------------------------------------------------------------------
		for (i = 0; i < length; i++)
		{
			Flash_Buff1[i] =*p_FlashBuffer;
			p_FlashBuffer ++;
		}

		flash_write.m_Mode=FLASH_PROGRAM_HALF_PAGE;//FLASH_PROGRAM_HALF_PAGE //FLASH_PROGRAM_WORD
		flash_write.m_Zone=FLASH_PROGRAM_CFG;//FLASH_PROGRAM_CFG  FLASH_PROGRAM_CODE
		flash_write.m_Addr=address;
		flash_write.m_WriteSize=63;//半页编程不得为0，如果写1（+1）实际写的是2个双字 单字编程无影响
		flash_write.m_Data=Flash_Buff1;
		FLASH_Program_Configuration_RAM(&flash_write);

/*
		for (i = 0; i < length; i++)
		{
			Flash_Buff2[i] = *p_FlashBuffer;
			p_FlashBuffer ++;
		}

		flash_write.m_Mode=FLASH_PROGRAM_HALF_PAGE;//FLASH_PROGRAM_HALF_PAGE //FLASH_PROGRAM_WORD
		flash_write.m_Zone=FLASH_PROGRAM_CFG;//FLASH_PROGRAM_CFG  FLASH_PROGRAM_CODE
		flash_write.m_Addr=address+0x200;
		flash_write.m_WriteSize=63;//半页编程不得为0，如果写1（+1）实际写的是2个双字 单字编程无影响
		flash_write.m_Data=Flash_Buff2;
		FLASH_Program_Configuration_RAM(&flash_write);
*/

}


/**
  * 描述  flash 程序区写单字节数据
  * 输入  address： 指定flash地址
  *  p_FlashBuffer: 写入数据的指针
  *
  * 返回  无。
  * 注：写入前要先擦除地址所在的页
  */
void FLASH_WriteByte(uint32_t address,uint8_t p_FlashBuffer)
{
	    uint32_t num32[2];
	    uint32_t *flash_write_num;
	    num32[0]=p_FlashBuffer;
	    num32[1]=0;
		flash_write_num=num32;
		//--------------------------------------------------------------------------------------------

	    flash_write.m_Mode=FLASH_PROGRAM_WORD;//FLASH_PROGRAM_HALF_PAGE //FLASH_PROGRAM_WORD
		flash_write.m_Zone=FLASH_PROGRAM_CODE;//FLASH_PROGRAM_CFG  FLASH_PROGRAM_CODE
		flash_write.m_Addr=address;
		flash_write.m_WriteSize=1;//半页编程不得为0，如果写1（+1）实际写的是2个双字 单字编程无影响
		flash_write.m_Data=flash_write_num;
		FLASH_Program_Configuration_RAM(&flash_write);
}

/**
  * 描述  flash 程序区写半字数据
  * 输入  address： 指定flash地址
  *  p_FlashBuffer: 写入数据的指针
  *
  * 返回  无。
  * 注：写入前要先擦除地址所在的页
  */
void FLASH_WriteHalfWord(uint32_t address,uint16_t p_FlashBuffer)
{
	    uint32_t num32[2];
	    uint32_t *flash_write_num;
	    num32[0]=p_FlashBuffer;
	    num32[1]=0;
		flash_write_num=num32;
		//--------------------------------------------------------------------------------------------

	    flash_write.m_Mode=FLASH_PROGRAM_WORD;//FLASH_PROGRAM_HALF_PAGE //FLASH_PROGRAM_WORD
		flash_write.m_Zone=FLASH_PROGRAM_CODE;//FLASH_PROGRAM_CFG  FLASH_PROGRAM_CODE
		flash_write.m_Addr=address;
		flash_write.m_WriteSize=1;//半页编程不得为0，如果写1（+1）实际写的是2个双字 单字编程无影响
		flash_write.m_Data=flash_write_num;
		FLASH_Program_Configuration_RAM(&flash_write);
}

void FLASH_WriteHalfWord_Cfg(uint32_t address,uint16_t p_FlashBuffer)
{
	    uint32_t num32[2];
	    uint32_t *flash_write_num;
	    num32[0]=p_FlashBuffer;
	    num32[1]=0;
		flash_write_num=num32;
		//--------------------------------------------------------------------------------------------

	    flash_write.m_Mode=FLASH_PROGRAM_WORD;//FLASH_PROGRAM_HALF_PAGE //FLASH_PROGRAM_WORD
		flash_write.m_Zone=FLASH_PROGRAM_CFG;//FLASH_PROGRAM_CFG  FLASH_PROGRAM_CODE
		flash_write.m_Addr=address;
		flash_write.m_WriteSize=1;//半页编程不得为0，如果写1（+1）实际写的是2个双字 单字编程无影响
		flash_write.m_Data=flash_write_num;
		FLASH_Program_Configuration_RAM(&flash_write);
}

/**
  * 描述  flash 程序区写单字数据
  * 输入  address： 指定flash地址
  *  p_FlashBuffer: 写入数据的指针
  *
  * 返回  无。
  * 注：写入前要先擦除地址所在的页
  */
void FLASH_WriteWord(uint32_t address,uint32_t p_FlashBuffer)
{
	    uint32_t num32[2];
	    uint32_t *flash_write_num;
	    num32[0]=p_FlashBuffer;
	    num32[1]=0;
		flash_write_num=num32;
		//--------------------------------------------------------------------------------------------

	    flash_write.m_Mode=FLASH_PROGRAM_WORD;//FLASH_PROGRAM_HALF_PAGE //FLASH_PROGRAM_WORD
		flash_write.m_Zone=FLASH_PROGRAM_CODE;//FLASH_PROGRAM_CFG  FLASH_PROGRAM_CODE
		flash_write.m_Addr=address;
		flash_write.m_WriteSize=1;//半页编程不得为0，如果写1（+1）实际写的是2个双字 单字编程无影响
		flash_write.m_Data=flash_write_num;
		FLASH_Program_Configuration_RAM(&flash_write);
}





/**
  * 描述  flash 程序区写多个字节数据
  * 输入  address： 指定flash地址
  *  p_FlashBuffer: 写入数据的指针
  *    leng：数据的长度
  * 返回  无。
  * 注：写入前要先擦除地址所在的页
  */
void FLASH_WriteNByte(uint32_t address,uint8_t *p_FlashBuffer,uint32_t leng)//地址必须为被8整除
{
	uint32_t i;
	for(i=0;i<leng;i++)//一页1024byte，缓冲一次写64bit=8byte，128个缓冲块
	{
		FLASH_WriteByte(address,*p_FlashBuffer);//
		p_FlashBuffer++;
		address+=8;
	}
}

/**
  * 描述  flash 程序区读单字节数据
  * 输入  address： 指定flash地址
  *  p_FlashBuffer: 读出数据的指针
  *
  * 返回  read_buf:读出单字节数据
  *
  */
uint32_t FLASH_ReadByte(uint32_t address,uint8_t *p_FlashBuffer) //读代码区
{
	uint32_t read_buf;
	read_buf=Read_Flash_or_CFR_RAM(address,FLASH_PROGRAM_CODE);
	*p_FlashBuffer=read_buf;

	return read_buf;
}

/**
  * 描述  flash 程序区读半字数据
  * 输入  address： 指定flash地址
  *  p_FlashBuffer: 读出数据的指针
  *
  * 返回  read_buf:读出半字数据
  *
  */
uint32_t FLASH_ReadHalWord(uint32_t address,uint16_t *p_FlashBuffer)
{
	uint32_t read_buf;
	read_buf=Read_Flash_or_CFR_RAM(address,FLASH_PROGRAM_CODE);
	*p_FlashBuffer=read_buf;

	return read_buf;
}

/**
  * 描述  flash 程序区读单字数据
  * 输入  address： 指定flash地址
  *  p_FlashBuffer: 读出数据的指针
  *
  * 返回  read_buf:读出单字数据
  *
  */
uint32_t FLASH_ReadWord(uint32_t address,uint32_t *p_FlashBuffer)
{
	uint32_t read_buf;
	read_buf=Read_Flash_or_CFR_RAM(address,FLASH_PROGRAM_CODE);
	*p_FlashBuffer=read_buf;

	return read_buf;
}

/**
  * 描述  flash 程序区多次读单字节数据
  * 输入  address： 指定flash地址
  *  p_FlashBuffer: 读出数据的指针
  *   leng：指定数据长度
  * 返回  无
  *
  */
void FLASH_ReadNByte(uint32_t address,uint8_t *p_FlashBuffer,uint32_t leng)//地址必须为被8整除
{
	uint32_t i=0;
	uint32_t read_buf;
	for(i=0;i<leng;i++)//一页1024byte，缓冲一次写64bit=8byte，128个缓冲块
	{
		read_buf=Read_Flash_or_CFR_RAM(address,FLASH_PROGRAM_CODE);
		*p_FlashBuffer=read_buf;
		p_FlashBuffer++;
		address+=8;
	}
}

