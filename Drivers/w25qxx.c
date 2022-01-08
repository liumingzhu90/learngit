/******************** (C) COPYRIGHT  源地工作室 ********************************
 * 文件名  ：w25qxx.c
 * 描述    ：完成对W25XX的flash完成读写操作
 * 作者    ：lmz
 * 版本更新: 2021-08-30
 * 硬件连接  :PC12-/CS   PC9--CLK   PC10--MISO  PC11--MOSI
 * 调试方式：KungFu KF32DP2
**********************************************************************************/
#include "w25qxx.h"
#include "upgrade_common.h"
#include "spi.h"
#include "usart.h"
#include "stdlib.h"
#include "malloc.h"
uint16_t W25QXX_TYPE=W25X40B;					//默认是W25X40B

/* -----------------------局部函数声明------------------------------- */
uint16_t  W25QXX_ReadID(void);  	    	//读取FLASH ID
uint8_t	 W25QXX_ReadSR(void);        		//读取状态寄存器
void W25QXX_Write_SR(uint8_t sr);  			//写状态寄存器
void W25QXX_Write_Enable(void);  			//写使能
void W25QXX_Write_Disable(void);			//写保护
void W25QXX_FAST_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);		// 快速读操作
void W25QXX_Wait_Busy(void);           		//等待空闲
void W25QXX_PowerDown(void);        		//进入掉电模式
void W25QXX_WAKEUP(void);					//唤醒
AppVersion Split_APP_Version(uint8_t *version);

void delay_us( uint32_t nCount);
void delay_us( uint32_t nCount)
{
	nCount=24*nCount;
	while(nCount--)
	{
	}
}

//=============================================================================
//函数名称:W25QXX_Init
//功能概要:初始化SPI FLASH的IO(/CS)口
//参数名称:无
//函数返回:无
//=============================================================================
void W25QXX_Init(void)
{
	GPIO_SPI2();
	SPI_FLASH_CS_HIGH(); 						//SPI FLASH不选中
	SPI2_Init();		   						//初始化SPI
	W25QXX_TYPE=W25QXX_ReadID();				//读取FLASH ID.
	switch(W25QXX_TYPE)							// print debug
	{
	case W25X40B:fprintf(USART1_STREAM,"W25X40B\n");break;
	case W25Q80:fprintf(USART1_STREAM,"W25Q80\n");break;
	case W25Q16:fprintf(USART1_STREAM,"W25Q16\n");break;
	case W25Q32:fprintf(USART1_STREAM,"W25Q32\n");break;
	case W25Q64:fprintf(USART1_STREAM,"W25Q64\n");break;
	case W25Q128:fprintf(USART1_STREAM,"W25Q128\n");break;
	default:fprintf(USART1_STREAM,"external flash not find type.\n");break;
	}
}

//=============================================================================
//函数名称:W25QXX_ReadSR
//功能概要:读取W25QXX的状态寄存器
//参数名称:无
//函数返回:无
//说明：
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:默认0,状态寄存器保护位,配合WP使用
//TB,BP2,BP1,BP0:FLASH区域写保护设置
//WEL:写使能锁定
//BUSY:忙标记位(1,忙;0,空闲)
//默认:0x00
//=============================================================================
uint8_t W25QXX_ReadSR(void)
{
	uint8_t byte=0;
	SPI_FLASH_CS_LOW();							//使能器件

	SPI2_ReadWriteByte(W25X_ReadStatusReg);    	//发送读取状态寄存器命令0x05
	byte=SPI2_ReadWriteByte(0Xff);             	//读取一个字节

	SPI_FLASH_CS_HIGH();						//取消片选
	return byte;
}
//=============================================================================
//函数名称:W25QXX_Write_SR
//功能概要:写W25QXX状态寄存器
//参数名称:sr 状态寄存器
//函数返回:无
//说明：     只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写!!!
//=============================================================================
void W25QXX_Write_SR(uint8_t sr)
{
	SPI_FLASH_CS_LOW();                         //使能器件
	SPI2_ReadWriteByte(W25X_WriteStatusReg);   	//发送写取状态寄存器命令0x01
	SPI2_ReadWriteByte(sr);               		//写入一个字节
	SPI_FLASH_CS_HIGH();                        //取消片选
}
//=============================================================================
//函数名称:W25QXX_Write_Enable
//功能概要:W25QXX写使能
//参数名称:无
//函数返回:无
//说明：     将WEL置位
//=============================================================================
void W25QXX_Write_Enable(void)
{
	SPI_FLASH_CS_LOW();                        	//使能器件
    SPI2_ReadWriteByte(W25X_WriteEnable);      	//发送写使能0x06
	SPI_FLASH_CS_HIGH();                     	//取消片选
}
//=============================================================================
//函数名称:W25QXX_Write_Disable
//功能概要:W25QXX写禁止
//参数名称:无
//函数返回:无
//说明：     将WEL清零
//=============================================================================
void W25QXX_Write_Disable(void)
{
	SPI_FLASH_CS_LOW();                     	//使能器件
    SPI2_ReadWriteByte(W25X_WriteDisable);     	//发送写禁止指令0x04
	SPI_FLASH_CS_HIGH();                   		//取消片选
}
//=============================================================================
//函数名称:W25QXX_ReadID
//功能概要:读取芯片ID
//参数名称:无
//函数返回:无
//=============================================================================
uint16_t W25QXX_ReadID(void)
{
	uint16_t Temp = 0;
	SPI_FLASH_CS_LOW();

	SPI2_ReadWriteByte(W25X_ManufactDeviceID);	//发送读取ID命令
	SPI2_ReadWriteByte(0xFF);					//Dummy
	SPI2_ReadWriteByte(0xFF);
	SPI2_ReadWriteByte(0x00);
	Temp|=SPI2_ReadWriteByte(0xFF)<<8;
	Temp|=SPI2_ReadWriteByte(0xFF);

	SPI_FLASH_CS_HIGH();
	return Temp;
}
//=============================================================================
//函数名称:W25QXX_Read
//功能概要:读取SPI FLASH，在指定地址开始读取指定长度的数据
//参数名称:
//pBuffer:数据存储区
//ReadAddr:开始读取的地址(24bit)
//NumByteToRead:要读取的字节数(最大65535)
//函数返回:无
//=============================================================================
void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)
{
 	uint32_t i;
	SPI_FLASH_CS_LOW();                    		//使能器件

    SPI2_ReadWriteByte(W25X_ReadData);         	//发送读取命令0x03
    SPI2_ReadWriteByte((uint8_t)((ReadAddr&0xFF0000)>>16));	//发送24bit地址
    SPI2_ReadWriteByte((uint8_t)((ReadAddr&0xFF00)>>8));
    SPI2_ReadWriteByte((uint8_t)(ReadAddr&0xFF));
    for(i=0;i<NumByteToRead;i++)
	{
        pBuffer[i]=SPI2_ReadWriteByte(0XFF);   	//循环读数
    }
    pBuffer[i]='\0';

	SPI_FLASH_CS_HIGH();
}

void W25QXX_FAST_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)
{
 	uint16_t i;
	SPI_FLASH_CS_LOW();                         //使能器件

    SPI2_ReadWriteByte(W25X_FastReadData);    	//发送读取命令0x0B
    SPI2_ReadWriteByte((uint8_t)((ReadAddr&0xFF0000)>>16));	//发送24bit地址
    SPI2_ReadWriteByte((uint8_t)((ReadAddr&0xFF00)>>8));
    SPI2_ReadWriteByte((uint8_t)(ReadAddr&0xFF));
    SPI2_ReadWriteByte(0xFF);
    for(i=0;i<NumByteToRead;i++)
	{
        pBuffer[i]=SPI2_ReadWriteByte(0XFF);   //循环读数
    }
    pBuffer[i]='\0';

	SPI_FLASH_CS_HIGH();
}
//=============================================================================
//函数名称:W25QXX_Write_Page
//功能概要:在指定地址开始写入最大256字节的数据，SPI在一页(0~65535)内写入
//         少于256个字节的数据
//参数名称:
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!
//函数返回:无
//=============================================================================
void W25QXX_Write_Page(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{
 	uint16_t i;
    W25QXX_Write_Enable();                  	//SET WEL
	SPI_FLASH_CS_LOW();//                 		//使能器件

    SPI2_ReadWriteByte(W25X_PageProgram);      	//发送写页命令0x02
    SPI2_ReadWriteByte((uint8_t)((WriteAddr)>>16)); //发送24bit地址
    SPI2_ReadWriteByte((uint8_t)((WriteAddr)>>8));
    SPI2_ReadWriteByte((uint8_t)WriteAddr);

    for(i=0;i<NumByteToWrite;i++)
    	SPI2_ReadWriteByte(pBuffer[i]);			//循环写数

	SPI_FLASH_CS_HIGH();                		//取消片选
	W25QXX_Wait_Busy();					   		//等待写入结束
//	delay_us(1);
}
//=============================================================================
//函数名称:W25QXX_Write_NoCheck
//功能概要:无检验写SPI FLASH ,在指定地址开始写入指定长度的数据,但是要确保地址不越界!
//参数名称:
//					pBuffer:数据存储区
//					WriteAddr:开始写入的地址(24bit)
//					NumByteToWrite:要写入的字节数(最大65535)
//函数返回:无
//说明：  必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//        具有自动换页功能
//=============================================================================
void W25QXX_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
	uint16_t pageremain;

	if(NumByteToWrite == 0) return ;

	pageremain = PAGE_SIZE - WriteAddr % PAGE_SIZE; //单页剩余的字节数

	if(NumByteToWrite <= pageremain)
		pageremain = NumByteToWrite;			//不大于256个字节

	while(1)
	{
		W25QXX_Write_Page(pBuffer, WriteAddr, pageremain);
		if(NumByteToWrite == pageremain)break;	//写入结束了
	 	else //NumByteToWrite>pageremain
		{
			pBuffer			+= pageremain;
			WriteAddr		+= pageremain;

			NumByteToWrite	-= pageremain;		//减去已经写入了的字节数
			if(NumByteToWrite > PAGE_SIZE)
				pageremain 	= PAGE_SIZE; 		//一次可以写入256个字节
			else pageremain = NumByteToWrite; 	//不够256个字节了
		}
	}
}
//=============================================================================
//函数名称:W25QXX_Write
//功能概要:写SPI FLASH  在指定地址开始写入指定长度的数据
//参数名称:
// 最多能写写4096个字节，一个扇区的大小
//					pBuffer:数据存储区
//					WriteAddr:开始写入的地址(24bit)
//					NumByteToWrite:要写入的字节数(最大65535)
//函数返回:无
//=============================================================================
uint8_t W25QXX_BUF[SECTOR_SIZE] = {0};
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{
 	uint16_t i 		= 0;
	uint32_t pos	= WriteAddr / SECTOR_SIZE;		//扇区地址
	uint16_t off	= WriteAddr % SECTOR_SIZE;		//在扇区内的偏移
//	fprintf(USART1_STREAM,"pos:%d,off:%d,num:%d.\r\n",pos,off,NumByteToWrite);
	memset(W25QXX_BUF,0,sizeof(W25QXX_BUF));

	W25QXX_Read(W25QXX_BUF, pos*SECTOR_SIZE, SECTOR_SIZE);//读出整个扇区的内容
	for(i=0; i<NumByteToWrite; i++)	   		//复制
	{
		W25QXX_BUF[off+i] = pBuffer[i];
	}
//	fprintf(USART1_STREAM,"-->%s<--\r\n",W25QXX_BUF);
	/* 写入整个扇区 */
	W25QXX_Erase_Sector(pos);			//擦除这个扇区
	W25QXX_Write_NoCheck(W25QXX_BUF, pos*SECTOR_SIZE, SECTOR_SIZE);	// SECTOR_SIZE  strlen(W25QXX_BUF)
}

//=============================================================================
//函数名称:W25QXX_Erase_Chip
//功能概要:擦除整个芯片
//参数名称:
//函数返回:无
//说明     等待时间超长...
//=============================================================================
void W25QXX_Erase_Chip(void)
{
	W25QXX_Write_Enable();                  	//SET WEL
	W25QXX_Wait_Busy();
	SPI_FLASH_CS_LOW();                     	//使能器件

	SPI2_ReadWriteByte(W25X_ChipErase);        	//发送片擦除命令0xC7

	SPI_FLASH_CS_HIGH();                    	//取消片选
	W25QXX_Wait_Busy();   				   		//等待芯片擦除结束
}
//=============================================================================
//函数名称:W25QXX_Erase_Sector
//功能概要:擦除一个扇区
//参数名称:Dst_Addr:扇区地址 根据实际容量设置
//函数返回:无
//说明    擦除一个山区的最少时间:150ms
//=============================================================================
void W25QXX_Erase_Sector(uint32_t Dst_Addr)
{
	//监视falsh擦除情况,测试用
	Dst_Addr*=BUFFER_SIZE;
	W25QXX_Write_Enable();                  	//SET WEL
	W25QXX_Wait_Busy();
	SPI_FLASH_CS_LOW();                         //使能器件

	SPI2_ReadWriteByte(W25X_SectorErase);      	//发送扇区擦除指令 0x20
	SPI2_ReadWriteByte((uint8_t)((Dst_Addr)>>16));  //发送24bit地址
	SPI2_ReadWriteByte((uint8_t)((Dst_Addr)>>8));
	SPI2_ReadWriteByte((uint8_t)Dst_Addr);

	SPI_FLASH_CS_HIGH();                        //取消片选
	W25QXX_Wait_Busy();   				   		//等待擦除完成
}
//=============================================================================
//函数名称:W25QXX_Wait_Busy
//功能概要:等待空闲
//参数名称:无
//函数返回:无
//=============================================================================
void W25QXX_Wait_Busy(void)
{
	while((W25QXX_ReadSR()&0x01)==0x01);   		//等待BUSY位清空
}
//=============================================================================
//函数名称:W25QXX_PowerDown
//功能概要:进入掉电模式
//参数名称:无
//函数返回:无
//=============================================================================
void W25QXX_PowerDown(void)
{
	SPI_FLASH_CS_LOW();                     	//使能器件
	SPI2_ReadWriteByte(W25X_PowerDown);        	//发送掉电命令 0xB9
	SPI_FLASH_CS_HIGH();                      	//取消片选
	delay_us(3);                               	//等待TPD
}
//=============================================================================
//函数名称:W25QXX_WAKEUP
//功能概要:唤醒
//参数名称:无
//函数返回:无
//=============================================================================
void W25QXX_WAKEUP(void)
{
	SPI_FLASH_CS_LOW();                     	//使能器件
	SPI2_ReadWriteByte(W25X_ReleasePowerDown);  //send W25X_PowerDown command 0xAB
	SPI_FLASH_CS_HIGH();                        //取消片选
	delay_us(3);                              	//等待TRES1
}

/*
 * 获取程序运行版本号
 * 参数：版本号
 * 返回：无
 * 说明：在bootloader模式下，升级中使用
 */
void Get_Run_App_Version(uint8_t *read_version)
{
	uint8_t version[VERSION_SIZE];
	W25QXX_Read(version,OUT_FLASH_PARAM_START+BOOT_VERSION_OFFSET,VERSION_SIZE);
	uint8_t j = 0;
	for(uint8_t i=0; i<strlen(version); i++){
		if(version[i] == 0xFF){	// 去除0XFF空字符
			break;
		}else{
			read_version[j] = version[i];
			j++;
		}
	}
	read_version[j] = '\0';
}

/*
 * 获取版本信息
 * 参数：获取版本信息
 * 返回：无
 */
void Get_Upgrade_App_Version(uint8_t *read_version)
{
	uint8_t version[VERSION_SIZE] = {0};
	W25QXX_Read(version,OUT_FLASH_PARAM_START+APP_VERSION_OFFSET,VERSION_SIZE);
	uint8_t j = 0;
	for(uint8_t i=0; i<strlen(version); i++){
		if(version[i] == 0xFF){	// 去除0XFF空字符
			break;
		}else{
			read_version[j] = version[i];
			j++;
		}
	}
	read_version[j] = '\0';
//	fprintf(USART1_STREAM,"app ver:%s.\r\n",read_version);
}

/*
 * 获取升级包的大小
 * 参数：无
 * 返回：升级包大小
 */
uint32_t Get_Upgrade_PKG_size(void)
{
	uint8_t binSize[PKG_SIZE_SIZE]="";
	W25QXX_Read(binSize,OUT_FLASH_PARAM_START+PKG_SIZE_OFFSET,PKG_SIZE_SIZE);
//	fprintf(USART0_STREAM, "\r\n--%s--read bin size:%d.\r\n",appSize,atoi(appSize));
	return atoi(binSize);
}

/*
 * 解析出版本号中的对应版本数字
 * 参数：版本号v1.0.0.1
 * 返回：AppVersion结构体
 */
AppVersion Split_APP_Version(uint8_t *version_t)
{
	uint8_t version[30] = {0};
	strcpy(version,version_t);
	AppVersion ver={0};
	uint8_t pos = 0;
	uint8_t *p = strtok(version+1, ".");
	while(p)
	{
		switch(pos){
		case 0:ver.main_v = atoi(p);break;
		case 1:ver.sub_v = atoi(p);break;
		case 2:ver.rev_v = atoi(p);break;
//		case 3:ver.bld_v = atoi(p);break;
		case 3:
			strcpy(ver.sensor_v,p);
			ver.sensor_v[strlen(ver.sensor_v)] = '\0';
			break;
		default:break;
		}
		p = strtok(NULL, ".");
		pos++;
	}
	pos = 0;

	return ver;
}
/*
 * 获取应用程序版本号
 * 参数：无
 * 返回：版本号结构体
 */
//AppVersion Get_APP_Version_Struct()
//{
//	AppVersion ver={0};
//
//	uint8_t platform_version[VERSION_SIZE]={0};
//	memset(platform_version,0,VERSION_SIZE);
//	Get_APP_Version(platform_version);
//	ver = Split_APP_Version(platform_version);
////	fprintf(USART0_STREAM, "platfrom version main:%d,sub:%d,rev:%d,bld:%d\r\n",ver.main_v,ver.sub_v,\
////			ver.rev_v,ver.bld_v);
//
//	return ver;
//}
/*
 * 设置应用程序的版本号
 * 参数：
 * 返回：无
 * 说明：该版本号与平台是同步的
 */
void Set_Upgrade_App_Version(uint8_t *version)
{
	W25QXX_Write(version, OUT_FLASH_PARAM_START+APP_VERSION_OFFSET, VERSION_SIZE);
}
/*
 * 设置升级包的长度
 * 参数：长度
 * 返回：无
 */
void Set_Upgrade_PKG_size(uint32_t len)
{
	uint8_t len_s[PKG_SIZE_SIZE] = {0};
	sprintf(len_s,"%d",len);
	W25QXX_Write(len_s, OUT_FLASH_PARAM_START+PKG_SIZE_OFFSET, PKG_SIZE_SIZE);
}
/*
 * 设置http升级包下载地址
 * 参数：addr地址( https://qupinmate.oss-cn-beijing.aliyuncs.com/a/v1.0.0.6.bin )
 * 返回：无
 * 说明：仅存储路径( https://qupinmate.oss-cn-beijing.aliyuncs.com/a/ )
 */
void Set_Http_Download_Addr(uint8_t *addr)
{
	uint8_t addr_s[HTTP_ADDR_SIZE] = {0};
	// 先取有效地址：https://qupinmate.oss-cn-beijing.aliyuncs.com/a/
	for(uint8_t i=0;i<strlen(addr);i++){
		if(addr[i]=='/' && (addr[i+1]=='v' || addr[i+1]=='V')){
			strncpy(addr_s,addr,i+1);
			addr_s[i+2] = '\0';
			break;
		}
	}
//	fprintf(USART1_STREAM,"addr_s:%s\r\n",addr_s);
	/* 先擦除后写入 */
	W25QXX_Write(addr_s, OUT_FLASH_PARAM_START+HTTP_ADDR_OFFSET, HTTP_ADDR_SIZE);
}
/*
 * 设置AEB算法端参数
 * 参数1：数据；参数2：扇区ID
 */
void Set_AEB_Alg_Parameter(uint8_t *data,uint8_t id)
{
	uint32_t pos = (OUT_FLASH_ALGPPARM_START + id * SECTOR_SIZE) / SECTOR_SIZE;		//扇区地址
	W25QXX_Erase_Sector(pos);			//擦除这个扇区
	W25QXX_Write_NoCheck(data, pos*SECTOR_SIZE, ALG_PARA_SIZE);
}
/*
 * 设置AEB算法组ID，version
 */
void Set_AEB_Alg_Group_ID(uint8_t id)
{
	uint8_t data[ALG_PARA_GROUPID_SIZE] = {0};
	sprintf(data,"%d",id);
	W25QXX_Erase_Sector(OUT_FLASH_ALGPPARMGROUP_START/BUFFER_SIZE);			//擦除这个扇区
	W25QXX_Write_NoCheck(data, OUT_FLASH_ALGPPARMGROUP_START, ALG_PARA_GROUPID_SIZE);
}
/*
 * 获取http升级包下载地址
 * 参数：addr地址
 * 返回：无
 */
void Get_Http_Download_Addr(uint8_t *addr)
{
	uint8_t addr_t[HTTP_ADDR_SIZE] = {0};
	uint8_t j = 0;

	W25QXX_Read(addr_t, OUT_FLASH_PARAM_START+HTTP_ADDR_OFFSET, HTTP_ADDR_SIZE);

	for(uint8_t i=0; i<strlen(addr_t); i++){
		if(addr_t[i] == 0xFF){	// 去除0XFF空字符
			break;
		}else{
			addr[j] = addr_t[i];
			j++;
		}
	}
	addr[j] = '\0';
//	fprintf(USART1_STREAM, "http_addr:%s\r\n",addr);
}
/*
 * 获取AEB算法组ID，version
 */
uint8_t Get_AEB_Alg_Group_ID()
{
	uint8_t data[ALG_PARA_GROUPID_SIZE] = {0};
	W25QXX_Read(data, OUT_FLASH_ALGPPARMGROUP_START, ALG_PARA_GROUPID_SIZE);
	return atoi(data);
}
/*
 * 获取AEB 算法的参数
 * 参数1：数据；参数2：扇区ID
 */
void Get_AEB_Alg_Parameter(uint8_t *data,uint8_t id)
{
//	uint8_t buf[ALG_PARA_SIZE] = {0};
//	uint8_t j = 0;
	W25QXX_Read(data, OUT_FLASH_ALGPPARM_START + id * SECTOR_SIZE, ALG_PARA_SIZE);
//	fprintf(USART1_STREAM, "read:%s\r\n",buf);
//	for(uint8_t i=0; i<strlen(buf); i++){
//		if(buf[i] == 0xFF){	// 去除0XFF空字符
//			break;
//		}else{
//			data[j] = buf[i];
//			j++;
//		}
//	}
//	data[j] = '\0';
//	fprintf(USART1_STREAM, "read:%s\r\n",data);
	return ;
}

/*
 * 设置程序运行版本号
 * 参数：版本号
 * 返回：无
 * 说明：在bootloader模式下使用
 */
void Set_Run_App_Version(uint8_t *version)
{
	W25QXX_Write(version, OUT_FLASH_PARAM_START+BOOT_VERSION_OFFSET, VERSION_SIZE);
}

/*
 * 判断APP是否需要下载平台bin文件
 * 参数：平台版本号
 * 返回：1需要升级；0无需升级
 */
uint8_t Version_Compare(uint8_t *current_ver,uint8_t *upgrade_ver)
{
//	fprintf(USART1_STREAM,"%s,%s\r\n",current_ver,upgrade_ver);
	AppVersion current_v = Split_APP_Version(current_ver);//Get_Platform_APP_Version();
	AppVersion upgrade_v = Split_APP_Version(upgrade_ver);
	if(current_v.main_v < upgrade_v.main_v){
		return 1;
	}else if(current_v.sub_v < upgrade_v.sub_v){
		return 1;
	}else if(current_v.rev_v < upgrade_v.rev_v){
		return 1;
	}

	return 0;
}
/*
 * 打印输出外部FLASH的BIn文件数据
 * 参数：bin文件的大小
 * 返回：无
 */
void Print_Flash_bin_Content(uint32_t bin_size)
{
	uint32_t rw_address 	= OUT_FLASH_APP_START;
	uint8_t sector_num 		= (bin_size) / BUFFER_SIZE;
	uint16_t sector_remain 	= (bin_size) % BUFFER_SIZE;
	uint8_t read_data[BUFFER_SIZE];
	/* 输出整个BUFFER_SIZE大小的数据 */
	if(sector_num > 0){
		for(int k=0; k<sector_num; k++){
			memset(read_data, 0, BUFFER_SIZE);
			W25QXX_Read(read_data,rw_address,BUFFER_SIZE);
			rw_address += BUFFER_SIZE;

			for(int j=0;j<BUFFER_SIZE;j++)	// print
				fprintf(USART1_STREAM, "%02X",read_data[j]);
		}
	}
	if(sector_remain > 0){
		memset(read_data, 0, BUFFER_SIZE);
		/* 最后输出剩余大小的数据 */
		W25QXX_Read(read_data,rw_address,sector_remain);
		for(int j=0;j<sector_remain;j++)	// print
			fprintf(USART1_STREAM, "%02X",read_data[j]);
	}
}
/*
 * 设置设备SN号
 * 参数：SN号
 * 返回：无
 */
void Set_Device_SN(uint8_t *sn)
{
	W25QXX_Write(sn,OUT_FLASH_SN_START+DEV_SN_OFFSET,DEV_SN_SIZE);
}

/*
 * 获取设备SN号
 * 参数：传入参数
 * 返回：0失败；1成功
 */
uint8_t Get_Device_SN(char * device_sn)
{
	uint8_t sn[DEV_SN_SIZE] = {0};
	for(uint8_t i=0;i<10;i++){
		W25QXX_Read(sn, OUT_FLASH_SN_START+DEV_SN_OFFSET, DEV_SN_SIZE);
//		fprintf(USART1_STREAM,"sn:%s,len:%d\r\n",sn,strlen(sn));
		uint8_t j = 0;
		for(uint8_t i=0; i<strlen(sn); i++){
			if(sn[i] == 0xFF){	// 去除0XFF空字符
				break;
			}else{
				device_sn[j] = sn[i];
				j++;
			}
		}
		device_sn[j] = '\0';
		if(strlen(device_sn) > 8){
//			fprintf(USART1_STREAM,"sn:%s,%d\r\n",device_sn,strlen(device_sn));
			return 1;
		}
		memset(device_sn,0,sizeof(device_sn));
	}
	strcpy(device_sn,"");	// EC200UCNAAR02A01M08
	return 0;
}



