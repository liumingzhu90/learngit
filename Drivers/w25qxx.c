/******************** (C) COPYRIGHT  Դ�ع����� ********************************
 * �ļ���  ��w25qxx.c
 * ����    ����ɶ�W25XX��flash��ɶ�д����
 * ����    ��lmz
 * �汾����: 2021-08-30
 * Ӳ������  :PC12-/CS   PC9--CLK   PC10--MISO  PC11--MOSI
 * ���Է�ʽ��KungFu KF32DP2
**********************************************************************************/
#include "w25qxx.h"
#include "upgrade_common.h"
#include "spi.h"
#include "usart.h"
#include "stdlib.h"
#include "malloc.h"
uint16_t W25QXX_TYPE=W25X40B;					//Ĭ����W25X40B

/* -----------------------�ֲ���������------------------------------- */
uint16_t  W25QXX_ReadID(void);  	    	//��ȡFLASH ID
uint8_t	 W25QXX_ReadSR(void);        		//��ȡ״̬�Ĵ���
void W25QXX_Write_SR(uint8_t sr);  			//д״̬�Ĵ���
void W25QXX_Write_Enable(void);  			//дʹ��
void W25QXX_Write_Disable(void);			//д����
void W25QXX_FAST_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);		// ���ٶ�����
void W25QXX_Wait_Busy(void);           		//�ȴ�����
void W25QXX_PowerDown(void);        		//�������ģʽ
void W25QXX_WAKEUP(void);					//����
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
//��������:W25QXX_Init
//���ܸ�Ҫ:��ʼ��SPI FLASH��IO(/CS)��
//��������:��
//��������:��
//=============================================================================
void W25QXX_Init(void)
{
	GPIO_SPI2();
	SPI_FLASH_CS_HIGH(); 						//SPI FLASH��ѡ��
	SPI2_Init();		   						//��ʼ��SPI
	W25QXX_TYPE=W25QXX_ReadID();				//��ȡFLASH ID.
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
//��������:W25QXX_ReadSR
//���ܸ�Ҫ:��ȡW25QXX��״̬�Ĵ���
//��������:��
//��������:��
//˵����
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1,æ;0,����)
//Ĭ��:0x00
//=============================================================================
uint8_t W25QXX_ReadSR(void)
{
	uint8_t byte=0;
	SPI_FLASH_CS_LOW();							//ʹ������

	SPI2_ReadWriteByte(W25X_ReadStatusReg);    	//���Ͷ�ȡ״̬�Ĵ�������0x05
	byte=SPI2_ReadWriteByte(0Xff);             	//��ȡһ���ֽ�

	SPI_FLASH_CS_HIGH();						//ȡ��Ƭѡ
	return byte;
}
//=============================================================================
//��������:W25QXX_Write_SR
//���ܸ�Ҫ:дW25QXX״̬�Ĵ���
//��������:sr ״̬�Ĵ���
//��������:��
//˵����     ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д!!!
//=============================================================================
void W25QXX_Write_SR(uint8_t sr)
{
	SPI_FLASH_CS_LOW();                         //ʹ������
	SPI2_ReadWriteByte(W25X_WriteStatusReg);   	//����дȡ״̬�Ĵ�������0x01
	SPI2_ReadWriteByte(sr);               		//д��һ���ֽ�
	SPI_FLASH_CS_HIGH();                        //ȡ��Ƭѡ
}
//=============================================================================
//��������:W25QXX_Write_Enable
//���ܸ�Ҫ:W25QXXдʹ��
//��������:��
//��������:��
//˵����     ��WEL��λ
//=============================================================================
void W25QXX_Write_Enable(void)
{
	SPI_FLASH_CS_LOW();                        	//ʹ������
    SPI2_ReadWriteByte(W25X_WriteEnable);      	//����дʹ��0x06
	SPI_FLASH_CS_HIGH();                     	//ȡ��Ƭѡ
}
//=============================================================================
//��������:W25QXX_Write_Disable
//���ܸ�Ҫ:W25QXXд��ֹ
//��������:��
//��������:��
//˵����     ��WEL����
//=============================================================================
void W25QXX_Write_Disable(void)
{
	SPI_FLASH_CS_LOW();                     	//ʹ������
    SPI2_ReadWriteByte(W25X_WriteDisable);     	//����д��ָֹ��0x04
	SPI_FLASH_CS_HIGH();                   		//ȡ��Ƭѡ
}
//=============================================================================
//��������:W25QXX_ReadID
//���ܸ�Ҫ:��ȡоƬID
//��������:��
//��������:��
//=============================================================================
uint16_t W25QXX_ReadID(void)
{
	uint16_t Temp = 0;
	SPI_FLASH_CS_LOW();

	SPI2_ReadWriteByte(W25X_ManufactDeviceID);	//���Ͷ�ȡID����
	SPI2_ReadWriteByte(0xFF);					//Dummy
	SPI2_ReadWriteByte(0xFF);
	SPI2_ReadWriteByte(0x00);
	Temp|=SPI2_ReadWriteByte(0xFF)<<8;
	Temp|=SPI2_ReadWriteByte(0xFF);

	SPI_FLASH_CS_HIGH();
	return Temp;
}
//=============================================================================
//��������:W25QXX_Read
//���ܸ�Ҫ:��ȡSPI FLASH����ָ����ַ��ʼ��ȡָ�����ȵ�����
//��������:
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
//��������:��
//=============================================================================
void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)
{
 	uint32_t i;
	SPI_FLASH_CS_LOW();                    		//ʹ������

    SPI2_ReadWriteByte(W25X_ReadData);         	//���Ͷ�ȡ����0x03
    SPI2_ReadWriteByte((uint8_t)((ReadAddr&0xFF0000)>>16));	//����24bit��ַ
    SPI2_ReadWriteByte((uint8_t)((ReadAddr&0xFF00)>>8));
    SPI2_ReadWriteByte((uint8_t)(ReadAddr&0xFF));
    for(i=0;i<NumByteToRead;i++)
	{
        pBuffer[i]=SPI2_ReadWriteByte(0XFF);   	//ѭ������
    }
    pBuffer[i]='\0';

	SPI_FLASH_CS_HIGH();
}

void W25QXX_FAST_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)
{
 	uint16_t i;
	SPI_FLASH_CS_LOW();                         //ʹ������

    SPI2_ReadWriteByte(W25X_FastReadData);    	//���Ͷ�ȡ����0x0B
    SPI2_ReadWriteByte((uint8_t)((ReadAddr&0xFF0000)>>16));	//����24bit��ַ
    SPI2_ReadWriteByte((uint8_t)((ReadAddr&0xFF00)>>8));
    SPI2_ReadWriteByte((uint8_t)(ReadAddr&0xFF));
    SPI2_ReadWriteByte(0xFF);
    for(i=0;i<NumByteToRead;i++)
	{
        pBuffer[i]=SPI2_ReadWriteByte(0XFF);   //ѭ������
    }
    pBuffer[i]='\0';

	SPI_FLASH_CS_HIGH();
}
//=============================================================================
//��������:W25QXX_Write_Page
//���ܸ�Ҫ:��ָ����ַ��ʼд�����256�ֽڵ����ݣ�SPI��һҳ(0~65535)��д��
//         ����256���ֽڵ�����
//��������:
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!
//��������:��
//=============================================================================
void W25QXX_Write_Page(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{
 	uint16_t i;
    W25QXX_Write_Enable();                  	//SET WEL
	SPI_FLASH_CS_LOW();//                 		//ʹ������

    SPI2_ReadWriteByte(W25X_PageProgram);      	//����дҳ����0x02
    SPI2_ReadWriteByte((uint8_t)((WriteAddr)>>16)); //����24bit��ַ
    SPI2_ReadWriteByte((uint8_t)((WriteAddr)>>8));
    SPI2_ReadWriteByte((uint8_t)WriteAddr);

    for(i=0;i<NumByteToWrite;i++)
    	SPI2_ReadWriteByte(pBuffer[i]);			//ѭ��д��

	SPI_FLASH_CS_HIGH();                		//ȡ��Ƭѡ
	W25QXX_Wait_Busy();					   		//�ȴ�д�����
//	delay_us(1);
}
//=============================================================================
//��������:W25QXX_Write_NoCheck
//���ܸ�Ҫ:�޼���дSPI FLASH ,��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//��������:
//					pBuffer:���ݴ洢��
//					WriteAddr:��ʼд��ĵ�ַ(24bit)
//					NumByteToWrite:Ҫд����ֽ���(���65535)
//��������:��
//˵����  ����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//        �����Զ���ҳ����
//=============================================================================
void W25QXX_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
	uint16_t pageremain;

	if(NumByteToWrite == 0) return ;

	pageremain = PAGE_SIZE - WriteAddr % PAGE_SIZE; //��ҳʣ����ֽ���

	if(NumByteToWrite <= pageremain)
		pageremain = NumByteToWrite;			//������256���ֽ�

	while(1)
	{
		W25QXX_Write_Page(pBuffer, WriteAddr, pageremain);
		if(NumByteToWrite == pageremain)break;	//д�������
	 	else //NumByteToWrite>pageremain
		{
			pBuffer			+= pageremain;
			WriteAddr		+= pageremain;

			NumByteToWrite	-= pageremain;		//��ȥ�Ѿ�д���˵��ֽ���
			if(NumByteToWrite > PAGE_SIZE)
				pageremain 	= PAGE_SIZE; 		//һ�ο���д��256���ֽ�
			else pageremain = NumByteToWrite; 	//����256���ֽ���
		}
	}
}
//=============================================================================
//��������:W25QXX_Write
//���ܸ�Ҫ:дSPI FLASH  ��ָ����ַ��ʼд��ָ�����ȵ�����
//��������:
// �����дд4096���ֽڣ�һ�������Ĵ�С
//					pBuffer:���ݴ洢��
//					WriteAddr:��ʼд��ĵ�ַ(24bit)
//					NumByteToWrite:Ҫд����ֽ���(���65535)
//��������:��
//=============================================================================
uint8_t W25QXX_BUF[SECTOR_SIZE] = {0};
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{
 	uint16_t i 		= 0;
	uint32_t pos	= WriteAddr / SECTOR_SIZE;		//������ַ
	uint16_t off	= WriteAddr % SECTOR_SIZE;		//�������ڵ�ƫ��
//	fprintf(USART1_STREAM,"pos:%d,off:%d,num:%d.\r\n",pos,off,NumByteToWrite);
	memset(W25QXX_BUF,0,sizeof(W25QXX_BUF));

	W25QXX_Read(W25QXX_BUF, pos*SECTOR_SIZE, SECTOR_SIZE);//������������������
	for(i=0; i<NumByteToWrite; i++)	   		//����
	{
		W25QXX_BUF[off+i] = pBuffer[i];
	}
//	fprintf(USART1_STREAM,"-->%s<--\r\n",W25QXX_BUF);
	/* д���������� */
	W25QXX_Erase_Sector(pos);			//�����������
	W25QXX_Write_NoCheck(W25QXX_BUF, pos*SECTOR_SIZE, SECTOR_SIZE);	// SECTOR_SIZE  strlen(W25QXX_BUF)
}

//=============================================================================
//��������:W25QXX_Erase_Chip
//���ܸ�Ҫ:��������оƬ
//��������:
//��������:��
//˵��     �ȴ�ʱ�䳬��...
//=============================================================================
void W25QXX_Erase_Chip(void)
{
	W25QXX_Write_Enable();                  	//SET WEL
	W25QXX_Wait_Busy();
	SPI_FLASH_CS_LOW();                     	//ʹ������

	SPI2_ReadWriteByte(W25X_ChipErase);        	//����Ƭ��������0xC7

	SPI_FLASH_CS_HIGH();                    	//ȡ��Ƭѡ
	W25QXX_Wait_Busy();   				   		//�ȴ�оƬ��������
}
//=============================================================================
//��������:W25QXX_Erase_Sector
//���ܸ�Ҫ:����һ������
//��������:Dst_Addr:������ַ ����ʵ����������
//��������:��
//˵��    ����һ��ɽ��������ʱ��:150ms
//=============================================================================
void W25QXX_Erase_Sector(uint32_t Dst_Addr)
{
	//����falsh�������,������
	Dst_Addr*=BUFFER_SIZE;
	W25QXX_Write_Enable();                  	//SET WEL
	W25QXX_Wait_Busy();
	SPI_FLASH_CS_LOW();                         //ʹ������

	SPI2_ReadWriteByte(W25X_SectorErase);      	//������������ָ�� 0x20
	SPI2_ReadWriteByte((uint8_t)((Dst_Addr)>>16));  //����24bit��ַ
	SPI2_ReadWriteByte((uint8_t)((Dst_Addr)>>8));
	SPI2_ReadWriteByte((uint8_t)Dst_Addr);

	SPI_FLASH_CS_HIGH();                        //ȡ��Ƭѡ
	W25QXX_Wait_Busy();   				   		//�ȴ��������
}
//=============================================================================
//��������:W25QXX_Wait_Busy
//���ܸ�Ҫ:�ȴ�����
//��������:��
//��������:��
//=============================================================================
void W25QXX_Wait_Busy(void)
{
	while((W25QXX_ReadSR()&0x01)==0x01);   		//�ȴ�BUSYλ���
}
//=============================================================================
//��������:W25QXX_PowerDown
//���ܸ�Ҫ:�������ģʽ
//��������:��
//��������:��
//=============================================================================
void W25QXX_PowerDown(void)
{
	SPI_FLASH_CS_LOW();                     	//ʹ������
	SPI2_ReadWriteByte(W25X_PowerDown);        	//���͵������� 0xB9
	SPI_FLASH_CS_HIGH();                      	//ȡ��Ƭѡ
	delay_us(3);                               	//�ȴ�TPD
}
//=============================================================================
//��������:W25QXX_WAKEUP
//���ܸ�Ҫ:����
//��������:��
//��������:��
//=============================================================================
void W25QXX_WAKEUP(void)
{
	SPI_FLASH_CS_LOW();                     	//ʹ������
	SPI2_ReadWriteByte(W25X_ReleasePowerDown);  //send W25X_PowerDown command 0xAB
	SPI_FLASH_CS_HIGH();                        //ȡ��Ƭѡ
	delay_us(3);                              	//�ȴ�TRES1
}

/*
 * ��ȡ�������а汾��
 * �������汾��
 * ���أ���
 * ˵������bootloaderģʽ�£�������ʹ��
 */
void Get_Run_App_Version(uint8_t *read_version)
{
	uint8_t version[VERSION_SIZE];
	W25QXX_Read(version,OUT_FLASH_PARAM_START+BOOT_VERSION_OFFSET,VERSION_SIZE);
	uint8_t j = 0;
	for(uint8_t i=0; i<strlen(version); i++){
		if(version[i] == 0xFF){	// ȥ��0XFF���ַ�
			break;
		}else{
			read_version[j] = version[i];
			j++;
		}
	}
	read_version[j] = '\0';
}

/*
 * ��ȡ�汾��Ϣ
 * ��������ȡ�汾��Ϣ
 * ���أ���
 */
void Get_Upgrade_App_Version(uint8_t *read_version)
{
	uint8_t version[VERSION_SIZE] = {0};
	W25QXX_Read(version,OUT_FLASH_PARAM_START+APP_VERSION_OFFSET,VERSION_SIZE);
	uint8_t j = 0;
	for(uint8_t i=0; i<strlen(version); i++){
		if(version[i] == 0xFF){	// ȥ��0XFF���ַ�
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
 * ��ȡ�������Ĵ�С
 * ��������
 * ���أ���������С
 */
uint32_t Get_Upgrade_PKG_size(void)
{
	uint8_t binSize[PKG_SIZE_SIZE]="";
	W25QXX_Read(binSize,OUT_FLASH_PARAM_START+PKG_SIZE_OFFSET,PKG_SIZE_SIZE);
//	fprintf(USART0_STREAM, "\r\n--%s--read bin size:%d.\r\n",appSize,atoi(appSize));
	return atoi(binSize);
}

/*
 * �������汾���еĶ�Ӧ�汾����
 * �������汾��v1.0.0.1
 * ���أ�AppVersion�ṹ��
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
 * ��ȡӦ�ó���汾��
 * ��������
 * ���أ��汾�Žṹ��
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
 * ����Ӧ�ó���İ汾��
 * ������
 * ���أ���
 * ˵�����ð汾����ƽ̨��ͬ����
 */
void Set_Upgrade_App_Version(uint8_t *version)
{
	W25QXX_Write(version, OUT_FLASH_PARAM_START+APP_VERSION_OFFSET, VERSION_SIZE);
}
/*
 * �����������ĳ���
 * ����������
 * ���أ���
 */
void Set_Upgrade_PKG_size(uint32_t len)
{
	uint8_t len_s[PKG_SIZE_SIZE] = {0};
	sprintf(len_s,"%d",len);
	W25QXX_Write(len_s, OUT_FLASH_PARAM_START+PKG_SIZE_OFFSET, PKG_SIZE_SIZE);
}
/*
 * ����http���������ص�ַ
 * ������addr��ַ( https://qupinmate.oss-cn-beijing.aliyuncs.com/a/v1.0.0.6.bin )
 * ���أ���
 * ˵�������洢·��( https://qupinmate.oss-cn-beijing.aliyuncs.com/a/ )
 */
void Set_Http_Download_Addr(uint8_t *addr)
{
	uint8_t addr_s[HTTP_ADDR_SIZE] = {0};
	// ��ȡ��Ч��ַ��https://qupinmate.oss-cn-beijing.aliyuncs.com/a/
	for(uint8_t i=0;i<strlen(addr);i++){
		if(addr[i]=='/' && (addr[i+1]=='v' || addr[i+1]=='V')){
			strncpy(addr_s,addr,i+1);
			addr_s[i+2] = '\0';
			break;
		}
	}
//	fprintf(USART1_STREAM,"addr_s:%s\r\n",addr_s);
	/* �Ȳ�����д�� */
	W25QXX_Write(addr_s, OUT_FLASH_PARAM_START+HTTP_ADDR_OFFSET, HTTP_ADDR_SIZE);
}
/*
 * ����AEB�㷨�˲���
 * ����1�����ݣ�����2������ID
 */
void Set_AEB_Alg_Parameter(uint8_t *data,uint8_t id)
{
	uint32_t pos = (OUT_FLASH_ALGPPARM_START + id * SECTOR_SIZE) / SECTOR_SIZE;		//������ַ
	W25QXX_Erase_Sector(pos);			//�����������
	W25QXX_Write_NoCheck(data, pos*SECTOR_SIZE, ALG_PARA_SIZE);
}
/*
 * ����AEB�㷨��ID��version
 */
void Set_AEB_Alg_Group_ID(uint8_t id)
{
	uint8_t data[ALG_PARA_GROUPID_SIZE] = {0};
	sprintf(data,"%d",id);
	W25QXX_Erase_Sector(OUT_FLASH_ALGPPARMGROUP_START/BUFFER_SIZE);			//�����������
	W25QXX_Write_NoCheck(data, OUT_FLASH_ALGPPARMGROUP_START, ALG_PARA_GROUPID_SIZE);
}
/*
 * ��ȡhttp���������ص�ַ
 * ������addr��ַ
 * ���أ���
 */
void Get_Http_Download_Addr(uint8_t *addr)
{
	uint8_t addr_t[HTTP_ADDR_SIZE] = {0};
	uint8_t j = 0;

	W25QXX_Read(addr_t, OUT_FLASH_PARAM_START+HTTP_ADDR_OFFSET, HTTP_ADDR_SIZE);

	for(uint8_t i=0; i<strlen(addr_t); i++){
		if(addr_t[i] == 0xFF){	// ȥ��0XFF���ַ�
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
 * ��ȡAEB�㷨��ID��version
 */
uint8_t Get_AEB_Alg_Group_ID()
{
	uint8_t data[ALG_PARA_GROUPID_SIZE] = {0};
	W25QXX_Read(data, OUT_FLASH_ALGPPARMGROUP_START, ALG_PARA_GROUPID_SIZE);
	return atoi(data);
}
/*
 * ��ȡAEB �㷨�Ĳ���
 * ����1�����ݣ�����2������ID
 */
void Get_AEB_Alg_Parameter(uint8_t *data,uint8_t id)
{
//	uint8_t buf[ALG_PARA_SIZE] = {0};
//	uint8_t j = 0;
	W25QXX_Read(data, OUT_FLASH_ALGPPARM_START + id * SECTOR_SIZE, ALG_PARA_SIZE);
//	fprintf(USART1_STREAM, "read:%s\r\n",buf);
//	for(uint8_t i=0; i<strlen(buf); i++){
//		if(buf[i] == 0xFF){	// ȥ��0XFF���ַ�
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
 * ���ó������а汾��
 * �������汾��
 * ���أ���
 * ˵������bootloaderģʽ��ʹ��
 */
void Set_Run_App_Version(uint8_t *version)
{
	W25QXX_Write(version, OUT_FLASH_PARAM_START+BOOT_VERSION_OFFSET, VERSION_SIZE);
}

/*
 * �ж�APP�Ƿ���Ҫ����ƽ̨bin�ļ�
 * ������ƽ̨�汾��
 * ���أ�1��Ҫ������0��������
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
 * ��ӡ����ⲿFLASH��BIn�ļ�����
 * ������bin�ļ��Ĵ�С
 * ���أ���
 */
void Print_Flash_bin_Content(uint32_t bin_size)
{
	uint32_t rw_address 	= OUT_FLASH_APP_START;
	uint8_t sector_num 		= (bin_size) / BUFFER_SIZE;
	uint16_t sector_remain 	= (bin_size) % BUFFER_SIZE;
	uint8_t read_data[BUFFER_SIZE];
	/* �������BUFFER_SIZE��С������ */
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
		/* ������ʣ���С������ */
		W25QXX_Read(read_data,rw_address,sector_remain);
		for(int j=0;j<sector_remain;j++)	// print
			fprintf(USART1_STREAM, "%02X",read_data[j]);
	}
}
/*
 * �����豸SN��
 * ������SN��
 * ���أ���
 */
void Set_Device_SN(uint8_t *sn)
{
	W25QXX_Write(sn,OUT_FLASH_SN_START+DEV_SN_OFFSET,DEV_SN_SIZE);
}

/*
 * ��ȡ�豸SN��
 * �������������
 * ���أ�0ʧ�ܣ�1�ɹ�
 */
uint8_t Get_Device_SN(char * device_sn)
{
	uint8_t sn[DEV_SN_SIZE] = {0};
	for(uint8_t i=0;i<10;i++){
		W25QXX_Read(sn, OUT_FLASH_SN_START+DEV_SN_OFFSET, DEV_SN_SIZE);
//		fprintf(USART1_STREAM,"sn:%s,len:%d\r\n",sn,strlen(sn));
		uint8_t j = 0;
		for(uint8_t i=0; i<strlen(sn); i++){
			if(sn[i] == 0xFF){	// ȥ��0XFF���ַ�
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



