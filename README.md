从内部FLASH中启动地址配置，需要2部操作；

步骤1：需要修改KF32A152MQV.ld中的第9行信息即可实现：

设置bootloader在内部FLASH启动地址
flash      : ORIGIN =  0x00000000, LENGTH = 0x00010000
注：起始地址为0，大小为64KB。

设置应用程序在内部FLASH启动地址
flash      : ORIGIN =  0x00010000, LENGTH = 0x00042000
注：起始地址为0x10000，大小为200KB。

步骤2：修了个如上配置后，仍需需要如下操作才能使其生效
项目属性-C/C++构建-设置-工具设置-通过设定-芯片脚本文件
将内容改成 -T"../KF32A152MQV.ld"



默认配置
flash      : ORIGIN =  0x00000000, LENGTH = 0x00080000
注：起始地址为0，大小为512KB。

---------------------------------查询指令-----------------------------------
通过USART1设置，115200，8，1，0，0
(1) <ZKHYCHK*VERSION>							获取版本号
(2) <ZKHYSET*VERSION:V1.1.0.S1C6,V1.1.0.S1C6>	设置版本号
(3)	<ZKHYSET*DEVICESN:EC200UCNAAR02A01M08>  	设置SN号
(4) <ZKHYCHK*DEVICESN> 							查询SN号
(5) <ZKHYSET*PACKAGESIZE:V1.1.0.S1C6,32000> 	控制串口升级
(6) <ZKHYSET*ERASESECTOR>						清空配置参数，可理解为恢复到出厂设置，有风险需谨慎 
(7) <ZKHYCHK*UPDATE:NEWEST>						OTA检测更新，若有更新会直接更新
(8) <ZKHYSET*UPDATE:V1.1.0.L1C6,60436>			手动设置OTA升级到固定版本，参数1：版本号；参数2：升级包大小
