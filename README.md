当前程序编译后flash起始地址为0x8000  需要配合bootloader程序才能启动
如果不使用bootloader程序启动，则修改KF32A151MQV.ld文件中第9行
  flash      : ORIGIN =  0x00008000, LENGTH = 0x00078000
  改成
  flash      : ORIGIN =  0x00000000, LENGTH = 0x00080000
 然后再编译工程下载到板卡上