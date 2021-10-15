
#ifndef HWREG
#define HWREG(x) (*((volatile unsigned int *)(x)))
#endif
//#################     auto value variable from tool 	 #############//
extern unsigned int __text_end__;
extern unsigned int __bss_start__;
extern unsigned int __bss_end__;
extern unsigned int __data_start__;

//#define Project_Type__cplusplus
#ifdef  Project_Type__cplusplus
extern unsigned int __init_class_start;
extern unsigned int __init_class_end;
#endif
//####################################################################//
extern int main();
//####################################################################//
void startup()
{
	unsigned int *s,*begin,*end;	
#ifdef  Project_Type__cplusplus
	void (*pf)(void);
#endif
//############# 		init work for the chip 		    #############//
//	HWREG(0x40000000)=1;
	HWREG(0x40000000)=0;
//############# init variable who have initialization	#############//
	s = (unsigned int*)&__text_end__;		
	begin = (unsigned int*)&__data_start__;
	end = (unsigned int*)&__bss_start__;
	while(begin < end)
		*begin++ = *s++;
//############# init class who have initialization(C++)	#############//
#ifdef Project_Type__cplusplus
	begin = (unsigned int*)&__init_class_start;
	end = (unsigned int*)&__init_class_end;
	while(begin<end)
	{
		pf=(void *)(*begin++);
		pf();
	}
#endif		
//############# init variable who have no initialization	#############//
#if 1	 //  0 not init this type variable
	begin = (unsigned int*)&__bss_start__;
	end = (unsigned int*)&__bss_end__;
	while(begin < end)
		*begin++ = 0;
#endif		
//############# begin to run main function	#############//		
	main();
}
//####################################################################//