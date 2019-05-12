#ifndef __BEEP_H
#define __BEEP_H	 
#include "sys.h" 


#define BEEP_ON  GPIO_SetBits(GPIOG, GPIO_Pin_7)	     //高电平 响
#define BEEP_OFF GPIO_ResetBits(GPIOG, GPIO_Pin_7)	     //低电平 不响


#define BEEP PGout(7)	// 蜂鸣器控制IO 

void BEEP_Init(void);//初始化		 				    
#endif

















