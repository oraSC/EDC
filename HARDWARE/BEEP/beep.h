#ifndef __BEEP_H
#define __BEEP_H	 
#include "sys.h" 


#define BEEP_ON  GPIO_SetBits(GPIOG, GPIO_Pin_7)	     //�ߵ�ƽ ��
#define BEEP_OFF GPIO_ResetBits(GPIOG, GPIO_Pin_7)	     //�͵�ƽ ����


#define BEEP PGout(7)	// ����������IO 

void BEEP_Init(void);//��ʼ��		 				    
#endif

















