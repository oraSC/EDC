#include "beep.h" 


//��ʼ��PG7  Ϊ�����		   
//BEEP IO��ʼ��
void BEEP_Init(void)
{   
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);//ʹ��GPIOGʱ��
  
  //��ʼ����������Ӧ����GPIOG7
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
  GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��GPIO
	
  GPIO_ResetBits(GPIOF,GPIO_Pin_7);  //��������Ӧ����GPIOF8���ͣ� 
}






