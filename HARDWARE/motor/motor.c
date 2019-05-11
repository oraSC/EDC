//author:ora
//email:1301912993@qq.com

#include "motor.h"

#define PRESCALER	84
#define T14_CLK	84000000


/*
*˵����ʹ�� TIM4, TIM4_CH1 -> PD12 , TIM4_CH2 -> PD13 
*/
void motor_init(u32 Fre)
{
	
	u32 prescaler = PRESCALER;
	u32 period = T14_CLK / prescaler / Fre - 1;
	
	GPIO_InitTypeDef	GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	//1.ʹ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	//2.GPIOF9����Ϊ��ʱ��4
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);	

	
	//3.��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//3.��ʱ����ʼ��
	TIM_TimeBaseInitStructure.TIM_Prescaler = prescaler - 1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = period;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

	//4.��ʼ��PWM���
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	//TIM_OCInitStructure.TIM_Pulse = pulse;
	//CH1��CH2
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	
	//4.ʹ��
	TIM_OC1PreloadConfig(TIM4, TIM_OCFast_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCFast_Enable);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	
	
}

void set_angle(float x_angle, float y_angle)
{
	//��������
	int PWM_x;
	int PWM_y;
	
	
	
	PWM_x = x_angle / (float)180 * 2000 + 500;
	PWM_y = y_angle / (float)180 * 2000 + 500;
	
	TIM_SetCompare1(TIM4,PWM_x);	//�޸ıȽ�ֵ���޸�ռ�ձ�      ���pid����ֵ
	TIM_SetCompare2(TIM4,PWM_y);	//�޸ıȽ�ֵ���޸�ռ�ձ�
	
}





