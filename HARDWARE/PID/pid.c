#include "pid.h"
#include "stm32f4xx.h"
#include "usart.h"

void pid_init(pid_t *Pid, float Kp, float Ki, float Kd)
{
	
	//��ʼ��pid����
	Pid->Pv = 0;
	Pid->Sv = 0;
	
	Pid->E_now = 0;
	Pid->E_last = 0;
	Pid->E_lastlast = 0;
	Pid->E_sum = 0;
	
	Pid->Kp = Kp;
	Pid->Ki = Ki;
	Pid->Kd = Kd;
	
	Pid->Pout = 0;
	Pid->Iout = 0;
	Pid->Dout = 0;

	//��ʼ��С��λ��
	Pid->ball_min_X = 160;
	Pid->ball_min_Y = 200;
	Pid->ball_max_X = 0;
	Pid->ball_max_Y = 0;
	Pid->ball_center_X = 80;
	Pid->ball_center_Y = 100;
	Pid->ball_last_center_X = -1;
	Pid->ball_last_center_Y = -1;
	

}

#define T3_PRESCALER	84
#define T3_CLK			84000000

void TIM3_init(int Fre)
{
	u32 prescaler = T3_PRESCALER;
	u32 period = T3_CLK / prescaler / Fre - 1;
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitTypeStructure;
	
	//1.ʹ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	//2.��ʱ��������ʽ
	TIM_TimeBaseInitStructure.TIM_Prescaler = prescaler - 1;
	TIM_TimeBaseInitStructure.TIM_Period = period;    // 30HZ
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	//3.�����ж�
	NVIC_InitTypeStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitTypeStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitTypeStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitTypeStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitTypeStructure);
	
	//4.����ʱ�������ж�
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	//5.ʹ�ܶ�ʱ��
	TIM_Cmd(TIM3, ENABLE);


}













	


