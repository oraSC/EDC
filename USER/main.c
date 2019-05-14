#include "stm32f4xx.h"
#include "usart.h"
#include "usart3.h"
#include "delay.h"
#include "motor.h"
#include "lcd.h"
#include "dcmi.h"	
#include "ov2640.h"	
#include "pid.h"
#include "init.h"
#include "beep.h"


#define Width_Start		10
#define Width_End		150
#define Height_Start	10
#define Height_End		190
#define ENABLE_I		1
#define DISABLE_I		0


//displayModeģʽѡ��
#define RGB				1
#define BinaryZation	-1
#define NEW_frame		1
#define OLD_frame		0

//��ʼֵ
int displayMode;
u16 rgb_buf[WINDOW_HEIGHT][WINDOW_WIDTH]; 
u16 gray;

aim_t Aim[20];
int PWM_init_X[20];
int PWM_init_Y[20];
	
int success = 0;
int aim_routine[TASK_num][TASK_node_num] = {	{0},
							{AIM_2_index, 5.5*2, A_Task_Finish},						//����һ
							{AIM_5_index, 2.5*2, A_Task_Finish},						//�����
							{AIM_4_index, 5, AIM_5_index, 5, A_Task_Finish},			//������
							{buffer_1_index, -1, buffer_2_index, -1, buffer_4_index, 1, AIM_9_index, 2.5 * 2,  A_Task_Finish},						//������
							{AIM_2_index, -1, AIM_6_index, -1, buffer_4_index, 1, AIM_9_index, 2.5 * 2, A_Task_Finish},			
							{0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, 2.5*2, A_Task_Finish},
							{buffer_1_index, 1, buffer_2_index, 1, buffer_4_index, 1, buffer_3_index, 1,\
							buffer_1_index, 1, buffer_2_index, 1, buffer_4_index, 1, buffer_3_index, 1,\
							buffer_1_index, 1, buffer_2_index, 1, buffer_4_index, 1, buffer_3_index, 1,\
							buffer_1_index, 1, buffer_2_index, 1, buffer_4_index, 1, AIM_9_index, 2.5 * 2,A_Task_Finish}
							};     //��Ӧ��� - 1
int Task_index = TASK_1_index;
int *aim_index = aim_routine[TASK_1_index];
char have_ball = 0;
char task_6_num = 0;	//��¼task 6 ������Ŀ
							
//===================PID����==============//
pid_t pid_X;
pid_t pid_Y;


float PWM_X = 0;
float PWM_Y = 0;
u8 ov_frame_flag;  						//֡��
void pid_calculate(void);
u8 ov_frame = 0;

//��ʱ����
char success_enable_timing = 0;
char success_timeout = 0;				
char success_timing = 0;
u8 task_time = 0;
u8 on_task = 0;	
	
int main(void)
{

	
	u16 i,j;
	
	displayMode = RGB;
	ov_frame_flag = OLD_frame;
	
	sys_init();

	/*******************************************************************************/

	/*************************************���**************************************/
	motor_init(50);			//50HZ -> 20ms
//	TIM_SetCompare1(TIM4, 1100);	//�޸ıȽ�ֵ���޸�ռ�ձ�      ���pid����ֵ
//	TIM_SetCompare2(TIM4, 1100);	//�޸ıȽ�ֵ���޸�ռ�ձ�
	set_angle(PWM_init_X[AIM_2_index], PWM_init_Y[AIM_2_index]);
	
	/********************************pid*********************************************/
	pid_init(&pid_X, 3, 0.005, 10); //pid_init(pid_t *Pid, float Kp, float Ki, float Kd)
	pid_init(&pid_Y, 3, 0.005, 10);
	TIM3_init(2); //����֡���ж�
	BEEP_Init();
	
	

	while(1)
	{
		if(displayMode == BinaryZation)
		{
			on_task = 1;
		}	
		
		//�ȴ��µ�һ֡
		while(ov_frame_flag == OLD_frame);
		ov_frame_flag = OLD_frame;
		LCD_SetCursor(0,0);  
		LCD_WriteRAM_Prepare();		//��ʼд��GRAM
		for(i=0;i<WINDOW_HEIGHT;i++)
		{
			 
			for(j=0; j < WINDOW_WIDTH; j++)
			{
				if(j == WINDOW_WIDTH - 1)
				{
					LCD_SetCursor(0,i+1);  
					LCD_WriteRAM_Prepare();		//��ʼд��GRAM
				}
				
				
				/*************************************************��ֵ��*******************************************/
				
					
				gray=((rgb_buf[i][j]>>11)*19595+((rgb_buf[i][j]>>5)&0x3f)*38469 +(rgb_buf[i][j]&0x1f)*7472)>>16;  //�Ҷȼ��㡣��ʽ��ٶ�
				if(gray>=22)  //�̶���ֵ��ֵ��
				{			  
						//if(i>8&&i<136&&j<200&&j>16)  //�˴�����ͼ��Ѱ��С���������� ���� �����ĸ�������
						{
							if(j > pid_X.ball_max_X)	pid_X.ball_max_X = j;
							if(j < pid_X.ball_min_X) 	pid_X.ball_min_X = j;
						 
							if(i > pid_Y.ball_max_Y)	pid_Y.ball_max_Y = i;
							if(i < pid_Y.ball_min_Y) 	pid_Y.ball_min_Y = i;
					 
						}
						
						if(displayMode == BinaryZation)
						{
							LCD->LCD_RAM=WHITE;
							have_ball = 1;
						}
				}
				else
				{		
					if(displayMode == BinaryZation)
					{
						LCD->LCD_RAM=BLACK;
						
					}
					
				}
			
				/***********************************************RGB****************************************************/
				if(displayMode == RGB)
				{
					LCD->LCD_RAM=rgb_buf[i][j];
					pid_X.E_sum = 0;
					pid_Y.E_sum = 0;
					
				}
			}
			
		}
			pid_X.ball_center_X = (pid_X.ball_max_X + pid_X.ball_min_X) / 2;
			pid_Y.ball_center_Y = (pid_Y.ball_max_Y + pid_Y.ball_min_Y) / 2;     //ͨ���ĸ����������С������
			
		if(displayMode == BinaryZation)
		{
		//�ɹ�
		if(abs(pid_X.ball_center_X - Aim[*aim_index].X) <= Aim[*aim_index].success_distance \
			&& abs(pid_Y.ball_center_Y - Aim[*aim_index].Y) <= Aim[*aim_index].success_distance)
		{
			success++;
			if(success >= 7)
			{
				if(*aim_index < 10)
				{
					pid_X.Kp = 0;
					pid_Y.Kp = 0;
					pid_X.Kd = 0;
					pid_Y.Kd = 0;
				}
			}
			if(success >= 10)
			{
				pid_X.E_sum = 0;
				pid_Y.E_sum = 0;
				//Ŀ���
				if(*(aim_index + 1) >= 0)
				{
					success_enable_timing = 1;
					
					if(success_timeout == 1)
					{
						aim_index += 2;
						BEEP_ON;
						
						
						if(*aim_index == A_Task_Finish)
						{
							//ָ����һ������
							Task_index++;
							aim_index = aim_routine[Task_index];
							displayMode = RGB;
							on_task = 0;
							task_time = 0;
						}

						pid_X.E_sum = 0;
						pid_Y.E_sum = 0;
						success = 0;
						success_enable_timing = 0;
						success_timeout = 0;
						success_timing = 0;
					}
				
				}
				//�����
				else
				{
					aim_index += 2;
					if(*aim_index == A_Task_Finish)
					{
						//ָ����һ������
						Task_index++;
						aim_index = aim_routine[Task_index];
						displayMode = RGB;
					}

					pid_X.E_sum = 0;
					pid_Y.E_sum = 0;
					success = 0;
				}
			}
			
		}

		//����
		else if(abs(pid_X.ball_center_X - Aim[*aim_index].X) <= Aim[*aim_index].middle_distance \
			&& abs(pid_Y.ball_center_Y - Aim[*aim_index].Y) <= Aim[*aim_index].middle_distance)
		{

			pid_X.Kp = Aim[*aim_index].PID_p_x;
			pid_Y.Kp = Aim[*aim_index].PID_p_y;
			pid_X.Kd = Aim[*aim_index].PID_d_x;
			pid_Y.Kd = Aim[*aim_index].PID_d_y;

			//����
			success = 0;		
			success_enable_timing = 0;
			success_timeout = 0;
			success_timing = 0;

		}
		//Զ��
			else 
			{
//�ɹ�������һ�������ٶȿ��Խ���
//				pid_X.Kp = 2.0;
//				pid_Y.Kp = 2.0;
//				pid_X.Kd = 50;
//				pid_Y.Kd = 50;

				if(*aim_index == 5)
				{
					pid_X.Kp = 2;
					pid_Y.Kp = 2;
					pid_X.Kd = 50;
					pid_Y.Kd = 50;
				
				}
				else if(*aim_index == 9)
				{
					pid_X.Kp = 1.7;
					pid_Y.Kp = 1.7;
					pid_X.Kd = 50;
					pid_Y.Kd = 50;
				
				
				}
//				else if(*aim_index == 7)
//				{
//					pid_X.Kp = 1;
//					pid_Y.Kp = 1;
//					pid_X.Kd = 30;
//					pid_Y.Kd = 30;
//				
//				
//				}

//				else if(*aim_index == 8)
//				{
//					pid_X.Kp = 1.7;
//					pid_Y.Kp = 1.7;
//					pid_X.Kd = 50;
//					pid_Y.Kd = 50;
//				
//				
//				}
				else if(*aim_index == 3)
				{
					pid_X.Kp = 1;
					pid_Y.Kp = 1;
					pid_X.Kd = 40;
					pid_Y.Kd = 40;
				
				
				}
				else if(*aim_index == 6)
				{
					pid_X.Kp = 1.7;
					pid_Y.Kp = 1.7;
					pid_X.Kd = 35;
					pid_Y.Kd = 35;
				
				
				}
				else
				{
					pid_X.Kp = 1.7;
					pid_Y.Kp = 1.7;
					pid_X.Kd = 50;
					pid_Y.Kd = 50;
				}
				success = 0;
//				pid_X.E_sum = 0;
//				pid_Y.E_sum = 0;

			}
		
		}
			pid_X.ball_max_X = 0;
			pid_X.ball_min_X = 160;
			pid_Y.ball_max_Y = 0;
			pid_Y.ball_min_Y = 200;   //������������������ٴα������ֵ ��Сֵ
				
			//pid����
			pid_calculate();
			ov_frame++;

	}
}


void pid_calculate(void)
{

	char str1[30] = {0};
	char str2[30] = {0};
	char str3[30] = {0};
	char str4[30] = {0};

	
	
	
	if(displayMode == BinaryZation)
	{
		//����ƫ��
		pid_X.E_now = Aim[*aim_index].X - pid_X.ball_center_X;
		pid_Y.E_now = Aim[*aim_index].Y - pid_Y.ball_center_Y;
		
		//�����������
		pid_X.Pout = pid_X.Kp * pid_X.E_now;
		pid_Y.Pout = pid_Y.Kp * pid_Y.E_now;
		
		pid_X.Iout = pid_X.Ki * pid_X.E_sum;
		pid_Y.Iout = pid_Y.Ki * pid_Y.E_sum;	
		pid_X.E_sum += pid_X.E_now;
		pid_Y.E_sum += pid_Y.E_now;
		
		//�ж��Ѿ����ɲ��
		pid_X.Dout = pid_X.Kd * (pid_X.E_now - pid_X.E_last);
		pid_Y.Dout = pid_Y.Kd * (pid_Y.E_now - pid_Y.E_last);
		
		if(have_ball == 1)
		{
			
			PWM_X = PWM_init_X[*aim_index] + pid_X.Pout + pid_X.Iout + pid_X.Dout; 
			PWM_Y = PWM_init_Y[*aim_index] + pid_Y.Pout + pid_Y.Iout + pid_Y.Dout;
			
			if(PWM_X < 0) PWM_X = 1;
			else if(PWM_X > 150) PWM_X = 100;
			if(PWM_Y < 0) PWM_Y = 1;
			else if(PWM_Y > 150) PWM_Y = 100;
			
			
			set_angle(PWM_X, PWM_Y);
		}
		
		else
		{
			PWM_X = PWM_init_X[*aim_index];  
			PWM_Y = PWM_init_Y[*aim_index];
			pid_X.E_sum = 0;
			pid_Y.E_sum = 0;
			set_angle(70, 70);
			//set_angle(PWM_init_X[*aim_index], PWM_init_Y[*aim_index]);
		}    
		have_ball = 0;	
		
		pid_X.E_last = pid_X.E_now;  //KD
		pid_Y.E_last = pid_Y.E_now;
		
	}
	else 
	{
		PWM_X = PWM_init_X[*aim_index];
		PWM_Y = PWM_init_Y[*aim_index];
		sprintf(str1, "P_X: %d, %d", PWM_X, PWM_init_X[*aim_index]);
		sprintf(str2, "P_Y: %d, %d", PWM_Y, PWM_init_Y[*aim_index]);
		sprintf(str3, "b_X: %d A_X:%d", pid_X.ball_center_X, Aim[*aim_index].X);
		sprintf(str4, "b_Y: %d A_Y:%d", pid_Y.ball_center_Y, Aim[*aim_index].Y);
		
		LCD_ShowString(150, 80 , 240, 16, 16, "                    ");
		LCD_ShowString(150, 110 , 240, 16, 16, "                    ");

		LCD_ShowString(150, 80 , 240, 16, 16, str1);
		LCD_ShowString(150, 110 , 240, 16, 16, str2);

		LCD_ShowString(150, 140 , 240, 16, 16, "                    ");
		LCD_ShowString(150, 170 , 240, 16, 16, "                    ");
		LCD_ShowString(150, 140 , 240, 16, 16, str3);
		LCD_ShowString(150, 170 , 240, 16, 16, str4);


	}
	
		
		
		
		
		

		
	
	//TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

}



//DCMI�жϷ�����
void DCMI_IRQHandler(void)
{
	if(DCMI_GetITStatus(DCMI_IT_FRAME)==SET)//����һ֡ͼ��
	{
		
		DCMI_ClearITPendingBit(DCMI_IT_FRAME);//���֡�ж�
		
		ov_frame_flag = NEW_frame;
		
		//printf("%d  \n", ov_frame_flag);
		
	}
} 
void TIM3_IRQHandler(void)
{

	
	
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		BEEP_OFF;
		
		LCD_ShowNum(150+24*4, 20 , ov_frame, 2, 24);
		
		if(on_task == 1)
		{
			task_time++;
			LCD_ShowNum(150+24*3, 50, task_time / 2, 2, 24);
		
		}
		ov_frame = 0;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		
		
		
		//�ɹ���ʹ
		if(success_enable_timing == 1)
		{
			success_timing++;
			if(success_timing >= *(aim_index + 1))
			{
				success_timeout = 1;
			}
		
		}
		
		
		
	}



}


