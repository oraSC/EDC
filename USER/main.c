#include "stm32f4xx.h"
#include "usart.h"
#include "usart3.h"
#include "delay.h"
#include "motor.h"
#include "lcd.h"
#include "dcmi.h"	
#include "ov2640.h"	
#include "pid.h"

#define WINDOW_WIDTH 	160
#define WINDOW_HEIGHT	200
#define Width_Start		10
#define Width_End		150
#define Height_Start	10
#define Height_End		190


//displayModeģʽѡ��
#define RGB				1
#define BinaryZation	-1



//��ʼֵ
int displayMode;
u16 rgb_buf[WINDOW_HEIGHT][WINDOW_WIDTH]; 
u16 gray;
u16 hang=0;




//===================PID����==============//
pid_t pid_X;
pid_t pid_Y;

int PWM_init_X = 1100;
int PWM_init_Y = 1100;

//Ŀ���
float Aim_X = 80;
float Aim_Y = 100;
int PWM_X = 0;
int PWM_Y = 0;


int main(void)
{

	
	u16 i,j;
	displayMode = BinaryZation;

	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);      //��ʼ����ʱ����
	uart_init(115200);		//��ʼ�����ڲ�����Ϊ115200
	usart3_init(9600);
 	LCD_Init();           //��ʼ��LCD FSMC�ӿ�
	POINT_COLOR=RED;      //������ɫ����ɫ
	
	
	/********************************����ͷ׼��*********************************/
	while(OV2640_Init())//��ʼ��OV2640
	{
		LCD_ShowString(30,130,240,16,16,"OV2640 ERR");
		delay_ms(200);
	    LCD_Fill(30,130,239,170,WHITE);
		delay_ms(200);
	}	
//	OV2640_ImageSize_Set(1600, 1200);
	OV2640_ImageWin_Set(0, 0, 800, 600);
	
	LCD_ShowString(30,130,200,16,16,"OV2640 OK");  
	delay_ms(1000);
	OV2640_RGB565_Mode();	//JPEGģʽ
	My_DCMI_Init();			//DCMI����
	DCMI_DMA_Init((u32)rgb_buf,0, sizeof(rgb_buf)/4,DMA_MemoryDataSize_HalfWord,DMA_MemoryInc_Enable);//DCMI DMA����
	//DCMI_DMA_Init((u32)&LCD->LCD_RAM,0, 1,DMA_MemoryDataSize_HalfWord,DMA_MemoryInc_Disable);//DCMI DMA??  
 	printf("%d", lcddev.width);
	printf("%d", lcddev.height); 
	//OV2640_OutSize_Set( lcddev.width, lcddev.height );
	//OV2640_OutSize_Set(WINDOW_WIDTH, WINDOW_HEIGHT); 
	OV2640_OutSize_Set(WINDOW_WIDTH, WINDOW_HEIGHT);
	
	//OV2640_OutSize_Set(lcddev.width, lcddev.height);
	
	DCMI_Start(); 			//�������� 
	/*******************************************************************************/
	
	/*************************************���**************************************/
	motor_init(50);			//50HZ -> 20ms
	TIM_SetCompare1(TIM4, PWM_init_X);	//�޸ıȽ�ֵ���޸�ռ�ձ�      ���pid����ֵ
	TIM_SetCompare2(TIM4, PWM_init_Y);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		
	
	/********************************pid*********************************************/
	pid_init(&pid_X, 3, 0.001, 40); //pid_init(pid_t *Pid, float Kp, float Ki, float Kd)
	pid_init(&pid_Y, 3, 0.001, 40);
	TIM3_init(30); //30HZ -> 33ms ����ͷ���30֡
	
	//while(1);
	while(1)
	{
		hang=0;
		LCD_SetCursor(0,0);  
		LCD_WriteRAM_Prepare();		//��ʼд��GRAM
		for(i=0;i<WINDOW_HEIGHT;i++)
		{
			
			for(j=0; j < WINDOW_WIDTH; j++)
			{
				if(j == WINDOW_WIDTH - 1)
				{
					hang++;
					LCD_SetCursor(0,i+1);  
					LCD_WriteRAM_Prepare();		//��ʼд��GRAM
				}
				
				
				/*************************************************��ֵ��*******************************************/
				if(displayMode == BinaryZation)
				{
					//�ü�
					if(i < Height_Start | i > Height_End | j < Width_Start || j > Width_End)
					{
						LCD->LCD_RAM=WHITE;
						continue;
					}
					gray=((rgb_buf[i][j]>>11)*19595+((rgb_buf[i][j]>>5)&0x3f)*38469 +(rgb_buf[i][j]&0x1f)*7472)>>16;  //�Ҷȼ��㡣��ʽ��ٶ�
					if(gray>=23)  //�̶���ֵ��ֵ��
					{			  
							//if(i>8&&i<136&&j<200&&j>16)  //�˴�����ͼ��Ѱ��С���������� ���� �����ĸ�������
							{
								if(j > pid_X.ball_max_X)	pid_X.ball_max_X = j;
								if(j < pid_X.ball_min_X) 	pid_X.ball_min_X = j;
							 
								if(i > pid_Y.ball_max_Y)	pid_Y.ball_max_Y = i;
								if(i < pid_Y.ball_min_Y) 	pid_Y.ball_min_Y = i;
						 
							}
							
							
							LCD->LCD_RAM=WHITE;
							
					}
					else
					{		
						
						LCD->LCD_RAM=BLACK;
					}
				}
				/***********************************************RGB****************************************************/
				else if(displayMode == RGB)
				{
					LCD->LCD_RAM=rgb_buf[i][j];
				}
			}
			
		}
			  pid_X.ball_center_X = (pid_X.ball_max_X + pid_X.ball_min_X) / 2;
			  pid_Y.ball_center_Y = (pid_Y.ball_max_Y + pid_Y.ball_min_Y) / 2;     //ͨ���ĸ����������С������
					
			  pid_X.ball_max_X = 0;
			  pid_X.ball_min_X = 160;
			  pid_Y.ball_max_Y = 0;
			  pid_Y.ball_min_Y = 200;   //������������������ٴα������ֵ ��Сֵ
					

				
		//		TIM_SetCompare1(TIM14,9340);	//�޸ıȽ�ֵ���޸�ռ�ձ�   ���Զ��ʹ��
		
		//		TIM_SetCompare1(TIM11,9300);	//�޸ıȽ�ֵ���޸�ռ�ձ�   ���Զ��ʹ��
		
	}
}


void TIM3_IRQHandler()
{

	char str1[30] = {0};
	char str2[30] = {0};
	char str3[30] = {0};
	char str4[30] = {0};
	//char str5[30] = {0};
	
	
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		if(displayMode == BinaryZation)
		{
			//����ƫ��
			pid_X.E_now = Aim_X - pid_X.ball_center_X;
			pid_Y.E_now = Aim_Y - pid_Y.ball_center_Y;
			
			PWM_X = PWM_init_X                                + \
			        pid_X.Kp * pid_X.E_now                    + \
			        pid_X.Ki * pid_X.E_sum                    + \
			        pid_X.Kd * (pid_X.E_now - pid_X.E_last);
			PWM_Y = PWM_init_Y                                + \
			        pid_Y.Kp * pid_Y.E_now                    + \
			        pid_Y.Ki * pid_Y.E_sum                    + \
			        pid_Y.Kd * (pid_Y.E_now - pid_Y.E_last);
			
			pid_X.E_last = pid_X.E_now;  //KD
			pid_Y.E_last = pid_Y.E_now;
			
			pid_X.E_sum += pid_X.E_now;  //KI
			pid_Y.E_sum += pid_Y.E_now;
			
			if(PWM_Y > 2500)	PWM_Y = 2500;                                    //pid����޷��� ��ֹ���
			if(PWM_Y < 500)		PWM_Y = 500;
			
			if(PWM_X > 2500)	PWM_X=2500;
			if(PWM_X < 500)		PWM_X=500;
		}
		else 
		{
			PWM_X = PWM_init_X;
			PWM_Y = PWM_init_Y;
		}
		
		TIM_SetCompare1(TIM4,PWM_X);	//�޸ıȽ�ֵ���޸�ռ�ձ�      ���pid����ֵ
		TIM_SetCompare2(TIM4,PWM_Y);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		
		
		
		sprintf(str3, "b_X: %d       Ki: %.2f", pid_X.ball_center_X, pid_X.Ki);
		sprintf(str4, "b_Y: %d", pid_Y.ball_center_Y);
		sprintf(str1, "P_X: %d, %d,  Kp: %.2f", PWM_X, PWM_init_X, pid_X.Kp);
		sprintf(str2, "P_Y: %d, %d,  Kd: %.2f", PWM_Y, PWM_init_Y, pid_X.Kd);
		//sprintf(str5, "Kp: %.2f  Kd: %.2f", pid_X.Kp, pid_X.Kd);
		
		LCD_ShowString(10, 220 , 240, 16, 16, "                    ");
		LCD_ShowString(10, 250 , 240, 16, 16, "                    ");
	
		LCD_ShowString(10, 220 , 240, 16, 16, str1);
		LCD_ShowString(10, 250 , 240, 16, 16, str2);
		
		LCD_ShowString(10, 270 , 240, 16, 16, "                    ");
		LCD_ShowString(10, 300 , 240, 16, 16, "                    ");
		LCD_ShowString(10, 270 , 240, 16, 16, str3);
		LCD_ShowString(10, 300 , 240, 16, 16, str4);
		
//		LCD_ShowString(30, 290 , 240, 16, 16, "                    ");
//		LCD_ShowString(30, 290 , 240, 16, 16, str5);
		
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

}


 