#include "stm32f4xx.h"
#include "usart.h"
#include "usart3.h"
#include "delay.h"
#include "motor.h"
#include "lcd.h"
#include "dcmi.h"	
#include "ov2640.h"	
#include "pid.h"

#define AIM_1_index		1
#define AIM_2_index		2
#define AIM_3_index		3
#define AIM_4_index		4
#define AIM_5_index		5
#define AIM_6_index		6
#define AIM_7_index		7
#define AIM_8_index		8
#define AIM_9_index		9
#define buffer_1_index	10
#define buffer_2_index	11
#define buffer_3_index	12
#define buffer_4_index	13


#define WINDOW_WIDTH 	148
#define WINDOW_HEIGHT	196
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
u16 hang=0;

//�㣺(X, Y)
typedef struct point{
	
	int X;
	int Y;

}point_t;

//ֱ�ߣ�Y = kX + b
typedef struct line{

	float k;
	float b;

}line_t;
//����
typedef struct rect{
	
	line_t left;
	line_t top;
	line_t right;
	line_t bottom;
	point_t center;

}rect_t;



point_t Aim[10];

	
int success = 0;
int aim_routine[9] = {AIM_5_index};     //��Ӧ��� - 1
int *aim_index = aim_routine;


//===================PID����==============//
pid_t pid_X;
pid_t pid_Y;

int PWM_init_X = 1100;
int PWM_init_Y = 1100;

//Ŀ���
float Aim_X = 70;
float Aim_Y = 70;
int PWM_X = 0;
int PWM_Y = 0;
u8 ov_frame_flag;  						//֡��
u32 ball_static = 0;
char Enable_I_flag = 0;

void pid_calculate(void);
rect_t fix_position();
line_t find_line(point_t start, point_t end);

int ov_frame = 0;
rect_t rect;


int main(void)
{

	
	u16 i,j;
	
	//Aim
	Aim[AIM_1_index].X = 19;
	Aim[AIM_1_index].Y = 30;
	Aim[AIM_2_index].X = 71;
	Aim[AIM_2_index].Y = 29;
	Aim[AIM_3_index].X = 121;
	Aim[AIM_3_index].Y = 29;
	Aim[AIM_4_index].X = 21;
	Aim[AIM_4_index].Y = 96;
	Aim[AIM_5_index].X = 71;
	Aim[AIM_5_index].Y = 95;
	Aim[AIM_6_index].X = 122;
	Aim[AIM_6_index].Y = 95;
	Aim[AIM_7_index].X = 22;
	Aim[AIM_7_index].Y = 161;
	Aim[AIM_8_index].X = 71;
	Aim[AIM_8_index].Y = 161;
	Aim[AIM_9_index].X = 122;
	Aim[AIM_9_index].Y = 160;
	
	//buffer
	Aim[buffer_1_index].X = 45;
	Aim[buffer_1_index].Y = 63;
	Aim[buffer_2_index].X = 96;
	Aim[buffer_2_index].Y = 62;
	Aim[buffer_3_index].X = 46;
	Aim[buffer_3_index].Y = 128;
	Aim[buffer_4_index].X = 96;
	Aim[buffer_4_index].Y = 128;

	
	displayMode = BinaryZation;
	ov_frame_flag = OLD_frame;
	
	
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
	pid_init(&pid_X, 10, 0.4, 180); //pid_init(pid_t *Pid, float Kp, float Ki, float Kd)
	pid_init(&pid_Y, 10, 0.4, 180);
	TIM3_init(1); //30HZ -> 33ms ����ͷ���30֡
	
	//��У��λ��
	while(ov_frame_flag == OLD_frame);
	rect = fix_position();
	
	
	//while(1);
	while(1)
	{
		//�ȴ��µ�һ֡
		while(ov_frame_flag == OLD_frame);
		ov_frame_flag = OLD_frame;
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
//					
//						if((rect.top.k * j + rect.top.b) > i)
//						{
//							LCD->LCD_RAM=WHITE;
//							continue;
//						}
//						else if((rect.bottom.k * j + rect.bottom.b) < i)
//						{
//							LCD->LCD_RAM=WHITE;
//							continue;
//							
//						}
//						else if( ((i - rect.left.b) / rect.left.k) > j)
//						{
//							LCD->LCD_RAM=WHITE;
//							continue;
//							
//						}
//						else if( ((i - rect.right.b) / rect.right.k) < j)
//						{
//							LCD->LCD_RAM=WHITE;
//							continue;
//							
//						}
						
					
					gray=((rgb_buf[i][j]>>11)*19595+((rgb_buf[i][j]>>5)&0x3f)*38469 +(rgb_buf[i][j]&0x1f)*7472)>>16;  //�Ҷȼ��㡣��ʽ��ٶ�
					if(gray>=22)  //�̶���ֵ��ֵ��
					//if(gray >= 8)
					
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
//			
//			//�ж��Ƿ���뻺����
//			if(*aim_index == buffer_1_index || *aim_index == buffer_2_index || *aim_index == buffer_3_index || *aim_index == buffer_4_index)
//			{
//				if(pid_X.ball_center_X <= Aim[*aim_index].X + 10 && pid_X.ball_center_X >= Aim[*aim_index].X - 10 \
//					&& pid_Y.ball_center_Y <= Aim[*aim_index].Y + 10 && pid_Y.ball_center_Y >= Aim[*aim_index].Y - 10)
//				{
//					success++;
//					if(success >= 60)
//					{
//						aim_index++;
//						success = 0;
//					}
//				
//				}
//				else 
//				{
//					ball_static = 0;
//					success = 0;
//					pid_X.ball_last_center_X = pid_X.ball_center_X;
//					pid_Y.ball_last_center_Y = pid_Y.ball_center_Y;
//				}
//				
//			
//			
//			
//			}
		
		
		
			//�ж�λ���Ƿ񲻶�
			if(pid_X.ball_last_center_X <= pid_X.ball_center_X + 1 && pid_X.ball_last_center_X >= pid_X.ball_center_X - 1  \
				&& pid_Y.ball_last_center_Y <= pid_Y.ball_center_Y + 1 && pid_Y.ball_last_center_Y >= pid_Y.ball_center_Y - 1)
			{
				if(pid_X.ball_center_X <= Aim[*aim_index].X + 3 && pid_X.ball_center_X >= Aim[*aim_index].X - 3 \
					&& pid_Y.ball_center_Y <= Aim[*aim_index].Y + 3 && pid_Y.ball_center_Y >= Aim[*aim_index].Y - 3)
				{
					//success
					success++;
					if(success >= 60)
					{
						aim_index++;
						success = 0;
						
					}
					
					
					
				}
				else 
				{
					success = 0;
					ball_static++;
					if(ball_static > 40)
					{
						Enable_I_flag = ENABLE_I;
					}
				}
			}
			else 
			{
				ball_static = 0;
				success = 0;
				pid_X.ball_last_center_X = pid_X.ball_center_X;
				pid_Y.ball_last_center_Y = pid_Y.ball_center_Y;
			}
		
			pid_X.ball_max_X = 0;
			pid_X.ball_min_X = 160;
			pid_Y.ball_max_Y = 0;
			pid_Y.ball_min_Y = 200;   //������������������ٴα������ֵ ��Сֵ
					
			
		
			//pid����
			pid_calculate();
			ov_frame++;
				
		//		TIM_SetCompare1(TIM14,9340);	//�޸ıȽ�ֵ���޸�ռ�ձ�   ���Զ��ʹ��
		
		//		TIM_SetCompare1(TIM11,9300);	//�޸ıȽ�ֵ���޸�ռ�ձ�   ���Զ��ʹ��
		
	}
}



#define POINT_SIZE 20

rect_t fix_position()
{
	int cols = 0;
	int rows = 0;
	int counter = 1;
	
	
	
	point_t first, second, third, fourth;
	point_t left_top, left_bottom, right_top, right_bottom;
	rect_t myrect;
	
	//�����ĸ�����	
	for(rows = 0; rows < WINDOW_HEIGHT; rows++)
	{
		for(cols = 0; cols < WINDOW_WIDTH; cols++)
		{
			gray=((rgb_buf[rows][cols]>>11)*19595+((rgb_buf[rows][cols]>>5)&0x3f)*38469 +(rgb_buf[rows][cols]&0x1f)*7472)>>16;  //�Ҷȼ��㡣��ʽ��ٶ�
			//��ɫ
			if(gray >= 23)
			{
				switch(counter)
				{
					case 1:
						first.X = cols;
						first.Y = rows;
						counter++;
						break;
					case 2:
						//��Ϊ��һ��
						if(cols <= first.X + POINT_SIZE && cols >= first.X - POINT_SIZE && \
							rows <= first.Y  + POINT_SIZE && rows >= first.Y - POINT_SIZE)
						{
							break;
						}
						second.X = cols;
						second.Y = rows;
						counter++;
						break;
					case 3:
						//��Ϊ��һ��
						if(cols <= first.X + POINT_SIZE && cols >= first.X - POINT_SIZE && \
							rows <= first.Y  + POINT_SIZE && rows >= first.Y - POINT_SIZE)
						{
							break;
						}
						//��Ϊ�ڶ���
						if(cols <= second.X + POINT_SIZE && cols >= second.X - POINT_SIZE && \
							rows <= second.Y  + POINT_SIZE && rows >= second.Y - POINT_SIZE)
						{
							break;
						}
						third.X = cols;
						third.Y = rows;
						counter++;
						break;
					case 4:
						//��Ϊ��һ��
						if(cols <= first.X + POINT_SIZE && cols >= first.X - POINT_SIZE && \
							rows <= first.Y  + POINT_SIZE && rows >= first.Y - POINT_SIZE)
						{
							break;
						}
						//��Ϊ�ڶ���
						if(cols <= second.X + POINT_SIZE && cols >= second.X - POINT_SIZE && \
							rows <= second.Y  + POINT_SIZE && rows >= second.Y - POINT_SIZE)
						{
							break;
						}
						//��Ϊ������
						if(cols <= third.X + POINT_SIZE && cols >= third.X - POINT_SIZE && \
							rows <= third.Y  + POINT_SIZE && rows >= third.Y - POINT_SIZE)
						{
							break;
						}
						fourth.X = cols;
						fourth.Y = rows;
						counter++;
						break;
				
				}
			
			
			}
		
		
		}
		
	
	
	}
	//�ж��ĸ�����
	if(first.X <= second.X)
	{
		left_top 	= first;
		right_top 	= second;
	}
	else
	{
		left_top	= second;
		right_top	= first;
	}
	
	if(third.X <= fourth.X)
	{
		left_bottom	= third;
		right_bottom = fourth;
		
	}
	else 
	{
		left_bottom	= fourth;
		right_bottom = third;
	}
	
	printf("left_top:(%d , %d) ", left_top.X, left_top.Y);
	printf("right_top:(%d , %d) ", right_top.X, right_top.Y);
	printf("left_bottom:(%d , %d) ", left_bottom.X, left_bottom.Y);
	printf("right_bottom:(%d , %d) ", right_bottom.X, right_bottom.Y);
	printf("\n");
	//Ѱ���߶�
	//line_top
	myrect.top = find_line(left_top, right_top);
	myrect.left = find_line(left_top, left_bottom);
	myrect.right = find_line(right_top, right_bottom);
	myrect.bottom = find_line(left_bottom, right_bottom);
	
	return myrect;
	

}

line_t find_line(point_t start, point_t end)
{
	line_t line;
	
	line.k = (float)(start.Y - end.Y) / (float)(start.X - end.X); 
	line.b = start.Y - line.k * start.X;

	return line;
}



void pid_calculate(void)
{
	static u8 I_count = 0;
	char str1[30] = {0};
	char str2[30] = {0};
	char str3[30] = {0};
	char str4[30] = {0};
	//char str5[30] = {0};
	
	
	//if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		if(displayMode == BinaryZation)
		{
			//����ƫ��
			pid_X.E_now = Aim[*aim_index].X - pid_X.ball_center_X;
			pid_Y.E_now = Aim[*aim_index].Y - pid_Y.ball_center_Y;
			
			//�����������
			pid_X.Pout = pid_X.Kp * pid_X.E_now;
			pid_Y.Pout = pid_Y.Kp * pid_Y.E_now;
			
			if(Enable_I_flag == ENABLE_I)
			{
				I_count++;
				if(I_count >= 40)
				{
					I_count = 0;
					Enable_I_flag = DISABLE_I;
					pid_X.E_sum = pid_Y.E_sum = 0;
					
				}
				pid_X.E_sum += pid_X.E_now;  //KI
				pid_Y.E_sum += pid_Y.E_now;
				pid_X.Iout = pid_X.Ki * pid_X.E_sum ;
				pid_Y.Iout = pid_Y.Ki * pid_Y.E_sum;
			}
			else 
			{
				pid_X.Iout = 0;
				pid_Y.Iout = 0;
			}
			
			
			pid_X.Dout = pid_X.Kd * (pid_X.E_now - pid_X.E_last);
			pid_Y.Dout = pid_Y.Kd * (pid_Y.E_now - pid_Y.E_last);
			
			PWM_X = PWM_init_X + pid_X.Pout + pid_X.Iout + pid_X.Dout;  
			PWM_Y = PWM_init_Y + pid_Y.Pout + pid_Y.Iout + pid_Y.Dout;
			        
			
			pid_X.E_last = pid_X.E_now;  //KD
			pid_Y.E_last = pid_Y.E_now;
			
			
			
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
		
		
		
		
		sprintf(str1, "P_X: %d, %d,  Kp: %.2f", PWM_X, PWM_init_X, pid_X.Kp);
		sprintf(str2, "P_Y: %d, %d,  Kd: %.2f", PWM_Y, PWM_init_Y, pid_X.Kd);
		sprintf(str3, "b_X: %d A_X:%d Ki: %.2f %d", pid_X.ball_center_X, Aim[*aim_index].X, pid_X.Ki, Enable_I_flag);
		sprintf(str4, "b_Y: %d A_Y:%d", pid_Y.ball_center_Y, Aim[*aim_index].Y);
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
	char frame[6];
	
	
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		
		sprintf(frame, "frame: %d", ov_frame);
		LCD_ShowString(150, 20 , 100, 16, 16, "          ");
		LCD_ShowString(150, 20 , 100, 16, 16, frame);
		ov_frame = 0;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}



}




 