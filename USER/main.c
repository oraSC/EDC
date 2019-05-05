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


//displayMode模式选择
#define RGB				1
#define BinaryZation	-1
#define NEW_frame		1
#define OLD_frame		0

//初始值
int displayMode;
u16 rgb_buf[WINDOW_HEIGHT][WINDOW_WIDTH]; 
u16 gray;

//点：(X, Y)
typedef struct point{
	
	int X;
	int Y;

}point_t;

point_t Aim[20];
int PWM_init_X[20];
int PWM_init_Y[20];
	
int success = 0;
int aim_routine[9] = {AIM_4_index, AIM_5_index};     //对应编号 - 1
int *aim_index = aim_routine;
char have_ball = 0;

//===================PID变量==============//
pid_t pid_X;
pid_t pid_Y;

//int PWM_init_X = 1100;
//int PWM_init_Y = 1100;

//目标点
float Aim_X = 70;
float Aim_Y = 70;
int PWM_X = 0;
int PWM_Y = 0;
u8 ov_frame_flag;  						//帧率
u32 ball_static = 0;
char Enable_I_flag = 0;

void pid_calculate(void);

int ov_frame = 0;
int close_counter = 0;
int close_distance = 5;

int main(void)
{

	
	u16 i,j;
	
	//Aim
	Aim[AIM_1_index].X = 12;
	Aim[AIM_1_index].Y = 28;
	Aim[AIM_2_index].X = 62;
	Aim[AIM_2_index].Y = 25;
	Aim[AIM_3_index].X = 114;
	Aim[AIM_3_index].Y = 24;
	Aim[AIM_4_index].X = 16;
	Aim[AIM_4_index].Y = 102;
	Aim[AIM_5_index].X = 67;
	Aim[AIM_5_index].Y = 100;
	Aim[AIM_6_index].X = 115;
	Aim[AIM_6_index].Y = 90;
	Aim[AIM_7_index].X = 15;
	Aim[AIM_7_index].Y = 157;
	Aim[AIM_8_index].X = 65;
	Aim[AIM_8_index].Y = 156;
	Aim[AIM_9_index].X = 116;
	Aim[AIM_9_index].Y = 155;
	
	//buffer
	Aim[buffer_1_index].X = 45;
	Aim[buffer_1_index].Y = 63;
	
	Aim[buffer_2_index].X = 96;
	Aim[buffer_2_index].Y = 62;
	
	Aim[buffer_3_index].X = 46;
	Aim[buffer_3_index].Y = 128;
	
	Aim[buffer_4_index].X = 96;
	Aim[buffer_4_index].Y = 128;

	//初始值
	PWM_init_X[AIM_4_index] = 1300;
	PWM_init_Y[AIM_4_index] = 1100;
	PWM_init_X[AIM_5_index] = 1100;
	PWM_init_Y[AIM_5_index] = 1100;
	
	displayMode = BinaryZation;
	ov_frame_flag = OLD_frame;
	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);      //初始化延时函数
	uart_init(115200);		//初始化串口波特率为115200
	usart3_init(9600);
 	LCD_Init();           //初始化LCD FSMC接口
	POINT_COLOR=RED;      //画笔颜色：红色
	
	
	/********************************摄像头准备*********************************/
	while(OV2640_Init())//初始化OV2640
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
	OV2640_RGB565_Mode();	//JPEG模式
	My_DCMI_Init();			//DCMI配置
	DCMI_DMA_Init((u32)rgb_buf,0, sizeof(rgb_buf)/4,DMA_MemoryDataSize_HalfWord,DMA_MemoryInc_Enable);//DCMI DMA配置
	//DCMI_DMA_Init((u32)&LCD->LCD_RAM,0, 1,DMA_MemoryDataSize_HalfWord,DMA_MemoryInc_Disable);//DCMI DMA??  
 	printf("%d", lcddev.width);
	printf("%d", lcddev.height); 
	//OV2640_OutSize_Set( lcddev.width, lcddev.height );
	//OV2640_OutSize_Set(WINDOW_WIDTH, WINDOW_HEIGHT); 
	OV2640_OutSize_Set(WINDOW_WIDTH, WINDOW_HEIGHT);
	
	//OV2640_OutSize_Set(lcddev.width, lcddev.height);
	
	DCMI_Start(); 			//启动传输 
	/*******************************************************************************/
	
	/*************************************舵机**************************************/
	motor_init(50);			//50HZ -> 20ms
	TIM_SetCompare1(TIM4, 1100);	//修改比较值，修改占空比      输出pid计算值
	TIM_SetCompare2(TIM4, 1100);	//修改比较值，修改占空比
		
	
	/********************************pid*********************************************/
	pid_init(&pid_X, 10, 0, 170); //pid_init(pid_t *Pid, float Kp, float Ki, float Kd)
	pid_init(&pid_Y, 10, 0, 170);
	TIM3_init(1); //计算帧数中断
	
	while(1)
	{
		//等待新的一帧
		while(ov_frame_flag == OLD_frame);
		ov_frame_flag = OLD_frame;
		LCD_SetCursor(0,0);  
		LCD_WriteRAM_Prepare();		//开始写入GRAM
		for(i=0;i<WINDOW_HEIGHT;i++)
		{
			
			for(j=0; j < WINDOW_WIDTH; j++)
			{
				if(j == WINDOW_WIDTH - 1)
				{
					LCD_SetCursor(0,i+1);  
					LCD_WriteRAM_Prepare();		//开始写入GRAM
				}
				
				
				/*************************************************二值化*******************************************/
				if(displayMode == BinaryZation)
				{
					
					gray=((rgb_buf[i][j]>>11)*19595+((rgb_buf[i][j]>>5)&0x3f)*38469 +(rgb_buf[i][j]&0x1f)*7472)>>16;  //灰度计算。公式请百度
					if(gray>=22)  //固定阈值二值化
					{			  
							//if(i>8&&i<136&&j<200&&j>16)  //此处遍历图像寻找小球最上最下 最左 最右四个点坐标
							{
								if(j > pid_X.ball_max_X)	pid_X.ball_max_X = j;
								if(j < pid_X.ball_min_X) 	pid_X.ball_min_X = j;
							 
								if(i > pid_Y.ball_max_Y)	pid_Y.ball_max_Y = i;
								if(i < pid_Y.ball_min_Y) 	pid_Y.ball_min_Y = i;
						 
							}
							
							
							LCD->LCD_RAM=WHITE;
							have_ball = 1;
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
			pid_Y.ball_center_Y = (pid_Y.ball_max_Y + pid_Y.ball_min_Y) / 2;     //通过四个点坐标计算小球质心
			
			
//			
////			//判断位置是否不动
////			if(pid_X.ball_last_center_X <= pid_X.ball_center_X + 1 && pid_X.ball_last_center_X >= pid_X.ball_center_X - 1  \
////				&& pid_Y.ball_last_center_Y <= pid_Y.ball_center_Y + 1 && pid_Y.ball_last_center_Y >= pid_Y.ball_center_Y - 1)
////			//{
//			//靠近
			if(abs(pid_X.ball_center_X - Aim[*aim_index].X) <= 3 && abs(pid_Y.ball_center_Y - Aim[*aim_index].Y) <= 3)
			{
////				close_counter ++;
////				if(close_counter >= 4)
////				{
					//close_distance = 20;
					//判断位置
					//成功1：kp = 5， kd = 10     //
					//成功2：kp = 5, kd = 5
				pid_X.Kp = 5;
				pid_Y.Kp = 5;
				pid_X.Kd = 5;
				pid_Y.Kd = 5;
////				}
				//成功
				if(abs(pid_X.ball_center_X - Aim[*aim_index].X) <= 1 && abs(pid_Y.ball_center_Y - Aim[*aim_index].Y) <= 1)
//				{
					success++;
					if(success >= 15)
					{
						pid_X.Kp = 0;
						pid_Y.Kp = 0;
						pid_X.Kd = 0;
						pid_Y.Kd = 0;
					}
					if(success >= 70)
					{
						aim_index++;
					
					}
					
				}
//				
//				
//				//success
//				success++;
//					if(success >= 60)
//					{
//						aim_index++;
//						success = 0;
//					}
//			}
//			//远离
			else 
			{
				
//				close_counter --;
//				if(close_counter <= 0)
//				{
//					close_distance = 5;
//				}
				//成功1：kp = 15， kd = 170     //
				//成功2：kp = 15, kd = 170
				pid_X.Kp = 15;
				pid_Y.Kp = 15;
				pid_X.Kd = 170;
				pid_Y.Kd = 170;
				success = 0;
				
//				ball_static++;
//				if(ball_static > 40)
//				{
//					Enable_I_flag = ENABLE_I;
//				}
			}
//		

////			{
////				ball_static = 0;
////				success = 0;
////				pid_X.ball_last_center_X = pid_X.ball_center_X;
////				pid_Y.ball_last_center_Y = pid_Y.ball_center_Y;
////			}
		
			pid_X.ball_max_X = 0;
			pid_X.ball_min_X = 160;
			pid_Y.ball_max_Y = 0;
			pid_Y.ball_min_Y = 200;   //清除掉本次坐标用于再次遍历最大值 最小值
				
		
			//pid计算
			pid_calculate();
			ov_frame++;

	}
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
			//计算偏差
			pid_X.E_now = Aim[*aim_index].X - pid_X.ball_center_X;
			pid_Y.E_now = Aim[*aim_index].Y - pid_Y.ball_center_Y;
			
			//计算各项作用
			pid_X.Pout = pid_X.Kp * pid_X.E_now;
			pid_Y.Pout = pid_Y.Kp * pid_Y.E_now;
			
//			if(Enable_I_flag == ENABLE_I)
//			{
//				I_count++;
//				if(I_count >= 40)
//				{
//					I_count = 0;
//					Enable_I_flag = DISABLE_I;
//					pid_X.E_sum = pid_Y.E_sum = 0;
//					
//				}
//				pid_X.E_sum += pid_X.E_now;  //KI
//				pid_Y.E_sum += pid_Y.E_now;
//				pid_X.Iout = pid_X.Ki * pid_X.E_sum ;
//				pid_Y.Iout = pid_Y.Ki * pid_Y.E_sum;
//			}
//			else 
//			{
//				pid_X.Iout = 0;
//				pid_Y.Iout = 0;
//			}
			
			
			
			//判断已经完成刹车
			pid_X.Dout = pid_X.Kd * (pid_X.E_now - pid_X.E_last);
			pid_Y.Dout = pid_Y.Kd * (pid_Y.E_now - pid_Y.E_last);
			
			if(have_ball == 1)
			{
				PWM_X = PWM_init_X[*aim_index] + pid_X.Pout + pid_X.Iout + pid_X.Dout;  
				PWM_Y = PWM_init_Y[*aim_index] + pid_Y.Pout + pid_Y.Iout + pid_Y.Dout;
				
			}
			
			else
			{
				PWM_X = PWM_init_X[*aim_index];  
				PWM_Y = PWM_init_Y[*aim_index];
			}    
			have_ball = 0;	
			
			pid_X.E_last = pid_X.E_now;  //KD
			pid_Y.E_last = pid_Y.E_now;
			
			
			
			if(PWM_Y > 2500)	PWM_Y = 2500;                                    //pid输出限幅度 防止抽风
			if(PWM_Y < 500)		PWM_Y = 500;
			
			if(PWM_X > 2500)	PWM_X=2500;
			if(PWM_X < 500)		PWM_X=500;
		}
		else 
		{
			PWM_X = PWM_init_X[*aim_index];
			PWM_Y = PWM_init_Y[*aim_index];
		}
		
		TIM_SetCompare1(TIM4,PWM_X);	//修改比较值，修改占空比      输出pid计算值
		TIM_SetCompare2(TIM4,PWM_Y);	//修改比较值，修改占空比
		
		
		
		
		sprintf(str1, "P_X: %d, %d,  Kp: %.2f", PWM_X, PWM_init_X[*aim_index], pid_X.Kp);
		sprintf(str2, "P_Y: %d, %d,  Kd: %.2f", PWM_Y, PWM_init_Y[*aim_index], pid_X.Kd);
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



//DCMI中断服务函数
void DCMI_IRQHandler(void)
{
	if(DCMI_GetITStatus(DCMI_IT_FRAME)==SET)//捕获到一帧图像
	{
		
		DCMI_ClearITPendingBit(DCMI_IT_FRAME);//清除帧中断
		
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


