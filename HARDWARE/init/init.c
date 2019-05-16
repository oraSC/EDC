#include "init.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "usart3.h"
#include "delay.h"
#include "motor.h"
#include "lcd.h"
#include "dcmi.h"	
#include "ov2640.h"	

extern u16 rgb_buf[WINDOW_HEIGHT][WINDOW_WIDTH];
extern aim_t Aim[20];
extern int PWM_init_X[20];
extern int PWM_init_Y[20];


void sys_init(void)
{
	//Aim x 、 y 、distance
	Aim[AIM_1_index].X = 24;
	Aim[AIM_1_index].Y = 33;
	Aim[AIM_1_index].success_distance = 4;
	Aim[AIM_1_index].middle_distance = 6;
	Aim[AIM_1_index].PID_p_x = 0.01;
	Aim[AIM_1_index].PID_p_y = 0.01;
	Aim[AIM_1_index].PID_d_x = 13;
	Aim[AIM_1_index].PID_d_y = 13;
	
	
	Aim[AIM_2_index].X = 72;
	Aim[AIM_2_index].Y = 34;
	Aim[AIM_2_index].success_distance = 4;
	Aim[AIM_2_index].middle_distance = 5;
	Aim[AIM_2_index].PID_p_x = 0.015;
	Aim[AIM_2_index].PID_p_y = 0.015;
	Aim[AIM_2_index].PID_d_x = 13;
	Aim[AIM_2_index].PID_d_y = 13;
	
	Aim[AIM_3_index].X = 121;
	Aim[AIM_3_index].Y = 37;
	Aim[AIM_3_index].success_distance = 5;
	Aim[AIM_3_index].middle_distance = 6;
	Aim[AIM_3_index].PID_p_x = 0.01;
	Aim[AIM_3_index].PID_p_y = 0.01;
	Aim[AIM_3_index].PID_d_x = 10;
	Aim[AIM_3_index].PID_d_y = 10;
	
	Aim[AIM_4_index].X = 22;
	Aim[AIM_4_index].Y = 95;
	Aim[AIM_4_index].success_distance = 4;
	Aim[AIM_4_index].middle_distance = 6;
	Aim[AIM_4_index].PID_p_x = 0.01;
	Aim[AIM_4_index].PID_p_y = 0.01;
	Aim[AIM_4_index].PID_d_x = 12;
	Aim[AIM_4_index].PID_d_y = 12;
	
	Aim[AIM_5_index].X = 71;
	Aim[AIM_5_index].Y = 97;
	Aim[AIM_5_index].success_distance = 4;
	Aim[AIM_5_index].middle_distance = 6;
	Aim[AIM_5_index].PID_p_x = 0.02;
	Aim[AIM_5_index].PID_p_y = 0.02;
	Aim[AIM_5_index].PID_d_x = 13;
	Aim[AIM_5_index].PID_d_y = 13;
	
	Aim[AIM_6_index].X = 120;
	Aim[AIM_6_index].Y = 100;
	Aim[AIM_6_index].success_distance = 4;
	Aim[AIM_6_index].middle_distance = 6;
	Aim[AIM_6_index].PID_p_x = 0.01;
	Aim[AIM_6_index].PID_p_y = 0.01;
	Aim[AIM_6_index].PID_d_x = 13;
	Aim[AIM_6_index].PID_d_y = 13;
	
	
	Aim[AIM_7_index].X = 21;
	Aim[AIM_7_index].Y = 158;
	Aim[AIM_7_index].success_distance = 4;
	Aim[AIM_7_index].middle_distance = 6;
	Aim[AIM_7_index].PID_p_x = 0.01;
	Aim[AIM_7_index].PID_p_y = 0.01;
	Aim[AIM_7_index].PID_d_x = 13;
	Aim[AIM_7_index].PID_d_y = 13;
	
	Aim[AIM_8_index].X = 69;
	Aim[AIM_8_index].Y = 161;
	Aim[AIM_8_index].success_distance = 4;
	Aim[AIM_8_index].middle_distance = 6;
	Aim[AIM_8_index].PID_p_x = 0.01;
	Aim[AIM_8_index].PID_p_y = 0.01;
	Aim[AIM_8_index].PID_d_x = 13;
	Aim[AIM_8_index].PID_d_y = 13;
	
	Aim[AIM_9_index].X = 118;
	Aim[AIM_9_index].Y = 163;
	Aim[AIM_9_index].success_distance = 2;
	Aim[AIM_9_index].middle_distance = 6;
	Aim[AIM_9_index].PID_p_x = 0.01;
	Aim[AIM_9_index].PID_p_y = 0.01;
	Aim[AIM_9_index].PID_d_x = 12;
	Aim[AIM_9_index].PID_d_y = 12;
	
	//buffer x , y
	Aim[buffer_1_index].X = 46;
	Aim[buffer_1_index].Y = 61;
	Aim[buffer_1_index].success_distance = 20;
	Aim[buffer_1_index].middle_distance = 30;
	Aim[buffer_1_index].PID_p_x = 1.7;
	Aim[buffer_1_index].PID_p_y = 1.7;
	Aim[buffer_1_index].PID_d_x = 50;
	Aim[buffer_1_index].PID_d_y = 50;
	
	Aim[buffer_2_index].X = 85;
	Aim[buffer_2_index].Y = 59;
	Aim[buffer_2_index].success_distance = 20;
	Aim[buffer_2_index].middle_distance = 30;
	Aim[buffer_2_index].PID_p_x = 1.7;
	Aim[buffer_2_index].PID_p_y = 1.7;
	Aim[buffer_2_index].PID_d_x = 50;
	Aim[buffer_2_index].PID_d_y = 50;
	
	Aim[buffer_3_index].X = 55;
	Aim[buffer_3_index].Y = 130;
	Aim[buffer_3_index].success_distance = 20;
	Aim[buffer_3_index].middle_distance = 30;
	Aim[buffer_3_index].PID_p_x = 1.7;
	Aim[buffer_3_index].PID_p_y = 1.7;
	Aim[buffer_3_index].PID_d_x = 50;
	Aim[buffer_3_index].PID_d_y = 50;
	
	Aim[buffer_4_index].X = 96;
	Aim[buffer_4_index].Y = 135;
	Aim[buffer_4_index].success_distance = 15;
	Aim[buffer_4_index].middle_distance = 20;
	Aim[buffer_4_index].PID_p_x = 1.7;
	Aim[buffer_4_index].PID_p_y = 1.7;
	Aim[buffer_4_index].PID_d_x = 60;
	Aim[buffer_4_index].PID_d_y = 60;
	

	//初始值
	PWM_init_X[AIM_1_index] = 65;
	PWM_init_Y[AIM_1_index] = 65;
	PWM_init_X[AIM_2_index] = 50;
	PWM_init_Y[AIM_2_index] = 72;
	PWM_init_X[AIM_3_index] = 30;
	PWM_init_Y[AIM_3_index] = 75;
	PWM_init_X[AIM_4_index] = 68;
	PWM_init_Y[AIM_4_index] = 53;
	PWM_init_X[AIM_5_index] = 51;
	PWM_init_Y[AIM_5_index] = 54;
	
	PWM_init_X[AIM_6_index] = 32;
	PWM_init_Y[AIM_6_index] = 54;
	PWM_init_X[AIM_7_index] = 67;
	PWM_init_Y[AIM_7_index] = 35;
	PWM_init_X[AIM_8_index] = 55;
	PWM_init_Y[AIM_8_index] = 37;
	PWM_init_X[AIM_9_index] = 37;
	PWM_init_Y[AIM_9_index] = 37;
	PWM_init_X[buffer_1_index] = 50;
	PWM_init_Y[buffer_1_index] = 50;
	PWM_init_X[buffer_2_index] = 45;
	PWM_init_Y[buffer_2_index] = 70;
	PWM_init_X[buffer_3_index] = 60;
	PWM_init_Y[buffer_3_index] = 30;
	PWM_init_X[buffer_4_index] = 45;
	PWM_init_Y[buffer_4_index] = 50;

			//外设初始化
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
	
	//show
	LCD_ShowString(150, 20 , 100, 24, 24, "frame: ");
	LCD_ShowString(150, 50 , 100, 24, 24, "time: ");

}







