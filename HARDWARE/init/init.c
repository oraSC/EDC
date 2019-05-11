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
	Aim[AIM_1_index].X = 12;
	Aim[AIM_1_index].Y = 28;
	Aim[AIM_1_index].success_distance = 2;
	Aim[AIM_1_index].close_distance = 4;
	Aim[AIM_1_index].middle_distance = 10;
//	Aim[AIM_1_index].far_distance;
	
	Aim[AIM_2_index].X = 62;
	Aim[AIM_2_index].Y = 25;
	
	Aim[AIM_3_index].X = 114;
	Aim[AIM_3_index].Y = 24;
	
	Aim[AIM_4_index].X = 22;
	Aim[AIM_4_index].Y = 96;
	Aim[AIM_4_index].success_distance = 2;
	Aim[AIM_4_index].close_distance = 4;
	Aim[AIM_4_index].middle_distance = 5;
	Aim[AIM_4_index].PID_p_x = 0.05;
	Aim[AIM_4_index].PID_p_y = 0.05;
	Aim[AIM_4_index].PID_d_x = 20;
	Aim[AIM_4_index].PID_d_y = 20;
	
	Aim[AIM_5_index].X = 70;
	Aim[AIM_5_index].Y = 96;
	Aim[AIM_5_index].success_distance = 2;
	Aim[AIM_5_index].close_distance = 2;
	Aim[AIM_5_index].middle_distance = 3;
	Aim[AIM_5_index].PID_p_x = 0.05;
	Aim[AIM_5_index].PID_p_y = 0.05;
	Aim[AIM_5_index].PID_d_x = 20;
	Aim[AIM_5_index].PID_d_y = 20;
	
	Aim[AIM_6_index].X = 115;
	Aim[AIM_6_index].Y = 90;
	Aim[AIM_7_index].X = 15;
	Aim[AIM_7_index].Y = 157;
	Aim[AIM_8_index].X = 67;
	Aim[AIM_8_index].Y = 164;
	
	Aim[AIM_9_index].X = 118;
	Aim[AIM_9_index].Y = 162;
	Aim[AIM_9_index].success_distance = 2;
	Aim[AIM_9_index].close_distance = 4;
	Aim[AIM_9_index].middle_distance = 10;
	
	//buffer x , y
	Aim[buffer_1_index].X = 45;
	Aim[buffer_1_index].Y = 63;
	Aim[buffer_1_index].success_distance = 20;
	Aim[buffer_1_index].close_distance = 20;
	Aim[buffer_1_index].middle_distance = 20;
	
	Aim[buffer_2_index].X = 96;
	Aim[buffer_2_index].Y = 62;
	Aim[buffer_2_index].success_distance = 20;
	Aim[buffer_2_index].close_distance = 20;
	Aim[buffer_2_index].middle_distance = 20;
	
	Aim[buffer_3_index].X = 46;
	Aim[buffer_3_index].Y = 128;
	Aim[buffer_3_index].success_distance = 20;
	Aim[buffer_3_index].close_distance = 20;
	Aim[buffer_3_index].middle_distance = 20;
	
	Aim[buffer_4_index].X = 96;
	Aim[buffer_4_index].Y = 128;
	Aim[buffer_4_index].success_distance = 10;
	Aim[buffer_4_index].close_distance = 10;
	Aim[buffer_4_index].middle_distance = 10;
	

	//初始值
	PWM_init_X[AIM_4_index] = 70;
	PWM_init_Y[AIM_4_index] = 60;
	PWM_init_X[AIM_5_index] = 55;
	PWM_init_Y[AIM_5_index] = 55;
	PWM_init_X[AIM_8_index] = 1200;
	PWM_init_Y[AIM_8_index] = 1000;
	PWM_init_X[AIM_9_index] = 1000;
	PWM_init_Y[AIM_9_index] = 1000;
	PWM_init_X[buffer_1_index] = 1100;
	PWM_init_Y[buffer_1_index] = 1100;
	PWM_init_X[buffer_2_index] = 1100;
	PWM_init_Y[buffer_2_index] = 1100;
	PWM_init_X[buffer_3_index] = 1100;
	PWM_init_Y[buffer_3_index] = 1100;
	PWM_init_X[buffer_4_index] = 1100;
	PWM_init_Y[buffer_4_index] = 1100;

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



}
