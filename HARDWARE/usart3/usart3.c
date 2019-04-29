#include "sys.h"
#include "usart3.h"	 
#include "pid.h"
#include "string.h"


/*
*说明：使用usart3 usart_RX -> PB11、usart_TX -> PB10
*/
void usart3_init(u32 bound)
{  	 
  
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //PB10,PB11,复用功能,上拉输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA2，PA3
	
 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11复用为USART3
  
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode =   USART_Mode_Tx | USART_Mode_Rx;	// 发模式
	USART_Init(USART3, &USART_InitStructure); //初始化串口
	
	USART_Cmd(USART3, ENABLE);  //使能串口 

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、



}


//displayMode模式选择
#define RGB				1
#define BinaryZation	-1
extern int PWM_X, PWM_Y;
extern pid_t pid_X;
extern pid_t pid_Y;
extern int PWM_init_X;
extern int PWM_init_Y;

#define USART3_REC_LEN 20
u8 USART3_RX_BUF[USART3_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
//接收'm'   - > 切换显示模式

u16 USART3_RX_STA=0;       //接收状态标记	
extern int displayMode;

void USART3_IRQHandler(void)
{
	u8 Res;
	  
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART3);//(USART3->DR);	//读取接收到的数据
		
		if(Res == 'm')
		{
			displayMode = -displayMode;
			printf("m\n");
		}
		else 
		{
			if((USART3_RX_STA&0x8000)==0)//接收未完成
			{
				if(USART3_RX_STA&0x4000)//接收到了0x0d
				{
					if(Res!=0x0a)USART3_RX_STA=0;//接收错误,重新开始
					else 
					
					{
						USART3_RX_STA|=0x8000;	//接收完成了 
						
					    if(RGB == displayMode)
						{
							PWM_init_X =(USART3_RX_BUF[0]-'0')*1000+(USART3_RX_BUF[1]-'0')*100+(USART3_RX_BUF[2]-'0')*10+(USART3_RX_BUF[3]-'0')*1;
							PWM_init_Y =(USART3_RX_BUF[4]-'0')*1000+(USART3_RX_BUF[5]-'0')*100+(USART3_RX_BUF[6]-'0')*10+(USART3_RX_BUF[7]-'0')*1;
						}
						
						if(USART3_RX_BUF[0] == 'K' && USART3_RX_BUF[1]=='P') //kp
						{
							pid_Y.Kp = pid_X.Kp = (USART3_RX_BUF[2] - '0' )*100 + (USART3_RX_BUF[3] - '0' ) * 10 +(USART3_RX_BUF[4] - '0' ) * 1 + (USART3_RX_BUF[6] - '0') * 0.1 + (USART3_RX_BUF[7] - '0' ) * 0.01 ;
							
						}
						else if(USART3_RX_BUF[0] == 'K' && USART3_RX_BUF[1]=='D') //kd
						{
							pid_Y.Kd = pid_X.Kd = (USART3_RX_BUF[2] - '0' )*100 + (USART3_RX_BUF[3] - '0' ) * 10 +(USART3_RX_BUF[4] - '0' ) * 1 + (USART3_RX_BUF[6] - '0') * 0.1 + (USART3_RX_BUF[7] - '0' ) * 0.01 ;							
						}
						else if(USART3_RX_BUF[0] == 'K' && USART3_RX_BUF[1]=='I') //ki
						{
							pid_Y.Ki = pid_X.Ki = (USART3_RX_BUF[2] - '0' )*100 + (USART3_RX_BUF[3] - '0' ) * 10 +(USART3_RX_BUF[4] - '0' ) * 1 + (USART3_RX_BUF[6] - '0') * 0.1 + (USART3_RX_BUF[7] - '0' ) * 0.01 + (USART3_RX_BUF[8] - '0' ) * 0.001 ;							
						}
						
						memset(USART3_RX_BUF, 0, sizeof(USART3_RX_BUF));
						
						USART3_RX_STA = 0;//标志位清0
					}
						
				}
				else //还没收到0X0D
				{	
					if(Res==0x0d)USART3_RX_STA|=0x4000;
					else
					{
						USART3_RX_BUF[USART3_RX_STA&0X3FFF]=Res ;
						USART3_RX_STA++;
						if(USART3_RX_STA>(USART3_REC_LEN-1))USART3_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}  
		}
		USART_ClearFlag(USART3, USART_IT_RXNE);
		
  } 




}







