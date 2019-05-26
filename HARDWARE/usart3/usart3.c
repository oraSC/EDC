#include "sys.h"
#include "usart3.h"	 
#include "pid.h"
#include "string.h"
#include "motor.h"
#include "init.h"
/*
*˵����ʹ��usart3 usart_RX -> PB11��usart_TX -> PB10
*/
void usart3_init(u32 bound)
{  	 
  
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //PB10,PB11,���ù���,�������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù��� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PA2��PA3
	
 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10����ΪUSART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11����ΪUSART3
  
	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode =   USART_Mode_Tx | USART_Mode_Rx;	// ��ģʽ
	USART_Init(USART3, &USART_InitStructure); //��ʼ������
	
	USART_Cmd(USART3, ENABLE);  //ʹ�ܴ��� 

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����



}


//displayModeģʽѡ��
#define RGB				1
#define BinaryZation	-1
extern int PWM_X, PWM_Y;
extern pid_t pid_X;
extern pid_t pid_Y;

extern int aim_routine[TASK_num][TASK_node_num];
extern int Task_index;
extern int *aim_index;


#define USART3_REC_LEN 20
u8 USART3_RX_BUF[USART3_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
//����'m'   - > �л���ʾģʽ

u16 USART3_RX_STA=0;       //����״̬���	
extern int displayMode;
extern u8 on_task;
extern u8 task_time;
extern char task_6_num;


void USART3_IRQHandler(void)
{
	u8 Res;
	int PWM_X;
	int PWM_Y;
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART3);//(USART3->DR);	//��ȡ���յ�������
		
		if(Res == 'm')
		{
			
			displayMode = -displayMode;
			pid_X.E_sum = 0;
			pid_Y.E_sum = 0;
			
			//printf("m\n");
		}
		else 
		{
			if((USART3_RX_STA&0x8000)==0)//����δ���
			{
				if(USART3_RX_STA&0x4000)//���յ���0x0d
				{
					if(Res!=0x0a)USART3_RX_STA=0;//���մ���,���¿�ʼ
					else 
					
					{
						USART3_RX_STA|=0x8000;	//��������� 
						
					    
						
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
						else if(USART3_RX_BUF[0] == 'T')
						{
							//ת����һ������
							on_task = 0;
							task_time = 0;
							
							Task_index =  USART3_RX_BUF[1] - '0';
							aim_index = aim_routine[Task_index];
							//������������������������
							if(Task_index == TASK_6_index)
							{
								task_6_num = 0;
							}
							
						}
						else if(USART3_RX_BUF[0] == 'n')
						{
							
							aim_routine[TASK_6_index][task_6_num] = USART3_RX_BUF[1] - '0';
							if(task_6_num != 0)
							{
							switch(USART3_RX_BUF[1] - '0')
							{
								case 1:
								case 2:
								case 4:
								aim_routine[TASK_6_index][task_6_num - 2] = buffer_1_index;
								break;
								case 3:
								case 6:
								case 5:	
								aim_routine[TASK_6_index][task_6_num - 2] = buffer_2_index;
								break;
								case 7:
								case 8:
								aim_routine[TASK_6_index][task_6_num - 2] = buffer_3_index;
								break;
								case 9:
								aim_routine[TASK_6_index][task_6_num - 2] = buffer_4_index;
								break;
								
							}
							}
							task_6_num += 4;
						
						}
						
						
						
						else if(RGB == displayMode)
						{
							PWM_X =(USART3_RX_BUF[0]-'0')*1000+(USART3_RX_BUF[1]-'0')*100+(USART3_RX_BUF[2]-'0')*10+(USART3_RX_BUF[3]-'0')*1;
							PWM_Y =(USART3_RX_BUF[4]-'0')*1000+(USART3_RX_BUF[5]-'0')*100+(USART3_RX_BUF[6]-'0')*10+(USART3_RX_BUF[7]-'0')*1;
							
							set_angle(PWM_X, PWM_Y);
						}
						
						memset(USART3_RX_BUF, 0, sizeof(USART3_RX_BUF));
						
						USART3_RX_STA = 0;//��־λ��0
					}
						
				}
				else //��û�յ�0X0D
				{	
					if(Res==0x0d)USART3_RX_STA|=0x4000;
					else
					{
						USART3_RX_BUF[USART3_RX_STA&0X3FFF]=Res ;
						USART3_RX_STA++;
						if(USART3_RX_STA>(USART3_REC_LEN-1))USART3_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}  
		}
		USART_ClearFlag(USART3, USART_IT_RXNE);
		
  } 




}







