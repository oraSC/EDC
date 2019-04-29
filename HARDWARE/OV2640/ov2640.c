#include "ov2640.h"
#include "ov2640cfg.h"
  		 
#include "sccb.h"	
 
  
//��ʼ��OV2640 
//�������Ժ�,Ĭ�������1600*1200�ߴ��ͼƬ!! 
//����ֵ:0,�ɹ�
//    ����,�������
u8 OV2640_Init(void)
{ 
	u16 i=0;
	u16 reg;
	//����IO     	   
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOG, ENABLE);
  //GPIOG9,15��ʼ������
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PG9,15�������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //�������
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//PG9,15�������
  GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��
 
 	OV2640_PWDN=0;	//POWER ON
	delay_ms(10);
	OV2640_RST=0;	//��λOV2640
	delay_ms(10);
	OV2640_RST=1;	//������λ 
  SCCB_Init();        		//��ʼ��SCCB ��IO��	 
	SCCB_WR_Reg(OV2640_DSP_RA_DLMT, 0x01);	//����sensor�Ĵ���
 	SCCB_WR_Reg(OV2640_SENSOR_COM7, 0x80);	//��λOV2640
	delay_ms(50); 
	reg=SCCB_RD_Reg(OV2640_SENSOR_MIDH);	//��ȡ����ID �߰�λ
	reg<<=8;
	reg|=SCCB_RD_Reg(OV2640_SENSOR_MIDL);	//��ȡ����ID �Ͱ�λ
	if(reg!=OV2640_MID)
	{
//		printf("MID:%d\r\n",reg);
		return 1;
	}
	reg=SCCB_RD_Reg(OV2640_SENSOR_PIDH);	//��ȡ����ID �߰�λ
	reg<<=8;
	reg|=SCCB_RD_Reg(OV2640_SENSOR_PIDL);	//��ȡ����ID �Ͱ�λ
	if(reg!=OV2640_PID)
	{
//		printf("HID:%d\r\n",reg);
		return 2;
	}  
#if SVGA	
	//��ʼ�� OV2640,����SVGA�ֱ���(800*600)  
	for(i=0;i<sizeof(ov2640_svga_init_reg_tbl)/2;i++)
	{
	   	SCCB_WR_Reg(ov2640_svga_init_reg_tbl[i][0],ov2640_svga_init_reg_tbl[i][1]);
 	}
#elif SXGA
	//��ʼ�� OV2640,����SXGA�ֱ���(1600*1200) 
	for(i=0;i<sizeof(ov2640_sxga_init_reg_tbl)/2;i++)
	{
	   	SCCB_WR_Reg(ov2640_sxga_init_reg_tbl[i][0],ov2640_sxga_init_reg_tbl[i][1]);
 	}
	
#endif
//void set_Cmos2640reg(void)   //�Ĵ������ñ���
//{


//	SCCB_WR_Reg(0xff, 0); 
//	SCCB_WR_Reg(0xff, 1); /*switch to register list 1 */
//	SCCB_WR_Reg(0x12, 0x80); /*system reset */
//	
//	
//	SCCB_WR_Reg(0xff, 0); /*switch to register list 0 */
//	SCCB_WR_Reg(0x2c, 0xff); 
//	SCCB_WR_Reg(0x2e, 0xdf); 
//	SCCB_WR_Reg(0xff, 0x1); /*switch to register list 1 */
//	SCCB_WR_Reg(0x3c, 0x32);
//	
//	SCCB_WR_Reg(0x11, 0x00);/*master clk */
//	SCCB_WR_Reg(0x09, 0x02); 
//	SCCB_WR_Reg(0x04, 0x28);  //
//	SCCB_WR_Reg(0x13, 0xe5);
//	SCCB_WR_Reg(0x14, 0xa8);  //
//	SCCB_WR_Reg(0x15, 0x00);   ////
//	SCCB_WR_Reg(0x2c, 0x0c); 
//	SCCB_WR_Reg(0x33, 0x78); 
//	SCCB_WR_Reg(0x3a, 0x33);
//	SCCB_WR_Reg(0x3b, 0xfb);
//	
//	SCCB_WR_Reg(0x3e, 0x00); 
//	SCCB_WR_Reg(0x43, 0x11); 
//	SCCB_WR_Reg(0x16, 0x10);
//	
//	
//	SCCB_WR_Reg(0x39, 0x92); 
//	
//	SCCB_WR_Reg(0x35, 0xda); 
//	SCCB_WR_Reg(0x22, 0x1a); 
//	SCCB_WR_Reg(0x23, 0x00); 
//								//37
//	SCCB_WR_Reg(0x34, 0xc0); 
//								//36
//	SCCB_WR_Reg(0x06, 0x88);
//	SCCB_WR_Reg(0x07, 0xc0); 
//	SCCB_WR_Reg(0x0d, 0x87); 
//	SCCB_WR_Reg(0x0e, 0x41);
//	SCCB_WR_Reg(0x4c, 0x00);
//	
//	SCCB_WR_Reg(0x48, 0x00);
//	SCCB_WR_Reg(0x5b, 0x00); 
//	SCCB_WR_Reg(0x42, 0x03); 
//	
//	SCCB_WR_Reg(0x4a, 0x81); 
//	SCCB_WR_Reg(0x21, 0x99); 
//	
//	SCCB_WR_Reg(0x24, 0x40); 
//	SCCB_WR_Reg(0x25, 0x38);
//	SCCB_WR_Reg(0x26, 0x82); 
//	SCCB_WR_Reg(0x5c, 0x00); 
//	SCCB_WR_Reg(0x63, 0x00); 
//									//46
//											//0c
//	
//	SCCB_WR_Reg(0x61, 0x70); 
//	SCCB_WR_Reg(0x62, 0x80); 
//	SCCB_WR_Reg(0x7c, 0x05);
//	
//	SCCB_WR_Reg(0x20, 0x80); 
//	SCCB_WR_Reg(0x28, 0x30); 
//	SCCB_WR_Reg(0x6c, 0x00); 
//	SCCB_WR_Reg(0x6d, 0x80);	
//	SCCB_WR_Reg(0x6e, 0); 
//	SCCB_WR_Reg(0x70, 0x02); 
//	SCCB_WR_Reg(0x71, 0x94); 
//	SCCB_WR_Reg(0x73, 0xc1);
//									//3d
//									//5a
//									
//	SCCB_WR_Reg(0x0c, 0x3c); ////
//	
//	SCCB_WR_Reg(0x5d, 0x55); ////
//	SCCB_WR_Reg(0x5e, 0x7d); ////
//	SCCB_WR_Reg(0x5f, 0x7d);	////
//	SCCB_WR_Reg(0x60, 0x55); ////
//	
//	
//	SCCB_WR_Reg(0x12, 0x00); 
//	SCCB_WR_Reg(0x32, 0x09);////
//	SCCB_WR_Reg(0x17, 0x11); 
//	SCCB_WR_Reg(0x18, 0x75); 
//	SCCB_WR_Reg(0x19, 0x01);
//	SCCB_WR_Reg(0x1a, 0x97);
//	SCCB_WR_Reg(0x32, 0x36);  
//									//03
//	SCCB_WR_Reg(0x37, 0x42);
//	
//	SCCB_WR_Reg(0x4f, 0xbb);	
//	SCCB_WR_Reg(0x50, 0x9c); 
//	SCCB_WR_Reg(0x6d, 0x80); 
//	SCCB_WR_Reg(0x35, 0x88);
//	SCCB_WR_Reg(0x22, 0x0a);
//	SCCB_WR_Reg(0x6d, 0x80);
//	SCCB_WR_Reg(0x3d, 0x2e); 
//	SCCB_WR_Reg(0xff, 0x00); /* switch to reg list 0 */
//	SCCB_WR_Reg(0xe5, 0x7f); 
//	SCCB_WR_Reg(0xf9, 0xc0);
//	SCCB_WR_Reg(0x41, 0x24); 
//	SCCB_WR_Reg(0x44, 0x06);
//	SCCB_WR_Reg(0xe0, 0x14); 
//	SCCB_WR_Reg(0x76, 0xff); 
//	SCCB_WR_Reg(0x33, 0xa0); 
//	SCCB_WR_Reg(0x42, 0x20);
//	SCCB_WR_Reg(0x43, 0x18); 
//	SCCB_WR_Reg(0x4c, 0x00); 
//	SCCB_WR_Reg(0x87, 0xd0); 
//	SCCB_WR_Reg(0x88, 0x3f);
//	SCCB_WR_Reg(0xd7, 0x03); 
//	SCCB_WR_Reg(0xd9, 0x10); 
//	SCCB_WR_Reg(0xd3, 0x82); 
//	SCCB_WR_Reg(0xc8, 0x08);
//	SCCB_WR_Reg(0xc9, 0x80); 
//	SCCB_WR_Reg(0x7c, 0x00); 
//	SCCB_WR_Reg(0x7d, 0x00); 
//	SCCB_WR_Reg(0x7c, 0x03);
//	SCCB_WR_Reg(0x7d, 0x48); 
//	SCCB_WR_Reg(0x7d, 0x48); 
//	SCCB_WR_Reg(0x7c, 0x08); 
//	SCCB_WR_Reg(0x7d, 0x20);
//	SCCB_WR_Reg(0x7d, 0x10); 
//	SCCB_WR_Reg(0x7d, 0x0e); 
//	SCCB_WR_Reg(0x90, 0x00); 
//	SCCB_WR_Reg(0x91, 0x0e);
//	SCCB_WR_Reg(0x91, 0x1a); 
//	SCCB_WR_Reg(0x91, 0x31); 
//	SCCB_WR_Reg(0x91, 0x5a); 
//	SCCB_WR_Reg(0x91, 0x69);
//	SCCB_WR_Reg(0x91, 0x75); 
//	SCCB_WR_Reg(0x91, 0x7e); 
//	SCCB_WR_Reg(0x91, 0x88); 
//	SCCB_WR_Reg(0x91, 0x8f);
//	SCCB_WR_Reg(0x91, 0x96); 
//	SCCB_WR_Reg(0x91, 0xa3); 
//	SCCB_WR_Reg(0x91, 0xaf); 
//	SCCB_WR_Reg(0x91, 0xc4);
//	SCCB_WR_Reg(0x91, 0xd7); 
//	SCCB_WR_Reg(0x91, 0xe8); 
//	SCCB_WR_Reg(0x91, 0x20); 
//	SCCB_WR_Reg(0x92, 0x00);
//	SCCB_WR_Reg(0x93, 0x06); 
//	SCCB_WR_Reg(0x93, 0xe3); 
//	SCCB_WR_Reg(0x93, 0x03); 
//	SCCB_WR_Reg(0x93, 0x03);
//	SCCB_WR_Reg(0x93, 0x00); 
//	SCCB_WR_Reg(0x93, 0x02); 
//	SCCB_WR_Reg(0x93, 0x00); 
//	SCCB_WR_Reg(0x93, 0x00);
//	SCCB_WR_Reg(0x93, 0x00); 
//	SCCB_WR_Reg(0x93, 0x00); 
//	SCCB_WR_Reg(0x93, 0x00); 
//	SCCB_WR_Reg(0x93, 0x00);
//	SCCB_WR_Reg(0x93, 0x00); 
//	SCCB_WR_Reg(0x96, 0x00); 
//	SCCB_WR_Reg(0x97, 0x08); 
//	SCCB_WR_Reg(0x97, 0x19);
//	SCCB_WR_Reg(0x97, 0x02); 
//	SCCB_WR_Reg(0x97, 0x0c); 
//	SCCB_WR_Reg(0x97, 0x24); 
//	SCCB_WR_Reg(0x97, 0x30);
//	SCCB_WR_Reg(0x97, 0x28); 
//	SCCB_WR_Reg(0x97, 0x26); 
//	SCCB_WR_Reg(0x97, 0x02); 
//	SCCB_WR_Reg(0x97, 0x98);
//	SCCB_WR_Reg(0x97, 0x80); 
//	SCCB_WR_Reg(0x97, 0x00); 
//	SCCB_WR_Reg(0x97, 0x00); 
//	SCCB_WR_Reg(0xa4, 0x00);
//	SCCB_WR_Reg(0xa8, 0x00); 
//	SCCB_WR_Reg(0xc5, 0x11); 
//	SCCB_WR_Reg(0xc6, 0x51); 
//	SCCB_WR_Reg(0xbf, 0x80);
//	SCCB_WR_Reg(0xc7, 0x10); 
//	SCCB_WR_Reg(0xb6, 0x66); 
//	SCCB_WR_Reg(0xb8, 0xa5); 
//	SCCB_WR_Reg(0xb7, 0x64);
//	SCCB_WR_Reg(0xb9, 0x7c); 
//	SCCB_WR_Reg(0xb3, 0xaf); 
//	SCCB_WR_Reg(0xb4, 0x97); 
//	SCCB_WR_Reg(0xb5, 0xff);
//	SCCB_WR_Reg(0xb0, 0xc5); 
//	SCCB_WR_Reg(0xb1, 0x94); 
//	SCCB_WR_Reg(0xb2, 0x0f); 
//	SCCB_WR_Reg(0xc4, 0x5c);
//	SCCB_WR_Reg(0xa6, 0x00); 
//	SCCB_WR_Reg(0xa7, 0x20); 
//	SCCB_WR_Reg(0xa7, 0xd8); 
//	SCCB_WR_Reg(0xa7, 0x1b);
//	SCCB_WR_Reg(0xa7, 0x31); 
//	SCCB_WR_Reg(0xa7, 0x00); 
//	SCCB_WR_Reg(0xa7, 0x18); 
//	SCCB_WR_Reg(0xa7, 0x20);
//	SCCB_WR_Reg(0xa7, 0xd8); 
//	SCCB_WR_Reg(0xa7, 0x19); 
//	SCCB_WR_Reg(0xa7, 0x31); 
//	SCCB_WR_Reg(0xa7, 0x00);
//	SCCB_WR_Reg(0xa7, 0x18); 
//	SCCB_WR_Reg(0xa7, 0x20); 
//	SCCB_WR_Reg(0xa7, 0xd8); 
//	SCCB_WR_Reg(0xa7, 0x19);
//	SCCB_WR_Reg(0xa7, 0x31); 
//	SCCB_WR_Reg(0xa7, 0x00); 
//	SCCB_WR_Reg(0xa7, 0x18); 
//	SCCB_WR_Reg(0xe0, 0x04); 
//	SCCB_WR_Reg(0xc0, 0xc8); 
//	SCCB_WR_Reg(0xc1, 0x96); 
//	SCCB_WR_Reg(0x86, 0x3d); 
//	SCCB_WR_Reg(0x50, 0x92); 
//	SCCB_WR_Reg(0x51, 0x90); 
//	SCCB_WR_Reg(0x52, 0x2c); 
//	SCCB_WR_Reg(0x53, 0x00); 
//	SCCB_WR_Reg(0x54, 0x00); 
//	SCCB_WR_Reg(0x55, 0x88); 
//	SCCB_WR_Reg(0x57, 0x00); 
//	SCCB_WR_Reg(0x5a, 0x50); 
//	SCCB_WR_Reg(0x5b, 0x3c); 
//	SCCB_WR_Reg(0x5c, 0x00); 
//	SCCB_WR_Reg(0xd3, 0x04);
//	SCCB_WR_Reg(0xe0, 0x00);	
//	SCCB_WR_Reg(0xc3, 0xef); 
//	SCCB_WR_Reg(0x7f, 0x00); 
//	SCCB_WR_Reg(0xda, 0x08); 
//	SCCB_WR_Reg(0xe5, 0x1f);
//	SCCB_WR_Reg(0xe1, 0x67); 
//	SCCB_WR_Reg(0xdd, 0x7f);	
//	SCCB_WR_Reg(0x05, 0x00);
//	SCCB_WR_Reg(0x98, 0x00);
//	SCCB_WR_Reg(0x99, 0x00);
//	SCCB_WR_Reg(0x00, 0x00);

//}
	
	
	
  	return 0x00; 	//ok
} 
//OV2640�л�ΪJPEGģʽ
void OV2640_JPEG_Mode(void) 
{
	u16 i=0;
	//����:YUV422��ʽ
	for(i=0;i<(sizeof(ov2640_yuv422_reg_tbl)/2);i++)
	{
		SCCB_WR_Reg(ov2640_yuv422_reg_tbl[i][0],ov2640_yuv422_reg_tbl[i][1]); 
	} 
	
	//����:���JPEG����
	for(i=0;i<(sizeof(ov2640_jpeg_reg_tbl)/2);i++)
	{
		SCCB_WR_Reg(ov2640_jpeg_reg_tbl[i][0],ov2640_jpeg_reg_tbl[i][1]);  
	}  
}
//OV2640�л�ΪRGB565ģʽ
void OV2640_RGB565_Mode(void) 
{
	u16 i=0;
	//����:RGB565���
	for(i=0;i<(sizeof(ov2640_rgb565_reg_tbl)/2);i++)
	{
		SCCB_WR_Reg(ov2640_rgb565_reg_tbl[i][0],ov2640_rgb565_reg_tbl[i][1]); 
	} 
} 
//�Զ��ع����ò�����,֧��5���ȼ�
const static u8 OV2640_AUTOEXPOSURE_LEVEL[5][8]=
{
	{
		0xFF,0x01,
		0x24,0x20,
		0x25,0x18,
		0x26,0x60,
	},
	{
		0xFF,0x01,
		0x24,0x34,
		0x25,0x1c,
		0x26,0x00,
	},
	{
		0xFF,0x01,	
		0x24,0x3e,	
		0x25,0x38,
		0x26,0x81,
	},
	{
		0xFF,0x01,
		0x24,0x48,
		0x25,0x40,
		0x26,0x81,
	},
	{
		0xFF,0x01,	
		0x24,0x58,	
		0x25,0x50,	
		0x26,0x92,	
	},
}; 
//OV2640�Զ��ع�ȼ�����
//level:0~4
void OV2640_Auto_Exposure(u8 level)
{  
	u8 i;
	u8 *p=(u8*)OV2640_AUTOEXPOSURE_LEVEL[level];
	for(i=0;i<4;i++)
	{ 
		SCCB_WR_Reg(p[i*2],p[i*2+1]); 
	} 
}  
//��ƽ������
//0:�Զ�
//1:̫��sunny
//2,����cloudy
//3,�칫��office
//4,����home
void OV2640_Light_Mode(u8 mode)
{
	u8 regccval=0X5E;//Sunny 
	u8 regcdval=0X41;
	u8 regceval=0X54;
	switch(mode)
	{ 
		case 0://auto 
			SCCB_WR_Reg(0XFF,0X00);	 
			SCCB_WR_Reg(0XC7,0X10);//AWB ON 
			return;  	
		case 2://cloudy
			regccval=0X65;
			regcdval=0X41;
			regceval=0X4F;
			break;	
		case 3://office
			regccval=0X52;
			regcdval=0X41;
			regceval=0X66;
			break;	
		case 4://home
			regccval=0X42;
			regcdval=0X3F;
			regceval=0X71;
			break;	
	}
	SCCB_WR_Reg(0XFF,0X00);	 
	SCCB_WR_Reg(0XC7,0X40);	//AWB OFF 
	SCCB_WR_Reg(0XCC,regccval); 
	SCCB_WR_Reg(0XCD,regcdval); 
	SCCB_WR_Reg(0XCE,regceval);  
}
//ɫ������
//0:-2
//1:-1
//2,0
//3,+1
//4,+2
void OV2640_Color_Saturation(u8 sat)
{ 
	u8 reg7dval=((sat+2)<<4)|0X08;
	SCCB_WR_Reg(0XFF,0X00);		
	SCCB_WR_Reg(0X7C,0X00);		
	SCCB_WR_Reg(0X7D,0X02);				
	SCCB_WR_Reg(0X7C,0X03);			
	SCCB_WR_Reg(0X7D,reg7dval);			
	SCCB_WR_Reg(0X7D,reg7dval); 		
}
//��������
//0:(0X00)-2
//1:(0X10)-1
//2,(0X20) 0
//3,(0X30)+1
//4,(0X40)+2
void OV2640_Brightness(u8 bright)
{
  SCCB_WR_Reg(0xff, 0x00);
  SCCB_WR_Reg(0x7c, 0x00);
  SCCB_WR_Reg(0x7d, 0x04);
  SCCB_WR_Reg(0x7c, 0x09);
  SCCB_WR_Reg(0x7d, bright<<4); 
  SCCB_WR_Reg(0x7d, 0x00); 
}
//�Աȶ�����
//0:-2
//1:-1
//2,0
//3,+1
//4,+2
void OV2640_Contrast(u8 contrast)
{
	u8 reg7d0val=0X20;//Ĭ��Ϊ��ͨģʽ
	u8 reg7d1val=0X20;
  	switch(contrast)
	{
		case 0://-2
			reg7d0val=0X18;	 	 
			reg7d1val=0X34;	 	 
			break;	
		case 1://-1
			reg7d0val=0X1C;	 	 
			reg7d1val=0X2A;	 	 
			break;	
		case 3://1
			reg7d0val=0X24;	 	 
			reg7d1val=0X16;	 	 
			break;	
		case 4://2
			reg7d0val=0X28;	 	 
			reg7d1val=0X0C;	 	 
			break;	
	}
	SCCB_WR_Reg(0xff,0x00);
	SCCB_WR_Reg(0x7c,0x00);
	SCCB_WR_Reg(0x7d,0x04);
	SCCB_WR_Reg(0x7c,0x07);
	SCCB_WR_Reg(0x7d,0x20);
	SCCB_WR_Reg(0x7d,reg7d0val);
	SCCB_WR_Reg(0x7d,reg7d1val);
	SCCB_WR_Reg(0x7d,0x06);
}
//��Ч����
//0:��ͨģʽ    
//1,��Ƭ
//2,�ڰ�   
//3,ƫ��ɫ
//4,ƫ��ɫ
//5,ƫ��ɫ
//6,����	    
void OV2640_Special_Effects(u8 eft)
{
	u8 reg7d0val=0X00;//Ĭ��Ϊ��ͨģʽ
	u8 reg7d1val=0X80;
	u8 reg7d2val=0X80; 
	switch(eft)
	{
		case 1://��Ƭ
			reg7d0val=0X40; 
			break;	
		case 2://�ڰ�
			reg7d0val=0X18; 
			break;	 
		case 3://ƫ��ɫ
			reg7d0val=0X18; 
			reg7d1val=0X40;
			reg7d2val=0XC0; 
			break;	
		case 4://ƫ��ɫ
			reg7d0val=0X18; 
			reg7d1val=0X40;
			reg7d2val=0X40; 
			break;	
		case 5://ƫ��ɫ
			reg7d0val=0X18; 
			reg7d1val=0XA0;
			reg7d2val=0X40; 
			break;	
		case 6://����
			reg7d0val=0X18; 
			reg7d1val=0X40;
			reg7d2val=0XA6; 
			break;	 
	}
	SCCB_WR_Reg(0xff,0x00);
	SCCB_WR_Reg(0x7c,0x00);
	SCCB_WR_Reg(0x7d,reg7d0val);
	SCCB_WR_Reg(0x7c,0x05);
	SCCB_WR_Reg(0x7d,reg7d1val);
	SCCB_WR_Reg(0x7d,reg7d2val); 
}
//��������
//sw:0,�رղ���
//   1,��������(ע��OV2640�Ĳ����ǵ�����ͼ�������)
void OV2640_Color_Bar(u8 sw)
{
	u8 reg;
	SCCB_WR_Reg(0XFF,0X01);
	reg=SCCB_RD_Reg(0X12);
	reg&=~(1<<1);
	if(sw)reg|=1<<1; 
	SCCB_WR_Reg(0X12,reg);
}
//����ͼ��������� 
//sx,sy,��ʼ��ַ
//width,height:���(��Ӧ:horizontal)�͸߶�(��Ӧ:vertical)
void OV2640_Window_Set(u16 sx,u16 sy,u16 width,u16 height)
{
	u16 endx;
	u16 endy;
	u8 temp; 
	endx=sx+width/2;	//V*2
 	endy=sy+height/2;
	
	SCCB_WR_Reg(0XFF,0X01);			
	temp=SCCB_RD_Reg(0X03);				//��ȡVref֮ǰ��ֵ
	temp&=0XF0;
	temp|=((endy&0X03)<<2)|(sy&0X03);
	SCCB_WR_Reg(0X03,temp);				//����Vref��start��end�����2λ
	SCCB_WR_Reg(0X19,sy>>2);			//����Vref��start��8λ
	SCCB_WR_Reg(0X1A,endy>>2);			//����Vref��end�ĸ�8λ
	
	temp=SCCB_RD_Reg(0X32);				//��ȡHref֮ǰ��ֵ
	temp&=0XC0;
	temp|=((endx&0X07)<<3)|(sx&0X07);
	SCCB_WR_Reg(0X32,temp);				//����Href��start��end�����3λ
	SCCB_WR_Reg(0X17,sx>>3);			//����Href��start��8λ
	SCCB_WR_Reg(0X18,endx>>3);			//����Href��end�ĸ�8λ
}
//����ͼ�������С
//OV2640���ͼ��Ĵ�С(�ֱ���),��ȫ�ɸĺ���ȷ��
//width,height:���(��Ӧ:horizontal)�͸߶�(��Ӧ:vertical),width��height������4�ı���
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 OV2640_OutSize_Set(u16 width,u16 height)
{
	u16 outh;
	u16 outw;
	u8 temp; 
	if(width%4)return 1;
	if(height%4)return 2;
	outw=width/4;
	outh=height/4; 
	SCCB_WR_Reg(0XFF,0X00);	
	SCCB_WR_Reg(0XE0,0X04);			
	SCCB_WR_Reg(0X5A,outw&0XFF);		//����OUTW�ĵͰ�λ
	SCCB_WR_Reg(0X5B,outh&0XFF);		//����OUTH�ĵͰ�λ
	temp=(outw>>8)&0X03;
	temp|=(outh>>6)&0X04;
	SCCB_WR_Reg(0X5C,temp);				//����OUTH/OUTW�ĸ�λ 
	SCCB_WR_Reg(0XE0,0X00);	
	return 0;
}
//����ͼ�񿪴���С
//��:OV2640_ImageSize_Setȷ������������ֱ��ʴӴ�С.
//�ú������������Χ������п���,����OV2640_OutSize_Set�����
//ע��:�������Ŀ�Ⱥ͸߶�,������ڵ���OV2640_OutSize_Set�����Ŀ�Ⱥ͸߶�
//     OV2640_OutSize_Set���õĿ�Ⱥ͸߶�,���ݱ��������õĿ�Ⱥ͸߶�,��DSP
//     �Զ��������ű���,������ⲿ�豸.
//width,height:���(��Ӧ:horizontal)�͸߶�(��Ӧ:vertical),width��height������4�ı���
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 OV2640_ImageWin_Set(u16 offx,u16 offy,u16 width,u16 height)
{
	u16 hsize;
	u16 vsize;
	u8 temp; 
	if(width%4)return 1;
	if(height%4)return 2;
	hsize=width/4;
	vsize=height/4;
	SCCB_WR_Reg(0XFF,0X00);	
	SCCB_WR_Reg(0XE0,0X04);					
	SCCB_WR_Reg(0X51,hsize&0XFF);		//����H_SIZE�ĵͰ�λ
	SCCB_WR_Reg(0X52,vsize&0XFF);		//����V_SIZE�ĵͰ�λ
	SCCB_WR_Reg(0X53,offx&0XFF);		//����offx�ĵͰ�λ
	SCCB_WR_Reg(0X54,offy&0XFF);		//����offy�ĵͰ�λ
	temp=(vsize>>1)&0X80;
	temp|=(offy>>4)&0X70;
	temp|=(hsize>>5)&0X08;
	temp|=(offx>>8)&0X07; 
	SCCB_WR_Reg(0X55,temp);				//����H_SIZE/V_SIZE/OFFX,OFFY�ĸ�λ
	SCCB_WR_Reg(0X57,(hsize>>2)&0X80);	//����H_SIZE/V_SIZE/OFFX,OFFY�ĸ�λ
	SCCB_WR_Reg(0XE0,0X00);	
	return 0;
} 
//�ú�������ͼ��ߴ��С,Ҳ������ѡ��ʽ������ֱ���
//UXGA:1600*1200,SVGA:800*600,CIF:352*288
//width,height:ͼ���Ⱥ�ͼ��߶�
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 OV2640_ImageSize_Set(u16 width,u16 height)
{ 
	u8 temp; 
	SCCB_WR_Reg(0XFF,0X00);			
	SCCB_WR_Reg(0XE0,0X04);			
	SCCB_WR_Reg(0XC0,(width)>>3&0XFF);		//����HSIZE��10:3λ
	SCCB_WR_Reg(0XC1,(height)>>3&0XFF);		//����VSIZE��10:3λ
	temp=(width&0X07)<<3;
	temp|=height&0X07;
	temp|=(width>>4)&0X80; 
	SCCB_WR_Reg(0X8C,temp);	
	SCCB_WR_Reg(0XE0,0X00);				 
	return 0;
}

















