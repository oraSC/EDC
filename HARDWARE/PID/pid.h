#ifndef _pid_h_
#define _pid_h_


typedef struct
{
	float Pv;			//��ǰֵ
	float Sv;			//Ŀ��ֵ
	
	//���
	float E_now;		//��ǰƫ��
	float E_last;		//��һ��ƫ��
	float E_lastlast;	//���ϴ�ƫ��
	float E_sum;		//ƫ���ܺ�

	//pidϵ��
	float Kp;			//����ϵ��
	float Ki;			//����ϵ��
	float Kd;			//΢��ϵ��
	
	//���
	float Pout;			//�������
	float Iout;			//�������
	float Dout;			//΢�����
	float out;			//�����
	
	
	//С��λ�ò���
	int ball_max_X;
	int ball_max_Y;
	int ball_min_X;
	int ball_min_Y;
	int ball_center_X;
	int ball_center_Y;
	int ball_last_center_X;
	int ball_last_center_Y;

}pid_t;




void pid_init(pid_t *Pid, float Kp, float Ki, float Kd);
void TIM3_init(int Fre);




#endif

