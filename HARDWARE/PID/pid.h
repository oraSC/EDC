#ifndef _pid_h_
#define _pid_h_


typedef struct
{
	float Pv;			//当前值
	float Sv;			//目标值
	
	//误差
	float E_now;		//当前偏差
	float E_last;		//上一次偏差
	float E_lastlast;	//上上次偏差
	float E_sum;		//偏差总和

	//pid系数
	float Kp;			//比例系数
	float Ki;			//积分系数
	float Kd;			//微分系数
	
	//输出
	float Pout;			//比例输出
	float Iout;			//积分输出
	float Dout;			//微分输出
	float out;			//总输出
	
	
	//小球位置参数
	int ball_max_X;
	int ball_max_Y;
	int ball_min_X;
	int ball_min_Y;
	int ball_center_X;
	int ball_center_Y;

}pid_t;




void pid_init(pid_t *Pid, float Kp, float Ki, float Kd);
void TIM3_init(int Fre);




#endif

