#ifndef _init_h_
#define _init_h_
typedef struct {
	
	//Î»ÖÃ
	int X;
	int Y;
	
	//³É¹¦·¶Î§
	int success_distance;
	//¿¿½ü·¶Î§
	int middle_distance;
//	//Ô¶Àë·¶Î§
//	int far_distance;
	
	//pid²ÎÊý
	float PID_p_x;
	float PID_p_y;
	float PID_d_x;
	float PID_d_y;

}aim_t;

#define TASK_num 		10
#define TASK_node_num	50


#define TASK_1_index	1
#define TASK_2_index	2
#define	TASK_3_index	3
#define TASK_4_index	4
#define TASK_5_index	5
#define TASK_6_index	6
#define TASK_7_index	7
#define TASK_8_index	8


#define A_Task_Finish	0
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

void sys_init(void);
void two_exchange(void);

#endif
