//author:ora
//email:1301912993@qq.com

#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"

/*
*功能:初始化舵机
*参数:
*	参1:频率(HZ)
*返回值:
*	void
*/
void motor_init(u32 Fre);
void set_angle(float x_angle, float y_angle);


#endif
