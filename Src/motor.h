#ifndef __MOTOR_H
#define __MOTOR_H
#include "main.h"

uint32_t Read_Encoder(uint8_t TIM_x);	//±àÂëÆ÷¿ØÖÆÖÜÆÚ10ms
void car_go(int left,int right);
void car_stop(void);
void car_back(void);

#endif
