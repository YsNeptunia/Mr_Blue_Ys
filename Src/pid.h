#ifndef __PID_H
#define __PID_H
#include "main.h"

int Vertical(float Angle,float Gyro_y);
int Velocity(int Encoder_Left,int Encoder_Right,int move);

#endif
