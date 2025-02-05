#include "pid.h"

/*************************************************************************** 
直立环PD控制器：Kp*Ek+Kd*Ek_D
入口：Med:机械中值(期望角度)，Angle:真实角度，gyro_Y:真实角速度
出口：PMW数值
******************************************************************************/

float PWM_out;

int Vertical(float Angle,float Gyro_y) 
{
	float Vertical_Kp= -192.0,Vertical_Kd= -0.8;	//直立环Kp,Kd   -378*0.6     -1.0*0.6//-320 -1.1
	PWM_out = Vertical_Kp*(Angle-0.2)+Vertical_Kd*(Gyro_y-0.01);	
	
	if(PWM_out>5000)
		PWM_out=5000;
	else if(PWM_out<-5000)
		PWM_out=-5000;
	return PWM_out;	
}


/********************************************************************* 
速度环PI控制器：Kp*Ek+Ki*Ek_S(Ek_S：偏差的积分)
入口：左右编码器测到的数值
出口：                   
**********************************************************************/
int Velocity(int Encoder_Left,int Encoder_Right,int move)
{
	// 定义成静态变量，保存在静态存储器，使得变量不丢掉
	static float Velocity,Encoder_Err,Encoder; //速度，偏差，编码器
	static float Encoder_Integral;					  //编码器数值积分
		
	
	float kp=-50.0,ki=-0.2;//200倍	//-50,-0.2
//	float kp=0,ki=0;
	
	// 1.计算速度偏差 	
	Encoder_Err = ((Encoder_Left+Encoder_Right)-0);	
 
	// 2.对速度偏差进行--低通滤波--
	
	Encoder *= 0.7;
	Encoder += Encoder_Err*0.3;
	
	
//	Encoder = Encoder_Err*0.3 + Encoder_last*0.7;// 使得波形更加平滑
//	Encoder_last = Encoder; 							// 防止速度过大影响直立环的正常工作

	// 3.对速度偏差积分出位移,遥控的速度通过积分融入速度控制器，减缓速度突变对直立控制的影响	
  Encoder_Integral += Encoder-move;	


	
	

	// 4.积分限幅	
	if(Encoder_Integral>1000)  	Encoder_Integral=1000;   
	if(Encoder_Integral<-1000)	   Encoder_Integral=-1000;           	

//	if(Moto_Flag == 1||Start_Flag ==0) 			Encoder_Integral=0;     		//===电机关闭后或者复位清除积分
    //5.速度环控制输出	
  Velocity=Encoder*kp+Encoder_Integral*ki;
	
	return Velocity;
}

// /*********************************************************************
// 转向环：系数*Z轴角速度+系数*遥控数据
// 入口：左右电机编码器测得数值，Z轴角速度
// **********************************************************************/
// int Turn(int Encoder_Left,int Encoder_Right,float gyro)
// {
//    float Turn_Target,Turn_PWM,Bias,Encoder_temp,Turn_Convert=70,Turn_Count; 
//    float Turn_Amplitude=100,Turn_Kp=10,Turn_Kd=0; 
	
	
// 	 if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude; //转速限幅    
//     if(Turn_Target<-Turn_Amplitude)  Turn_Target=-Turn_Amplitude;
	
	
// 	 //=============turing PD controller==================//	
// 		Turn_PWM= -Turn_Target*Turn_Kp+gyro*Turn_Kd;
 
//   return Turn_PWM;
// }

