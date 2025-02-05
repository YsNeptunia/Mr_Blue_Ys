#include "pid.h"

/*************************************************************************** 
ֱ����PD��������Kp*Ek+Kd*Ek_D
��ڣ�Med:��е��ֵ(�����Ƕ�)��Angle:��ʵ�Ƕȣ�gyro_Y:��ʵ���ٶ�
���ڣ�PMW��ֵ
******************************************************************************/

float PWM_out;

int Vertical(float Angle,float Gyro_y) 
{
	float Vertical_Kp= -192.0,Vertical_Kd= -0.8;	//ֱ����Kp,Kd   -378*0.6     -1.0*0.6//-320 -1.1
	PWM_out = Vertical_Kp*(Angle-0.2)+Vertical_Kd*(Gyro_y-0.01);	
	
	if(PWM_out>5000)
		PWM_out=5000;
	else if(PWM_out<-5000)
		PWM_out=-5000;
	return PWM_out;	
}


/********************************************************************* 
�ٶȻ�PI��������Kp*Ek+Ki*Ek_S(Ek_S��ƫ��Ļ���)
��ڣ����ұ������⵽����ֵ
���ڣ�                   
**********************************************************************/
int Velocity(int Encoder_Left,int Encoder_Right,int move)
{
	// ����ɾ�̬�����������ھ�̬�洢����ʹ�ñ���������
	static float Velocity,Encoder_Err,Encoder; //�ٶȣ�ƫ�������
	static float Encoder_Integral;					  //��������ֵ����
		
	
	float kp=-50.0,ki=-0.2;//200��	//-50,-0.2
//	float kp=0,ki=0;
	
	// 1.�����ٶ�ƫ�� 	
	Encoder_Err = ((Encoder_Left+Encoder_Right)-0);	
 
	// 2.���ٶ�ƫ�����--��ͨ�˲�--
	
	Encoder *= 0.7;
	Encoder += Encoder_Err*0.3;
	
	
//	Encoder = Encoder_Err*0.3 + Encoder_last*0.7;// ʹ�ò��θ���ƽ��
//	Encoder_last = Encoder; 							// ��ֹ�ٶȹ���Ӱ��ֱ��������������

	// 3.���ٶ�ƫ����ֳ�λ��,ң�ص��ٶ�ͨ�����������ٶȿ������������ٶ�ͻ���ֱ�����Ƶ�Ӱ��	
  Encoder_Integral += Encoder-move;	


	
	

	// 4.�����޷�	
	if(Encoder_Integral>1000)  	Encoder_Integral=1000;   
	if(Encoder_Integral<-1000)	   Encoder_Integral=-1000;           	

//	if(Moto_Flag == 1||Start_Flag ==0) 			Encoder_Integral=0;     		//===����رպ���߸�λ�������
    //5.�ٶȻ��������	
  Velocity=Encoder*kp+Encoder_Integral*ki;
	
	return Velocity;
}

// /*********************************************************************
// ת�򻷣�ϵ��*Z����ٶ�+ϵ��*ң������
// ��ڣ����ҵ�������������ֵ��Z����ٶ�
// **********************************************************************/
// int Turn(int Encoder_Left,int Encoder_Right,float gyro)
// {
//    float Turn_Target,Turn_PWM,Bias,Encoder_temp,Turn_Convert=70,Turn_Count; 
//    float Turn_Amplitude=100,Turn_Kp=10,Turn_Kd=0; 
	
	
// 	 if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude; //ת���޷�    
//     if(Turn_Target<-Turn_Amplitude)  Turn_Target=-Turn_Amplitude;
	
	
// 	 //=============turing PD controller==================//	
// 		Turn_PWM= -Turn_Target*Turn_Kp+gyro*Turn_Kd;
 
//   return Turn_PWM;
// }

