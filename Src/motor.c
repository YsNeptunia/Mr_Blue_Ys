#include "motor.h"
#include "tim.h"

/**************************************************************************
�������ܣ���λʱ���ȡ����������
��ڲ�������ʱ��
����  ֵ���ٶ�ֵ
**************************************************************************/
uint32_t Read_Encoder(uint8_t TIM_x)	//��������������10ms
{
	
	uint32_t Encoder_TIM;
	switch(TIM_x)
	{
		case 3: //������λ����Ϊ�ٶ�ֵ
					Encoder_TIM = (short)__HAL_TIM_GET_COUNTER(&htim3);//�ɼ���������ֵ������
					Encoder_TIM = Encoder_TIM*2.2;
				  __HAL_TIM_SET_COUNTER(&htim3,0);							//����ʱ���ļ���ֵ����
		break;	
		
		case 4: Encoder_TIM = (short)__HAL_TIM_GET_COUNTER(&htim4);
					__HAL_TIM_SET_COUNTER(&htim4,0);
		break;
		
		default: Encoder_TIM = 0;
	
	}
  return Encoder_TIM;	
}

void car_go(int left,int right)
{
    if(left > 0)
    {
		HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET);
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,left);
    }
    else
    {
		HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,-left);
    }

    if(right > 0)
     {
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET);
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,right);
     }

    else
     {
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,-right);
     }
}

void car_stop()
{
    HAL_GPIO_WritePin(Encoder_Left_A_GPIO_Port,Encoder_Left_A_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Encoder_Right_A_GPIO_Port,Encoder_Right_A_Pin,GPIO_PIN_RESET);
    __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,0);
	
    HAL_GPIO_WritePin(Encoder_Left_B_GPIO_Port,Encoder_Left_B_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Encoder_Right_B_GPIO_Port,Encoder_Right_B_Pin,GPIO_PIN_RESET);
    __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,0);

}

void car_back()
{
    HAL_GPIO_WritePin(Encoder_Left_A_GPIO_Port,Encoder_Left_A_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(Encoder_Right_A_GPIO_Port,Encoder_Right_A_Pin,GPIO_PIN_RESET);
    __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,500);
	
    HAL_GPIO_WritePin(Encoder_Left_B_GPIO_Port,Encoder_Left_B_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(Encoder_Right_B_GPIO_Port,Encoder_Right_B_Pin,GPIO_PIN_RESET);
    __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,500);

}


// void car_go(int left,int right)
// {
//     if(left > 0)
//     {
//         HAL_GPIO_WritePin(motor_A1_GPIO_Port,motor_A1_Pin,GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(motor_A2_GPIO_Port,motor_A2_Pin,GPIO_PIN_SET);
//         __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,left);
//     }
//     else
//     {
//         HAL_GPIO_WritePin(motor_A1_GPIO_Port,motor_A1_Pin,GPIO_PIN_SET);
//         HAL_GPIO_WritePin(motor_A2_GPIO_Port,motor_A2_Pin,GPIO_PIN_RESET);
//         __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,-left);
//     }

//     if(right > 0)
//      {
//         HAL_GPIO_WritePin(motor_B1_GPIO_Port,motor_B1_Pin,GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(motor_B2_GPIO_Port,motor_B2_Pin,GPIO_PIN_SET);
//         __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,right);
//      }

//     else
//      {
//         HAL_GPIO_WritePin(motor_B1_GPIO_Port,motor_B1_Pin,GPIO_PIN_SET);
//         HAL_GPIO_WritePin(motor_B2_GPIO_Port,motor_B2_Pin,GPIO_PIN_RESET);
//         __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,-right );
//      }
// }

// void car_stop()
// {
//     HAL_GPIO_WritePin(motor_A1_GPIO_Port,motor_A1_Pin,GPIO_PIN_RESET);
//     HAL_GPIO_WritePin(motor_A2_GPIO_Port,motor_A2_Pin,GPIO_PIN_RESET);
//     __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,0);
	
//     HAL_GPIO_WritePin(motor_B1_GPIO_Port,motor_B1_Pin,GPIO_PIN_RESET);
//     HAL_GPIO_WritePin(motor_B2_GPIO_Port,motor_B2_Pin,GPIO_PIN_RESET);
//     __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,0);

// }

// void car_back()
// {
//     HAL_GPIO_WritePin(motor_A1_GPIO_Port,motor_A1_Pin,GPIO_PIN_SET);
//     HAL_GPIO_WritePin(motor_A2_GPIO_Port,motor_A2_Pin,GPIO_PIN_RESET);
//     __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,500);
	
//     HAL_GPIO_WritePin(motor_B1_GPIO_Port,motor_B1_Pin,GPIO_PIN_SET);
//     HAL_GPIO_WritePin(motor_B2_GPIO_Port,motor_B2_Pin,GPIO_PIN_RESET);
//     __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,500);

// }
