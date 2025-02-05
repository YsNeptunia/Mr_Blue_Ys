/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "OLED.h"
// #include "mpu6050.h"
// #include "inv_mpu.h"
// #include "inv_mpu_dmp_motion_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char Buff[50];//存储数据
uint8_t Data;	//暂存数据
int Cnt = 0;	//计数
 
/* 蓝牙回调函数 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
			Buff[Cnt++] = (char)Data;
			if(Buff[Cnt-1] == 0x0A)//判断是否为回车结尾
      {
        OLED_ShowString(4,1,(char *)Buff);//oled显示接收到的字符串
        Cnt = 0;
      }
			// {
			// 	printf("\n蓝牙返回的数据为：\n");
			// 	HAL_UART_Transmit_IT(&huart1, (uint8_t *)Buff, Cnt);//显示在串口助手
			// 	
			// }
		HAL_UART_Receive_IT(&huart1, &Data, sizeof(Data));//继续接收数据
	}
}
//外部中断
uint8_t cnt_data;
// float pitch,roll,yaw;
float pitch,roll,yaw; 								  			 //欧拉角(姿态角)
short aacx,aacy,aacz;													 //加速度传感器原始数据
short gyrox,gyroy,gyroz;											 //陀螺仪原始数据
float temp; 								  								 //温度
int ver_out,vel_out,PWM_end,move;
uint32_t Encoder_Left,Encoder_Right,a;
// uint8_t keyflag = 1;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT_Pin)
    {
        if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
        {
          if(++cnt_data == 4)//刷新速率
          {
            // OLED_ShowSignedNum(1,8,(int32_t)(pitch * 10000),7);
            // OLED_ShowSignedNum(2,8,(int32_t)(roll * 10000),7);
            // OLED_ShowSignedNum(3,8,(int32_t)(yaw * 10000),7);

            OLED_ShowNum(1,8,ver_out,7);
            OLED_ShowNum(2,8,vel_out,7);
            OLED_ShowNum(3,8,PWM_end,7);

            cnt_data = 0;
          }
        }
        HAL_GPIO_TogglePin(INT_GPIO_Port, INT_Pin);
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &Data, sizeof(Data));
  // HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//PWMA启动函数
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//PWMB启动函数
	HAL_TIM_Base_Start_IT(&htim1);//开启10ms中断
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);//编码器
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim3);//开启中断
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1);//编码器
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim4);//开启中断	

  // __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,3600);
  // __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,3600);
  // Set_Pwmb(3600);
  // Set_Pwma(3600);

  OLED_Init();//OLED初始化

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  OLED_ShowString(1,1,"MPU Initialzing");
  OLED_ShowString(2,1,"Please");
  OLED_ShowString(3,1,"Wait...");
  OLED_DrawBMP(70,2,120,7,kaf);
  while(mpu_dmp_init()) HAL_Delay(50);
  OLED_Clear();
  HAL_Delay(20);
  OLED_ShowString(4,1,"INIT FINISHED!");
  OLED_ShowString(1,1,"ver:");
  OLED_ShowString(2,1,"vel:");
  OLED_ShowString(3,1,"PWM:");
  while(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin));

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    mpu_dmp_get_data(&pitch,&roll,&yaw);			//得到姿态角即欧拉角
		MPU6050_Read_Gyro();	//得到陀螺仪数据,读取角速度			

		ver_out = Vertical(pitch,Mpu6050_Data.Gyro_Y);
		vel_out = Velocity(Encoder_Left,Encoder_Right,move);
		PWM_end = ver_out + vel_out;
		
    // if(keyflag)
		  car_go(PWM_end*0.9,PWM_end); 
    // else
    //   car_stop();
		// mpu_dmp_get_data(&pitch,&roll,&yaw);
		// printf("%.3f,%.3f,%.3f,%d\r\n",pitch,roll,yaw,MPU_Get_Temperature());

    // if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == 0){
    //     Set_Pwmb(-7199);
    //     Set_Pwma(-7199);
    // }
    // else{
    //    Set_Pwmb(3000);
    //    Set_Pwma(3000);
    // }
    // HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    // HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t Encoder_time;//,keycnt
	if (htim == (&htim1))
	{
		Encoder_time++;
		// OLED_cnt++;
		
// 		if(OLED_cnt == 500)
// 		{
// //			OLED_ShowSignedNum(3,1,Encoder_Left,3);
// //			OLED_ShowSignedNum(4,1,Encoder_Right,3);
// 			OLED_ShowSignedNum(3,1,move/10,3);
// 			OLED_cnt = 0;
// 		}
		// if(keycnt >= 80)
		// {
		// 	if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == 0){
    //     keyflag = 1;
    //   }
    //   else  keyflag = 0;
		// 	keycnt=0;
		// }
		
		if(Encoder_time >= 10)//编码器控制周期10ms
		{
			Encoder_Left = Read_Encoder(3);
			Encoder_Right = Read_Encoder(4);
			Encoder_time = 0;
			
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
