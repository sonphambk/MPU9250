/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "IMU/MPU9250.h"
#include "IMU/MPU9250_register.h"
#include "IMU/Madgwick/Madgwick.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define pi 3.141592653589793238462
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float a12, a22, a31, a32, a33;
uint8_t buf[100]={0};

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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		//Get_magnetometer();
		Process_IMU();

		q[0] = q0;
		q[1] = q1;
		q[2] = q2;
		q[3] = q3;
		a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
		a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
		a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
		a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
		a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
		float sinp = a32;
		if (abs(sinp) >= 1)
			pitch = copysign(M_PI/2,sinp);
		else
			pitch =  asin(sinp);
		//pitch = -asinf(a32);
		roll  = atan2f(a31, a33);
		yaw   = atan2f(a12, a22);
		pitch *= 180.0f / pi;
		yaw = atan2f(sinf(yaw),cosf(yaw));
		yaw   *= 180.0f / pi;

		roll  *= 180.0f / pi;

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
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  init_IMU();
  init_magnetometer();
  if (Check_Connection(0x71) == 1)
  {
//	  sprintf((char*)buf,"Value of WHO_I_AM:0x%x\n\r",temp);
//	  HAL_UART_Transmit(&huart3,(uint8_t*)buf,sizeof(buf),1000);
//	  HAL_UART_Transmit(&huart3,(uint8_t*)"MPU9250 is online...\n\r",24,1000);
	  Calibration_IMU();
	  HAL_Delay(1000);

  }
  else
  {
	 // HAL_UART_Transmit(&huart3,(uint8_t*)"check connection...\n",22,1000);
  }

  Calib_magnetometer();
//  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //dt = 0.00055;

	  // max = 339  360  10
//		min  -40  -17  -412
	 // sprintf((char*)buf,"%.2f  %.2f  %.2f\r\n",Gyro_X,Gyro_Y,Gyro_Z);

		//sprintf((char*)buf,"Scale_X:%.2f  Scale_Y:%.2f  Scale_Z:%.2f\r\n",scale_x,scale_y,scale_z);
	  //sprintf((char*)buf,"%.3f  %.3f  %.3f\r\n",Mag_X_calib,Mag_Y_calib,Mag_Z_calib);
		//sprintf((char*)buf,"%.2f  %.2f  %.2f\n",(float)mag_bias[0],(float)mag_bias[1],(float)mag_bias[2]);

	 // sprintf((char*)buf,"yaw_kalman: %.2f\n\r",new_yaw);
	  //sprintf((char*)buf,"qo:%.5f  q1:%.5f  q2:%.5f  q3:%.5f\r\n",q[0],q[1],q[2],q[3]);
	  //sprintf((char*)buf,"qo:%.5f  q1:%.5f  q2:%.5f  q3:%.5f\r\n",q0,q1,q2,q3);
	//  sprintf((char*)buf,"%d  %.d  %d\r\n",Mag_x,Mag_y,Mag_z);

	  //sprintf((char*)buf,"Gyro_X_bias: %.2f \t Gyro_Y_bias: %.2f \t Gyro_Z_bias: %.2f\n\r",Gyro_x_bias,Gyro_y_bias,Gyro_z_bias);

//	  HAL_UART_Transmit(&huart3,(uint8_t*)buf,strlen(buf),1000);
//	  HAL_Delay(10);
	  //HAL_UART_Transmit(&huart3,(uint8_t*)mess,(sizeof(mess)),1000);
	  sprintf((char*)buf,"%.2f  %.2f  %.2f\r\n",roll,pitch,yaw);
	  HAL_UART_Transmit(&huart3,(uint8_t*)buf,sizeof(buf),1000);

	  HAL_Delay(100);





    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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

//#ifdef __GNUC__
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */
//
///**
//  * @brief  Retargets the C library printf function to the USART.
//  * @param  None
//  * @retval None
//  */
//PUTCHAR_PROTOTYPE
//{
//  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
//  return ch;
//}
//int __io_putchar(int ch) {
//HAL_UART_Transmit_IT(&huart3, (uint8_t *)&ch, 1);
//while (UartReady != SET) // Wait for the end of the transfer to avoid 2nd Tx before 1st complete
//{
//}
//UartReady = RESET; // Reset transmission flag
//return ch;
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
