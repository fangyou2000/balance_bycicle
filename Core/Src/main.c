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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//PID输出限幅值
#define  limit_pid_max 2000
#define  limit_pid_min 700

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

#include "mpu6050.h"
#include <stdio.h>

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
int DMP_test(void);
//int DMP_gett(void);
int pd_control(float);
unsigned char mpu_dmp_get_data(float *pitch,float *roll,float *yaw);
unsigned char tx_data[20];
unsigned char sumcheck = 0;
unsigned char addcheck = 0;
unsigned char i = 0;
int pwmval;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  pwmval = 1200;  //500是0度，每加100转9度；最大为180度，对应2500
	uint8_t tx = 49;
		int a;
	float pitch,roll,yaw;         //欧拉角
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Transmit(&huart1,"hello\n",6,HAL_MAX_DELAY/4);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	//HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 1200);//duty is 5%
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 12000);//duty is 60%
	printf("回正");
  OLED_Init();//OLED初始化  
	OLED_Clear();//清屏
	//参数分别是列号、行号和输出的字
    OLED_ShowCHinese(18,0,0);//硬
		OLED_ShowCHinese(36,0,1);//件
		OLED_ShowCHinese(54,0,2);//课
		OLED_ShowCHinese(72,0,3);//设
	OLED_ShowString(6,3,"speed is: m/s");//显示字符串
		OLED_ShowString(0,6,"yingjian");  
		OLED_ShowString(65,6,"keshe");  
		OLED_ShowNum(78,3,1,1,1);//参数是列、行、数值、长度、大小

		//printf("Uart init OK!\r\n");
		DMP_test();
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//uint8_t rx = 0; //
		//HAL_UART_Receive(&huart1, &rx, 1, HAL_MAX_DELAY);

//	  if (rx == '1')
//		{HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
//		printf("engien on");}
				
				if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
				{
				//printf("%f\n",roll);
				static int pid_out = 0;
				pid_out = pd_control(roll);//执行pd控制
        //PID输出幅度限幅
				//printf("%d\n",pid_out);//printf("限幅%d",pid_out);
        if(pid_out > limit_pid_max){pid_out = limit_pid_max;}
        if(pid_out < limit_pid_min){pid_out = limit_pid_min;}
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pid_out);//更新舵机角度	
					//printf("changing");
				}
		 HAL_Delay(10);
			//printf("while is working");
  }//end of while
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	printf("engien on");
	//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	static uint16_t TIM3_Cnt1 = 0;
    //
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
