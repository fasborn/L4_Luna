/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <mine.h>
#include "sk6812.h"
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
extern uint32_t BUF_DMA [ARRAY_LEN];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TEST_8

uint8_t rate[] = {0x5a, 0x06, 0x03, 0x32, 0x00, 0x00};

int leds(int i, int number){
  if ((i+number)<0){
    return 0;
  }
  else if((i+number)>299){
    return 299;
  }
  return i+number;
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
  MX_DMA_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

#ifdef TEST_1
	uint8_t data[9];
	newos();
	HAL_UART_Receive_IT(&huart2, data, 9);

#endif


#ifdef TEST_3

	//HAL_UART_Transmit(&huart2, rate, 6, 1000);
	
	HAL_UART_Transmit(&huart4, "Hi\r\n", 4, 1000);

	uint8_t data[9];
	newos();
	HAL_UART_Receive_IT(&huart2, data, 9);

#endif

#ifdef TEST_5
	uint8_t data[9];
	newos();
	HAL_UART_Receive_IT(&huart2, data, 9);

#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
//	for(int i = 0; i<ARRAY_LEN; i++){
//		BUF_DMA[i] = 0;
//	}
	
//	fill_buffer(0x01000000, 0,  1);
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, (ARRAY_LEN*2)+130);		
//	HAL_Delay(12);
//	
//	fill_buffer(0x01000000, 1,  2);
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, (ARRAY_LEN*2)+1);		
//	HAL_Delay(12);
//	
//	fill_buffer(0x01000000, 2,  3);
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, (ARRAY_LEN*2)+1);		
//	HAL_Delay(12);	

		#ifdef TEST_6
		
			fill_buffer(0xFF000000, 0,  5);
			HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, (ARRAY_LEN*2)+130);
		#endif
	
//	uint32_t i = 0;
	
  while (1)
  {
//			i+=10;
//			fill_buffer(0x0000FF00, 0,  297);
//			HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, (ARRAY_LEN*2)+130);
//			HAL_Delay(100);
//			if (i >= (0xFFFFFFFF - 0x12C)){
//				i = 0;
//			}

		#ifdef TEST_1
		if(huart2.RxXferCount==0)
		{
//			int j = 0;
//			j = data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7];
//			uint8_t k = 0;
//			k = j&0xff;
//			
//			int os = 0;

//			os = get_data(data);
//			
//			uint8_t left = os>>8;
//			uint8_t right = os & 0xff;
			
			//HAL_UART_Transmit(&huart4, &right, 1, 1000);
			//HAL_UART_Receive_IT(&huart2, data, 9);

			int j = 0;
			j = data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7];
			uint8_t k = 0;
			k = j&0xff;
			
			
			int os = 0;

			os = get_data(data);

			uint8_t left = os>>8;
			uint8_t right = os & 0xff;


			//HAL_UART_Transmit(&huart4, &k, 1, 1000);
			HAL_UART_Transmit(&huart4, &right, 1, 1000);
			HAL_UART_Receive_IT(&huart2, data, 9);
			//HAL_UART_Transmit(&huart4, data, 9, 1000);
			//HAL_Delay(1000);
		}
		#endif
		
		#ifdef TEST_2
		for (int yas = 0; yas < LED_COUNT; yas++ ) {
			
			fill_buffer(0x01000000, leds(yas, -20),  leds(yas, +20));
			HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, 19329);		
			HAL_Delay(12);
		}
		#endif
		
		#ifdef TEST_4
		for (int yas = 0; yas < LED_COUNT; yas++ ) {
			
			fill_buffer(0x0F0F0000, leds(yas, -2),  leds(yas, +2));
			//fill_buffer(0x000000FF, 0,  leds(yas, -2));
			//fill_buffer(0x000000FF, leds(yas, +2),  297);
			HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, (ARRAY_LEN*2)+130);		
			HAL_Delay(40);
		}
		#endif		
		
		#ifdef TEST_3
		if(huart2.RxXferCount==0)
		{
			int os = 0;

			os = get_data(data);

			uint8_t left = os>>8;
			uint8_t right = os & 0xff;

			HAL_UART_Transmit(&huart4, &right, 1, 1000);
			HAL_UART_Receive_IT(&huart2, data, 9);
		}
		#endif
		
		#ifdef TEST_5
		
		if(huart2.RxXferCount==0)
		{
			int os = 0;

			os = get_data(data);

			uint8_t left = os>>8;
			uint8_t right = os & 0xff;

			float temp = get_position(os/100.0);
			int to_fill = (int) temp;
			
		
			HAL_UART_Transmit(&huart4, &right, 1, 1000);
			HAL_UART_Receive_IT(&huart2, data, 9);
			
			fill_buffer(0xFF000000, leds(to_fill, -1),  leds(to_fill, +1));
			HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, (ARRAY_LEN*2)+130);		
		}		
		

		#endif
		
		
		#ifdef TEST_7
			for(int i = DELAY_LEN; i<ARRAY_LEN-DELAY_LEN; i++){
				BUF_DMA[i] = LOW;
			}			
			fill_led(0xFF000000, 30);
			fill_buff_leds(0x00FF0000, 5, 15);
			
			fill_with_background(0xFF000000, 6, 8, 0x00FF0000);
			
			HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, (ARRAY_LEN*2)+130);			
			HAL_Delay(40);
		#endif
			
		#ifdef TEST_8
		for (int yas = 0; yas < LED_COUNT; yas++ ) {
			clear_buffer();
			fill_with_background(0x0000FF00, leds(yas, -2),  leds(yas, +2), 0x000000FF);
			//fill_buffer(0x0F0F0000, leds(yas, -2),  leds(yas, +2));
			HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, (ARRAY_LEN*2)+130);		
			HAL_Delay(40);
		}
		#endif			
			
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART4;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
