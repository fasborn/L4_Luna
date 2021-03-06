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
#define TEST_2
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
//	SET_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE);
	uint8_t data[9];
	//HAL_UART_Receive_IT(&huart2, data, 9);
	newos();
	HAL_UART_Receive_IT(&huart2, data, 9);
	uint8_t data_2[9];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		#ifdef TEST_1
		if(UART4->ISR&USART_ISR_RXNE){
//			int k;
//			k	= UART4->RDR;
//			uint16_t data;
//			data = k & 0x1FF;
//			
//			//HAL_UART_Transmit(&huart4, (uint8_t*) data, 1, 1000);
//			UART4->TDR = k;
			
			uint8_t k;// = UART4->RDR;
			
			HAL_UART_Receive(&huart4, &k, 1, 1000);
			HAL_UART_Transmit(&huart4, &k, 1, 1000);
			HAL_UART_Transmit(&huart4, (uint8_t*) "Hi\r\n", 4, 1000);
//			HAL_Delay(1000);	
		}
		
		#endif
		
		#ifdef TEST_3
		if(USART2->ISR&USART_ISR_RXNE){
			uint8_t k;
			k	= USART2->RDR;
//			uint16_t data;
//			data = k & 0x1FF;
			
			//HAL_UART_Transmit(&huart4, (uint8_t*) data, 1, 1000);
//			UART4->TDR = (uint8_t) data;
			
//			uint8_t k;// = UART4->RDR;
			
//			HAL_UART_Receive(&huart2, &k, 1, 1000);
			HAL_UART_Transmit(&huart4, &k, 1, 1000);
			HAL_UART_Transmit(&huart4, (uint8_t*) "\r\n", 2, 1000);
		}
	#endif
	
//	HAL_UART_Transmit(&huart4, (uint8_t*) "Hi\r\n", 4, 1000);
//	
//	HAL_Delay(1000);
		if(huart2.RxXferCount==0)

		{
//			strncpy(data_2, data, 9);
			//HAL_UART_Transmit(&huart4, (uint8_t*) get_data(data), 1, 1000);
//			HAL_UART_Transmit(&huart4, data, 9, 1000);
//			uint8_t k = j&0xff;
			//uint8_t j = 0;// = data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7];


			
//			for(int i = 0; i<8; i++){
////				HAL_UART_Transmit(&huart4, &data[i], 1, 1000);
//				j+=data[i];
////				k = j&0xff;
////				HAL_UART_Transmit(&huart4, &k, 1, 1000);
//			}
			int j = 0;
			j = data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7];
			uint8_t k = 0;
			k = j&0xff;
			
			int os = 0;

			os = get_data(data);
			
			uint8_t left = os>>8;
			uint8_t right = os & 0xff;
			
//			HAL_UART_Transmit(&huart4, &k, 1, 1000);
//			HAL_UART_Transmit(&huart4, &left, 1, 1000);
			HAL_UART_Transmit(&huart4, &right, 1, 1000);
//			HAL_UART_Transmit(&huart4, (uint8_t*) (j&0xff00)>>, 1, 1000);
//			HAL_UART_Transmit(&huart4, (uint8_t*) k, 1, 1000);
			HAL_UART_Receive_IT(&huart2, data, 9);
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
