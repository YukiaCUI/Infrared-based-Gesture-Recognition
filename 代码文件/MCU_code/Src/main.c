/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SAMP 8
#define K 2300
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t dma_end_flag = 0, result = 0;
uint32_t AD_DMA[SAMP]={0};
float adc1[SAMP*2]={0};
float Rock_Centre[] = 
		{3685.81461676, 2361.55793226, 2398.58823529, 3623.04099822, 
		 3585.92691622, 1968.82174688, 1608.16934046, 3610.14438503, 
		 3579.26737968, 1573.21033868, 1980.67557932, 3628.56684492, 
		 3636.65418895, 2410.41889483, 2380.05525847, 3706.92869875};
float Scissor_Centre[] = 
		{3641.67158672, 3643.9298893,  3646.26937269, 3758.28782288,
		 2293.78597786, 1348.15498155, 2806.79335793, 1447.67896679,
  	 1292.50922509, 2615.43173432, 1456.73431734, 2430.17712177,
		 3736.11808118, 3688.66420664, 3687.68265683, 3698.05535055};
float Paper_Centre[] = 
		{3631.26689189, 3556.02702703, 3583.10472973, 3643.47297297,
		 2471.62837838, 1768.22297297, 1116.31418919, 3299.77702703,
		 3500.36148649, 1357.77027027, 1646.34797297, 2265.94256757,
		 3637.96959459, 3521.86824324, 3480.91216216, 3539.62837838};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f) {
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)		//DMA采集完成中断服务函数
{
	if(hadc->Instance == ADC1)
	{
		dma_end_flag = 1;
	}
}
int pd(float adc[SAMP*2])
{
	int total = 0;
	float rock_dis = 0, scissor_dis = 0, paper_dis = 0;
	for(int i=0;i<16;i++)
	{
		 if(adc[i]<2300)
			 total++;
		 rock_dis += (adc[i]-Rock_Centre[i])*(adc[i]-Rock_Centre[i]);
		 scissor_dis += (adc[i]-Scissor_Centre[i])*(adc[i]-Scissor_Centre[i]);
		 paper_dis += (adc[i]-Paper_Centre[i])*(adc[i]-Paper_Centre[i]);
	}
	if(total==0)
		return 3;
	int minn, flag=0;
	minn = rock_dis;
	if(scissor_dis < minn)
	{
		minn = scissor_dis;
		flag = 1;
	}
	if(paper_dis < minn)
	{
		minn = paper_dis;
		flag = 2;
	}
	return flag;	
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_TIM_Base_Start(&htim3);
	HAL_Delay(10);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//set 1 = open; set 0 = close
		HAL_GPIO_WritePin(BANK0_GPIO_Port,BANK0_Pin,1);
		HAL_GPIO_WritePin(BANK1_GPIO_Port,BANK1_Pin,0);
		HAL_Delay(10);
		HAL_ADC_Start_DMA(&hadc1, AD_DMA, SAMP);
		HAL_Delay(100);
		if(dma_end_flag == 1)
		{
			dma_end_flag = 0;
			for(uint8_t i=0;i<8;i++)
			{
				adc1[i] = (float)AD_DMA[i];
				printf("adc%d = %f\r\n",i,adc1[i]);
			}
		}
		
		HAL_GPIO_WritePin(BANK0_GPIO_Port,BANK0_Pin,0);
		HAL_GPIO_WritePin(BANK1_GPIO_Port,BANK1_Pin,1);
		HAL_Delay(10);
		HAL_ADC_Start_DMA(&hadc1, AD_DMA, SAMP);
		HAL_Delay(100);
		if(dma_end_flag == 1)
		{
			dma_end_flag = 0;
			for(uint8_t i=0;i<8;i++)
			{
				adc1[i+8] = (float)AD_DMA[i];
				printf("adc%d = %f\r\n",i+8,adc1[i+8]);
			}
		}
		
		result = pd(adc1);
		
		if(result == 0)
		{
			//0 = rock
			HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		}
		if(result == 1)
		{
			//1 = scissor
			HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		}
		if(result == 2)
		{
			//2 = paper
			HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		}
		if(result == 3)
		{
			HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		}
		
		HAL_Delay(10);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
