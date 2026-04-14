/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <nextion.h>
#include <encoder.h>
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

// user test values

uint8_t amp_waveform = 0;
uint8_t amp_waveform_1 = 0;

uint8_t Rx_Data[4];
uint8_t ASCOM_Data[8];
volatile uint8_t waveform_enable = 0;
volatile uint8_t page_switch = 0;

uint32_t wave_delay_time_0 = 0;
uint32_t wave_delay_time_1 = 0;
uint16_t counter = 0;

uint16_t pwm_signal_1 = 0;
uint16_t pwm_signal_2 = 0;
uint16_t pwm_value_1 = 1;
uint16_t pwm_value_2 = 1;

uint16_t current_engine_value[2];

float filtered_value_0 = 2160;
float filtered_value_1 = 2160;
float angle_to_display = 0;
float amps_0 = 0;
float amps_1 = 0;

int16_t amps_to_display_0 = 0;
int16_t amps_to_display_1 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t encoder_cnt_get();
float encoder_angle_get(uint16_t positioning);
float get_current_offset(uint16_t dma_adc_index, float offset);
float get_amps(float filter_value);
void encoder_display_angle(uint16_t position);
void Nextion_SendString(char *ID, float info);
void Nextion_Waveform(uint8_t wave1, uint8_t wave2);
void Nextion_SendString_Current(int16_t current1, int16_t current2);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_UART_Receive_IT(&huart1, Rx_Data, 4);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)current_engine_value, 2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	counter = encoder_cnt_get();
	angle_to_display = encoder_angle_get(counter);

	if(page_switch) {
		if(HAL_GetTick() - wave_delay_time_0 > 20) {

			wave_delay_time_0 = HAL_GetTick();

			Nextion_SendString("x0", angle_to_display);
			//encoder_display_angle(counter);
		}
	}

	//pwm_signal_1 = __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_value_1);
	//pwm_signal_2 = __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_value_2);

	// filtr cyfrowy 1 rzędu

	// skalowanie pradu - wzór funkcji liniowej

	filtered_value_0 = get_current_offset(current_engine_value[0], filtered_value_0);
	filtered_value_1 = get_current_offset(current_engine_value[1], filtered_value_1);

	amps_0 = get_amps(filtered_value_0);
	amps_1 = get_amps(filtered_value_1);

	amps_to_display_0 = (int16_t)(amps_0*100);
	amps_to_display_1 = (int16_t)(amps_1*100);

	amp_waveform = (uint8_t)(255.0/42.0)*amps_0 + 22.0*(255.0/42.0);
	amp_waveform_1 = (uint8_t)(255.0/42.0)*amps_1 + 22.0*(255.0/42.0);

	if(waveform_enable) {

		if(HAL_GetTick() - wave_delay_time_0 > 15) {
			wave_delay_time_0 = HAL_GetTick();
			Nextion_Waveform(amp_waveform, amp_waveform_1);
		}

		if(HAL_GetTick() - wave_delay_time_1 > 20) {
			wave_delay_time_1 = HAL_GetTick();
			Nextion_SendString_Current(amps_to_display_0, amps_to_display_1);
		}
	}

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1) {

		if(Rx_Data[1] == 0x00 && Rx_Data[2] == 0x02)
			page_switch = 1;
		else if(Rx_Data[1] == 0x01 && Rx_Data[2] == 0x04)
			page_switch = 0;
		else if(Rx_Data[1] == 0x00 && Rx_Data[2] == 0x06)
			waveform_enable = 1;
		else if(Rx_Data[1] == 0x02 && Rx_Data[2] == 0x03)
			waveform_enable = 0;

		HAL_UART_Receive_IT(&huart1, Rx_Data, 4);
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
