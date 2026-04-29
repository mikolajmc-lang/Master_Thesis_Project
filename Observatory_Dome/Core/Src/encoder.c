/*
 * encoder.c
 *
 *  Created on: Mar 13, 2026
 *      Author: 48694
 */

#include "main.h"
#include "usart.h"
#include "tim.h"
#include "i2c.h"

#include <encoder.h>
#include <nextion.h>
#include <vl53l0x_api.h>

#define alpha 0.05

extern uint8_t ToF_Data[2];


void tof_sensor_init()
{
	VL53L0X_DataInit(0);
	VL53L0X_StaticInit(0);
}

void i2c_check()
{
	uint8_t tof_data = 0;

	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c1, 0x53, &tof_data, 1, 100);

	if(status == HAL_OK)
	  HAL_UART_Transmit(&huart2, (uint8_t*)"OK.\r\n", strlen("OK.\r\n"), 100);
	else
	  HAL_UART_Transmit(&huart2, (uint8_t*)"FAILED.\r\n", strlen("FAILED.\r\n"), 100);
}

void tof_data_request()
{
	HAL_I2C_Mem_Read_DMA(&hi2c1, 0x52, 0x14, I2C_MEMADD_SIZE_8BIT, ToF_Data, 2);
}

uint16_t encoder_cnt_get()
{
	uint16_t cnt = 0;
	cnt = __HAL_TIM_GET_COUNTER(&htim3);

	return cnt;
}

float encoder_ASCOM_preset(float angle_position)
{
	float angle = angle_position*10;

	return angle;
}

float encoder_angle_get(uint16_t positioning)
{
	float angle = (float) ((360.0/14400.0)* positioning)*10;

	return angle;
}

float get_current_offset(uint16_t dma_adc_index, float offset)
{
	offset = (alpha * dma_adc_index) + ((1.0 - alpha) * offset);

	return offset;
}

float get_amps(float filter_value)
{
	// skalowanie pradu - wzór funkcji liniowej
	float amp = (float)((20.0/1927.0)*filter_value - 22.5);

	return amp;
}

void encoder_display_angle(uint16_t position)
{
	char bufor_danych[64];
	float angle = (float)(360.0/14400.0)* position;

	snprintf(bufor_danych, sizeof(bufor_danych), "Angle: %.2f \r\n", angle);
	HAL_UART_Transmit(&huart2, (uint8_t*)bufor_danych, strlen(bufor_danych), 50);
}

void ride_left()
{
	  HAL_GPIO_WritePin(GPIOB, ENGINE1P_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, ENGINE1L_Pin, GPIO_PIN_SET);
}

void ride_right()
{
	  HAL_GPIO_WritePin(GPIOC, ENGINE1L_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, ENGINE1P_Pin, GPIO_PIN_SET);
}


