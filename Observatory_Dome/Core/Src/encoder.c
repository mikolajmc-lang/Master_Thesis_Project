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


void i2c_check()
{
	uint8_t tof_data = 0;

	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c1, 0x53, &tof_data, 1, 100);

	if(status == HAL_OK)
	  HAL_UART_Transmit(&huart2, (uint8_t*)"OK.\r\n", strlen("OK.\r\n"), 100);
	else
	  HAL_UART_Transmit(&huart2, (uint8_t*)"FAILED.\r\n", strlen("FAILED.\r\n"), 100);
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
	HAL_GPIO_WritePin(GPIOB, ENGINE2P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, ENGINE2L_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, ENGINE1P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, ENGINE1L_Pin, GPIO_PIN_SET);
}

void ride_right()
{
	HAL_GPIO_WritePin(GPIOB, ENGINE2P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, ENGINE2L_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, ENGINE1L_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, ENGINE1P_Pin, GPIO_PIN_SET);
}

void ride_open()
{
	HAL_GPIO_WritePin(GPIOB, ENGINE1P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, ENGINE1L_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, ENGINE2P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, ENGINE2L_Pin, GPIO_PIN_SET);
}

void ride_close()
{
	HAL_GPIO_WritePin(GPIOB, ENGINE1P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, ENGINE1L_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, ENGINE2L_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, ENGINE2P_Pin, GPIO_PIN_SET);
}

void dont_ride()
{
	HAL_GPIO_WritePin(GPIOB, ENGINE1P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, ENGINE1L_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, ENGINE2L_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, ENGINE2P_Pin, GPIO_PIN_RESET);
}

void soft_start_func(uint8_t *pwm, uint8_t channel, uint32_t *tick)
{
	if(HAL_GetTick() - *tick > 50){

		if(*pwm < 60) {
			(*pwm)++;

			if(channel == 0)
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, *pwm);
			else
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *pwm);
		}
		*tick = HAL_GetTick();
	}
}


void soft_stop_func(uint8_t *pwm, uint8_t *pwm2, uint32_t *tick)
{
	if(HAL_GetTick() - *tick > 50){

		if(*pwm > 0){
		--(*pwm);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, *pwm);
		}

		if(*pwm2 > 0){
		--(*pwm2);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *pwm2);
		}

		*tick = HAL_GetTick();
	}
}

