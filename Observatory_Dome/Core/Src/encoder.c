/*
 * encoder.c
 *
 *  Created on: Mar 13, 2026
 *      Author: 48694
 */

#include "main.h"
#include "usart.h"
#include "tim.h"

#include <encoder.h>
#include <nextion.h>

#define alpha 0.05


uint16_t encoder_cnt_get()
{
	uint16_t cnt = 0;
	cnt = __HAL_TIM_GET_COUNTER(&htim3);

	return cnt;
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
