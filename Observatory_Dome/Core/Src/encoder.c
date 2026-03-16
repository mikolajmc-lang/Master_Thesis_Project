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



uint16_t encoder_cnt_get()
{
	uint16_t cnt = 0;
	cnt = __HAL_TIM_GET_COUNTER(&htim3);

	return cnt;
}

void encoder_display_angle(uint16_t position)
{
	char bufor_danych[64];
	float angle = (float)(360.0/80.0)* position;

	snprintf(bufor_danych, sizeof(bufor_danych), "Angle: %.f \r\n", angle);
	HAL_UART_Transmit(&huart2, (uint8_t*)bufor_danych, strlen(bufor_danych), 50);
}
