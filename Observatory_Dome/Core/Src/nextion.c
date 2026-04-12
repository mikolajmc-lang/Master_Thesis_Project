/*
 * nextion.c
 *
 *  Created on: Mar 22, 2026
 *      Author: 48694
 */

#include "main.h"
#include "usart.h"
#include "tim.h"

#include <stdlib.h>
#include <encoder.h>
#include <nextion.h>

uint8_t cmd_end[3] = {0xFF,0xFF,0xFF}; // command end sequence

void Nextion_SendString(char *ID, float info)
{
	static char buf[50];

	if(huart1.gState == HAL_UART_STATE_READY) {
		int len = sprintf(buf, "%s.val=%.f\xFF\xFF\xFF", ID, info);

		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buf, len);
	}
}

void Nextion_Waveform(uint8_t wave1, uint8_t wave2)
{
	static char msg[64];

	if(huart1.gState == HAL_UART_STATE_READY) {
		int len = snprintf(msg, sizeof(msg), "add 1,0,%u\xFF\xFF\xFF" "add 4,0,%u\xFF\xFF\xFF",wave1,wave2);

		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msg, len);
	}
}
