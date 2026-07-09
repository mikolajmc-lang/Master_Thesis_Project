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

void Nextion_SendText(char *ID, char *string)
{
	static char buf[50];

	if(huart1.gState == HAL_UART_STATE_READY) {
		int len = sprintf(buf, "%s.txt=\"%s\"\xFF\xFF\xFF", ID, string);

		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buf, len);
	}
}

void Nextion_SendString_Current(int16_t current1, int16_t current2)
{
	static char buf[50];

	if(huart1.gState == HAL_UART_STATE_READY) {
		int len = sprintf(buf, "x5.val=%d\xFF\xFF\xFF" "x6.val=%d\xFF\xFF\xFF", current1, current2);

		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buf, len);
	}
}

void Nextion_SendString_Rain(char *text_ID, char *info)
{
	static char buf[50];

	if(huart1.gState == HAL_UART_STATE_READY) {
		int len = sprintf(buf, "t4.txt=\"%s\"\xFF\xFF\xFFt4.bco=%s\xFF\xFF\xFF", text_ID, info);

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
