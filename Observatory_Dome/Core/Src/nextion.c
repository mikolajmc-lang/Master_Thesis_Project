/*
 * nextion.c
 *
 *  Created on: Mar 22, 2026
 *      Author: 48694
 */

#include "main.h"
#include "usart.h"
#include "tim.h"

#include <encoder.h>
#include <nextion.h>

uint8_t cmd_end[3] = {0xFF,0xFF,0xFF}; // command end sequence

void Nextion_SendString(char *ID, float info)
{
	char buf[50];

	int len = sprintf(buf, "%s.txt=\"Angle: %.f\"", ID, info);

	HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);
	HAL_UART_Transmit(&huart1, cmd_end, 3, 100);
}
