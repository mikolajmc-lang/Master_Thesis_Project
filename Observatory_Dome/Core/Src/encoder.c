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


// Parametry układu i czujnika
#define V_REF           3.3f
#define ADC_MAX         4095.0f
#define R1_VAL          985.0f
#define R2_VAL          2150.0f
#define SENSITIVITY     0.100f  // 0.100f dla 20A, 0.066f dla 30A, 0.185f dla 5A
#define V_ZERO          2.500f  // Napięcie przy 0A (warto zmierzyć lub skalibrować programowo)

// Obliczone stałe
#define DIVIDER_RATIO   (R2_VAL / (R1_VAL + R2_VAL))
//#define SCALE_FACTOR    (V_REF / (ADC_MAX * DIVIDER_RATIO * SENSITIVITY))
#define OFFSET_CURRENT  (V_ZERO / SENSITIVITY)

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
	//float amp = (float)((20.0/1895.0)*filter_value - 22.16);
	//return amp;
	const float ADC_ZERO = 2225.0f;     // Wartość z DMA przy I = 0 A
	const float SCALE_FACTOR = 0.01176f; // (V_REF / (4095 * k_div * Sensitivity))

	    return (filter_value - ADC_ZERO) * SCALE_FACTOR;
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
	HAL_GPIO_WritePin(GPIOB, ENGINE2P_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, ENGINE2L_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, ENGINE1P_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, ENGINE1L_Pin, GPIO_PIN_RESET);
}

void ride_right()
{
	HAL_GPIO_WritePin(GPIOB, ENGINE2P_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, ENGINE2L_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, ENGINE1L_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, ENGINE1P_Pin, GPIO_PIN_RESET);
}

void ride_open()
{
	HAL_GPIO_WritePin(GPIOB, ENGINE1P_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, ENGINE1L_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, ENGINE2P_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, ENGINE2L_Pin, GPIO_PIN_RESET);
}

void ride_close()
{
	HAL_GPIO_WritePin(GPIOB, ENGINE1P_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, ENGINE1L_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, ENGINE2L_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, ENGINE2P_Pin, GPIO_PIN_RESET);
}

void dont_ride()
{
	HAL_GPIO_WritePin(GPIOB, ENGINE1P_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, ENGINE1L_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, ENGINE2L_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, ENGINE2P_Pin, GPIO_PIN_SET);
}

void dont_ride_shutter() {
    // Gasi tylko piny klapy (Silnik 1)
    HAL_GPIO_WritePin(GPIOB, ENGINE2P_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, ENGINE2L_Pin, GPIO_PIN_SET);
}

void dont_ride_rot() {
    // Gasi tylko piny obrotu (Silnik 2)
    HAL_GPIO_WritePin(GPIOC, ENGINE1L_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, ENGINE1P_Pin, GPIO_PIN_SET);
}

void soft_start_func(volatile uint8_t *pwm, uint8_t channel, uint32_t *tick)
{
	if(HAL_GetTick() - *tick > 20){

		if(*pwm >= 0 && *pwm < 65) {

			if(*pwm < 42){
				*pwm = 42;
			} else {
				(*pwm)++;
			}

			if(channel == 0)
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, *pwm);
			else
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *pwm);
		}

		*tick = HAL_GetTick();
	}
}


void soft_stop_func(volatile uint8_t *pwm, volatile uint8_t *pwm2, uint32_t *tick)
{
	if(HAL_GetTick() - *tick > 20){

		if(*pwm > 0){
			//if(*pwm <= 42) {
			//	*pwm = 0; // Skok od razu do 0
			//}else{
				--(*pwm);
			//}
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, *pwm);
		}

		if(*pwm2 > 0){
			//if(*pwm2 <= 42) {
			//*pwm2 = 0;
			//} else {
			--(*pwm2);
			//}
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *pwm2);
		}

		*tick = HAL_GetTick();
	}
}

void soft_stop_func_alternate(volatile uint8_t *pwm, uint8_t channel, uint32_t *tick)
{
	if(HAL_GetTick() - *tick > 20){

		if(*pwm > 0){
			--(*pwm);

		if(channel == 0)
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, *pwm);
		else
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *pwm);

		}

		*tick = HAL_GetTick();
	}


}

