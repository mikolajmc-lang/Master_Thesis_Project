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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <nextion.h>
#include <encoder.h>
#include <vl53l0x_api.h>
#include <vl53l0x_platform.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	TOF_IDLE,
	TOF_TRIGGER_MEASURE,
	TOF_WAIT_FOR_DATA,
	TOF_GET_RESULT
} ToF_State_t;

typedef enum {
	DIR_NONE = 0,
	DIR_OPEN,
	DIR_CLOSE,
	DIR_LEFT,
	DIR_RIGHT
} MotorState;

typedef union {
	float float_val;
	uint32_t uint32_t_val;
	uint8_t bytes[4];
} ByteConverter;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ASCOM_Buffer 20

#define RAIN_STATE_NONE          0
#define RAIN_STATE_GO_HOME       1
#define RAIN_STATE_STOP_HOME     2
#define RAIN_STATE_CLOSE_SHUTTER 3
#define RAIN_STATE_STOP_SHUTTER  4
#define RAIN_STATE_RESUME_DRIVE  5
#define RAIN_STATE_FINISHED      6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// user test values

uint8_t label_is_home = 0; // 0 - Preset, 1 - Angle.
uint8_t i,j,k,l = 0; // Zabezpieczenie ponownego załączenia obrotów
uint8_t i2c_transmit_flag = 0;
uint8_t angle_conversion_flag = 0;
uint8_t ride_left_flag = 0;
uint8_t ride_right_flag = 0;
uint8_t ride_open_flag = 0;
uint8_t ride_close_flag = 0;
volatile uint8_t abort_slew_flag = 0;
volatile uint8_t nextion_rain_on = 0;
volatile uint8_t nextion_rain_off = 0;
volatile uint8_t is_home = 0;

uint8_t pwm_run_1 = 0;
uint8_t pwm_run_2 = 0;
uint8_t soft_start_automatic = 0;
uint8_t soft_start = 0;
uint8_t soft_stop = 0;

uint8_t amp_waveform = 0;
uint8_t amp_waveform_1 = 0;

// Sekcja komunikacji dwustronnej ASCOM - PC

uint8_t ASCOM_rx_buffer[7]; // Bufor na rozkaz z PC (7 bajtów)
uint8_t ASCOM_tx_buffer[11]; // Bufor na odpowiedź ze statusem (11 bajtów)

uint8_t current_shutter_state = 1; // // 0=Open, 1=Closed, 2=Opening, 3=Closing, 4=Error
uint8_t is_slewing = 0;           // Flaga ruchu obrotowego kopuły
uint8_t rain_alert = 0;           // Flaga alarmu pogodowego

volatile uint8_t ASCOM_frame_ready = 0;

uint8_t Rx_Data[4];
uint8_t ASCOM_Data[ASCOM_Buffer];
uint8_t ASCOM_Index = 0;
uint8_t ASCOM_Byte = 0;
uint16_t UART_value = 0;

volatile uint8_t open = 0;
volatile uint8_t close = 0;
volatile uint8_t left = 0;
volatile uint8_t right = 0;

volatile uint8_t raindrop_signal = 0;
volatile uint8_t waveform_enable = 0;
volatile uint8_t page_auto = 0;
volatile uint8_t page_manual = 0;

uint32_t tick_start_1 = 0, tick_stop_1 = 0;
uint32_t ToF_Tick = 0;
uint32_t ToF_delay_time = 0;
uint32_t wave_delay_time_0 = 0;
uint32_t wave_delay_time_1 = 0;
uint32_t preset_delay_time = 0;
uint16_t counter = 0;

uint16_t pwm_signal_1 = 0;
uint16_t pwm_signal_2 = 0;
volatile uint8_t pwm_value_1 = 0;
volatile uint8_t pwm_value_2 = 0;

uint16_t current_engine_value[2];

float target_angle = 0.0f;
float string_to_angle = 0;
float filtered_value_0 = 2160;
float filtered_value_1 = 2160;
float angle_to_display = 0;
float preset_to_display = 0;
float distance_to_display = 0;
float amps_0 = 0;
float amps_1 = 0;

int16_t half = 180.0;
int16_t amps_to_display_0 = 0;
int16_t amps_to_display_1 = 0;

uint16_t tof_distance_mm = 0;

uint8_t comparision_mode = 1;
volatile uint8_t print_home = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t encoder_cnt_get();
float encoder_angle_get(uint16_t positioning);
float get_current_offset(uint16_t dma_adc_index, float offset);
float get_amps(float filter_value);
float encoder_ASCOM_preset(float angle_position);
void encoder_display_angle(uint16_t position);
void Nextion_SendText(char *ID, char *string);
void Nextion_SendString(char *ID, float info);
void Nextion_SendString_Rain(char *text_ID, char *info);
void Nextion_Waveform(uint8_t wave1, uint8_t wave2);
void Nextion_SendString_Current(int16_t current1, int16_t current2);
void soft_start_func(volatile uint8_t *pwm, uint8_t channel, uint32_t *tick);
void soft_stop_func(volatile uint8_t *pwm, volatile uint8_t *pwm2, uint32_t *tick);
void soft_stop_func_alternate(volatile uint8_t *pwm, uint8_t channel, uint32_t *tick);
void ride_left(void);
void ride_right(void);
void ride_open(void);
void ride_close(void);
void dont_ride(void);
void i2c_check(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
MotorState CurrentDir = DIR_NONE;

VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_Dev_t mySensor;
VL53L0X_DEV pDev = &mySensor;

void tof_sensor_init(VL53L0X_DEV Dev)
{
	  uint32_t refSpadCount;
	  uint8_t isApertureSpads;
	  uint8_t VhvSettings;
	  uint8_t PhaseCal;

	  VL53L0X_Error status = VL53L0X_DataInit(Dev);

	  if(status == VL53L0X_ERROR_NONE) {
		 status = VL53L0X_StaticInit(Dev);
	  }
	  if(status == VL53L0X_ERROR_NONE) {
		 status = VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
	  }
	  if(status == VL53L0X_ERROR_NONE) {
		 status = VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
	  }
	  if(status == VL53L0X_ERROR_NONE) {
		 status = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	  }
	  if(status == VL53L0X_ERROR_NONE) {
		 status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
	  }
	  if(status == VL53L0X_ERROR_NONE) {
		 status = VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	  }
	  if(status == VL53L0X_ERROR_NONE) {
		 status = VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	  }
	  if(status == VL53L0X_ERROR_NONE) {
		  status = VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,(FixPoint1616_t)(0.1*65536));
	  }
	  if (status == VL53L0X_ERROR_NONE) {
		  status = VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
	  }
	  if (status == VL53L0X_ERROR_NONE) {
		  status = VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	  }
	  if (status == VL53L0X_ERROR_NONE) {
		  status = VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
	  }
}

uint16_t tof_data_request(void)
{
	uint16_t distance;
	VL53L0X_Error status = VL53L0X_PerformSingleRangingMeasurement(pDev, &RangingData);

	if(status == VL53L0X_ERROR_NONE){
		distance = RangingData.RangeMilliMeter;
	}

	return distance;
}

uint16_t tof_data_request_nonblocking_mode(void)
{
	static ToF_State_t state = TOF_TRIGGER_MEASURE;
	static uint32_t lastTick = 0;
	static uint16_t distance = 0;

	if(HAL_GetTick() - lastTick >= 25) {
		switch(state) {
			case TOF_TRIGGER_MEASURE:
				if(VL53L0X_PerformSingleMeasurement(pDev) == VL53L0X_ERROR_NONE) {
					state = TOF_WAIT_FOR_DATA;
					//lastTick = HAL_GetTick();
				}
			break;

			case TOF_WAIT_FOR_DATA:
					uint8_t DataReady = 0;

					VL53L0X_GetMeasurementDataReady(pDev, &DataReady);

					if(DataReady){
						state = TOF_GET_RESULT;
					}
			break;

			case TOF_GET_RESULT:

				if(VL53L0X_GetRangingMeasurementData(pDev, &RangingData) == VL53L0X_ERROR_NONE) {
					distance = RangingData.RangeMilliMeter;
				}
				VL53L0X_ClearInterruptMask(pDev, 0);

				state = TOF_TRIGGER_MEASURE;
			break;
		}

		lastTick = HAL_GetTick();
	}

	return distance;
}

void ASCOM_SendStatus(void)
{
	ByteConverter az_conv;
	uint8_t motor_flags = 0;

	// Zabezpieczenie danych
	az_conv.float_val = (float)(angle_to_display/10.0);

	// Maska bitowa statusu motor_flags
	if(is_slewing)
		motor_flags |= 0x01; // Bit 0 (kopula obraca sie)
	if(rain_alert)
		motor_flags |= 0x02; // Bit 1 (Deszcz trwa)
	if(angle_to_display == 0.0f)
		motor_flags |= 0x04; // Kopuła jest w pozycji Home

	// Konstrukcja stałej długości ramki danych
	ASCOM_tx_buffer[0] = 0x23; // Znak początku #

	// Rzutowanie danych Azimuth
	ASCOM_tx_buffer[1] = az_conv.bytes[0];
	ASCOM_tx_buffer[2] = az_conv.bytes[1];
	ASCOM_tx_buffer[3] = az_conv.bytes[2];
	ASCOM_tx_buffer[4] = az_conv.bytes[3];

	ASCOM_tx_buffer[5] = current_shutter_state;

	// Pakowanie dystansu ToF (16-bitowa)

	ASCOM_tx_buffer[6] = (uint8_t)(tof_distance_mm & 0xFF); // LSB
	ASCOM_tx_buffer[7] = (uint8_t)(tof_distance_mm >> 8 & 0xFF); // MSB
	ASCOM_tx_buffer[8] = 0x00;
	ASCOM_tx_buffer[9] = 0x00;

	ASCOM_tx_buffer[10] = motor_flags; // Bajt flag kontrolnych otwierania/zamykania klapy

	uint32_t tx_timeout = HAL_GetTick();

	while(huart2.gState == HAL_UART_STATE_BUSY_TX)
	{
		if(HAL_GetTick() - tx_timeout > 3) {
			return;
		}
	}

	HAL_UART_Transmit_DMA(&huart2, ASCOM_tx_buffer, 11);
}

void ASCOM_ParseCommand(void)
{
	// Weryfikacja ramki, sprawdzenie czy przyszlo : oraz $
	if(ASCOM_rx_buffer[0] == 0x3A && ASCOM_rx_buffer[6] == 0x24)
	{
		uint8_t cmd_code = ASCOM_rx_buffer[1];
		ByteConverter target_az;

		// Bezpośrednie zmapowanie 4 bajtów danych z bufora na wartość float (Azimuth)
		target_az.bytes[0] = ASCOM_rx_buffer[2];
		target_az.bytes[1] = ASCOM_rx_buffer[3];
		target_az.bytes[2] = ASCOM_rx_buffer[4];
		target_az.bytes[3] = ASCOM_rx_buffer[5];

		// System bezpieczenstwa
		if (rain_alert && (cmd_code == 1 || cmd_code == 2 || cmd_code == 4)) {
			ASCOM_SendStatus();
			return;
		}

		switch(cmd_code)
		{
			case 1: // FIND HOME
				target_angle = 0.0f;
				// page_auto = 1;
				is_slewing = 1;
				print_home = 1;
			break;

			case 2: // SLEW
				if(target_az.float_val >= 0.0f && target_az.float_val < 360.0f) {
					target_angle = (target_az.float_val*10);
					// page_auto = 1;
					is_slewing = 1;
				}
			break;

			case 3: // CLOSE SHUTTER
				if(current_shutter_state != 1 && current_shutter_state != 3)
				{
					current_shutter_state = 3;
					target_angle = 0.0f;
					// page_auto = 1;

				}
			break;

			case 4: // OPEN SHUTTER
				if(current_shutter_state != 0 && current_shutter_state != 2)
				{
					current_shutter_state = 2;
					target_angle = 0.0f;
					// page_auto = 1;
				}
			break;

			case 5: // GET STATUS

			break;

			case 6: // ZATRZYMANIE AWARYJNE (Abort Slew / N.I.N.A. Stop)
				abort_slew_flag = 1;
			break;

			default:

			break;
		}

		ASCOM_SendStatus();
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  pDev->I2CHandle = &hi2c1;
  pDev->I2cDevAddr = 0x52; //Adres czujnika ToF
  pDev->comms_type = 1;
  pDev->comms_speed_khz = 400;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Sprzętowy reset czujnika ToF
  HAL_GPIO_WritePin(GPIOB, XSHUT_Pin, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(GPIOB, XSHUT_Pin, GPIO_PIN_SET);
  HAL_Delay(20);

  tof_sensor_init(pDev);
  //i2c_check();

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_UART_Receive_IT(&huart1, Rx_Data, 4);
  //HAL_UART_Receive_IT(&huart2, &ASCOM_Byte, 1);
  HAL_UART_Receive_IT(&huart2, &ASCOM_Byte, 1);
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

	/* if(ASCOM_frame_ready) {
		ASCOM_frame_ready = 0;

		ASCOM_ParseCommand();
	} */


	if(HAL_GetTick() - ToF_delay_time >= 30)
	{
		tof_distance_mm = tof_data_request_nonblocking_mode();
		ToF_delay_time = HAL_GetTick();
	}

	//tof_distance_mm = tof_data_request_nonblocking_mode();
	distance_to_display = (float)(tof_distance_mm*10);

	static uint8_t reset_pwm = 0;
	static uint8_t rain_string = 0;
	static uint8_t timer_reset = 0;

	// --- ZMIENNE MASZYNY STANÓW AUTO ---
	static uint8_t start_automation = 0;
	static uint8_t automation_active = 0;
	static uint8_t nextion_off = 0;
	static uint32_t auto_delay_tick = 0;
	static uint8_t rain_state = RAIN_STATE_NONE;

	counter = encoder_cnt_get();
	angle_to_display = encoder_angle_get(counter);

	if (abort_slew_flag) {
	    // 1. Twarde odcięcie sprzętowe
	    dont_ride();
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 42); // PWM jałowe dla szczeliny
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 42); // PWM jałowe dla obrotu
	    pwm_value_1 = 0;
	    pwm_value_2 = 0;

	    // 2. Czyszczenie flag ruchu
	    open = 0; close = 0; left = 0; right = 0;
	    ride_left_flag = 0; ride_right_flag = 0;
	    ride_open_flag = 0; ride_close_flag = 0;

	    // 3. Resetowanie logiki napędu
	    CurrentDir = DIR_NONE;
	    soft_start = 0;
	    soft_stop = 0;
	    pwm_run_1 = 0;
	    pwm_run_2 = 0;

	    // 4. Synchronizacja celu - KLUCZOWE!
	    // Oszukujemy maszynę stanów, że jesteśmy u celu, żeby nie wznowiła jazdy
	    target_angle = angle_to_display;
	    preset_to_display = angle_to_display;
	    comparision_mode = 1;
	    is_slewing = 0;

	    // Zatrzymanie awaryjne skonsumowane
	    abort_slew_flag = 0;
	}


	if(page_manual && !page_auto){

		static uint8_t dont_ride_flag = 0;
		static uint8_t string_queue = 0;

		reset_pwm = 0;

		if(huart1.gState == HAL_UART_STATE_READY) {

			if(HAL_GetTick() - wave_delay_time_0 > 15) {
				wave_delay_time_0 = HAL_GetTick();

				switch(string_queue) {
					case 0:
						Nextion_SendString("x3", angle_to_display);
						string_queue++;
					break;

					case 1:
						Nextion_SendString("x4", distance_to_display);
						string_queue = 0;
					break;
				}
			}
		}

		if(open){
			if(CurrentDir== DIR_OPEN) {
			        // DRUGIE KLIKNIECIE: Silnik jedzie w tym samym kierunku, więc go zatrzymujemy
			        soft_stop = 1;
			        soft_start = 0;
			        CurrentDir = DIR_NONE; // Ważne: zmieniamy stan na NONE, żeby system wiedział, że dążymy do stopu
			        open = 0;               // "Konsumujemy" flagę
			} else {
				// PIERWSZE KLIKNIECIE (lub zmiana z innego kierunku):
				if(pwm_value_1 == 0 && pwm_value_2 == 0 && soft_stop == 0){
					ride_open();
					CurrentDir = DIR_OPEN;
					pwm_run_1 = 1;
					pwm_run_2 = 0;
					soft_start = 1;
					open = 0;
				} else {
					soft_stop = 1;
					soft_start = 0;
				}
			}

		}

		if(close){
			if(CurrentDir == DIR_CLOSE) {
			        soft_stop = 1;
			        soft_start = 0;
			        CurrentDir = DIR_NONE;
			        close = 0;               // "Konsumujemy" flagę
			} else {
				if(pwm_value_1 == 0 && pwm_value_2 == 0 && soft_stop == 0){
					ride_close();
					CurrentDir = DIR_CLOSE;
					pwm_run_1 = 1;
					pwm_run_2 = 0;
					soft_start = 1;
					close = 0;
				} else {
					soft_stop = 1;
					soft_start = 0;
				}
			}
		}

		if(left){
			if(CurrentDir == DIR_LEFT) {
			        soft_stop = 1;
			        soft_start = 0;
			        CurrentDir = DIR_NONE;
			        left = 0;               // "Konsumujemy" flagę
			} else {
				if(pwm_value_1 == 0 && pwm_value_2 == 0 && soft_stop == 0){
					ride_left();
					CurrentDir = DIR_LEFT;
					pwm_run_1 = 0;
					pwm_run_2 = 1;
					soft_start = 1;
					left = 0;
				} else {
					soft_stop = 1;
					soft_start = 0;
				}
			}
		}

		if(right){
			if(CurrentDir == DIR_RIGHT) {
			        soft_stop = 1;
			        soft_start = 0;
			        CurrentDir = DIR_NONE;
			        right = 0;               // "Konsumujemy" flagę
			} else {
				if(pwm_value_1 == 0 && pwm_value_2 == 0 && soft_stop == 0){
					ride_right();
					CurrentDir = DIR_RIGHT;
					pwm_run_1 = 0;
					pwm_run_2 = 1;
					soft_start = 1;
					right = 0;
				} else {
					soft_stop = 1;
					soft_start = 0;
				}
			}
		}

		if(soft_start){

			if(pwm_run_1)
				soft_start_func(&pwm_value_1, 0, &tick_start_1);

			if(pwm_run_2)
				soft_start_func(&pwm_value_2, 1, &tick_start_1);

			if(pwm_value_1 >= 65 || pwm_value_2 >= 65) // Silnik musi się rozpedzić
				soft_start = 0;

		}

		if(soft_stop){

			if(!dont_ride_flag) {
				dont_ride();
				dont_ride_flag = 1;
			}

			//soft_stop_func(&pwm_value_1, &pwm_value_2, &tick_stop_1);
			if(pwm_run_1)
				soft_stop_func_alternate(&pwm_value_1, 0, &tick_stop_1);

			if(pwm_run_2)
				soft_stop_func_alternate(&pwm_value_2, 1, &tick_stop_1);


			if(pwm_value_1 == 0 && pwm_value_2 == 0){
				soft_stop = 0;
				dont_ride_flag = 0;

				if(pwm_run_1)
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 42);

				if(pwm_run_2)
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 42);
				//dont_ride();

				if(open) {
					ride_open();
					CurrentDir = DIR_OPEN;
					pwm_run_1 = 1;
					pwm_run_2 = 0;
					soft_start = 1;
					open = 0;
				}else if(close){
					ride_close();
					CurrentDir = DIR_CLOSE;
					pwm_run_1 = 1;
					pwm_run_2 = 0;
					soft_start = 1;
					close = 0;
				}else if(left){
					ride_left();
					CurrentDir = DIR_LEFT;
					pwm_run_2 = 1;
					pwm_run_1 = 0;
					soft_start = 1;
					left = 0;
				}else if(right){
					ride_right();
					CurrentDir = DIR_RIGHT;
					pwm_run_2 = 1;
					pwm_run_1 = 0;
					soft_start = 1;
					right = 0;
				}else {
					CurrentDir = DIR_NONE;
					pwm_run_1 = 0;
					pwm_run_2 = 0;
					soft_start = 0;
				}

			}
		}

	} else if(!page_manual && page_auto)   {
		// --- 1. SEKCJA NEXTIONA (Round-Robin) ---
		    static uint32_t nextion_update_tick = 0;
		    static uint8_t nextion_seq = 0;
		    float angle_Home = 0.0;

		    if (huart1.gState == HAL_UART_STATE_READY) {
		        if (!rain_string && !waveform_enable) {
		            Nextion_SendString_Rain("OFF", "RED");
		            rain_string = 1;
		        }
		        else if (nextion_rain_on) {
		            Nextion_SendString_Rain("ON", "GREEN");
		            nextion_rain_on = 0;
		        }
		        else if (nextion_rain_off) {
		            Nextion_SendString_Rain("OFF", "RED");
		            nextion_rain_off = 0;
		        }
		        else if ((current_shutter_state == 2 || current_shutter_state == 3 || rain_alert || print_home) && !label_is_home) {
		            Nextion_SendText("t1", "Home:"); // Pokazujemy Home w trakcie powrotu
		            print_home = 0;
		            label_is_home = 1;
		        }
		        else if (current_shutter_state <= 1 && !rain_alert && label_is_home && !is_slewing) {
		            Nextion_SendText("t1", "Preset:");
		            label_is_home = 0;
		        }
		        else if (HAL_GetTick() - nextion_update_tick > 15) {
		            nextion_update_tick = HAL_GetTick();
		            switch(nextion_seq) {
		                case 0: Nextion_SendString("x0", angle_to_display); nextion_seq++; break;
		                case 1:
		                    if(label_is_home) Nextion_SendString("x1", angle_Home);
		                    else Nextion_SendString("x1", target_angle);
		                    nextion_seq++; break;
		                case 2: Nextion_SendString("x2", distance_to_display); nextion_seq = 0; break;
		            }
		        }
		    }

		    // --- 2. DELAY AUTOMATYZACJI ---
		    if (HAL_GetTick() - auto_delay_tick > 2000) {
		        start_automation = 1;
		        automation_active = 1;
		        auto_delay_tick = HAL_GetTick();
		    }

		    // --- 3. WYKRYCIE DESZCZU (Hardware Override) ---
		    if (raindrop_signal && !rain_alert) {
		        rain_alert = 1;
		        nextion_rain_on = 1;
		        current_shutter_state = 3; // Wymuś status Closing
		        target_angle = angle_Home; // Wymuś powrót do 0.0
		        comparision_mode = 1;
		    }
		    else if (!raindrop_signal && rain_alert) {
		        rain_alert = 0;
		        nextion_rain_off = 1;
		    }

		    // --- 4. LOGIKA DOJAZDU (Silnik 2 - Obrót) ---
		    if (angle_to_display == target_angle) {
		        // JESTEŚMY U CELU OBROTOWEGO
		        if (pwm_value_2 > 0) {
		            soft_stop_func_alternate(&pwm_value_2, 1, &tick_stop_1);
		        } else {
		            dont_ride();
		            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 42);
		            ride_left_flag = 0;
		            ride_right_flag = 0;
		            soft_start = 0;
		            comparision_mode = 1;
		            is_slewing = 0;
		        }
		    } else {
		        // NIE JESTEŚMY U CELU -> KRĘCIMY KOPUŁĄ
		        if (start_automation) {
		            if (pwm_value_1 > 0) {
		               // Klapa się rusza - całkowita blokada obrotu! Czekamy.
		            } else {
		                is_slewing = 1; // Flaga dla ASCOM (jedziemy)

		                if (comparision_mode) {
		                    float delta = target_angle - angle_to_display;

		                    if (delta > 1800.0) delta -= 3600.0;
		                    else if (delta < -1800.0) delta += 3600.0;

		                    uint8_t target_left = (delta < 0.0) ? 1 : 0;
		                    uint8_t target_right = (delta > 0.0) ? 1 : 0;

		                    if ((target_left && ride_right_flag) || (target_right && ride_left_flag)) {
		                        if (pwm_value_2 > 0) {
		                            soft_stop_func_alternate(&pwm_value_2, 1, &tick_stop_1);
		                        } else {
		                            dont_ride();
		                            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 42);
		                            ride_left_flag = target_left;
		                            ride_right_flag = target_right;
		                            soft_start = 0;
		                            comparision_mode = 0;
		                        }
		                    } else {
		                        ride_left_flag = target_left;
		                        ride_right_flag = target_right;
		                        comparision_mode = 0;
		                    }
		                }

		                if (!comparision_mode) {
		                    if (ride_left_flag && !soft_start) { ride_left(); soft_start = 1; }
		                    else if (ride_right_flag && !soft_start) { ride_right(); soft_start = 1; }

		                    if (soft_start) {
		                        soft_start_func(&pwm_value_2, 1, &tick_start_1);
		                        if (pwm_value_2 == 65) soft_start = 0;
		                    }
		                }
		            }
		        }
		    }

		    // --- 5. SEKWENCJA PRACY SZCZELINY (Silnik 1) ---
		    uint8_t is_at_home = (angle_to_display == angle_Home);

		    if (is_at_home && pwm_value_2 == 0) {
		        // PROCEDURA ZAMYKANIA KLAPY
		        if (current_shutter_state == 3) {
		            if (!ride_close_flag) {
		                ride_close();
		                ride_close_flag = 1;
		                ride_open_flag = 0;
		                soft_start = 1;
		            }

		            if (soft_start) {
		                soft_start_func(&pwm_value_1, 0, &tick_start_1);
		                if (pwm_value_1 == 65) soft_start = 0;
		            }

		            if (tof_distance_mm <= 50.0) {
		                if (pwm_value_1 > 0) {
		                    soft_stop_func_alternate(&pwm_value_1, 0, &tick_stop_1);
		                } else {
		                    dont_ride();
		                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 42);
		                    ride_close_flag = 0;
		                    current_shutter_state = 1; // Zamknięto
		                }
		            }
		        }

		        // PROCEDURA OTWIERANIA KLAPY
		        else if (current_shutter_state == 2) {
		            if (!ride_open_flag) {
		                ride_open();
		                ride_open_flag = 1;
		                ride_close_flag = 0;
		                soft_start = 1;
		            }

		            if (soft_start) {
		                soft_start_func(&pwm_value_1, 0, &tick_start_1);
		                if (pwm_value_1 == 65) soft_start = 0;
		            }

		            if (tof_distance_mm >= 200.0) {
		                if (pwm_value_1 > 0) {
		                    soft_stop_func_alternate(&pwm_value_1, 0, &tick_stop_1);
		                } else {
		                    dont_ride();
		                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 42);
		                    ride_open_flag = 0;
		                    current_shutter_state = 0; // Otwarto
		                }
		            }
		        }
		    }

	} else if (!page_manual && !page_auto) {
		// --- 1. RESET PODSTAWOWY ---
		    timer_reset = 0;
		    rain_string = 0;
		    label_is_home = 0;

		    // --- 2. RESET MASZYNY STANÓW AUTO ---
		    start_automation = 0;
		    automation_active = 0;
		    comparision_mode = 1;
		    nextion_off = 0;
		    rain_alert = 0;
		    rain_state = RAIN_STATE_NONE;
		    auto_delay_tick = HAL_GetTick();

		    ride_open_flag = 0;
		    ride_close_flag = 0;
		    ride_left_flag = 0;
		    ride_right_flag = 0;

		    // --- 3. MIĘKKIE ZATRZYMANIE (Przy wychodzeniu z trybów) ---
		    if(pwm_value_1 > 0 || pwm_value_2 > 0) {
		        soft_stop_func(&pwm_value_1, &pwm_value_2, &tick_stop_1);
		    } else {
		        dont_ride();
		        soft_stop = 0;
		        soft_start = 0;
		        pwm_run_1 = 0;
		        pwm_run_2 = 0;

		        CurrentDir = DIR_NONE;
		        open = 0; close = 0; left = 0; right = 0;

		        if(!reset_pwm) {
		            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
		            MX_TIM2_Init();
		            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		            reset_pwm = 1;
		        }
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

		static uint8_t waveform_queue = 0;

		rain_string = 0;

		if(huart1.gState == HAL_UART_STATE_READY) {
			if(HAL_GetTick() - wave_delay_time_0 > 15) {
				wave_delay_time_0 = HAL_GetTick();

				switch(waveform_queue) {
					case 0:
						Nextion_Waveform(amp_waveform, amp_waveform_1);
						waveform_queue++;
					break;

					case 1:
						Nextion_SendString_Current(amps_to_display_0, amps_to_display_1);
						waveform_queue = 0;
					break;
				}

			}
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

		if(Rx_Data[1] == 0x00 && Rx_Data[2] == 0x04)
			page_manual = 1;
		else if(Rx_Data[1] == 0x00 && Rx_Data[2] == 0x03)
			page_auto = 1;
		else if(Rx_Data[1] == 0x01 && Rx_Data[2] == 0x0E)
			page_manual = 0;
		else if(Rx_Data[1] == 0x03 && Rx_Data[2] == 0x04)
			page_auto = 0;
		else if(Rx_Data[1] == 0x01 && Rx_Data[2] == 0x0D)
			waveform_enable = 1;
		else if(Rx_Data[1] == 0x03 && Rx_Data[2] == 0x08)
			waveform_enable = 1;
		else if(Rx_Data[1] == 0x02 && Rx_Data[2] == 0x03)
			waveform_enable = 0;
		else if(Rx_Data[1] == 0x01 && Rx_Data[2] == 0x03)
			open = 1;
		else if(Rx_Data[1] == 0x01 && Rx_Data[2] == 0x04)
			close = 1;
		else if(Rx_Data[1] == 0x01 && Rx_Data[2] == 0x05)
			left = 1;
		else if(Rx_Data[1] == 0x01 && Rx_Data[2] == 0x06)
			right = 1;

		HAL_UART_Receive_IT(&huart1, Rx_Data, 4);
	}

	/*if(huart->Instance == USART2) {
			if(ASCOM_Byte == '\n') {
				// Znak końca - parsujemy!
				ASCOM_Data[ASCOM_Index] = '\0'; // ZAWSZE zamykaj string znakiem NULL przed użyciem atof!
				angle_conversion_flag = 1;
			}
	        else if (ASCOM_Byte != '\r') {
				// Zabezpieczenie przed przepełnieniem bufora
				if (ASCOM_Index < (ASCOM_Buffer - 1)) {
					ASCOM_Data[ASCOM_Index++] = ASCOM_Byte;
				}
	            // Jeśli przyszło więcej znaków niż rozmiar bufora, po prostu je ignorujemy
	            // aż do momentu nadejścia '\n'
			}

			HAL_UART_Receive_IT(&huart2, &ASCOM_Byte, 1);
	} */

	if(huart->Instance == USART2) {

		for(int i = 0; i < 6; i++)
		{
			ASCOM_rx_buffer[i] = ASCOM_rx_buffer[i+1];
		}

		ASCOM_rx_buffer[6] = ASCOM_Byte;

		if(ASCOM_rx_buffer[0] == 0x3A && ASCOM_rx_buffer[6] == 0x24)
		{
			ASCOM_ParseCommand(); // Wywołaj przetworzenie - ramka jest prawidłowa!
		}

		//ASCOM_frame_ready = 1;

		HAL_UART_Receive_IT(&huart2, &ASCOM_Byte, 1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint32_t last_interrupt_time = 0;

	uint32_t current_time = HAL_GetTick();

	if(GPIO_Pin == RAINDROP_Pin) {

		if(current_time - last_interrupt_time > 300) {

			raindrop_signal = 1;
			nextion_rain_on = 1;

			last_interrupt_time = current_time;
		}
	}

	if(GPIO_Pin == HOME_Position_Pin) {

		static uint32_t last_tick = 0;
		uint32_t current_tick = HAL_GetTick();

		if(current_tick - last_tick > 150) {
			is_home = 1;
		}

		last_tick = current_tick;
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
