/*
 * foc.c
 *
 *  Created on: Jul 16, 2023
 *      Author: chris
 */

#include "foc.h"

#define ADC_PER_VOLT 1241 //4095/3.3
#define UAMP_PER_ADC 52351

#define ANG_FILT_LVL 4 //
#define ADC_FILT_LVL 8

#define Q16_2_3 43691 // (2/3) << 16
#define Q16_SQRT3_2 56756 // (sqrt(3)/2) << 16
#define Q16_1_2 32768 // (1/2) << 16

static uint8_t step;
static uint16_t mag;

static uint16_t m_angle;
static uint16_t m_angle_prev;
static int16_t revs;
static int32_t cont_angle;
static int32_t cont_angle_prev;
static int32_t rpm;

static uint16_t e_offset;
static uint32_t e_angle;

static int16_t adc_U_offset = 3; //How much the adc values are off at no current
static int16_t adc_V_offset = -10;
static int16_t adc_W_offset = -4;

static uint32_t adc_U_accum = 0;
static uint32_t adc_V_accum = 0;
static uint32_t adc_W_accum = 0;

static uint16_t adc_U = 0;
static uint16_t adc_V = 0;
static uint16_t adc_W = 0;

static int32_t curr_U = 0;
static int32_t curr_V = 0;
static int32_t curr_W = 0;

static uint8_t angle_lut = 0;
static int16_t sin_t = 0;
static int16_t cos_t = 0;

static int32_t I_d = 0;
static int32_t I_q = 0;

static uint32_t count = 0; //incremented every loop, reset at 100Hz
static uint16_t loop_freq = 0; //Hz, calculated at 100Hz using count

void foc_startup() {
	//disable RS485 tranceiver driver
	HAL_GPIO_WritePin(USART_DE_GPIO_Port, USART_DE_Pin, 0);

	HAL_ADC_Stop(&hadc); //stop adc before calibration
	HAL_Delay(1);
	HAL_ADCEx_Calibration_Start(&hadc); //seems like this uses VREFINT_CAL

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // turn on complementary channel
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); // turn on complementary channel
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); // turn on complementary channel

	HAL_TIM_Base_Start_IT(&htim2); //100Hz timer for printing

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	//green, wait 2 seconds, then red to give time for flashing
	LED_RED;
	HAL_Delay(2000);
	LED_GREEN;
	HAL_Delay(100);

	HAL_I2C_EnableListen_IT(&hi2c1);

	//get out of standby mode to allow gate drive
	ENABLE_DRIVE
	;

	//move to step 0
	TIM1->CCR1 = 20;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	HAL_Delay(1000);

	for (int i = 0; i < 10; i++) { //take some angle measurements to let the sensor settle
		HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 0);
		HAL_SPI_TransmitReceive(&hspi1, p.spi_TX, p.spi_RX, 2, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 1);
	}
	// 780.19 angle counts per 1/6th of an electrical cycle
	// 4681.14 angle counts per electrical cycle
	// 90º out of phase would be 1/4th of an electrical cycle, so 1170.285 angle counts
	m_angle = (uint16_t) ((p.spi_RX[0] << 8) + p.spi_RX[1] + 16384); // 0 to 32767
	e_offset = (m_angle * PPAIRS - e_offset) & (32768 - 1); //convert to electrical angle, modulo 32768
	e_angle = 0;

	step = 0;
	mag = 20;

	m_angle_prev = m_angle;
	revs = 0;
	cont_angle = 0;
	cont_angle_prev = 0;
	rpm = 0;

}

void foc_loop() {
	//read MA702 magnetic angle
	HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi1, p.spi_TX, p.spi_RX, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 1);

	m_angle = ((uint16_t) (p.spi_RX[0]) << 8) + p.spi_RX[1] + 16384; // 0 to 32,767 (~91 per degree)
	e_angle = (m_angle * PPAIRS - e_offset) & (32768 - 1); //convert to electrical angle, modulo 32768

	if (m_angle_prev < 8192 && m_angle > 24576) { //detect angle wraparound and increment a revolution
		revs -= 32768;
	} else if (m_angle < 8192 && m_angle_prev > 24576) {
		revs += 32768;
	}
	cont_angle = m_angle + revs;
	m_angle_prev = m_angle;


	//Handle i2c commands
	int cmd = p.i2c_RX[0];
	if (cmd == 0) {
		mag = 0;
	} else if (cmd >= 1 && cmd <= 8) {
		ENABLE_DRIVE;

		if (cont_angle > 36500) {
			step = ((e_angle + 300) % 360) / 60;
			mag = (cont_angle - 36500) / 100;
		} else if (cont_angle < 35500) {
			step = ((e_angle + 120) % 360) / 60;
			mag = (35500 - cont_angle) / 100;
		} else {
			mag = 0;
		}

		if (mag > cmd * 10) {
			mag = cmd * 10;
		}

	} else if (cmd == 9) {
		ENABLE_DRIVE;
		step = ((e_angle + 120) % 360) / 60;
	}

	if (step == 0) {
		TIM1->CCR1 = mag;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = 0;
	}
	if (step == 1) {
		TIM1->CCR1 = mag;
		TIM1->CCR2 = mag;
		TIM1->CCR3 = 0;
	}
	if (step == 2) {
		TIM1->CCR1 = 0;
		TIM1->CCR2 = mag;
		TIM1->CCR3 = 0;
	}
	if (step == 3) {
		TIM1->CCR1 = 0;
		TIM1->CCR2 = mag;
		TIM1->CCR3 = mag;
	}
	if (step == 4) {
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = mag;
	}
	if (step == 5) {
		TIM1->CCR1 = mag;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = mag;
	}

	//read all ADCs
	HAL_ADC_Start_DMA(&hadc, (uint32_t*) p.adc_vals, NBR_ADC);  // start the adc in dma mode

	adc_U = adc_U_accum >> ADC_FILT_LVL;
	adc_U_accum = adc_U_accum - adc_U + (p.adc_vals[3] - adc_U_offset); //https://stackoverflow.com/questions/38918530/simple-low-pass-filter-in-fixed-point

	adc_V = adc_V_accum >> ADC_FILT_LVL;
	adc_V_accum = adc_V_accum - adc_V + (p.adc_vals[0] - adc_V_offset);

	adc_W = adc_W_accum >> ADC_FILT_LVL;
	adc_W_accum = adc_W_accum - adc_W + (p.adc_vals[1] - adc_W_offset);

	curr_U = UAMP_PER_ADC * (adc_U - 2048) / 1000;
	curr_V = UAMP_PER_ADC * (adc_V - 2048) / 1000;
	curr_W = UAMP_PER_ADC * (adc_W - 2048) / 1000;

	//Convert phase currents to DQ currents
	angle_lut = e_angle >> 7; //scale angle to 0-255 for lookup table

	sin_t = sin_lut[angle_lut];
	cos_t = sin_lut[(64 - angle_lut) & (256 - 1)]; //64 out of 256 is the equilvalent of 90º/360º. Modulo 256.

	//calculations shifted 16 bits up
	I_d = ( Q16_2_3 * (cos_t*a + ( Q16_SQRT3_2*sin_t - Q16_1_2*cos_t)*b + (-Q16_SQRT3_2*sin_t - Q16_1_2*cos_t)*c)) >> 16;
	I_q = (-Q16_2_3 * (sin_t*a + (-Q16_SQRT3_2*cos_t - Q16_1_2*sin_t)*b + ( Q16_SQRT3_2*cos_t - Q16_1_2*sin_t)*c)) >> 16;

//	I_d = 0.6666667f * (cf * a + (SQRT3_2 * sf - .5f * cf) * b + (-SQRT3_2 * sf - .5f * cf) * c);   ///Faster DQ0 Transform
//	I_q = 0.6666667f * (-sf * a - (-SQRT3_2*cf-.5f*sf)*b - (SQRT3_2*cf-.5f*sf)*c);

	count++;

	if (p.print_flag) { //100Hz clock

		rpm = ((cont_angle - cont_angle_prev)*100*60) >> 15; //should be accurate within reasonable RPM range if 32-bit

		loop_freq = count * 100;

		memset(p.uart_TX, 0, sizeof(p.uart_TX));

		sprintf((char*) p.uart_TX, " U_mamp: %d \n V_mamp: %d \n W_mamp: %d \n \t", adc_U, adc_V, adc_W);

//		sprintf((char*) p.uart_TX, "Helloo  \r\n\t");
		HAL_UART_Transmit_DMA(&huart1, p.uart_TX, UARTSIZE);

		p.print_flag = 0;
		count = 0;

		//restart I2C listener after a transfer
		if (p.i2c_complete_flag == 1) {
			HAL_I2C_EnableListen_IT(&hi2c1);
			p.i2c_complete_flag = 0;
		}
	}
}

