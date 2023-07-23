/*
 * sixstep.c
 *
 *  Created on: Jul 16, 2023
 *      Author: chris
 */

#include "sixstep.h"

#define curr_avg_window 100
#define adc_per_volt 1241 //4095/3.3

uint8_t step;
uint16_t mag;

uint32_t m_angle;
uint32_t m_angle_prev;
int32_t revs;
int32_t cont_angle;
int32_t cont_angle_prev;
int32_t rpm;

uint16_t e_offset;
uint16_t e_angle;

int16_t adc_U_offset = 3; //How much the adc values are off at no current
int16_t adc_V_offset = -10;
int16_t adc_W_offset = -4;



uint32_t adc_U_buf[curr_avg_window] = {0};
uint32_t adc_V_buf[curr_avg_window] = {0};
uint32_t adc_W_buf[curr_avg_window] = {0};
uint8_t adc_buf_index = 0;

uint16_t adc_U = 0;
uint16_t adc_V = 0;
uint16_t adc_W = 0;

uint16_t curr_U = 0;
uint16_t curr_V = 0;
uint16_t curr_W = 0;

uint32_t sum = 0;

void sixstep_startup() {
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
	LED_red;
	HAL_Delay(2000);
	LED_green;
	HAL_Delay(100);

	HAL_I2C_EnableListen_IT(&hi2c1);

	//get out of standby mode to allow gate drive
	HAL_GPIO_WritePin(GPIOF, OC_TH_STBY1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, OC_TH_STBY2_Pin, GPIO_PIN_SET);
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
	m_angle = (uint32_t) ((p.spi_RX[0] << 8) + p.spi_RX[1] + 16384); // 0 to 32,767
	m_angle = m_angle * 36000 / 32768; //0 - 36000 (hundreths of degrees)
	e_offset = (m_angle / 100 * PPAIRS) % 360;
	e_angle = 0;

	step = 0;
	mag = 20;

	m_angle_prev = m_angle;
	revs = 0;
	cont_angle = 0;
	cont_angle_prev = 0;
	rpm = 0;
}

void sixstep_loop() {
	//read MA702 magnetic angle
	HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi1, p.spi_TX, p.spi_RX, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 1);
	m_angle = (uint32_t) ((p.spi_RX[0] << 8) + p.spi_RX[1] + 16384); // 0 to 32,767
	m_angle = (m_angle * 36000 / 32768); //0 - 36000 (hundreths of degrees)
	e_angle = ((m_angle / 100 * PPAIRS) - e_offset) % 360;

	m_angle %= 36000;

	if (m_angle_prev < 9000 && m_angle > 27000) {
		revs -= 36000;
	} else if (m_angle < 9000 && m_angle_prev > 27000) {
		revs += 36000;
	}
	cont_angle = (99 * cont_angle + 1 * (m_angle + revs)) / 100;

	int cmd = p.i2c_RX[0];
	if (cmd == 0) {
		mag = 0;
//		HAL_GPIO_WritePin(GPIOF, OC_TH_STBY1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOF, OC_TH_STBY2_Pin, GPIO_PIN_RESET);
	} else if (cmd >= 1 && cmd <= 8) {
		HAL_GPIO_WritePin(GPIOF, OC_TH_STBY1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF, OC_TH_STBY2_Pin, GPIO_PIN_SET);
		//			step = cmd - 1;

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
		HAL_GPIO_WritePin(GPIOF, OC_TH_STBY1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF, OC_TH_STBY2_Pin, GPIO_PIN_SET);
		step = ((e_angle + 120) % 360) / 60;
	}

	m_angle_prev = m_angle;

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


	adc_U_buf[adc_buf_index] = p.adc_vals[3] - adc_U_offset;
	adc_V_buf[adc_buf_index] = p.adc_vals[0] - adc_V_offset;
	adc_W_buf[adc_buf_index] = p.adc_vals[1] - adc_W_offset;

	adc_buf_index++;
	if (adc_buf_index >= curr_avg_window) {
		adc_buf_index = 0;
	}

	sum = 0;
	for (int i = 0; i < curr_avg_window; i++) sum += adc_U_buf[i];
	adc_U = (uint16_t)(sum/curr_avg_window);
	sum = 0;
	for (int i = 0; i < curr_avg_window; i++) sum += adc_V_buf[i];
	adc_V = (uint16_t)(sum/curr_avg_window);
	sum = 0;
	for (int i = 0; i < curr_avg_window; i++) sum += adc_W_buf[i];
	adc_W = (uint16_t)(sum/curr_avg_window);

//	adc_U = avg(adc_U_buf);
//	adc_V = avg(adc_V_buf);
//	adc_W = avg(adc_W_buf);

//	adc_U = p.adc_vals[3] * 1000 / adc_per_volt;
//	adc_V = p.adc_vals[0] * 1000 / adc_per_volt;
//	adc_W = p.adc_vals[1] * 1000 / adc_per_volt;

//	adc_U_filt = (100 * p.adc_vals[3] + 9900 * adc_U_filt) / 10000;
//	adc_V_filt = (100 * p.adc_vals[0] + 9900 * adc_V_filt) / 10000;
//	adc_W_filt = (100 * p.adc_vals[1] + 9900 * adc_W_filt) / 10000;



	if (p.print_flag) {

		rpm = (cont_angle - cont_angle_prev) / 60;
		cont_angle_prev = cont_angle;

		memset(p.uart_TX, 0, sizeof(p.uart_TX));

		sprintf((char*) p.uart_TX, " U_filt: %d \n V_filt: %d \n W_filt: %d \n \t",
				adc_U, adc_V, adc_W);

//		sprintf((char*) p.uart_TX, "Helloo  \r\n\t");
		HAL_UART_Transmit_DMA(&huart1, p.uart_TX, UARTSIZE);

		p.print_flag = 0;

		//restart I2C listener after a transfer
		if (p.i2c_complete_flag == 1) {
			HAL_I2C_EnableListen_IT(&hi2c1);
			p.i2c_complete_flag = 0;
		}
	}
}


uint16_t avg(uint16_t *buf){
	sum = 0;
	uint8_t len = curr_avg_window;
	for (int i = 0; i < len; i++) {
		sum += buf[i];
	}
	return (uint16_t) (sum);
}

//uint32_t avg_filter(uint16_t new_val, uint16_t *buf){
//	uint8_t len = sizeof(buf) / sizeof(buf[0]);
//	uint32_t sum = 0;
//	for(uint8_t i = len - 1; i > 0; i--){
//		history[i] = history[i-1];
//		sum += history[i];
//	}
//	history[0] = new_val;
//	sum += history[0];
//
//	return sum / len;
//}





