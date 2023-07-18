/*
 * sixstep.c
 *
 *  Created on: Jul 16, 2023
 *      Author: chris
 */

#include "sixstep.h"





void sixstep_startup() {
	//disable RS485 tranceiver driver
	HAL_GPIO_WritePin(USART_DE_GPIO_Port, USART_DE_Pin, 0);

	//	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	//	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

	HAL_ADCEx_Calibration_Start(&hadc);

	//don't run when not connected to actual power i think
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

	//green, wait 3 seconds, then red to give time for flashing
	LED_red;
	HAL_Delay(2000);
	LED_green;
	HAL_Delay(100);

	HAL_StatusTypeDef status;
	status = HAL_I2C_EnableListen_IT(&hi2c1);
	if (status != HAL_OK) {
		/* Transfer error in reception process */
		Error_Handler();
	}

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
		HAL_SPI_TransmitReceive(&hspi1, spi_TX, spi_RX, 2, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 1);
	}
	// 780.19 angle counts per 1/6th of an electrical cycle
	// 4681.14 angle counts per electrical cycle
	// 90º out of phase would be 1/4th of an electrical cycle, so 1170.285 angle counts
	m_angle = (uint32_t) ((spi_RX[0] << 8) + spi_RX[1] + 16384); // 0 to 32,767
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
	HAL_SPI_TransmitReceive(&hspi1, spi_TX, spi_RX, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 1);
	m_angle = (uint32_t) ((spi_RX[0] << 8) + spi_RX[1] + 16384); // 0 to 32,767
	m_angle = (m_angle * 36000 / 32768); //0 - 36000 (hundreths of degrees)
	e_angle = ((m_angle / 100 * PPAIRS) - e_offset) % 360;

	m_angle %= 36000;

	if (m_angle_prev < 9000 && m_angle > 27000) {
		revs -= 36000;
	} else if (m_angle < 9000 && m_angle_prev > 27000) {
		revs += 36000;
	}
	cont_angle = (99 * cont_angle + 1 * (m_angle + revs)) / 100;

	int cmd = i2c_RX[0];
	if (cmd == 0) {
		HAL_GPIO_WritePin(GPIOF, OC_TH_STBY1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, OC_TH_STBY2_Pin, GPIO_PIN_RESET);
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
	HAL_ADC_Start_DMA(&hadc, (uint32_t*) adc_vals, NBR_ADC);  // start the adc in dma mode
//	int16_t temp = (1000 * (1908 - adc_vals[4])) / 5337;

	//		adc_to_volt = 3.0 * VREFINT_CAL / (adc_vals[0] * 4095.);
	//		float volt1 = adc_vals[1] * adc_to_volt;

	//		printf("temp: %d \n", temp);

	//		printf("ADC: \r\n");
	//		for(int i = 0; i < NBR_ADC; i++){
	//			if(i == 2){ //Vbus sensor
	//				printf(" %d", 330*adc_vals[i] * 512 / 4095);
	//			}else{
	//				printf(" %d", adc_vals[i]);
	//			}
	//		}
	//		printf("\r\n");

	if (do_print) {
		//			LED_red;

		rpm = (cont_angle - cont_angle_prev) / 60;
		cont_angle_prev = cont_angle;

		//			HAL_GPIO_TogglePin(GPIOF, LED_STATUS_Pin);

		//			HAL_UART_Transmit_DMA(&huart1, uart_TX, 100);

		sprintf((char*) uart_TX, " adc0: %d \n adc1: %d \n adc3: %d \n\t", adc_vals[0], adc_vals[1], adc_vals[3]);
		//			addprint("cont_angle: %ld \n", cont_angle);
		//			addprint("i2c_RX: %d \n", i2c_RX[0]);
		//			addprint("adc1: %d \n", adc_vals[1]);
		//			addprint("rpm: %ld \n", rpm);
		//			addprint("\t");
		HAL_UART_Transmit_DMA(&huart1, uart_TX, 100);

		//			HAL_UART_Transmit(&huart1, uart_TX, 100, HAL_MAX_DELAY);

		do_print = 0;

		//restart I2C listener after a transfer
		if (i2c_complete == 1) {
			HAL_I2C_EnableListen_IT(&hi2c1);
			i2c_complete = 0;
		}
		//			LED_green;
	}
}
