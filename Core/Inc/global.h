/*
 * global.h
 *
 *  Created on: Jul 16, 2023
 *      Author: chris
 */

#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_

#include "stm32f0xx_hal.h"


/* Size of Transmission and receive buffer */
#define I2CSIZE 2
#define UARTSIZE 100


/* 6 ADC channels in total:
 * [0] ADC0: Phase V current
 * [1] ADC3: Phase W current
 * [2] ADC4: VBUS sense (5.12x voltage divider)
 * [3] ADC9: Phase U current
 * [4] TEMP
 * [5] VREF
*/
#define NBR_ADC 6

#define PPAIRS 7 //pole pairs

//calibration values for temperature sensor and ADC internal refernce. See datasheet section 3.10.2
#define TS_CAL1 *((uint16_t*)0x1FFFF7B8)
#define VREFINT_CAL *((uint16_t*)0x1FFFF7BA)

#define LED_STATUS_Pin GPIO_PIN_0
#define LED_STATUS_GPIO_Port GPIOF
#define MAG_NCS_Pin GPIO_PIN_1
#define MAG_NCS_GPIO_Port GPIOF
#define OP_V_O_Pin GPIO_PIN_0
#define OP_V_O_GPIO_Port GPIOA
#define USART_DE_Pin GPIO_PIN_1
#define USART_DE_GPIO_Port GPIOA
#define OP_W_O_Pin GPIO_PIN_3
#define OP_W_O_GPIO_Port GPIOA
#define OP_U_O_Pin GPIO_PIN_1
#define OP_U_O_GPIO_Port GPIOB
#define OC_COMP_INT_Pin GPIO_PIN_12
#define OC_COMP_INT_GPIO_Port GPIOB
#define LSU_Pin GPIO_PIN_13
#define LSU_GPIO_Port GPIOB
#define LSV_Pin GPIO_PIN_14
#define LSV_GPIO_Port GPIOB
#define HSU_Pin GPIO_PIN_8
#define HSU_GPIO_Port GPIOA
#define HSV_Pin GPIO_PIN_9
#define HSV_GPIO_Port GPIOA
#define OC_SEL_Pin GPIO_PIN_11
#define OC_SEL_GPIO_Port GPIOA
#define OC_COMP_INT2_Pin GPIO_PIN_12
#define OC_COMP_INT2_GPIO_Port GPIOA
#define OC_TH_STBY2_Pin GPIO_PIN_6
#define OC_TH_STBY2_GPIO_Port GPIOF
#define OC_TH_STBY1_Pin GPIO_PIN_7
#define OC_TH_STBY1_GPIO_Port GPIOF

#define LED_RED HAL_GPIO_WritePin(GPIOF, LED_STATUS_Pin, 1)
#define LED_GREEN HAL_GPIO_WritePin(GPIOF, LED_STATUS_Pin, 0)

#define ENABLE_DRIVE \
	HAL_GPIO_WritePin(GPIOF, OC_TH_STBY1_Pin, 1); \
	HAL_GPIO_WritePin(GPIOF, OC_TH_STBY2_Pin, 1);
#define DISABLE_DRIVE \
	HAL_GPIO_WritePin(GPIOF, OC_TH_STBY1_Pin, 0); \
	HAL_GPIO_WritePin(GPIOF, OC_TH_STBY2_Pin, 0);
#define RS485_SET_RX \
	HAL_GPIO_WritePin(USART_DE_GPIO_Port, USART_DE_Pin, 0);
#define RS485_SET_TX \
	HAL_GPIO_WritePin(USART_DE_GPIO_Port, USART_DE_Pin, 1);


extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;


typedef struct {
	uint8_t i2c_TX[I2CSIZE];
	uint8_t i2c_RX[I2CSIZE];

	uint8_t uart_TX[UARTSIZE];
	uint8_t uart_RX[3];

	/* Buffer for raw ADC readings */
	uint16_t adc_vals[NBR_ADC];

	uint8_t spi_TX[2];
	uint8_t spi_RX[2];

	uint8_t print_flag;
	uint8_t i2c_complete_flag;
} PeripherialStruct;
extern PeripherialStruct p;



#endif /* INC_GLOBAL_H_ */
