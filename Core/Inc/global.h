/*
 * global.h
 *
 *  Created on: Jul 16, 2023
 *      Author: chris
 */

#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_

#include "stm32f0xx_hal.h"

// 4-bit address
#define UART_ADDR 0x5

#define DO_FOC 0

// Motor parameters
#define PPAIRS 7        // pole pairs
#define INVERT_MAG 1    //invert=0 for red motor, invert=1 for green motor
#define POS_KP 10
#define POS_THRES 10

#define MAX_INT32 0x7FFFFFFF
#define MIN_INT32 0x80000000 //used as the end char
#define MIN_INT8 0x80 //used as the end char

/* Size of Transmission and receive buffer */
#define I2CSIZE 2
#define UARTSIZE 10

/* 6 ADC channels in total:
 * [0] ADC0: Phase V current
 * [1] ADC3: Phase W current
 * [2] ADC4: VBUS sense (5.12x voltage divider)
 * [3] ADC9: Phase U current
 * [4] TEMP
 * [5] VREF
 */
#define NBR_ADC 6

// calibration values for temperature sensor and ADC internal refernce. See datasheet section 3.10.2
#define TS_CAL1 *((uint16_t *)0x1FFFF7B8)
#define VREFINT_CAL *((uint16_t *)0x1FFFF7BA)

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
#define LED_TOGGLE HAL_GPIO_TogglePin(GPIOF, LED_STATUS_Pin);

#define ENABLE_DRIVE                              \
    HAL_GPIO_WritePin(GPIOF, OC_TH_STBY1_Pin, 1); \
    HAL_GPIO_WritePin(GPIOF, OC_TH_STBY2_Pin, 1);
#define DISABLE_DRIVE                             \
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

    uint8_t uart_TX[UARTSIZE]; // send to Teensy
    uint8_t uart_RX[UARTSIZE]; // receive from Teensy (may be out of order)
    int16_t uart_cmd[UARTSIZE]; //first byte is command, rest is data packed into 16 bits

    /* Buffer for raw ADC readings */
    uint16_t adc_vals[NBR_ADC];

    uint8_t spi_TX[2];
    uint8_t spi_RX[2];

    uint8_t print_flag;
    uint8_t i2c_complete_flag;
    uint8_t uart_idle;
    uint8_t adc_conversion_flag;

    uint8_t uart_watchdog;
} PeripherialStruct;
extern PeripherialStruct p;

#endif /* INC_GLOBAL_H_ */
