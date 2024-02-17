/*
 * encoder.c
 *
 *  Created on: Dec 12, 2023
 *      Author: chris
 */

#include "foc.h"
#include "comdef.h"
#include "utils.h"
#include "global.h"
#include "stm32f0xx_hal_uart.h"
#include "main.h"

#define ADC_PER_VOLT 1241 // 4095/3.3
#define UAMP_PER_ADC 52351

#define MANGLE_FILT_LVL 4

 


static uint16_t m_angle_new = 0;
static uint32_t m_angle_accum = 0;
static uint16_t m_angle = 0;
static uint16_t m_angle_prev = 0;
static int32_t revs = 0;
static int32_t cont_angle = 0;
static int32_t cont_angle_prev = 0;
static int32_t rpm = 0;

static uint16_t e_offset = 0;
static uint16_t e_angle = 0;


static int16_t encoder_res = 7;


static uint32_t count = 0;     // incremented every loop, reset at 100Hz
static uint16_t loop_freq = 0; // Hz, calculated at 100Hz using count


void encoder_startup() {

    // no motor power in encoder mode
    DISABLE_DRIVE;
    
    HAL_TIM_Base_Start_IT(&htim2); // 100Hz timer


    // green, wait 2 seconds, then red to give time for flashing
    LED_RED;
    HAL_Delay(2000);
    LED_GREEN;
    HAL_Delay(1000);

    for (int i = 0; i < 10; i++) { // take some measurements to let the sensors settle
        HAL_GPIO_WritePin(MAG1_CS_GPIO_Port, MAG1_CS_Pin, 0);
        HAL_SPI_TransmitReceive(&hspi1, p.spi_TX, p.spi_RX, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(MAG1_CS_GPIO_Port, MAG1_CS_Pin, 1);

        HAL_UART_Receive(&huart1, p.uart_RX, 1, 1);
    }

	m_angle = (uint16_t)((p.spi_RX[0] << 8) + p.spi_RX[1] + 16384); // 0 to 32767
    m_angle_prev = m_angle;
    e_offset = (m_angle * PPAIRS - e_offset) & (32768 - 1);         // convert to electrical angle, modulo 32768

    HAL_UART_Receive_IT(&huart1, p.uart_RX, UARTSIZE);
}

void encoder_loop() {

    count++;

    // read MA702, update m_angle, e_angle, and cont_angle
    {
        HAL_GPIO_WritePin(MAG1_CS_GPIO_Port, MAG1_CS_Pin, 0);
        HAL_SPI_TransmitReceive(&hspi1, p.spi_TX, p.spi_RX, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(MAG1_CS_GPIO_Port, MAG1_CS_Pin, 1);


        // angles represented in [0,32767] (~91 per degree)
        m_angle_new = ((uint16_t)(p.spi_RX[0]) << 8) + p.spi_RX[1] + 16384;
        m_angle = m_angle_accum >> MANGLE_FILT_LVL;
		m_angle_accum = m_angle_accum - m_angle + m_angle_new;


        e_angle = (m_angle * PPAIRS - e_offset) & (32768 - 1); // convert to electrical angle and modulo

        if (m_angle_prev < 8192 && m_angle > 24576) { // detect angle wraparound and increment a revolution
            revs -= 32768;
        } else if (m_angle < 8192 && m_angle_prev > 24576) {
            revs += 32768;
        }
        cont_angle = m_angle + revs;
        m_angle_prev = m_angle;
    }


    if (p.uart_received_flag) {
        p.uart_received_flag = 0;

        // clear the uart buffer
        uint8_t temp_buffer[UARTSIZE];
        while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
            HAL_UART_Receive(&huart1, temp_buffer, 1, 1);
        }

        // check which of 3 bytes is the cmd and concat 14 data bytes into int16_t (signed)
        if (p.uart_RX[0] & 0x80) {
            p.uart_cmd[0] = p.uart_RX[0] & CMD_MASK;
            p.uart_cmd[1] = (p.uart_RX[1] << 7) | (p.uart_RX[2]);
        } else if (p.uart_RX[1] & 0x80) {
            p.uart_cmd[0] = p.uart_RX[1] & CMD_MASK;
            p.uart_cmd[1] = (p.uart_RX[2] << 7) | (p.uart_RX[0]);
        } else {
            p.uart_cmd[0] = p.uart_RX[2] & CMD_MASK;
            p.uart_cmd[1] = (p.uart_RX[0] << 7) | (p.uart_RX[1]);
        }
        p.uart_cmd[1] = pad14(p.uart_cmd[1]);

        if (p.uart_cmd[0] == CMD_SET_VOLTAGE) {

        } else if (p.uart_cmd[0] == CMD_SET_CURRENT) {

        } else if (p.uart_cmd[0] == CMD_SET_POSITION) {

        } else if (p.uart_cmd[0] == CMD_SET_SPEED) {
            
        } else if (p.uart_cmd[0] == CMD_GET_POSITION) {
            encoder_res = p.uart_cmd[1];
        }


        p.uart_TX[0] = (uint8_t)(cont_angle >> (encoder_res + 7)) & 0b01111111;
        p.uart_TX[1] = (uint8_t)(cont_angle >> (encoder_res + 0)) & 0b01111111;

        uint8_t checksum = 0;
        for(int i=0; i<2; i++){
            checksum += p.uart_TX[i];
        }
        p.uart_TX[2] = (uint8_t)(checksum) & 0b01111111;
        p.uart_TX[3] = MIN_INT8;

        // p.uart_txready_flag = 1;
    }

    if (p.clock_1khz_flag) { // 100Hz clock

        rpm = ((cont_angle - cont_angle_prev) * 100 * 60) >> 15; // should be accurate within reasonable RPM range if 32-bit
        cont_angle_prev = cont_angle;

        loop_freq = count * 100;
        count = 0;

        p.uart_watchdog++;
        if (p.uart_watchdog > 5) {
            p.uart_watchdog = 5;
            // duty = 0;
        }

        p.clock_1khz_flag = 0;
    }
    LED_GREEN;
}
