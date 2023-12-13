/*
 * encoder.c
 *
 *  Created on: Dec 12, 2023
 *      Author: chris
 */

#include "foc.h"
#include "comdef.h"
#include "utils.h"

#define ADC_PER_VOLT 1241 // 4095/3.3
#define UAMP_PER_ADC 52351

#define ADC_FILT_LVL 6
#define DQ_FILT_LVL 8

 
static uint8_t reverse = 0;
static uint16_t mag = 0;

static uint8_t step = 0;
static uint16_t duty = 0;
static uint16_t duty_offset = 0;
static uint16_t duty_offsetted = 0;

static uint16_t m_angle = 0;
static uint16_t m_angle_prev = 0;
static int32_t revs = 0;
static int32_t cont_angle = 0;
static int32_t cont_angle_prev = 0;
static int32_t rpm = 0;

static uint16_t e_offset = 0;
static uint16_t e_angle = 0;

static uint16_t I_max = 130;
static int32_t cont_angle_des = MIN_INT32; //min means no position tracking

static int16_t encoder_res = 7;

// How much the adc values are off at no current, offset by 2048 to center zero
// current at 0
static int16_t adc_U_offset = 2048 + 3 - 86;
static int16_t adc_V_offset = 2048 + -10 - 60;
static int16_t adc_W_offset = 2048 + -4 - 71;

// units are in ADC counts [-2048,2047]
static int16_t I_u = 0;
static int16_t I_v = 0;
static int16_t I_w = 0;
static int16_t I_phase = 0; //max phase current

// used for filtering
static int32_t I_u_accum = 0;
static int32_t I_v_accum = 0;
static int32_t I_w_accum = 0;

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
        HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 0);
        HAL_SPI_TransmitReceive(&hspi1, p.spi_TX, p.spi_RX, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 1);

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
        HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 0);
        HAL_SPI_TransmitReceive(&hspi1, p.spi_TX, p.spi_RX, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 1);

        // angles represented in [0,32767] (~91 per degree)
        m_angle = ((uint16_t)(p.spi_RX[0]) << 8) + p.spi_RX[1] + 16384;
        e_angle = (m_angle * PPAIRS - e_offset) & (32768 - 1); // convert to electrical angle and modulo

        if (m_angle_prev < 8192 && m_angle > 24576) { // detect angle wraparound and increment a revolution
            revs -= 32768;
        } else if (m_angle < 8192 && m_angle_prev > 24576) {
            revs += 32768;
        }
        cont_angle = m_angle + revs;
        m_angle_prev = m_angle;
    }


    if (p.uart_idle) {

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

        RS485_SET_TX;
        HAL_UART_Transmit_DMA(&huart1, p.uart_TX, 4); // DMA channel 4
        p.uart_idle = 0;
    }

    if (p.print_flag) { // 100Hz clock

        rpm = ((cont_angle - cont_angle_prev) * 100 * 60) >> 15; // should be accurate within reasonable RPM range if 32-bit
        cont_angle_prev = cont_angle;

        loop_freq = count * 100;
        count = 0;

        p.uart_watchdog++;
        if (p.uart_watchdog > 5) {
            p.uart_watchdog = 5;
            // duty = 0;
        }

        p.print_flag = 0;
    }
    LED_GREEN;
}