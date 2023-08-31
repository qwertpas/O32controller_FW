/*
 * foc.c
 *
 *  Created on: Jul 16, 2023
 *      Author: chris
 */

#include "foc.h"
#include "comdef.h"

#define ADC_PER_VOLT 1241 // 4095/3.3
#define UAMP_PER_ADC 52351

#define ADC_FILT_LVL 0
#define DQ_FILT_LVL 8

#define Q16_2_3 ((uint16_t)43691)     // (2/3) * 2^16
#define Q16_SQRT3_2 ((uint16_t)56756) // (sqrt(3)/2) * 2^16
#define Q16_1_2 ((uint16_t)32768)     // (1/2) * 2^16

#define KP_d 0
#define KI_d 0
#define KF_d 0
#define KP_q 0
#define KI_q 0
#define KF_q 0

static uint8_t reverse;
static uint16_t mag;

static uint8_t step;
static uint8_t duty_cycle;

static uint16_t m_angle;
static uint16_t m_angle_prev;
static int32_t revs;
static int32_t cont_angle;
static int32_t cont_angle_prev;
static int32_t rpm;

static uint16_t e_offset;
static uint16_t e_angle;

// How much the adc values are off at no current, offset by 2048 to center zero
// current at 0
static int16_t adc_U_offset = 2048 + 3;
static int16_t adc_V_offset = 2048 + -10;
static int16_t adc_W_offset = 2048 + -4;

// units are in ADC counts [-2048,2047]
static int16_t I_u = 0;
static int16_t I_v = 0;
static int16_t I_w = 0;

// used for filtering
static int32_t I_u_accum = 0;
static int32_t I_v_accum = 0;
static int32_t I_w_accum = 0;

// units are in ADC counts [-2048,2047]
static int16_t I_d = 0;
static int16_t I_q = 0;
static int32_t I_d_accum = 0;
static int32_t I_q_accum = 0;
static int16_t I_d_filt = 0;
static int16_t I_q_filt = 0;

// current setpoints
static int16_t I_d_des = 0; // usually 0 unless field weakening
static int16_t I_d_error = 0;
static int32_t I_d_error_int = 0;

static int16_t I_q_des = 0;
static int16_t I_q_error = 0;
static int32_t I_q_error_int = 0;

// DQ voltage commands
static int16_t V_d = 0; // usually 0 unless field weakening
static int16_t V_q = 0; // usually 0 unless field weakening

static uint32_t count = 0;     // incremented every loop, reset at 100Hz
static uint16_t loop_freq = 0; // Hz, calculated at 100Hz using count

int16_t clip(int16_t x, int16_t min, int16_t max) {
    if (x > max) {
        return max;
    } else if (x < min) {
        return min;
    } else {
        return x;
    }
}

void foc_startup() {

    HAL_ADC_Stop(&hadc); // stop adc before calibration
    HAL_Delay(1);
    HAL_ADCEx_Calibration_Start(&hadc); // seems like this uses VREFINT_CAL

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // turn on complementary channel
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); // turn on complementary channel
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); // turn on complementary channel

    HAL_TIM_Base_Start_IT(&htim2); // 100Hz timer for printing

    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    // green, wait 2 seconds, then red to give time for flashing
    LED_RED;
    HAL_Delay(2000);
    LED_GREEN;
    HAL_Delay(100);

    // get out of standby mode to allow gate drive
    ENABLE_DRIVE;

    // move to step 0
    TIM1->CCR1 = 20;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    HAL_Delay(1000);

    for (int i = 0; i < 10; i++) { // take some measurements to let the sensors settle
        HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 0);
        HAL_SPI_TransmitReceive(&hspi1, p.spi_TX, p.spi_RX, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 1);

        HAL_ADC_Start_DMA(&hadc, (uint32_t *)p.adc_vals, NBR_ADC); // start the adc in dma mode
    }
    // 780.19 angle counts per 1/6th of an electrical cycle
    // 4681.14 angle counts per electrical cycle
    // 90º out of phase would be 1/4th of an electrical cycle, so 1170.285 angle counts
    m_angle = (uint16_t)((p.spi_RX[0] << 8) + p.spi_RX[1] + 16384); // 0 to 32767
    e_offset = (m_angle * PPAIRS - e_offset) & (32768 - 1);         // convert to electrical angle, modulo 32768
    e_angle = 0;

    step = 0;
    mag = 20;

    m_angle_prev = m_angle;
    revs = 0;
    cont_angle = 0;
    cont_angle_prev = 0;
    rpm = 0;

    HAL_UART_Receive_IT(&huart1, p.uart_RX, UARTSIZE);
}

void foc_loop() {

    count++;

    // to collapse FOC calcs
    {
        // read MA702 magnetic angle
        HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 0);
        HAL_SPI_TransmitReceive(&hspi1, p.spi_TX, p.spi_RX, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 1);

        // angles represented in [0,32767] (~91 per degree)
        m_angle = ((uint16_t)(p.spi_RX[0]) << 8) + p.spi_RX[1] + 16384;
        e_angle = (m_angle * PPAIRS - e_offset) &
                  (32768 - 1); // convert to electrical angle and modulo

        if (m_angle_prev < 8192 &&
            m_angle > 24576) { // detect angle wraparound and increment a revolution
            revs -= 32768;
        } else if (m_angle < 8192 && m_angle_prev > 24576) {
            revs += 32768;
        }
        cont_angle = m_angle + revs;
        m_angle_prev = m_angle;

        // read ADCs
        HAL_ADC_Start_DMA(&hadc, (uint32_t *)p.adc_vals, NBR_ADC);

        // filter ADC values (https://stackoverflow.com/questions/38918530/simple-low-pass-filter-in-fixed-point)
        // phase currents are in adc units [-2048, 2047] (1 bit sign, 11 bit value)
        // to get current in milliamps, multiply by UAMP_PER_ADC then divide by 1000
        I_u = I_u_accum >> ADC_FILT_LVL;
        I_u_accum = I_u_accum - I_u + (p.adc_vals[3] - adc_U_offset);

        I_v = I_v_accum >> ADC_FILT_LVL;
        I_v_accum = I_v_accum - I_v + (p.adc_vals[0] - adc_V_offset);

        I_w = I_w_accum >> ADC_FILT_LVL;
        I_w_accum = I_w_accum - I_w + (p.adc_vals[1] - adc_W_offset);

        // Convert phase currents to DQ currents (DQ0 transform):
        uint8_t angle_lut = e_angle >> 7; // scale e_angle [0,32767] to [0,255] for lookup table

        // each term below has 15 fractional bits and is signed, floating point equilvalent < 1
        int16_t Q16_sin_t = sin_lut[angle_lut];
        int16_t Q16_cos_t;
        if (angle_lut < 64) {
            Q16_cos_t = sin_lut[(64 - angle_lut) & (256 - 1)]; /// 64 out of 256 is the equilvalent of /// 90º/360º. &255 is mod256.
        } else {
            Q16_cos_t = sin_lut[(63 - angle_lut) & (256 - 1)];
        }

        // some intermediate rounding, avg errors in Iq and Id are around 0.1%
        int16_t Q16_SQRT3_2_sin_t = (Q16_SQRT3_2 * Q16_sin_t) >> 16;
        int16_t Q16_SQRT3_2_cos_t = (Q16_SQRT3_2 * Q16_cos_t) >> 16;
        int16_t Q16_1_2_sin_t = (Q16_1_2 * Q16_sin_t) >> 16;
        int16_t Q16_1_2_cos_t = (Q16_1_2 * Q16_cos_t) >> 16;

        I_d = (Q16_cos_t * I_u + (Q16_SQRT3_2_sin_t - Q16_1_2_cos_t) * I_v + (-Q16_SQRT3_2_sin_t - Q16_1_2_cos_t) * I_w) >> 16;
        I_q = (Q16_sin_t * I_u + (-Q16_SQRT3_2_cos_t - Q16_1_2_sin_t) * I_v + (Q16_SQRT3_2_cos_t - Q16_1_2_sin_t) * I_w) >> 16;
        I_d = (I_d * Q16_2_3) >> 15;
        I_q = (I_q * -Q16_2_3) >> 15;

        I_d_filt = I_d_accum >> DQ_FILT_LVL;
        I_d_accum = I_d_accum - I_d_filt + I_d;

        I_q_filt = I_q_accum >> DQ_FILT_LVL;
        I_q_accum = I_q_accum - I_q_filt + I_q;

        I_d_error = I_d_des - I_d;
        I_q_error = I_q_des - I_q;

        I_d_error_int = clip(I_d_error_int + I_d_error, -32, 32);
        I_q_error_int = clip(I_q_error_int + I_q_error, -32, 32);

        // PI+feedforward control loop
        // Voltage is scaled to [-32768,32767]
        V_d = KP_d * I_d_error + KI_d * I_d_error_int + KF_d * I_d_des;
        V_q = KP_q * I_q_error + KI_q * I_q_error_int + KF_q * I_q_des;
        V_d = clip(V_d, -32, 32);
        V_q = clip(V_q, -32, 32);

        // Convert DQ voltages to phase voltages, avg error around 0.4%
        int32_t V_u = (Q16_cos_t * V_d - Q16_sin_t * V_q) >> 15;
        int32_t V_v = ((Q16_SQRT3_2_sin_t - Q16_1_2_cos_t) * V_d + (Q16_SQRT3_2_cos_t + Q16_1_2_sin_t) * V_q) >> 15;
        int32_t V_w = (-(Q16_SQRT3_2_sin_t + Q16_1_2_cos_t) * V_d - (Q16_SQRT3_2_cos_t - Q16_1_2_sin_t) * V_q) >> 15;
    }

    if (!reverse) {
	   step = ((-e_angle + 27307) & (32768 - 1)) / 5461; // divide 16 bit angle into sextants
    } else {
        step = ((-e_angle + 10923) & (32768 - 1)) / 5461;
    }

    // step = ((32768 - e_angle + 27307 + 0) & (32768 - 1)) / 5462; // divide 16 bit angle into sextants


    // step = 0;
//     if(count % 5000 == 0){
//         step = (step + 1) % 6;
//     }
//       duty_cycle = 10;

    // six-step commutation
    if (step == 0) {
        TIM1->CCR1 = duty_cycle;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
    }
    if (step == 1) {
        TIM1->CCR1 = duty_cycle;
        TIM1->CCR2 = duty_cycle;
        TIM1->CCR3 = 0;
    }
    if (step == 2) {
        TIM1->CCR1 = 0;
        TIM1->CCR2 = duty_cycle;
        TIM1->CCR3 = 0;
    }
    if (step == 3) {
        TIM1->CCR1 = 0;
        TIM1->CCR2 = duty_cycle;
        TIM1->CCR3 = duty_cycle;
    }
    if (step == 4) {
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = duty_cycle;
    }
    if (step == 5) {
        TIM1->CCR1 = duty_cycle;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = duty_cycle;
    }

    if (p.uart_idle) {

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

        // If negative, take the absolute value assuming two's complement
        reverse = (p.uart_cmd[1] >> 13) & 1;
        mag = reverse ? (~p.uart_cmd[1]) + 1 : p.uart_cmd[1];

        if (p.uart_cmd[0] == CMD_SET_VOLTAGE) {
            duty_cycle = mag >> 6;
        } else if (p.uart_cmd[0] == CMD_SET_SPEED) {
            // implement later
        }
        

        p.uart_TX[0] = (uint8_t)(rpm >> 7) & 0b01111111;
        p.uart_TX[1] = (uint8_t)(rpm) & 0b01111111;
        // p.uart_TX[2] = (uint8_t) (0);

        RS485_SET_TX;
        HAL_UART_Transmit_DMA(&huart1, p.uart_TX, 2); // DMA channel 4
        p.uart_idle = 0;
    }

    if (p.print_flag) { // 100Hz clock

        rpm = ((cont_angle - cont_angle_prev) * 100 * 60) >> 15; // should be accurate within reasonable RPM range if 32-bit
        cont_angle_prev = cont_angle;

        loop_freq = count * 100;
//        count = 0;

        // uint8_t print_TX[50];
		// sprintf((char*) print_TX, " freq: %d\n I_d_filt: %d\n I_q_filt: %d\n \t", loop_freq, I_d_filt, I_q_filt);
        // HAL_UART_Transmit_DMA(&huart1, print_TX, 50);

        p.print_flag = 0;
    }
    LED_GREEN;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { // gets called before all bits finish
    HAL_UART_Receive_IT(&huart1, p.uart_RX, UARTSIZE);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    RS485_SET_RX;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) { // receive overrun error happens once in a while, just restart RX
    HAL_UART_Receive_IT(&huart1, p.uart_RX, UARTSIZE);
    LED_RED;
}
