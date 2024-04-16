/*
 * sixstep.c
 *
 *  Created on: Jul 16, 2023
 *      Author: chris
 */

#include "foc.h"
#include "comdef.h"
#include "utils.h"

#define ADC_PER_VOLT 1241 // 4095/3.3
#define UAMP_PER_ADC 52351

#define ADC_FILT_LVL 6
#define DQ_FILT_LVL 8
#define VBUS_FILT_LVL 8

 
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
static int16_t adc_U_offset = 2048;
static int16_t adc_V_offset = 2048;
static int16_t adc_W_offset = 2048;

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

static int16_t vbus = 0;
static int32_t vbus_accum = 0;


void sixstep_startup() {

    HAL_ADC_Stop(&hadc); // stop adc before calibration
    HAL_Delay(1);
    HAL_ADCEx_Calibration_Start(&hadc); // seems like this uses VREFINT_CAL

    HAL_TIM_Base_Start(&htim1); //PWM and ADC timer
    TIM1->EGR = TIM_EGR_UG;
    htim1.Instance->RCR = 1; //timer interrupts for center of peak and trough, ignore troughs
    TIM1->EGR = TIM_EGR_UG; //update timer config


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
    TIM1->CCR1 = MAX_DUTY/16;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    HAL_Delay(1000);

    for (int i = 0; i < 10; i++) { // take some measurements to let the sensors settle
        HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 0);
        HAL_SPI_TransmitReceive(&hspi1, p.spi_TX, p.spi_RX, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOF, MAG_NCS_Pin, 1);

        // HAL_ADC_Start_DMA(&hadc, (uint32_t *)p.adc_vals, NBR_ADC); // start the adc in dma mode

        HAL_UART_Receive(&huart1, p.uart_RX, 1, 1);
    }

	m_angle = (uint16_t)((p.spi_RX[0] << 8) + p.spi_RX[1] + 16384); // 0 to 32767
    m_angle_prev = m_angle;
    e_offset = (m_angle * PPAIRS - e_offset) & (32768 - 1);         // convert to electrical angle, modulo 32768

    // stop motor
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    // HAL_UART_Receive_IT(&huart1, p.uart_RX, UARTSIZE);
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, p.uart_RX, UART_RX_SIZE);

}

void sixstep_loop() {

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

	//read currents from ADCs and low pass filter
	{
		// filter ADC values (https://stackoverflow.com/questions/38918530/simple-low-pass-filter-in-fixed-point)
		// phase currents are in adc units [-2048, 2047] (1 bit sign, 11 bit value)
		// to get current in milliamps, multiply by UAMP_PER_ADC then divide by 1000
		I_u = I_u_accum >> ADC_FILT_LVL;
		I_u_accum = I_u_accum - I_u + (p.adc_vals[3] - adc_U_offset);

		I_v = I_v_accum >> ADC_FILT_LVL;
		I_v_accum = I_v_accum - I_v + (p.adc_vals[0] - adc_V_offset);

		I_w = I_w_accum >> ADC_FILT_LVL;
		I_w_accum = I_w_accum - I_w + (p.adc_vals[1] - adc_W_offset);

		I_phase = abs16(I_u);
		if(abs16(I_v) > I_phase) I_phase = abs16(I_v);
		if(abs16(I_w) > I_phase) I_phase = abs16(I_w);
	}

	//position control
	if(cont_angle_des != MIN_INT32){
		//tracking position
		int32_t cont_angle_error = cont_angle_des - cont_angle;

		reverse = (cont_angle_error > 0) ? 0 : 1;

		duty = abs32(cont_angle_error) >> 6;
		// reverse = (cont_angle_des > cont_angle) ? 1 : 0;

		// duty = 20;
		duty = clip(duty, 0, 60);
	}

	//current limit
	if(I_phase > I_max){
        if(duty_offset < duty) duty_offset += 1; //is subtracted from duty cycle
	}else{
        if(duty_offset > 0) duty_offset -= 1;
    }


    // six-step commutation using e_angle, reverse, duty
    {
        // calculate step from e_angle
        if (INVERT_MAG) e_angle *= -1;
        if (reverse) {
            step = ((e_angle + 27307) & (32768 - 1)) / 5461; // divide 16 bit angle into sextants
        } else {
            step = ((e_angle + 10923) & (32768 - 1)) / 5461;
        }

        duty_offsetted = clip(duty - duty_offset, 0, MAX_DUTY);
        duty_offsetted = duty;

		//apply duty to phases according to step
		if (step == 0) {
			TIM1->CCR1 = duty_offsetted;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
		}
		if (step == 1) {
			TIM1->CCR1 = duty_offsetted;
			TIM1->CCR2 = duty_offsetted;
			TIM1->CCR3 = 0;
		}
		if (step == 2) {
			TIM1->CCR1 = 0;
			TIM1->CCR2 = duty_offsetted;
			TIM1->CCR3 = 0;
		}
		if (step == 3) {
			TIM1->CCR1 = 0;
			TIM1->CCR2 = duty_offsetted;
			TIM1->CCR3 = duty_offsetted;
		}
		if (step == 4) {
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = duty_offsetted;
		}
		if (step == 5) {
			TIM1->CCR1 = duty_offsetted;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = duty_offsetted;
		}  
    }



    if (p.uart_received_flag) {
        p.uart_received_flag = 0; //clear flag

        p.uart_cmd[0] = p.uart_RX[0] & CMD_MASK;
        p.uart_cmd[1] = (p.uart_RX[1] << 7) | (p.uart_RX[2]);
        p.uart_cmd[1] = pad14(p.uart_cmd[1]);

        if (p.uart_cmd[0] == CMD_SET_VOLTAGE) {
            reverse = (p.uart_cmd[1] >> 13) & 1;
            mag = reverse ? (~p.uart_cmd[1]) + 1 : p.uart_cmd[1]; // If negative, take the absolute value assuming two's complement
            duty = mag;
        } else if (p.uart_cmd[0] == CMD_SET_CURRENT) {
            I_max = p.uart_cmd[1]; //0-2048 range
        } else if (p.uart_cmd[0] == CMD_SET_POSITION) {
            cont_angle_des = p.uart_cmd[1] << 13;
        } else if (p.uart_cmd[0] == CMD_SET_SPEED) {
            // implement later
        } else if (p.uart_cmd[0] == CMD_GET_POSITION) {
            encoder_res = p.uart_cmd[1];
        }

        p.uart_TX[0] = (uint8_t)(cont_angle >> 21) & 0b01111111;
        p.uart_TX[1] = (uint8_t)(cont_angle >> 14) & 0b01111111;
        p.uart_TX[2] = (uint8_t)(cont_angle >> 07) & 0b01111111;
        p.uart_TX[3] = (uint8_t)(cont_angle >> 00) & 0b01111111;

        p.uart_TX[4] = (uint8_t)(rpm >> (1+7)) & 0b01111111;
        p.uart_TX[5] = (uint8_t)(rpm >> (1+0)) & 0b01111111;

        uint8_t temp_pcb = 0;
        p.uart_TX[6] = (uint8_t)(temp_pcb >> 0) & 0b01111111;
        p.uart_TX[7] = (uint8_t)(vbus >> 7) & 0b01111111;
        p.uart_TX[8] = (uint8_t)(vbus >> 0) & 0b01111111;

        RS485_SET_TX;
        HAL_UART_Transmit_DMA(&huart1, p.uart_TX, 9);
    }

    
    LED_GREEN;
}


void sixstep_slowloop() {

    rpm = ((cont_angle - cont_angle_prev) * 1000 * 60) >> 15; // should be accurate within reasonable RPM range if 32-bit
    cont_angle_prev = cont_angle;

    vbus = vbus_accum >> VBUS_FILT_LVL;
    vbus_accum = vbus_accum - vbus + p.adc_vals[3];

}


