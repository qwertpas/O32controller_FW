/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Function to get number of elements */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Size of Transmission and receive buffer */
#define I2CSIZE 2
/* 6 ADC channels in total:
 * ADC0: Phase V current
 * ADC3: Phase W current
 * ADC4: VBUS sense (5.12x voltage divider)
 * ADC9: Phase U current
 * then TEMP and VREF */
#define NBR_ADC 6

#define PPAIRS 7 //pole pairs

//calibration values for temperature sensor and ADC internal refernce. See datasheet section 3.10.2
#define TS_CAL1 *((uint16_t*)0x1FFFF7B8)
#define VREFINT_CAL *((uint16_t*)0x1FFFF7BA)

#define LED_red HAL_GPIO_WritePin(GPIOF, LED_STATUS_Pin, 1)
#define LED_green HAL_GPIO_WritePin(GPIOF, LED_STATUS_Pin, 0)

#define B_PAT "%c%c%c%c%c%c%c%c"
#define TO_B(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

uint8_t i2c_TX[I2CSIZE];
uint8_t i2c_RX[I2CSIZE];

uint8_t uart_TX[100];
uint16_t uart_TX_pos = 0;
uint8_t uart_RX[100];

/* Buffer for raw ADC readings */
uint16_t adc_vals[NBR_ADC];

uint8_t spi_TX[2];
uint8_t spi_RX[2];

uint8_t do_print = 0;
uint8_t i2c_complete = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

//overrides printf
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

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
	MX_I2C1_Init();
	MX_ADC_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

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
	uint32_t m_angle = (uint32_t) ((spi_RX[0] << 8) + spi_RX[1] + 16384); // 0 to 32,767
	m_angle = m_angle * 36000 / 32768; //0 - 36000 (hundreths of degrees)
	uint32_t e_offset = (m_angle / 100 * PPAIRS) % 360;
	int32_t e_angle = 0;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	int step = 0;
	int mag = 20;

	uint32_t m_angle_prev = m_angle;
	int32_t revs = 0;
	int32_t cont_angle = 0;
	int32_t cont_angle_prev = 0;
	int32_t rpm = 0;

	while (1) {

		//restart I2C listener after a transfer
//		if (Xfer_Complete ==1){
//			/* Put I2C peripheral in listen mode process */
//			status = HAL_I2C_EnableListen_IT(&hi2c1);
//			Xfer_Complete =0;
//		}
//		printf("i2c: %X \r\n", i2c_RX[0]);

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
		int16_t temp = (1000 * (1908 - adc_vals[4])) / 5337;

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

			sprintf((char*)uart_TX, " adc1: %d \n adc2: %d \n adc3: %d \n\t", adc_vals[1], adc_vals[2], adc_vals[3]);
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
				status = HAL_I2C_EnableListen_IT(&hi2c1);
				i2c_complete = 0;
			}
//			LED_green;
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x2000090E;
	hi2c1.Init.OwnAddress1 = 18;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 2;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 512;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
	sSlaveConfig.InputTrigger = TIM_TS_ETRF;
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_INVERTED;
	sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 10;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 47;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 10000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, LED_STATUS_Pin | MAG_NCS_Pin | OC_TH_STBY2_Pin | OC_TH_STBY1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USART_DE_GPIO_Port, USART_DE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LED_STATUS_Pin MAG_NCS_Pin OC_TH_STBY2_Pin OC_TH_STBY1_Pin */
	GPIO_InitStruct.Pin = LED_STATUS_Pin | MAG_NCS_Pin | OC_TH_STBY2_Pin | OC_TH_STBY1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pin : USART_DE_Pin */
	GPIO_InitStruct.Pin = USART_DE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USART_DE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OC_SEL_Pin */
	GPIO_InitStruct.Pin = OC_SEL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OC_SEL_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//Callback whenever a timer rolls over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) { //100Hz
		do_print = 1;
	}
}

/**
 * @brief  Tx Transfer completed callback.
 * @param  I2cHandle: I2C handle.
 * @note   This example shows a simple way to report end of IT Tx transfer, and
 *         you can add your own implementation.
 * @retval None
 */

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
	i2c_complete = 1;
}

/**
 * @brief  Rx Transfer completed callback.
 * @param  I2cHandle: I2C handle
 * @note   This example shows a simple way to report end of IT Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
}

/**
 * @brief  Slave Address Match callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  TransferDirection: Master request Transfer Direction (Write/Read), value of @ref I2C_XferOptions_definition
 * @param  AddrMatchCode: Address Match Code
 * @retval None
 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
//	LED_red;
	HAL_StatusTypeDef status;

	if (TransferDirection != 0) {
		status = HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, (uint8_t*) i2c_TX, I2CSIZE, I2C_FIRST_AND_LAST_FRAME);
	} else {
		status = HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, (uint8_t*) i2c_RX, I2CSIZE, I2C_FIRST_AND_LAST_FRAME);
		i2c_TX[0] = i2c_RX[0] + 1;
		i2c_TX[1] = i2c_RX[1] + 1;
	}
	if (status != HAL_OK) {
		Error_Handler();
	}

//	LED_green;
}

/**
 * @brief  I2C error callbacks.
 * @param  I2cHandle: I2C handle
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle) {
	/** Error_Handler() function is called when error occurs.
	 * 1- When Slave doesn't acknowledge its address, Master restarts communication.
	 * 2- When Master doesn't acknowledge the last data transferred, Slave doesn't care in this example.
	 */
	if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF) {
		Error_Handler();
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	LED_red;
	printf("ERROR HANDLER \n");
	__disable_irq();		//disable interrupts
	NVIC_SystemReset(); //reset microcontroller, clearing any I2C faults. Maybe change to only
						//reinit the I2C to save power.
	while (1) {
	}			//if reset takes a while, do nothing

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
