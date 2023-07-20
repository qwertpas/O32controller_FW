/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
