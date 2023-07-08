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
#include "stm32f4xx_hal.h"

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
#define INT_STATE_Pin GPIO_PIN_13
#define INT_STATE_GPIO_Port GPIOC
#define RECORD_STATE_Pin GPIO_PIN_14
#define RECORD_STATE_GPIO_Port GPIOC
#define LED_MIC_STATE_Pin GPIO_PIN_15
#define LED_MIC_STATE_GPIO_Port GPIOC
#define CAR_DIR_1_Pin GPIO_PIN_3
#define CAR_DIR_1_GPIO_Port GPIOA
#define CAR_DIR_2_Pin GPIO_PIN_4
#define CAR_DIR_2_GPIO_Port GPIOA
#define CAR_DIR_3_Pin GPIO_PIN_5
#define CAR_DIR_3_GPIO_Port GPIOA
#define CAR_DIR_4_Pin GPIO_PIN_6
#define CAR_DIR_4_GPIO_Port GPIOA
#define I2S2_CLOCK_Pin GPIO_PIN_10
#define I2S2_CLOCK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
