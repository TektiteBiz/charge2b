/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define FLOAT_EQUALS(A, B) (fabsf((A) - (B)) < 0.00001f)
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BATT1H_Pin GPIO_PIN_0
#define BATT1H_GPIO_Port GPIOA
#define BATT2H_Pin GPIO_PIN_1
#define BATT2H_GPIO_Port GPIOA
#define BATT1L_Pin GPIO_PIN_2
#define BATT1L_GPIO_Port GPIOA
#define BATT2L_Pin GPIO_PIN_3
#define BATT2L_GPIO_Port GPIOA
#define CTRL1_Pin GPIO_PIN_4
#define CTRL1_GPIO_Port GPIOA
#define CTRL2_Pin GPIO_PIN_5
#define CTRL2_GPIO_Port GPIOA
#define VBUS_Pin GPIO_PIN_6
#define VBUS_GPIO_Port GPIOA
#define LED2R_Pin GPIO_PIN_6
#define LED2R_GPIO_Port GPIOC
#define LED2G_Pin GPIO_PIN_7
#define LED2G_GPIO_Port GPIOC
#define LED2B_Pin GPIO_PIN_8
#define LED2B_GPIO_Port GPIOC
#define LED1R_Pin GPIO_PIN_8
#define LED1R_GPIO_Port GPIOA
#define LED1G_Pin GPIO_PIN_9
#define LED1G_GPIO_Port GPIOA
#define LED1B_Pin GPIO_PIN_10
#define LED1B_GPIO_Port GPIOA
#define ALERT_Pin GPIO_PIN_2
#define ALERT_GPIO_Port GPIOD
#define MODE2_Pin GPIO_PIN_3
#define MODE2_GPIO_Port GPIOB
#define MODE1_Pin GPIO_PIN_4
#define MODE1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
