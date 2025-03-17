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
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

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
#define VDDA_APPLI 3300
#define BATT_LOW1_Pin GPIO_PIN_0
#define BATT_LOW1_GPIO_Port GPIOA
#define BATT_LOW2_Pin GPIO_PIN_1
#define BATT_LOW2_GPIO_Port GPIOA
#define BATT_HIGH1_Pin GPIO_PIN_2
#define BATT_HIGH1_GPIO_Port GPIOA
#define BATT_HIGH2_Pin GPIO_PIN_3
#define BATT_HIGH2_GPIO_Port GPIOA
#define CTRL1_Pin GPIO_PIN_4
#define CTRL1_GPIO_Port GPIOA
#define CTRL2_Pin GPIO_PIN_5
#define CTRL2_GPIO_Port GPIOA
#define VBUS_Pin GPIO_PIN_6
#define VBUS_GPIO_Port GPIOA
#define LEDG1_Pin GPIO_PIN_7
#define LEDG1_GPIO_Port GPIOA
#define LEDR1_Pin GPIO_PIN_0
#define LEDR1_GPIO_Port GPIOB
#define LEDB1_Pin GPIO_PIN_1
#define LEDB1_GPIO_Port GPIOB
#define LEDB2_Pin GPIO_PIN_14
#define LEDB2_GPIO_Port GPIOB
#define LEDR2_Pin GPIO_PIN_15
#define LEDR2_GPIO_Port GPIOB
#define LEDG2_Pin GPIO_PIN_6
#define LEDG2_GPIO_Port GPIOC
#define MODE2_Pin GPIO_PIN_11
#define MODE2_GPIO_Port GPIOC
#define MODE1_Pin GPIO_PIN_3
#define MODE1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
