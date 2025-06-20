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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USR_BTN_Pin GPIO_PIN_13
#define USR_BTN_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_14
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_15
#define LED_2_GPIO_Port GPIOC
#define Status_LED_Pin GPIO_PIN_0
#define Status_LED_GPIO_Port GPIOF
#define Vbus_DC2_Pin GPIO_PIN_0
#define Vbus_DC2_GPIO_Port GPIOC
#define Vbus_DC1_Pin GPIO_PIN_1
#define Vbus_DC1_GPIO_Port GPIOC
#define I_CT_adc_Pin GPIO_PIN_2
#define I_CT_adc_GPIO_Port GPIOC
#define DAC1_Pin GPIO_PIN_4
#define DAC1_GPIO_Port GPIOA
#define DAC2_Pin GPIO_PIN_5
#define DAC2_GPIO_Port GPIOA
#define DAC3_Pin GPIO_PIN_6
#define DAC3_GPIO_Port GPIOA
#define IDC2_LEM_Pin GPIO_PIN_7
#define IDC2_LEM_GPIO_Port GPIOA
#define IDC1_LEM_Pin GPIO_PIN_4
#define IDC1_LEM_GPIO_Port GPIOC
#define Temp_Pin GPIO_PIN_2
#define Temp_GPIO_Port GPIOB
#define PWM_SX_LS2_Pin GPIO_PIN_12
#define PWM_SX_LS2_GPIO_Port GPIOB
#define PWM_SX_HS2_Pin GPIO_PIN_13
#define PWM_SX_HS2_GPIO_Port GPIOB
#define PWM_DX_LS2_Pin GPIO_PIN_14
#define PWM_DX_LS2_GPIO_Port GPIOB
#define PWM_DX_HS2_Pin GPIO_PIN_15
#define PWM_DX_HS2_GPIO_Port GPIOB
#define PWM_SX_HS1_Pin GPIO_PIN_8
#define PWM_SX_HS1_GPIO_Port GPIOA
#define PWM_SX_LS1_Pin GPIO_PIN_9
#define PWM_SX_LS1_GPIO_Port GPIOA
#define PWM_DX_HS1_Pin GPIO_PIN_10
#define PWM_DX_HS1_GPIO_Port GPIOA
#define PWM_DX_LS1_Pin GPIO_PIN_11
#define PWM_DX_LS1_GPIO_Port GPIOA
#define GPIO_2_Pin GPIO_PIN_10
#define GPIO_2_GPIO_Port GPIOC
#define GPIO_1_Pin GPIO_PIN_11
#define GPIO_1_GPIO_Port GPIOC
#define FAN_Pin GPIO_PIN_12
#define FAN_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
