/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LOAD_RES_ADC_Pin GPIO_PIN_0
#define LOAD_RES_ADC_GPIO_Port GPIOC
#define LOAD_CAP_FREQ_IN_Pin GPIO_PIN_1
#define LOAD_CAP_FREQ_IN_GPIO_Port GPIOA
#define GP22_SSN_Pin GPIO_PIN_4
#define GP22_SSN_GPIO_Port GPIOA
#define LENGTH_PULSE_TRIG_Pin GPIO_PIN_9
#define LENGTH_PULSE_TRIG_GPIO_Port GPIOE
#define PATH_LENGTH_EN_Pin GPIO_PIN_2
#define PATH_LENGTH_EN_GPIO_Port GPIOG
#define PATH_RESISTANCE_EN_Pin GPIO_PIN_3
#define PATH_RESISTANCE_EN_GPIO_Port GPIOG
#define PATH_CAPACITANCE_EN_Pin GPIO_PIN_4
#define PATH_CAPACITANCE_EN_GPIO_Port GPIOG
#define GP22_INTN_Pin GPIO_PIN_0
#define GP22_INTN_GPIO_Port GPIOE
#define GP22_INTN_EXTI_IRQn EXTI0_IRQn
#define GP22_RSTN_Pin GPIO_PIN_1
#define GP22_RSTN_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/*
 * .ioc已设置下列User Label；当前生成文件尚未重新生成，因此在USER CODE区
 * 保留受保护别名。下次由CubeMX生成同名宏后，这些定义会自动跳过。
 */
#ifndef GP22_SSN_Pin
#define GP22_SSN_Pin GPIO_PIN_4
#define GP22_SSN_GPIO_Port GPIOA
#endif

#ifndef GP22_INTN_Pin
#define GP22_INTN_Pin GPIO_PIN_0
#define GP22_INTN_GPIO_Port GPIOE
#endif

#ifndef GP22_RSTN_Pin
#define GP22_RSTN_Pin GPIO_PIN_1
#define GP22_RSTN_GPIO_Port GPIOE
#endif

#ifndef LENGTH_PULSE_TRIG_Pin
#define LENGTH_PULSE_TRIG_Pin GPIO_PIN_9
#define LENGTH_PULSE_TRIG_GPIO_Port GPIOE
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
