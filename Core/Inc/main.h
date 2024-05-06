/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define PWMOUT_Pin GPIO_PIN_0
#define PWMOUT_GPIO_Port GPIOC
#define Relay1_Pin GPIO_PIN_1
#define Relay1_GPIO_Port GPIOC
#define DIROUT_Pin GPIO_PIN_3
#define DIROUT_GPIO_Port GPIOC
#define LogicCon1_Pin GPIO_PIN_0
#define LogicCon1_GPIO_Port GPIOA
#define LogicCon2_Pin GPIO_PIN_1
#define LogicCon2_GPIO_Port GPIOA
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define LogicCon3_Pin GPIO_PIN_4
#define LogicCon3_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define EncoderA_Pin GPIO_PIN_6
#define EncoderA_GPIO_Port GPIOA
#define EncoderB_Pin GPIO_PIN_7
#define EncoderB_GPIO_Port GPIOA
#define LogicCon4_Pin GPIO_PIN_0
#define LogicCon4_GPIO_Port GPIOB
#define Relay2_Pin GPIO_PIN_1
#define Relay2_GPIO_Port GPIOB
#define Relay3_Pin GPIO_PIN_2
#define Relay3_GPIO_Port GPIOB
#define LimitTop_Pin GPIO_PIN_7
#define LimitTop_GPIO_Port GPIOC
#define BT1_Pin GPIO_PIN_8
#define BT1_GPIO_Port GPIOC
#define BT2_Pin GPIO_PIN_9
#define BT2_GPIO_Port GPIOC
#define BT3_Pin GPIO_PIN_8
#define BT3_GPIO_Port GPIOA
#define BT4_Pin GPIO_PIN_9
#define BT4_GPIO_Port GPIOA
#define BT5_Pin GPIO_PIN_10
#define BT5_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define LimitBottom_Pin GPIO_PIN_6
#define LimitBottom_GPIO_Port GPIOB
#define Relay4_Pin GPIO_PIN_9
#define Relay4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
