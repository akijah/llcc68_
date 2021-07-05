/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define WKUP_Pin GPIO_PIN_0
#define WKUP_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define NRST_Pin GPIO_PIN_0
#define NRST_GPIO_Port GPIOB
#define BUSY_Pin GPIO_PIN_1
#define BUSY_GPIO_Port GPIOB
#define DIO1_Pin GPIO_PIN_10
#define DIO1_GPIO_Port GPIOB
#define MODE_MS_Pin GPIO_PIN_12
#define MODE_MS_GPIO_Port GPIOB
#define TEST2_Pin GPIO_PIN_14
#define TEST2_GPIO_Port GPIOB
#define TEST1_Pin GPIO_PIN_15
#define TEST1_GPIO_Port GPIOB
#define RXEN_Pin GPIO_PIN_3
#define RXEN_GPIO_Port GPIOB
#define TXEN_Pin GPIO_PIN_4
#define TXEN_GPIO_Port GPIOB
#define DIO2_Pin GPIO_PIN_5
#define DIO2_GPIO_Port GPIOB
#define EXT6_BTN_Pin GPIO_PIN_6
#define EXT6_BTN_GPIO_Port GPIOB
#define EXT6_BTN_EXTI_IRQn EXTI9_5_IRQn
#define EXT7_DIO1_Pin GPIO_PIN_7
#define EXT7_DIO1_GPIO_Port GPIOB
#define EXT7_DIO1_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
