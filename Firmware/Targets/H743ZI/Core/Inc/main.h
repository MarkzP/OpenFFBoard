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
#include "stm32h7xx_hal.h"

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
void RebootDFU();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_A_Pin GPIO_PIN_13
#define BUTTON_A_GPIO_Port GPIOC
#define DIN0_Pin GPIO_PIN_0
#define DIN0_GPIO_Port GPIOF
#define DIN1_Pin GPIO_PIN_1
#define DIN1_GPIO_Port GPIOF
#define DIN2_Pin GPIO_PIN_2
#define DIN2_GPIO_Port GPIOF
#define DIN3_Pin GPIO_PIN_3
#define DIN3_GPIO_Port GPIOF
#define DIN4_Pin GPIO_PIN_4
#define DIN4_GPIO_Port GPIOF
#define DIN5_Pin GPIO_PIN_5
#define DIN5_GPIO_Port GPIOF
#define DIN6_Pin GPIO_PIN_6
#define DIN6_GPIO_Port GPIOF
#define DIN7_Pin GPIO_PIN_7
#define DIN7_GPIO_Port GPIOF
#define LED_SYS_Pin GPIO_PIN_0
#define LED_SYS_GPIO_Port GPIOB
#define LED_ERR_Pin GPIO_PIN_14
#define LED_ERR_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define ENCODER_Z_Pin GPIO_PIN_6
#define ENCODER_Z_GPIO_Port GPIOC
#define DRV_BRAKE_Pin GPIO_PIN_8
#define DRV_BRAKE_GPIO_Port GPIOC
#define DRV_ENABLE_Pin GPIO_PIN_9
#define DRV_ENABLE_GPIO_Port GPIOC
#define FLAG_Pin GPIO_PIN_2
#define FLAG_GPIO_Port GPIOD
#define FLAG_EXTI_IRQn EXTI2_IRQn
#define SPI1_SS1_Pin GPIO_PIN_6
#define SPI1_SS1_GPIO_Port GPIOB
#define SPI1_SS2_Pin GPIO_PIN_7
#define SPI1_SS2_GPIO_Port GPIOB
#define LED_CLIP_Pin GPIO_PIN_1
#define LED_CLIP_GPIO_Port GPIOE
void   MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
