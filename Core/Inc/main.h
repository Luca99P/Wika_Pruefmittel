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
//#include "nfc08a1.h"
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
HAL_StatusTypeDef spiTxRx(const uint8_t *txData, uint8_t *rxData, uint16_t length);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define IRQ_3911_Pin GPIO_PIN_0
#define IRQ_3911_GPIO_Port GPIOA
#define IRQ_3911_EXTI_IRQn EXTI0_IRQn
#define LED_F_Pin GPIO_PIN_1
#define LED_F_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_4
#define LED_B_GPIO_Port GPIOA
#define RS485_EN_TX_Pin GPIO_PIN_5
#define RS485_EN_TX_GPIO_Port GPIOC
#define LED_A_Pin GPIO_PIN_0
#define LED_A_GPIO_Port GPIOB
#define START_RFID_Pin GPIO_PIN_13
#define START_RFID_GPIO_Port GPIOB
#define CHECK_RFID_Pin GPIO_PIN_14
#define CHECK_RFID_GPIO_Port GPIOB
#define LED_FIELD_Pin GPIO_PIN_8
#define LED_FIELD_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define START_RS485_Pin GPIO_PIN_10
#define START_RS485_GPIO_Port GPIOC
#define CHECK_RS485_Pin GPIO_PIN_11
#define CHECK_RS485_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define SPI1_CS_Pin GPIO_PIN_6
#define SPI1_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
