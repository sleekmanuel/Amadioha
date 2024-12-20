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
#include "stm32l4xx_hal.h"

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
#define VBase_Pin GPIO_PIN_3
#define VBase_GPIO_Port GPIOA
#define Sense_CuttOff_Pin GPIO_PIN_4
#define Sense_CuttOff_GPIO_Port GPIOA
#define Switch_Pin GPIO_PIN_0
#define Switch_GPIO_Port GPIOB
#define Switch_EXTI_IRQn EXTI0_IRQn
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define V_REF 3.3 // ADC reference voltage (Vref) in volts
#define ADC_RESOLUTION 4095.0  // ADC resolution (12-bit gives values from 0 to 4095)
#define SENSITIVITY 0.185  // ACS712 sensitivity
#define Data_BUFFER_SIZE 12   // Transmission Buffer size
#define DEBOUNCE_DELAY_MS 50
#define ADDRESS_HIGH 0x13A200  // High address on Xbee devices
#define SAMPLE_COUNT 250 // Number of samples to take for RMS

// External declarations for UART handle and buffers
extern UART_HandleTypeDef huart1;          // UART handle
extern uint8_t rx_buffer[Data_BUFFER_SIZE]; // Buffer to store received data
extern uint8_t received_byte;              // Variable to store single received byte
extern uint8_t mySerialLow[8];             // Array to store Serial Number Low
extern uint8_t myDestLow[8];               // Array to store Destination Number Low
extern volatile uint8_t data_received_flag; // Flag to indicate data reception completion
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
