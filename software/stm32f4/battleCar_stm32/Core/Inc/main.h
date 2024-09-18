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
#define gray1_Pin GPIO_PIN_1
#define gray1_GPIO_Port GPIOA
#define gray2_Pin GPIO_PIN_2
#define gray2_GPIO_Port GPIOA
#define gray3_Pin GPIO_PIN_3
#define gray3_GPIO_Port GPIOA
#define gray4_Pin GPIO_PIN_4
#define gray4_GPIO_Port GPIOA
#define motor1_pwm_Pin GPIO_PIN_9
#define motor1_pwm_GPIO_Port GPIOE
#define motor2_pwm_Pin GPIO_PIN_11
#define motor2_pwm_GPIO_Port GPIOE
#define opi_tx_Pin GPIO_PIN_10
#define opi_tx_GPIO_Port GPIOB
#define opi_rx_Pin GPIO_PIN_11
#define opi_rx_GPIO_Port GPIOB
#define start_sw1_Pin GPIO_PIN_12
#define start_sw1_GPIO_Port GPIOB
#define start_sw2_Pin GPIO_PIN_13
#define start_sw2_GPIO_Port GPIOB
#define motor1_encoderA_Pin GPIO_PIN_12
#define motor1_encoderA_GPIO_Port GPIOD
#define motor2_encoderB_Pin GPIO_PIN_13
#define motor2_encoderB_GPIO_Port GPIOD
#define motor2_encoderA_Pin GPIO_PIN_6
#define motor2_encoderA_GPIO_Port GPIOC
#define motor2_encoderBC7_Pin GPIO_PIN_7
#define motor2_encoderBC7_GPIO_Port GPIOC
#define ir_sw1_Pin GPIO_PIN_9
#define ir_sw1_GPIO_Port GPIOA
#define ir_sw2_Pin GPIO_PIN_10
#define ir_sw2_GPIO_Port GPIOA
#define ir_sw3_Pin GPIO_PIN_11
#define ir_sw3_GPIO_Port GPIOA
#define ir_sw4_Pin GPIO_PIN_12
#define ir_sw4_GPIO_Port GPIOA
#define test_tx_Pin GPIO_PIN_12
#define test_tx_GPIO_Port GPIOC
#define test_rx_Pin GPIO_PIN_2
#define test_rx_GPIO_Port GPIOD
#define tof_tx_Pin GPIO_PIN_5
#define tof_tx_GPIO_Port GPIOD
#define tof_rx_Pin GPIO_PIN_6
#define tof_rx_GPIO_Port GPIOD
#define motor1_dir1_Pin GPIO_PIN_6
#define motor1_dir1_GPIO_Port GPIOB
#define motor1_dir2_Pin GPIO_PIN_7
#define motor1_dir2_GPIO_Port GPIOB
#define motor2_dir1_Pin GPIO_PIN_8
#define motor2_dir1_GPIO_Port GPIOB
#define motor2_dir2_Pin GPIO_PIN_9
#define motor2_dir2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
