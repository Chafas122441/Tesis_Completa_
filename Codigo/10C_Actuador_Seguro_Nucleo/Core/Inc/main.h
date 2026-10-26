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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define DOF2_Sensor_MOSI_Pin GPIO_PIN_1
#define DOF2_Sensor_MOSI_GPIO_Port GPIOC
#define Motor_DOF3_Pin GPIO_PIN_0
#define Motor_DOF3_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define DOF1_Sensor_SCK_Pin GPIO_PIN_5
#define DOF1_Sensor_SCK_GPIO_Port GPIOA
#define DOF1_Sensor_MOSI_Pin GPIO_PIN_7
#define DOF1_Sensor_MOSI_GPIO_Port GPIOA
#define DOF3_Sensor_MOSI_Pin GPIO_PIN_0
#define DOF3_Sensor_MOSI_GPIO_Port GPIOB
#define Motor_DOF2_Pin GPIO_PIN_10
#define Motor_DOF2_GPIO_Port GPIOB
#define DOF2_Sensor_SCK_Pin GPIO_PIN_13
#define DOF2_Sensor_SCK_GPIO_Port GPIOB
#define Portenta_Comm_TX_Pin GPIO_PIN_6
#define Portenta_Comm_TX_GPIO_Port GPIOC
#define Portenta_Comm_RX_Pin GPIO_PIN_7
#define Portenta_Comm_RX_GPIO_Port GPIOC
#define Motor_DOF1_Pin GPIO_PIN_9
#define Motor_DOF1_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define DOF3_Sensor_SCK_Pin GPIO_PIN_10
#define DOF3_Sensor_SCK_GPIO_Port GPIOC
#define Motor_DOF4_Pin GPIO_PIN_12
#define Motor_DOF4_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define CS_JointAxle_DOF3_Pin GPIO_PIN_4
#define CS_JointAxle_DOF3_GPIO_Port GPIOB
#define CS_MotorAxle_DOF3_Pin GPIO_PIN_5
#define CS_MotorAxle_DOF3_GPIO_Port GPIOB
#define CS_JointAxle_DOF2_Pin GPIO_PIN_6
#define CS_JointAxle_DOF2_GPIO_Port GPIOB
#define CS_MotorAxle_DOF2_Pin GPIO_PIN_7
#define CS_MotorAxle_DOF2_GPIO_Port GPIOB
#define CS_JointAxle_DOF1_Pin GPIO_PIN_8
#define CS_JointAxle_DOF1_GPIO_Port GPIOB
#define CS_MotorAxle_DOF1_Pin GPIO_PIN_9
#define CS_MotorAxle_DOF1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
