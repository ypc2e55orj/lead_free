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
#include "stm32f3xx_hal.h"

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
#define Switch1_Pin GPIO_PIN_0
#define Switch1_GPIO_Port GPIOF
#define Switch2_Pin GPIO_PIN_1
#define Switch2_GPIO_Port GPIOF
#define EncoderRightB_Pin GPIO_PIN_0
#define EncoderRightB_GPIO_Port GPIOA
#define EncoderRightA_Pin GPIO_PIN_1
#define EncoderRightA_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define LineSensorRightOut_Pin GPIO_PIN_3
#define LineSensorRightOut_GPIO_Port GPIOA
#define LineSensorRightIn_Pin GPIO_PIN_4
#define LineSensorRightIn_GPIO_Port GPIOA
#define LineSensorLeftIn_Pin GPIO_PIN_5
#define LineSensorLeftIn_GPIO_Port GPIOA
#define LineSensorLeftOut_Pin GPIO_PIN_6
#define LineSensorLeftOut_GPIO_Port GPIOA
#define BatteryVoltage_Pin GPIO_PIN_7
#define BatteryVoltage_GPIO_Port GPIOA
#define MarkerSensorRight_Pin GPIO_PIN_0
#define MarkerSensorRight_GPIO_Port GPIOB
#define MarkerSensorLeft_Pin GPIO_PIN_1
#define MarkerSensorLeft_GPIO_Port GPIOB
#define EncoderLeftA_Pin GPIO_PIN_8
#define EncoderLeftA_GPIO_Port GPIOA
#define EncoderLeftA_EXTI_IRQn EXTI9_5_IRQn
#define EncoderLeftB_Pin GPIO_PIN_9
#define EncoderLeftB_GPIO_Port GPIOA
#define EncoderLeftB_EXTI_IRQn EXTI9_5_IRQn
#define MotorLeftPhase_Pin GPIO_PIN_10
#define MotorLeftPhase_GPIO_Port GPIOA
#define MotorLeftEnable_Pin GPIO_PIN_11
#define MotorLeftEnable_GPIO_Port GPIOA
#define Buzzer_Pin GPIO_PIN_12
#define Buzzer_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define Led_Pin GPIO_PIN_3
#define Led_GPIO_Port GPIOB
#define MotorRightPhase_Pin GPIO_PIN_4
#define MotorRightPhase_GPIO_Port GPIOB
#define MotorRightEnable_Pin GPIO_PIN_5
#define MotorRightEnable_GPIO_Port GPIOB
#define Bmx055Scl_Pin GPIO_PIN_6
#define Bmx055Scl_GPIO_Port GPIOB
#define Bmx055Sda_Pin GPIO_PIN_7
#define Bmx055Sda_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
