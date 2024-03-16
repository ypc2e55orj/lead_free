#include "servo.h"

// libc
#include <stdbool.h>

// STM32CubeMX
#include <main.h>

/* Motor ---------------------------------------------------------------------*/
//! TIM1 Handler extern
extern TIM_HandleTypeDef htim1;
//! TIM17 Handler extern
extern TIM_HandleTypeDef htim17;
/**
 * @brief Initialize motors.
 */
void MOTOR_Init()
{
  TIM17->CCR1 = 0;
  TIM1->CCR4 = 0;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
}
/**
 * @brief Set a duty of right motor.
 */
void MOTOR_SetDutyRight(uint16_t duty, bool cw)
{
  TIM17->CCR1 = duty;
  HAL_GPIO_WritePin(MotorRightPhase_GPIO_Port, MotorRightPhase_Pin, cw ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
/**
 * @brief Set a duty of left motor
 */
void MOTOR_SetDutyLeft(uint16_t duty, bool cw)
{
  TIM1->CCR4 = duty;
  HAL_GPIO_WritePin(MotorLeftPhase_GPIO_Port, MotorLeftPhase_Pin, cw ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
