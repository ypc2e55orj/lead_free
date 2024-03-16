#include "servo.h"

// libc
#include <stdio.h>
#include <math.h>

// STM32CubeMX
#include <main.h>

// project
#include "sensor.h"
#include "odometry.h"
#include "pid.h"
#include "parameter.h"

/* Motor ---------------------------------------------------------------------*/
#define MOTOR_MAX_DUTY 3199
//! TIM1 Handler extern
extern TIM_HandleTypeDef htim1;
//! TIM17 Handler extern
extern TIM_HandleTypeDef htim17;
/**
 * @brief Initialize motors.
 */
static void MOTOR_Init()
{
  TIM17->CCR1 = 0;
  TIM1->CCR4 = 0;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
}
/**
 * @brief Set a duty of right motor.
 */
static void MOTOR_SetDutyRight(float duty)
{
  TIM17->CCR1 = (uint16_t)(fabsf(duty) * MOTOR_MAX_DUTY);
  HAL_GPIO_WritePin(MotorRightPhase_GPIO_Port, MotorRightPhase_Pin, duty > 0.0f ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
/**
 * @brief Set a duty of left motor
 */
static void MOTOR_SetDutyLeft(float duty)
{
  TIM1->CCR4 = (uint16_t)(fabsf(duty) * MOTOR_MAX_DUTY);
  HAL_GPIO_WritePin(MotorLeftPhase_GPIO_Port, MotorLeftPhase_Pin, duty > 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Servo ---------------------------------------------------------------------*/
static float servo_target_velocity;
static float servo_target_angular_velocity;
/**
 * @brief Initialize servos
 */
void SERVO_Init()
{
  MOTOR_Init();
}
/**
 * @brief Set target velocity
 */
void SERVO_SetTarget(float velo, float ang_velo)
{
  servo_target_velocity = velo;
  servo_target_angular_velocity = ang_velo;
}
/**
 * @brief Update servos
 */
void SERVO_Update()
{
  PARAMETER *param = PARAMETER_Get();
  const ODOMETRY *odom = ODOMETRY_GetCurrent();
  float bat_vol = (float)ADC_GetBatteryVoltage() / 1000.0f;

  float velo_err = PID_Update(&param->velocity_pid, servo_target_velocity, odom->velocity, 1.0f);
  float ang_velo_err = PID_Update(&param->angular_velocity_pid, servo_target_angular_velocity, odom->angular_velocity, 1.0f);

  float vol_r = velo_err + ang_velo_err;
  float vol_l = velo_err - ang_velo_err;

  float duty_r = vol_r / bat_vol;
  float duty_l = vol_l / bat_vol;

  printf("%04d, %04d\r\n", (int)(duty_r), (int)(duty_l));

  MOTOR_SetDutyRight(duty_r);
  MOTOR_SetDutyLeft(duty_l);
}
