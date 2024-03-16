#include "servo.h"

// libc
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

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
 * @brief Start motors.
 */
static void MOTOR_Start()
{
  TIM17->CCR1 = 0;
  TIM1->CCR4 = 0;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
}
/**
 * @brief Stop motors
 */
static void MOTOR_Stop()
{
  TIM17->CCR1 = 0;
  TIM1->CCR4 = 0;
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
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
#define SERVO_LIMIT_VOTAGE 4.5f
//! Servo enable flag
static volatile bool servo_running = false;
//! Target velocity
static float servo_target_velocity;
//! Target angular velocity
static float servo_target_angular_velocity;
/**
 * @brief Start servos
 */
void SERVO_Start()
{
  servo_running = true;
  MOTOR_Start();
}
/**
 * @brief Stop servos
 */
void SERVO_Stop()
{
  PARAMETER *param = PARAMETER_Get();

  servo_running = false;
  MOTOR_Stop();

  PID_Reset(&param->velocity_pid);
  PID_Reset(&param->angular_velocity_pid);
}
/**
 * @brief Update servos (1kHz)
 */
void SERVO_Update()
{
  if (!servo_running)
    return;

  PARAMETER *param = PARAMETER_Get();
  const ODOMETRY *odom = ODOMETRY_GetCurrent();
  float bat_vol = (float)ADC_GetBatteryVoltage() / 1000.0f;

  // Generate target velocity

  // Feedback
  float velo_err = PID_Update(&param->velocity_pid, servo_target_velocity, odom->velocity, 1.0f);
  float ang_velo_err = PID_Update(&param->angular_velocity_pid, servo_target_angular_velocity, odom->angular_velocity, 1.0f);

  float vol_r = velo_err + ang_velo_err;
  float vol_l = velo_err - ang_velo_err;

  if (vol_r < -SERVO_LIMIT_VOTAGE)
    vol_r = -SERVO_LIMIT_VOTAGE;
  else if (SERVO_LIMIT_VOTAGE < vol_r)
    vol_r = SERVO_LIMIT_VOTAGE;
  if (vol_l < -SERVO_LIMIT_VOTAGE)
    vol_l = -SERVO_LIMIT_VOTAGE;
  else if (SERVO_LIMIT_VOTAGE < vol_l)
    vol_l = SERVO_LIMIT_VOTAGE;

  float duty_r = vol_r / bat_vol;
  float duty_l = vol_l / bat_vol;

  MOTOR_SetDutyRight(duty_r);
  MOTOR_SetDutyLeft(duty_l);
}
