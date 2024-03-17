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
static float servo_target_velocity = 0;
//! Velocity PID
static PID servo_velocity_pid = {0};
//! Acceleration
static float servo_acceleration = 0;
//! Target angular velocity
static float servo_target_angular_velocity = 0;
//! Angular velocity PID
static PID servo_angular_velocity_pid = {0};
//! Angular acceleration
static float servo_angular_acceleration = 0;
/**
 * @brief Start servos
 */
void SERVO_Start()
{
  const PARAMETER *param = PARAMETER_Get();

  servo_velocity_pid.kp = param->velocity_pid[0];
  servo_velocity_pid.ki = param->velocity_pid[1];
  servo_velocity_pid.kd = param->velocity_pid[2];
  servo_angular_velocity_pid.kp = param->angular_velocity_pid[0];
  servo_angular_velocity_pid.ki = param->angular_velocity_pid[1];
  servo_angular_velocity_pid.kd = param->angular_velocity_pid[2];

  SERVO_Reset();
  servo_running = true;
  MOTOR_Start();
}
/**
 * @brief Reset servos
 */
void SERVO_Reset()
{
  PID_Reset(&servo_velocity_pid);
  PID_Reset(&servo_angular_velocity_pid);
}
/**
 * @brief Stop servos
 */
void SERVO_Stop()
{
  servo_running = false;
  MOTOR_Stop();
  SERVO_Reset();

  servo_target_velocity = 0.0f;
  servo_acceleration = 0.0f;
  servo_target_angular_velocity = 0.0f;
  servo_angular_acceleration = 0.0f;
}
/**
 * @brief Update servos (1kHz)
 */
void SERVO_Update()
{
  if (!servo_running)
    return;

  const PARAMETER *param = PARAMETER_Get();
  const ODOMETRY *odom = ODOMETRY_GetCurrent();
  float bat_vol = (float)ADC_GetBatteryVoltage() / 1000.0f;

  // Generate target velocity
  servo_target_velocity += servo_acceleration / 1000.0f;
  if (servo_target_velocity < -1.0f * param->max_velocity)
    servo_target_velocity = -1.0f * param->max_velocity;
  else if (param->max_velocity < servo_target_velocity)
    servo_target_velocity = param->max_velocity;

  servo_target_angular_velocity = servo_angular_acceleration / 1000.0f;
  if (servo_target_angular_velocity < -1.0f * param->max_angular_velocity)
    servo_target_angular_velocity = -1.0f * param->max_angular_velocity;
  else if (param->max_angular_velocity < servo_target_angular_velocity)
    servo_target_angular_velocity = param->max_angular_velocity;

  // Feedback
  float velo_err = PID_Update(&servo_velocity_pid, servo_target_velocity, odom->velocity, 1.0f);
  float ang_velo_err = PID_Update(&servo_angular_velocity_pid, servo_target_angular_velocity, odom->angular_velocity, 1.0f);

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
/**
 *  @brief Set target velocity
 */
void SERVO_SetTargetVelocity(float velo)
{
  servo_target_velocity = velo;
}
/**
 *  @brief Set acceleration
 */
void SERVO_SetAcceleration(float accel)
{
  const PARAMETER *param = PARAMETER_Get();
  if (accel < -1.0f * param->acceleration)
    accel = -1.0f * param->acceleration;
  else if (param->acceleration < accel)
    accel = param->acceleration;

  servo_acceleration = accel;
}
/**
 *  @brief Set target angular velocity
 */
void SERVO_SetTargetAngularVelocity(float ang_velo)
{
  servo_target_angular_velocity = ang_velo;
}
/**
 * @brief Set angular acceleration
 */
void SERVO_SetAngularAcceleration(float ang_accel)
{
  const PARAMETER *param = PARAMETER_Get();
  if (ang_accel < -1.0f * param->angular_acceleration)
    ang_accel = -1.0f * param->angular_acceleration;
  else if (param->angular_acceleration < ang_accel)
    ang_accel = param->angular_acceleration;

  servo_angular_acceleration = ang_accel;
}
/**
 *  @brief Get target velocity
 */
const float *SERVO_GetTargetVelocity()
{
  return &servo_target_velocity;
}
/**
 *  @brief Get target angular velocity
 */
const float *SERVO_GetTargetAngularVelocity()
{
  return &servo_target_angular_velocity;
}
