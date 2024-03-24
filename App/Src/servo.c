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
static volatile bool servoRunning = false;
//! Target velocity
static float servoTargetVelocity = 0;
//! Max velocity
static float servoMaxVelocity = 0;
//! Velocity PID
static PID servoVelocityPid = {0};
//! Acceleration
static float servoAcceleration = 0;
//! Target angular velocity
static float servoTargetAngularVelocity = 0;
//! Max angular velocity
static float servoMaxAngularVelocity = 0;
//! Angular velocity PID
static PID servoAngularVelocityPid = {0};
//! Angular acceleration
static float servoAngularAcceleration = 0;
/**
 * @brief Start servos
 */
void SERVO_Start(const float *veloPidGain, const float *angPidGain)
{
  servoVelocityPid.kp = veloPidGain[0];
  servoVelocityPid.ki = veloPidGain[1];
  servoVelocityPid.kd = veloPidGain[2];
  servoAngularVelocityPid.kp = angPidGain[0];
  servoAngularVelocityPid.ki = angPidGain[1];
  servoAngularVelocityPid.kd = angPidGain[2];

  SERVO_Reset();
  servoRunning = true;
  MOTOR_Start();
}
/**
 * @brief Reset servos
 */
void SERVO_Reset()
{
  PID_Reset(&servoVelocityPid);
  PID_Reset(&servoAngularVelocityPid);
}
/**
 * @brief Stop servos
 */
void SERVO_Stop()
{
  servoRunning = false;
  MOTOR_Stop();
  SERVO_Reset();

  servoTargetVelocity = 0.0f;
  servoAcceleration = 0.0f;
  servoTargetAngularVelocity = 0.0f;
  servoAngularAcceleration = 0.0f;
}
/**
 * @brief Update servos (1kHz)
 */
void SERVO_UpdateInterval()
{
  if (!servoRunning)
    return;

  const ODOMETRY *odom = ODOMETRY_GetCurrent();
  float batVol = (float)ADC_GetBatteryVoltage() / 1000.0f;

  // Generate target velocity
  servoTargetVelocity += servoAcceleration / 1000.0f;
  if (servoTargetVelocity < -1.0f * servoMaxVelocity)
    servoTargetVelocity = -1.0f * servoMaxVelocity;
  else if (servoMaxVelocity < servoTargetVelocity)
    servoTargetVelocity = servoMaxVelocity;

  servoTargetAngularVelocity += servoAngularAcceleration / 1000.0f;
  if (servoTargetAngularVelocity < -1.0f * servoMaxAngularVelocity)
    servoTargetAngularVelocity = -1.0f * servoMaxAngularVelocity;
  else if (servoMaxAngularVelocity < servoTargetAngularVelocity)
    servoTargetAngularVelocity = servoMaxAngularVelocity;

  // Feedback
  float veloError = PID_Update(&servoVelocityPid, servoTargetVelocity, odom->velocity, 1.0f);
  float angVeloError = PID_Update(&servoAngularVelocityPid, servoTargetAngularVelocity, odom->angularVelocity, 1.0f);

  float volRight = veloError + angVeloError;
  float volLeft = veloError - angVeloError;

  if (volRight < -SERVO_LIMIT_VOTAGE)
    volRight = -SERVO_LIMIT_VOTAGE;
  else if (SERVO_LIMIT_VOTAGE < volRight)
    volRight = SERVO_LIMIT_VOTAGE;
  if (volLeft < -SERVO_LIMIT_VOTAGE)
    volLeft = -SERVO_LIMIT_VOTAGE;
  else if (SERVO_LIMIT_VOTAGE < volLeft)
    volLeft = SERVO_LIMIT_VOTAGE;

  float dutyRight = volRight / batVol;
  float dutyLeft = volLeft / batVol;

  MOTOR_SetDutyRight(dutyRight);
  MOTOR_SetDutyLeft(dutyLeft);
}
/**
 *  @brief Set target velocity
 */
void SERVO_SetTargetVelocity(float velo)
{
  servoTargetVelocity = velo;
}
/**
 *  @brief Set acceleration
 */
void SERVO_SetAcceleration(float accel)
{
  servoAcceleration = accel;
}
/**
 *  @brief Set target angular velocity
 */
void SERVO_SetTargetAngularVelocity(float angVelo)
{
  servoTargetAngularVelocity = angVelo;
}
/**
 * @brief Set angular acceleration
 */
void SERVO_SetAngularAcceleration(float angAccel)
{
  servoAngularAcceleration = angAccel;
}
/**
 *  @brief Get target velocity
 */
const float *SERVO_GetTargetVelocity()
{
  return &servoTargetVelocity;
}
/**
 *  @brief Set target velocity
 */
void SERVO_SetMaxVelocity(float velo)
{
  servoMaxVelocity = velo;
}
/**
 *  @brief Get target angular velocity
 */
const float *SERVO_GetTargetAngularVelocity()
{
  return &servoTargetAngularVelocity;
}
/**
 *  @brief Set target angular velocity
 */
void SERVO_SetMaxAngularVelocity(float angVelo)
{
  servoMaxAngularVelocity = angVelo;
}
