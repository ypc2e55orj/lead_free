#include "app_main.h"

// libc
#include <stdio.h>
#include <math.h>

// STM32CubeMX
#include <main.h>

// project
#include "interval.h"
#include "sensor.h"
#include "odometry.h"
#include "servo.h"
#include "parameter.h"
#include "button.h"
#include "logger.h"

void RUN_Straight(float length, float accel, float min_velo, float max_velo, float end_velo)
{
  ODOMETRY_Reset();
  SERVO_Reset();
  SERVO_SetAcceleration(accel);

  const ODOMETRY *odom = ODOMETRY_GetCurrent();
  const float *tar_velo = SERVO_GetTargetVelocity();

  float accel_length = (max_velo * max_velo - *tar_velo * *tar_velo) / (2.0f * accel);
  float decel_length = (max_velo * max_velo - end_velo * end_velo) / (2.0f * accel);

  SERVO_SetAcceleration(accel);
  while (accel_length < odom->length)
    ;
  while (length - decel_length > odom->length)
    ;
  SERVO_SetAcceleration(-1.0f * accel);
  while (length > odom->length)
  {
    if (*tar_velo < min_velo)
    {
      SERVO_SetAcceleration(0.0f);
      SERVO_SetTargetVelocity(min_velo);
    }
  }
  SERVO_SetAcceleration(0.0f);
  SERVO_SetTargetVelocity(end_velo);
}

void app_main(void)
{
  PARAMETER_Init();
  SENSOR_Init();
  INTERVAL_Start();
  INTERVAL_Buzzer(50);

  while (1)
  {
    if (BUTTON_GetSw1())
    {
      while (BUTTON_GetSw1())
        ;
      INTERVAL_Buzzer(50);
      const PARAMETER *param = PARAMETER_Get();
      HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);
      SERVO_Start();
      LOGGER_Start(500);
      RUN_Straight(0.5f, param->acceleration, param->min_velocity, param->max_velocity, 0);
      LOGGER_Stop();
      SERVO_SetTargetVelocity(0.0f);
      HAL_Delay(4000);
      SERVO_Stop();
      HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
    }
    if (BUTTON_GetSw2())
    {
      while (BUTTON_GetSw2())
        ;
      INTERVAL_Buzzer(50);
      HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);
      LOGGER_Print();
      HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
    }
  }
}
