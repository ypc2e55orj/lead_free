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

void RUN_Straight(float length, float accel, float max_velo, float end_velo)
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
  while (length > odom->length && *tar_velo > end_velo)
    ;
  SERVO_SetAcceleration(0.0f);
  SERVO_SetTargetVelocity(end_velo);
}

void app_main(void)
{
  PARAMETER_Init();
  SENSOR_Init();
  INTERVAL_Start();
  INTERVAL_Buzzer(50);

  SERVO_Start();

  while (!BUTTON_GetSw2())
  {
  }
  while (BUTTON_GetSw2())
  {
    INTERVAL_Buzzer(10);
  }

  const PARAMETER *param = PARAMETER_Get();
  LOGGER_SetFrequency(1000);
  RUN_Straight(0.1, param->acceleration, param->max_velocity, 0.0f);

  while (1)
  {
    if (BUTTON_GetSw1())
    {
      INTERVAL_Buzzer(10);
      HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);
      LOGGER_Print();
      HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
    }
  }
}
