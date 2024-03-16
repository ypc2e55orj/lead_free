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

void RUN_Straight(float length, float accel, float max_velo, float end_velo)
{
  ODOMETRY_Reset();
  SERVO_Reset();
  SERVO_SetAcceleration(accel);

  const ODOMETRY *odom = ODOMETRY_GetCurrent();
  const float *tar_velo = SERVO_GetTargetVelocity();

  while (length > odom->length)
    ;

  SERVO_SetAcceleration(0.0f);
  SERVO_SetTargetVelocity(0.0f);
}

void app_main(void)
{
  PARAMETER_Init();
  SENSOR_Init();
  INTERVAL_Start();
  INTERVAL_Buzzer(50);

  SERVO_Start();

  const PARAMETER *param = PARAMETER_Get();
  RUN_Straight(0.1, param->acceleration, param->max_velocity, 0.0f);

  while (1)
  {
    if (BUTTON_GetSw1())
    {
      INTERVAL_Buzzer(10);
    }
    if (BUTTON_GetSw2())
    {
      HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    }
    const ODOMETRY *odom = ODOMETRY_GetCurrent();
    printf("%03d, %03d, %03d, %03d, %03d, %03d\r\n", (int)(odom->velocity * 1000), (int)(odom->angular_velocity * 1000), (int)(odom->length * 1000), (int)(odom->angle * 1000), (int)(odom->x * 1000), (int)(odom->y * 1000));
  }
}
