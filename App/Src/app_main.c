#include "app_main.h"

// libc
#include <stdio.h>

// STM32CubeMX
#include <main.h>

// project
#include "interval.h"
#include "sensor.h"
#include "odometry.h"
#include "servo.h"
#include "parameter.h"

void app_main(void)
{
  PARAMETER_Init();
  SENSOR_Init();
  SERVO_Init();
  INTERVAL_Start();

  while (1)
  {
  }
}
