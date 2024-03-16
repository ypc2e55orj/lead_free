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
#include "button.h"

void app_main(void)
{
  PARAMETER_Init();
  SENSOR_Init();
  SERVO_Start();
  INTERVAL_Start();

  INTERVAL_Buzzer(50);

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
