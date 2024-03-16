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
  }
}
