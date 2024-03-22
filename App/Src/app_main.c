#include "app_main.h"

// libc
#include <stdio.h>
#include <math.h>
#include <float.h>

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
#include "run.h"
#include "line.h"

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
      LOGGER_Start(250);
      RUN_Straight(0.18f, param->acceleration, param->minVelocity, param->maxVelocity, 0);
      RUN_Turn(180.0f, param->angularAcceleration, param->minAngularVelocity, param->maxAngularVelocity, 0);
      RUN_Straight(0.18f, param->acceleration, param->minVelocity, param->maxVelocity, 0);
      HAL_Delay(1000);
      LOGGER_Stop();
      INTERVAL_Buzzer(50);
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
    // LINE_Print();
  }
}
