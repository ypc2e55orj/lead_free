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

  const PARAMETER *param = PARAMETER_Get();
  while (1)
  {
    if (BUTTON_GetSw1())
    {
      while (BUTTON_GetSw1())
        ;
      INTERVAL_Buzzer(50);
      HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);
      SERVO_Start(param->velocityPid, param->angularVelocityPid);
      LOGGER_Start(250);
      LINE_StartCalibrateForward();
      RUN_Straight(RUN_DIRECTION_FORWARD, 0.1f, 0.5f, 0.01f, 0.1f, 0);
      LINE_StartCalibrateBack();
      RUN_Straight(RUN_DIRECTION_BACK, 0.1f, 0.5f, 0.01f, 0.1f, 0);
      LINE_StopCalibrate();
      LOGGER_Stop();
      HAL_Delay(500);
      INTERVAL_Buzzer(50);
      SERVO_Stop();
      HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
      printf("------------\r\n");
      const LINE_CALIBRATE *p = LINE_GetCalibrate();
      for (LINE_SENSOR_POS i = 0; i < NUM_SENSOR_POS; i++)
      {
        printf("%d: min %04d, max %04d \r\n", i, p->calibrateAverage[i].min, p->calibrateAverage[i].max);
      }
    }
    if (BUTTON_GetSw2())
    {
      while (BUTTON_GetSw2())
        ;
      INTERVAL_Buzzer(50);
      HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);
      SERVO_Start(param->velocityPid, param->angularVelocityPid);
      while (!BUTTON_GetSw2())
        ;
      while (BUTTON_GetSw2())
        ;
      SERVO_Stop();
      INTERVAL_Buzzer(50);
      HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
    }
    // LINE_Print();
  }
}
