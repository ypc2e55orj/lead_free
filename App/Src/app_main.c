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

void Delay(int ms)
{
  for (int i = 0; i < ms; i++)
  {
    HAL_Delay(1);
  }
}

void Calibrate(void)
{
  const PARAMETER *param = PARAMETER_Get();
  while (!BUTTON_GetSw1())
  {
    HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    Delay(10);
  }
  while (BUTTON_GetSw1())
    ;
  printf("Execute calibration...\r\n");
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
  Delay(500);
  INTERVAL_Buzzer(50);
  SERVO_Stop();
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
  const LINE_CALIBRATE *p = LINE_GetCalibrate();
  for (LINE_SENSOR_POS i = 0; i < NUM_SENSOR_POS; i++)
  {
    printf("%d: min %04d, max %04d \r\n", i, p->calibrateAverage[i].min, p->calibrateAverage[i].max);
  }
  printf("outOffset %04d, inOffset %04d\r\n", p->lineOffsetOut, p->lineOffsetIn);
  printf("startGoalThresh %04d, curvatureThresh %04d\r\n", p->markerThreshStartGoal, p->markerThreshCurvature);
}

void app_main(void)
{
  PARAMETER_Init();
  SENSOR_Init();
  INTERVAL_Start();
  INTERVAL_Buzzer(50);
  printf("\r\n\r\n ---------- LeadFree is started !!! ----------\r\n");
  printf("Waiting calibration...\r\n");
  Calibrate();

  while (1)
  {
    if (BUTTON_GetSw1())
    {
    }
    if (BUTTON_GetSw2())
    {
      while (BUTTON_GetSw2())
        ;
      INTERVAL_Buzzer(50);
      // SERVO_Start(param->velocityPid, param->angularVelocityPid);
      LINE_ResetStartGoalState();
      LINE_ResetCurvatureState();
      while (!BUTTON_GetSw2())
      {
        printf("startGoal: %d, curvature: %d\r\n", LINE_GetStartGoalState(), LINE_GetCurvatureState());
      }
      while (BUTTON_GetSw2())
        ;
      // SERVO_Stop();
      INTERVAL_Buzzer(50);
    }
  }
}
