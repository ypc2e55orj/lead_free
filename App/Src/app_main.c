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
  LINE_DisableFeedback();
  SERVO_Start(param->velocityPid, param->angularVelocityPid);
  LINE_StartCalibrateForward();
  RUN_Straight(RUN_DIRECTION_FORWARD, 0.1f, 0.5f, 0.01f, 0.1f, 0);
  LINE_StartCalibrateBack();
  RUN_Straight(RUN_DIRECTION_BACK, 0.1f, 0.5f, 0.01f, 0.1f, 0);
  LINE_StopCalibrate();
  Delay(500);
  INTERVAL_Buzzer(50);
  SERVO_Stop();
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
  const LINE_CALIBRATE *p = LINE_GetCalibrate();
  for (LINE_SENSOR_POS i = 0; i < NUM_SENSOR_POS; i++)
  {
    printf("%d: min %04d, max %04d \r\n", i, (int)(p->calibrateAverage[i].min * 1000), (int)(p->calibrateAverage[i].max * 1000));
  }
  printf("outOffset %04d, inOffset %04d\r\n", (int)(p->lineOffsetOut * 1000), (int)(p->lineOffsetIn * 1000));
  printf("startGoalThresh %04d, curvatureThresh %04d\r\n", (int)(p->threshold[MARKER_SENSOR_RIGHT] * 1000), (int)(p->threshold[MARKER_SENSOR_LEFT] * 1000));
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

  const PARAMETER *param = PARAMETER_Get();
  const ODOMETRY *odom = ODOMETRY_GetCurrent();
  while (1)
  {
    if (BUTTON_GetSw1())
    {
      while (BUTTON_GetSw2())
        ;
      INTERVAL_Buzzer(50);
      LOGGER_Print();
    }
    if (BUTTON_GetSw2())
    {
      while (BUTTON_GetSw2())
        ;
      INTERVAL_Buzzer(50);
      LINE_EnableFeedback(param->lineAngularVelocityPid);
      SERVO_Start(param->velocityPid, param->angularVelocityPid);
      LINE_ResetStartGoalState();
      LINE_ResetCurvatureState();
      SERVO_SetMaxVelocity(param->maxVelocity);
      SERVO_SetAcceleration(param->acceleration);
      SERVO_SetTargetVelocity(0.0f);
      while (LINE_GetStartGoalState() != STARTGOAL_MARKER_GOAL_PASSED)
      {
        if (LINE_GetStartGoalState() == STARTGOAL_MARKER_START_PASSED)
        {
          INTERVAL_Buzzer(50);
          LOGGER_Start(10);
        }
        if (LINE_GetCurvatureState() == CURVATURE_MARKER_PASSED)
        {
          INTERVAL_Buzzer(50);
        }
        float velo = LINE_GetAngularVelocity();
        float sign = velo < 0.0f ? -1.0f : 1.0f;
        SERVO_SetAngularAcceleration(sign * param->acceleration);
        SERVO_SetTargetAngularVelocity(velo);
        SERVO_SetMaxAngularVelocity(sign * velo);
      }
      LOGGER_Stop();
      INTERVAL_Buzzer(50);
      float stopLength = odom->length + 0.5f;
      while (odom->length < stopLength)
      {
        float velo = LINE_GetAngularVelocity();
        float sign = velo < 0.0f ? -1.0f : 1.0f;
        SERVO_SetAngularAcceleration(sign * param->acceleration);
        SERVO_SetTargetAngularVelocity(velo);
        SERVO_SetMaxAngularVelocity(sign * velo);
      }
      SERVO_SetAcceleration(-1.0f * param->acceleration);
      while (*SERVO_GetTargetVelocity() > param->minVelocity)
      {
        float velo = LINE_GetAngularVelocity();
        float sign = velo < 0.0f ? -1.0f : 1.0f;
        SERVO_SetAngularAcceleration(sign * param->acceleration);
        SERVO_SetTargetAngularVelocity(velo);
        SERVO_SetMaxAngularVelocity(sign * velo);
      }
      SERVO_Stop();
      INTERVAL_Buzzer(50);
    }
  }
}
