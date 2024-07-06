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
#include "search.h"

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
  printf("Waiting calibration...\r\n");
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

void EmergencyStop()
{
  SERVO_SetAcceleration(0.0f);
  SERVO_SetMaxVelocity(0.0f);
  SERVO_SetTargetVelocity(0.0f);
  SERVO_SetAngularAcceleration(0.0f);
  SERVO_SetMaxAngularVelocity(0.0f);
  SERVO_SetTargetAngularVelocity(0.0f);
  for (int i = 0; i < 4; i++)
  {
    INTERVAL_Buzzer(50);
    Delay(100);
  }
  INTERVAL_Buzzer(1000);
  Delay(1000);
  SERVO_Stop();
}

void TraceLine()
{
  const PARAMETER *param = PARAMETER_Get();
  const ODOMETRY *odom = ODOMETRY_GetCurrent();

  INTERVAL_Buzzer(50);
  LOGGER_SetMode(LOGGER_MODE_TARGET);
  LOGGER_Clear();
  ODOMETRY_Reset();
  SERVO_SetTargetVelocity(0.0f);
  SERVO_SetMaxVelocity(param->maxVelocity);
  SERVO_Start(param->velocityPid, param->angularVelocityPid);
  SERVO_SetAcceleration(param->acceleration);
  LINE_ResetStartGoalState();
  LINE_ResetCurvatureState();
  LINE_EnableFeedback(param->lineAngularVelocityPid);
  SEARCH_Start();
  while (LINE_GetStartGoalState() != STARTGOAL_MARKER_GOAL_PASSED)
  {
    if (LINE_GetStartGoalState() == STARTGOAL_MARKER_START_PASSED)
    {
      INTERVAL_Buzzer(50);
      LOGGER_Start(100);
    }
    if (LINE_GetCurvatureState() == CURVATURE_MARKER_PASSED)
    {
      INTERVAL_Buzzer(50);
    }
    if (LINE_GetLineState() == LINE_STATE_COURSE_OUT)
    {
      EmergencyStop();
      return;
    }
    RUN_LineFeedback();
  }
  SEARCH_Stop();
  LOGGER_Stop();
  INTERVAL_Buzzer(50);
  float stopLength = odom->length + 0.25f;
  while (odom->length < stopLength)
  {
    RUN_LineFeedback();
  }
  SERVO_SetAcceleration(-1.0f * param->acceleration);
  while (*SERVO_GetTargetVelocity() > param->minVelocity)
  {
    RUN_LineFeedback();
  }
  SERVO_Stop();
  INTERVAL_Buzzer(50);
}

void FastTraceLine()
{
  const PARAMETER *param = PARAMETER_Get();
  const ODOMETRY *odom = ODOMETRY_GetCurrent();

  INTERVAL_Buzzer(50);
  LOGGER_SetMode(LOGGER_MODE_TARGET);
  LOGGER_Clear();
  ODOMETRY_Reset();
  LINE_EnableFeedback(param->lineAngularVelocityPid);
  LINE_ResetStartGoalState();
  LINE_ResetCurvatureState();
  SERVO_SetTargetVelocity(0.0f);
  SERVO_SetMaxVelocity(param->minVelocity);
  SERVO_Start(param->velocityPid, param->angularVelocityPid);
  SERVO_SetAcceleration(param->acceleration);
  FAST_Start();
  while (LINE_GetStartGoalState() != STARTGOAL_MARKER_GOAL_PASSED)
  {
    if (LINE_GetStartGoalState() == STARTGOAL_MARKER_START_PASSED)
    {
      INTERVAL_Buzzer(50);
      LOGGER_Start(20);
    }
    if (LINE_GetCurvatureState() == CURVATURE_MARKER_PASSED)
    {
      INTERVAL_Buzzer(50);
    }
    if (LINE_GetLineState() == LINE_STATE_COURSE_OUT)
    {
      EmergencyStop();
      return;
    }

    if (LINE_GetCurvatureState() == CURVATURE_MARKER_PASSED ||
        LINE_GetStartGoalState() == STARTGOAL_MARKER_START_PASSED)
    {
      const COURSE *course = FAST_Get();
      if (course != NULL && course->pattern == COURSE_PATTERN_STRAIGHT)
      {
        float straightLength = course->length - 0.1;
        float accelLength = RUN_CalculateAccelLength(param->acceleration, param->maxVelocity, param->minVelocity);
        if (accelLength > straightLength)
        {
          RUN_LineFeedback();
        }
        else
        {
          RUN_Straight(RUN_DIRECTION_FORWARD, straightLength, param->acceleration, param->minVelocity, param->maxVelocity, param->minVelocity);
        }
      }
      if (LINE_GetCurvatureState() == CURVATURE_MARKER_PASSED)
      {
        while (LINE_GetCurvatureState() == CURVATURE_MARKER_PASSED)
          RUN_LineFeedback();
      }
      if (LINE_GetStartGoalState() == STARTGOAL_MARKER_START_PASSED)
      {
        while (LINE_GetStartGoalState() == STARTGOAL_MARKER_START_PASSED)
          RUN_LineFeedback();
      }
    }
    else
    {
      RUN_LineFeedback();
    }
  }
  FAST_Stop();
  LOGGER_Stop();
  INTERVAL_Buzzer(50);
  float stopLength = odom->length + 0.25f;
  while (odom->length < stopLength)
  {
    RUN_LineFeedback();
  }
  SERVO_SetAcceleration(-1.0f * param->acceleration);
  while (*SERVO_GetTargetVelocity() > param->minVelocity)
  {
    RUN_LineFeedback();
  }
  SERVO_Stop();
  INTERVAL_Buzzer(50);
}

void app_main(void)
{
  PARAMETER_Init();
  SENSOR_Init();
  INTERVAL_Start();
  INTERVAL_Buzzer(50);
  printf("\r\n\r\n ---------- LeadFree is started !!! ----------\r\n");
  Calibrate();

  while (1)
  {
    if (BUTTON_GetSw1())
    {
      while (BUTTON_GetSw1())
        ;
      PARAMETER_SetIndex(0);
      TraceLine();

      while (!BUTTON_GetSw1() && !BUTTON_GetSw2())
        ;
      if (BUTTON_GetSw1())
      {
        while (BUTTON_GetSw1())
          ;
        PARAMETER_SetIndex(2);
        FastTraceLine();
      }
      else if (BUTTON_GetSw2())
      {
        while (BUTTON_GetSw2())
          ;
        PARAMETER_SetIndex(1);
        TraceLine();
        break;
      }
    }
    if (BUTTON_GetSw2())
    {
      while (!BUTTON_GetSw2())
        ;
      INTERVAL_Buzzer(50);
      SEARCH_Print();
      LOGGER_Print();
      INTERVAL_Buzzer(50);
    }
  }
}
