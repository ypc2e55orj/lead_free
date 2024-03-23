#include "line.h"

// STM32CubeMX
#include <main.h>

// libc
#include <stdio.h>
#include <stdbool.h>

//<! state of start and goal marker
static STARTGOAL_MARKER lineMarkerStartGoal = STARTGOAL_MARKER_START_WAITING;
//<! state of curvature marker
static CURVATURE_MARKER lineMarkerCurvature = CURVATURE_MARKER_WAITING;
//<! calibrating flag
static bool lineMarkerCalibrating = false;
//<! calibrate value
static LINE_CALIBRATE lineMarkerCalibrateValue = {NULL};
static LINE_CALIBRATE_MINMAX *lineMarkerCalibrateValuePtr = NULL;
//<!

/**
 * @brief Update calibration data
 */
static void LINE_UpdateCalibration()
{
  for (LINE_SENSOR_POS i = 0; i < NUM_SENSOR_POS; i++)
  {
    uint16_t value = ADC_GetLineSensorValue(i);
    if (lineMarkerCalibrateValue.calibrateTemp[i].min > value)
    {
      lineMarkerCalibrateValue.calibrateTemp[i].min = value;
    }
    if (lineMarkerCalibrateValue.calibrateTemp[i].max < value)
    {
      lineMarkerCalibrateValue.calibrateTemp[i].max = value;
    }
  }
}
/**
 * @brief Update marker sensor state
 */
static void LINE_UpdateMarkerState()
{
  bool isStartGoal = ADC_GetLineSensorValue(MARKER_SENSOR_RIGHT) > lineMarkerCalibrateValue.markerThreshStartGoal;
  bool isCurvature = ADC_GetLineSensorValue(MARKER_SENSOR_LEFT) > lineMarkerCalibrateValue.markerThreshCurvature;

  if (isStartGoal && isCurvature)
  {
    if (lineMarkerStartGoal != STARTGOAL_MARKER_GOAL_PASSED)
    {
      lineMarkerStartGoal = STARTGOAL_MARKER_IGNORING;
    }
    if (lineMarkerCurvature != CURVATURE_MARKER_PASSED)
    {
      lineMarkerCurvature = CURVATURE_MARKER_IGNORING;
    }
  }
  switch (lineMarkerStartGoal)
  {
  default:
  case STARTGOAL_MARKER_START_WAITING: //!< wait start marker
    if (isStartGoal)
    {
      lineMarkerStartGoal = STARTGOAL_MARKER_START_PASSING;
    }
    break;
  case STARTGOAL_MARKER_START_PASSING: //!< on the start marker
    if (!isStartGoal)
    {
      lineMarkerStartGoal = STARTGOAL_MARKER_START_PASSED;
    }
    break;
  case STARTGOAL_MARKER_START_PASSED: //!< started
    lineMarkerStartGoal = STARTGOAL_MARKER_GOAL_WAITING;
    break;
  case STARTGOAL_MARKER_GOAL_WAITING: //!< wait goal marker
    if (isStartGoal)
    {
      lineMarkerStartGoal = STARTGOAL_MARKER_GOAL_PASSING;
    }
    break;
  case STARTGOAL_MARKER_GOAL_PASSING: //!< on the goal marker
    if (!isStartGoal)
    {
      lineMarkerStartGoal = STARTGOAL_MARKER_GOAL_PASSED;
    }
    break;
  case STARTGOAL_MARKER_GOAL_PASSED: //!< goaled
    break;
  case STARTGOAL_MARKER_IGNORING: //!< on the cross line
    if (!isStartGoal)
    {
      lineMarkerStartGoal = STARTGOAL_MARKER_GOAL_WAITING;
    }
    break;
  }
  switch (lineMarkerCurvature)
  {
  default:
  case CURVATURE_MARKER_WAITING: //!< wait curvature marker
    if (isCurvature)
    {
      lineMarkerCurvature = CURVATURE_MARKER_PASSING;
    }
    break;
  case CURVATURE_MARKER_PASSING: //!< on the curvature marker
    if (!isCurvature)
    {
      lineMarkerCurvature = CURVATURE_MARKER_PASSED;
    }
    break;
  case CURVATURE_MARKER_PASSED:
    lineMarkerCurvature = CURVATURE_MARKER_WAITING;
    break;
  case CURVATURE_MARKER_IGNORING: //!< on the cross line
    if (!isCurvature)
    {
      lineMarkerCurvature = CURVATURE_MARKER_WAITING;
    }
    break;
  }
}
/**
 * @brief Generate angular velocity
 */
static void LINE_GenerateAngularVelocity()
{
}
/**
 * @brief Update marker state
 */
void LINE_UpdateInterval()
{
  if (lineMarkerCalibrating)
  {
    LINE_UpdateCalibration();
  }
  else
  {
    LINE_UpdateMarkerState();
    LINE_GenerateAngularVelocity();
  }
}

/**
 * @brief Get start and goal marker state
 */
STARTGOAL_MARKER LINE_GetStartGoalState()
{
  return lineMarkerStartGoal;
}
/**
 * @brief Get curvature marker state
 */
CURVATURE_MARKER LINE_GetCurvatureState()
{
  return lineMarkerCurvature;
}
/**
 * @brief Reset start and goal marker state
 */
STARTGOAL_MARKER LINE_ResetStartGoalState()
{
  lineMarkerStartGoal = STARTGOAL_MARKER_START_WAITING;
}
/**
 * @brief Reset curvature marker state
 */
CURVATURE_MARKER LINE_ResetCurvatureState()
{
  lineMarkerCurvature = CURVATURE_MARKER_WAITING;
}
/**
 * @brief Clear temporary
 */
static void LINE_ClearCalibrationTemporary()
{
  for (LINE_SENSOR_POS i = 0; i < NUM_SENSOR_POS; i++)
  {
    lineMarkerCalibrateValue.calibrateTemp[i].min = UINT16_MAX;
    lineMarkerCalibrateValue.calibrateTemp[i].max = 0;
  }
}
/**
 * @brief Calculate sensor average
 */
static void LINE_CalculateCalibrationAverage()
{
  for (LINE_SENSOR_POS i = 0; i < NUM_SENSOR_POS; i++)
  {
    lineMarkerCalibrateValue.calibrateAverage[i].min += lineMarkerCalibrateValue.calibrateTemp[i].min;
    lineMarkerCalibrateValue.calibrateAverage[i].min /= 2;
    lineMarkerCalibrateValue.calibrateAverage[i].max += lineMarkerCalibrateValue.calibrateTemp[i].max;
    lineMarkerCalibrateValue.calibrateAverage[i].max /= 2;
  }
}
/**
 * @brief Start sensor calibration (forward)
 */
void LINE_StartCalibrateForward()
{
  lineMarkerCalibrating = false;
  for (LINE_SENSOR_POS i = 0; i < NUM_SENSOR_POS; i++)
  {
    lineMarkerCalibrateValue.calibrateAverage[i].min = 0;
    lineMarkerCalibrateValue.calibrateAverage[i].max = 0;
  }
  LINE_ClearCalibrationTemporary();
  lineMarkerCalibrating = true;
}
/**
 * @brief Start sensor calibration (back)
 */
void LINE_StartCalibrateBack()
{
  lineMarkerCalibrating = false;
  LINE_CalculateCalibrationAverage();
  LINE_ClearCalibrationTemporary();
  lineMarkerCalibrating = true;
}
/**
 * @brief Stop sensor calibration
 */
void LINE_StopCalibrate()
{
  lineMarkerCalibrating = false;
  LINE_CalculateCalibrationAverage();
  LINE_ClearCalibrationTemporary();

  uint16_t threshStartGoal = lineMarkerCalibrateValue.calibrateAverage[MARKER_SENSOR_RIGHT].max + lineMarkerCalibrateValue.calibrateAverage[MARKER_SENSOR_RIGHT].min;
  uint16_t threshCurvature = lineMarkerCalibrateValue.calibrateAverage[MARKER_SENSOR_LEFT].max + lineMarkerCalibrateValue.calibrateAverage[MARKER_SENSOR_LEFT].min;
  lineMarkerCalibrateValue.markerThreshStartGoal = threshStartGoal / 2;
  lineMarkerCalibrateValue.markerThreshCurvature = threshCurvature / 2;

  int offsetOut = lineMarkerCalibrateValue.calibrateAverage[LINE_SENSOR_LEFT_OUT].max - lineMarkerCalibrateValue.calibrateAverage[LINE_SENSOR_RIGHT_OUT].max;
  int offsetIn = lineMarkerCalibrateValue.calibrateAverage[LINE_SENSOR_LEFT_IN].max - lineMarkerCalibrateValue.calibrateAverage[LINE_SENSOR_RIGHT_IN].max;
  lineMarkerCalibrateValue.lineOffsetOut = offsetOut;
  lineMarkerCalibrateValue.lineOffsetIn = offsetIn;
}
/**
 * @brief Get calibration value
 */
const LINE_CALIBRATE *LINE_GetCalibrate()
{
  return &lineMarkerCalibrateValue;
}
/**
 * @brief Get generated angular velocity
 */
float LINE_GetAngularVelocity()
{
  return 0;
}
/**
 * @brief Print line sensor
 */
void LINE_Print()
{
#if 0
#else
  printf(
      ""
      "%04d, "
      "%04d, %04d, %04d, %04d, "
      "%04d"
      "\r\n",
      ADC_GetLineSensorValue(MARKER_SENSOR_RIGHT),
      ADC_GetLineSensorValue(LINE_SENSOR_RIGHT_OUT),
      ADC_GetLineSensorValue(LINE_SENSOR_RIGHT_IN),
      ADC_GetLineSensorValue(LINE_SENSOR_LEFT_IN),
      ADC_GetLineSensorValue(LINE_SENSOR_LEFT_OUT),
      ADC_GetLineSensorValue(MARKER_SENSOR_LEFT));
#endif
}
