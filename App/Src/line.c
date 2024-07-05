#include "line.h"

// STM32CubeMX
#include <main.h>

// libc
#include <stdio.h>
#include <math.h>
#include <float.h>

// project
#include "pid.h"
#include "parameter_static.h"

//<! State of start and goal marker
static STARTGOAL_MARKER_STATE lineMarkerStartGoalState = STARTGOAL_MARKER_START_WAITING;
//<! State of curvature marker
static CURVATURE_MARKER_STATE lineMarkerCurvatureState = CURVATURE_MARKER_WAITING;
//<! State of line
static LINE_STATE lineState = LINE_STATE_GOOD;
//<! Calibrating flag
static bool lineMarkerCalibrating = false;
//<! Calibrate value
static LINE_CALIBRATE lineMarkerCalibrateValue = {0};
//<! Line and marker value
static float lineMarkerValue[NUM_SENSOR_POS] = {0};
//<! Line in error
static float lineInError = 0;
//<! Line out error
static float lineOutError = 0;
//<! Enable line angular velocity feedback
static bool lineEnableFeedback = false;
//<! Line angular velocity
static float lineAngularVelocity = 0.0f;
//<! Angular velocity PID
static PID lineAngularVelocityPid = {0};

/**
 * @brief Update calibration data
 */
static void LINE_UpdateCalibration()
{
  for (LINE_SENSOR_POS i = 0; i < NUM_SENSOR_POS; i++)
  {
    float value = logf((float)ADC_GetLineSensorValue(i) + 1.0f);
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
  bool isStartGoal = ADC_GetLineSensorValue(MARKER_SENSOR_RIGHT) > lineMarkerCalibrateValue.threshold[MARKER_SENSOR_RIGHT];
  bool isCurvature = ADC_GetLineSensorValue(MARKER_SENSOR_LEFT) > lineMarkerCalibrateValue.threshold[MARKER_SENSOR_LEFT];

  if (isStartGoal && isCurvature)
  {
    if (lineMarkerStartGoalState != STARTGOAL_MARKER_GOAL_PASSED)
    {
      lineMarkerStartGoalState = STARTGOAL_MARKER_IGNORING;
    }
    if (lineMarkerCurvatureState != CURVATURE_MARKER_PASSED)
    {
      lineMarkerCurvatureState = CURVATURE_MARKER_IGNORING;
    }
  }
  switch (lineMarkerStartGoalState)
  {
  default:
  case STARTGOAL_MARKER_START_WAITING: //!< wait start marker
    if (isStartGoal)
    {
      lineMarkerStartGoalState = STARTGOAL_MARKER_START_PASSING;
    }
    break;
  case STARTGOAL_MARKER_START_PASSING: //!< on the start marker
    if (!isStartGoal)
    {
      lineMarkerStartGoalState = STARTGOAL_MARKER_START_PASSED;
    }
    break;
  case STARTGOAL_MARKER_START_PASSED: //!< started
    lineMarkerStartGoalState = STARTGOAL_MARKER_GOAL_WAITING;
    break;
  case STARTGOAL_MARKER_GOAL_WAITING: //!< wait goal marker
    if (isStartGoal)
    {
      lineMarkerStartGoalState = STARTGOAL_MARKER_GOAL_PASSING;
    }
    break;
  case STARTGOAL_MARKER_GOAL_PASSING: //!< on the goal marker
    if (!isStartGoal)
    {
      lineMarkerStartGoalState = STARTGOAL_MARKER_GOAL_PASSED;
    }
    break;
  case STARTGOAL_MARKER_GOAL_PASSED: //!< goaled
    break;
  case STARTGOAL_MARKER_IGNORING: //!< on the cross line
    if (!isStartGoal)
    {
      lineMarkerStartGoalState = STARTGOAL_MARKER_GOAL_WAITING;
    }
    break;
  }
  switch (lineMarkerCurvatureState)
  {
  default:
  case CURVATURE_MARKER_WAITING: //!< wait curvature marker
    if (isCurvature)
    {
      lineMarkerCurvatureState = CURVATURE_MARKER_PASSING;
    }
    break;
  case CURVATURE_MARKER_PASSING: //!< on the curvature marker
    if (!isCurvature)
    {
      lineMarkerCurvatureState = CURVATURE_MARKER_PASSED;
    }
    break;
  case CURVATURE_MARKER_PASSED:
    lineMarkerCurvatureState = CURVATURE_MARKER_WAITING;
    break;
  case CURVATURE_MARKER_IGNORING: //!< on the cross line
    if (!isCurvature)
    {
      lineMarkerCurvatureState = CURVATURE_MARKER_WAITING;
    }
    break;
  }
}
/**
 * @brief Update angular velocity
 */
static void LINE_UpdateAngularVelocity()
{
  if (!lineEnableFeedback)
  {
    lineAngularVelocity = 0.0f;
    return;
  }

  bool isRightOut = lineMarkerValue[LINE_SENSOR_RIGHT_OUT] > lineMarkerCalibrateValue.threshold[LINE_SENSOR_RIGHT_OUT];
  bool isRighIn = lineMarkerValue[LINE_SENSOR_RIGHT_IN] > lineMarkerCalibrateValue.threshold[LINE_SENSOR_RIGHT_IN];
  bool isLeftIn = lineMarkerValue[LINE_SENSOR_LEFT_IN] > lineMarkerCalibrateValue.threshold[LINE_SENSOR_LEFT_IN];
  bool isLeftOut = lineMarkerValue[LINE_SENSOR_LEFT_OUT] > lineMarkerCalibrateValue.threshold[LINE_SENSOR_LEFT_OUT];

  if (!isRightOut && !isRighIn && !isLeftIn && !isLeftOut)
  {
    // Course out
    lineAngularVelocity = 0.0f;
    lineState = LINE_STATE_COURSE_OUT;
  }
  else
  {
    // Good
    lineOutError = (lineMarkerValue[LINE_SENSOR_RIGHT_OUT] + lineMarkerCalibrateValue.lineOffsetOut) - lineMarkerValue[LINE_SENSOR_LEFT_OUT];
    lineInError = (lineMarkerValue[LINE_SENSOR_RIGHT_IN] + lineMarkerCalibrateValue.lineOffsetIn) - lineMarkerValue[LINE_SENSOR_LEFT_IN];
    lineAngularVelocity = PID_Update(&lineAngularVelocityPid, 0.0f, (lineInError + 2.0f * lineOutError), 1.0f);
    lineState = LINE_STATE_GOOD;
  }
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
    for (LINE_SENSOR_POS i = 0; i < NUM_SENSOR_POS; i++)
    {
      lineMarkerValue[i] = logf((float)ADC_GetLineSensorValue(i) + 1.0f) - lineMarkerCalibrateValue.calibrateAverage[i].min;
    }
    LINE_UpdateMarkerState();
    LINE_UpdateAngularVelocity();
  }
}
/**
 * @brief Get start and goal marker state
 */
STARTGOAL_MARKER_STATE LINE_GetStartGoalState()
{
  return lineMarkerStartGoalState;
}
/**
 * @brief Get curvature marker state
 */
CURVATURE_MARKER_STATE LINE_GetCurvatureState()
{
  return lineMarkerCurvatureState;
}
/**
 * @brief Reset start and goal marker state
 */
void LINE_ResetStartGoalState()
{
  lineMarkerStartGoalState = STARTGOAL_MARKER_START_WAITING;
}
/**
 * @brief Reset curvature marker state
 */
void LINE_ResetCurvatureState()
{
  lineMarkerCurvatureState = CURVATURE_MARKER_WAITING;
}
/**
 * @brief Clear temporary
 */
static void LINE_ClearCalibrationTemporary()
{
  for (LINE_SENSOR_POS i = 0; i < NUM_SENSOR_POS; i++)
  {
    lineMarkerCalibrateValue.calibrateTemp[i].min = logf(4095.0f); // 12 bit ADC
    lineMarkerCalibrateValue.calibrateTemp[i].max = logf(1.0f);
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
  for (LINE_SENSOR_POS i = 0; i < NUM_SENSOR_POS; i++)
  {
    lineMarkerCalibrateValue.threshold[i] = (lineMarkerCalibrateValue.calibrateAverage[i].max - lineMarkerCalibrateValue.calibrateAverage[i].min) * PARAMETER_STATIC_LINE_EXISTS_THRESHOLD;
  }
  float rightOut = lineMarkerCalibrateValue.calibrateAverage[LINE_SENSOR_RIGHT_OUT].max - lineMarkerCalibrateValue.calibrateAverage[LINE_SENSOR_RIGHT_OUT].min;
  float rightIn = lineMarkerCalibrateValue.calibrateAverage[LINE_SENSOR_RIGHT_IN].max - lineMarkerCalibrateValue.calibrateAverage[LINE_SENSOR_RIGHT_IN].min;
  float leftIn = lineMarkerCalibrateValue.calibrateAverage[LINE_SENSOR_LEFT_IN].max - lineMarkerCalibrateValue.calibrateAverage[LINE_SENSOR_LEFT_IN].min;
  float leftOut = lineMarkerCalibrateValue.calibrateAverage[LINE_SENSOR_LEFT_OUT].max - lineMarkerCalibrateValue.calibrateAverage[LINE_SENSOR_LEFT_OUT].min;
  lineMarkerCalibrateValue.lineOffsetOut = leftOut - rightOut;
  lineMarkerCalibrateValue.lineOffsetIn = leftIn - rightIn;
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
  return lineAngularVelocity;
}
/**
 * @brief Get line state
 */
LINE_STATE LINE_GetLineState()
{
  return lineState;
}
/**
 * @brief Reset line state
 */
void LINE_ResetLineState()
{
  lineState = LINE_STATE_SETUP;
}
/**
 * @brief Enable line feedback
 */
void LINE_EnableFeedback(const float *lineAngVeloPidGain)
{
  lineAngularVelocityPid.kp = lineAngVeloPidGain[0];
  lineAngularVelocityPid.ki = lineAngVeloPidGain[1];
  lineAngularVelocityPid.kd = lineAngVeloPidGain[2];
  PID_Reset(&lineAngularVelocityPid);
  lineEnableFeedback = true;
}
/**
 * @brief Disable line feedback
 */
void LINE_DisableFeedback()
{
  lineEnableFeedback = false;
}
/**
 * @brief Get feedback state
 */
bool LINE_IsFeedbackEnabled()
{
  return lineEnableFeedback;
}
/**
 * @brief Print line value
 */
void LINE_Print()
{
  printf("%d,%d,%d,%d,%d,%d\r\n",
         ADC_GetLineSensorValue(MARKER_SENSOR_LEFT),
         ADC_GetLineSensorValue(LINE_SENSOR_LEFT_OUT),
         ADC_GetLineSensorValue(LINE_SENSOR_LEFT_IN),
         ADC_GetLineSensorValue(LINE_SENSOR_RIGHT_IN),
         ADC_GetLineSensorValue(LINE_SENSOR_RIGHT_OUT),
         ADC_GetLineSensorValue(MARKER_SENSOR_RIGHT));
}
