#include "line.h"

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

/**
 * @brief Update marker state
 */
void LINE_UpdateMarkerState()
{
  if (lineMarkerCalibrating)
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
  else
  {
  }
}
/**
 * @brief Get start and goal marker state
 */
STARTGOAL_MARKER LINE_GetStartGoalState()
{
  return 0;
}
/**
 * @brief Get curvature marker state
 */
CURVATURE_MARKER LINE_GetCurvatureState()
{
  return 0;
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
  lineMarkerCalibrating = true;
}
/**
 * @brief Get calibration value
 */
const LINE_CALIBRATE *LINE_GetCalibrate()
{
  return &lineMarkerCalibrateValue;
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
