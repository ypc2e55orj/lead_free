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
static LINE_MINMAX lineMarkerCalibrateValueForward[NUM_SENSOR_POS] = {0};
static LINE_MINMAX lineMarkerCalibrateValueBack[NUM_SENSOR_POS] = {0};
static LINE_MINMAX *lineMarkerCalibrateValuePtr = NULL;

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
      if (lineMarkerCalibrateValuePtr[i].min > value)
      {
        lineMarkerCalibrateValuePtr[i].min = value;
      }
      if (lineMarkerCalibrateValuePtr[i].max < value)
      {
        lineMarkerCalibrateValuePtr[i].max = value;
      }
    }
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
 * @brief Start sensor calibration
 */
static void LINE_StartCalibrate(LINE_MINMAX *ptr)
{
  lineMarkerCalibrating = false;
  lineMarkerCalibrateValuePtr = ptr;
  for (LINE_SENSOR_POS i = 0; i < NUM_SENSOR_POS; i++)
  {
    lineMarkerCalibrateValuePtr[i].min = UINT16_MAX;
    lineMarkerCalibrateValuePtr[i].max = 0;
  }
  lineMarkerCalibrating = true;
}
/**
 * @brief Start sensor calibration (forward)
 */
void LINE_StartCalibrateForward()
{
  LINE_StartCalibrate(lineMarkerCalibrateValueForward);
}
/**
 * @brief Start sensor calibration (back)
 */
void LINE_StartCalibrateBack()
{
  LINE_StartCalibrate(lineMarkerCalibrateValueBack);
}
/**
 * @brief Stop sensor calibration
 */
void LINE_StopCalibrate()
{
  lineMarkerCalibrating = false;
}
/**
 * @brief Get calibration value
 */
const LINE_MINMAX *LINE_GetCalibrateForward(LINE_SENSOR_POS pos)
{
  return &lineMarkerCalibrateValueForward[pos];
}
/**
 * @brief Get calibration value (back)
 */
const LINE_MINMAX *LINE_GetCalibrateBack(LINE_SENSOR_POS pos)
{
  return &lineMarkerCalibrateValueBack[pos];
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
