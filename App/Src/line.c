#include "line.h"

// libc
#include <stdio.h>

// project
#include "sensor.h"

//<! state of start and goal marker
static STARTGOAL_MARKER lineMarkerStartGoal = STARTGOAL_MARKER_START_WAITING;
//<! state of curvature marker
static CURVATURE_MARKER lineMarkerCurvature = CURVATURE_MARKER_WAITING;

/**
 * @brief Update marker state
 */
void LINE_UpdateMarkerState()
{
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
