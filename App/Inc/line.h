#ifndef LINE_H

// libc
#include <stdint.h>

// project
#include "sensor.h"

//! Goal and start marker state
typedef enum
{
  STARTGOAL_MARKER_START_WAITING, //!< wait start marker
  STARTGOAL_MARKER_START_PASSING, //!< on the start marker
  STARTGOAL_MARKER_START_PASSED,  //!< started
  STARTGOAL_MARKER_GOAL_WAITING,  //!< wait goal marker
  STARTGOAL_MARKER_GOAL_PASSING,  //!< on the goal marker
  STARTGOAL_MARKER_GOAL_PASSED,   //!< goaled
  STARTGOAL_MARKER_IGNORING,      //!< on the cross line
} STARTGOAL_MARKER;

//! Curvature marker state
typedef enum
{
  CURVATURE_MARKER_WAITING,  //!< wait curvature marker
  CURVATURE_MARKER_PASSING,  //!< on the curvature marker
  CURVATURE_MARKER_PASSED,   //!< passed
  CURVATURE_MARKER_IGNORING, //!< on the cross line
} CURVATURE_MARKER;

//! Sensor calibrate data (min, max)
typedef struct
{
  uint16_t min, max;
} LINE_CALIBRATE_MINMAX;

//! Sensor threshold
typedef struct
{
  LINE_CALIBRATE_MINMAX calibrateTemp[NUM_SENSOR_POS];
  LINE_CALIBRATE_MINMAX calibrateAverage[NUM_SENSOR_POS];
  int lineOffsetOut;
  int lineOffsetIn;
  uint16_t markerThreshStartGoal;
  uint16_t markerThreshCurvature;
} LINE_CALIBRATE;

/**
 * @brief Update line sensor and marker sensor state
 */
void LINE_UpdateInterval();
/**
 * @brief Get start and goal marker state
 */
STARTGOAL_MARKER LINE_GetStartGoalState();
/**
 * @brief Get curvature marker state
 */
CURVATURE_MARKER LINE_GetCurvatureState();
/**
 * @brief Reset start and goal marker state
 */
STARTGOAL_MARKER LINE_ResetStartGoalState();
/**
 * @brief Reset curvature marker state
 */
CURVATURE_MARKER LINE_ResetCurvatureState();
/**
 * @brief Start sensor calibration (forward)
 */
void LINE_StartCalibrateForward();
/**
 * @brief Start sensor calibration (back)
 */
void LINE_StartCalibrateBack();
/**
 * @brief Stop sensor calibration
 */
void LINE_StopCalibrate();
/**
 * @brief Get calibration value
 */
const LINE_CALIBRATE *LINE_GetCalibrate();
/**
 * @brief Get generated angular velocity
 */
float LINE_GetAngularVelocity();
/**
 * @brief Print line sensor
 */
void LINE_Print();

#endif // LINE_H
