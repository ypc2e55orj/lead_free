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
} STARTGOAL_MARKER_STATE;

//! Curvature marker state
typedef enum
{
  CURVATURE_MARKER_WAITING,  //!< wait curvature marker
  CURVATURE_MARKER_PASSING,  //!< on the curvature marker
  CURVATURE_MARKER_PASSED,   //!< passed
  CURVATURE_MARKER_IGNORING, //!< on the cross line
} CURVATURE_MARKER_STATE;

//! Line state
typedef enum
{
  LINE_STATE_SETUP,
  LINE_STATE_GOOD,
  LINE_STATE_CROSS,
  LINE_STATE_COURSE_OUT,
} LINE_STATE;

//! Sensor calibrate data (min, max)
typedef struct
{
  float min, max;
} LINE_CALIBRATE_MINMAX;

//! Sensor threshold
typedef struct
{
  LINE_CALIBRATE_MINMAX calibrateTemp[NUM_SENSOR_POS];
  LINE_CALIBRATE_MINMAX calibrateAverage[NUM_SENSOR_POS];
  float lineOffsetOut;
  float lineOffsetIn;
  float threshold[NUM_SENSOR_POS];
} LINE_CALIBRATE;

/**
 * @brief Update line sensor and marker sensor state
 */
void LINE_UpdateInterval();
/**
 * @brief Get start and goal marker state
 */
STARTGOAL_MARKER_STATE LINE_GetStartGoalState();
/**
 * @brief Get curvature marker state
 */
CURVATURE_MARKER_STATE LINE_GetCurvatureState();
/**
 * @brief Reset start and goal marker state
 */
void LINE_ResetStartGoalState();
/**
 * @brief Reset curvature marker state
 */
void LINE_ResetCurvatureState();
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
 * @brief Reset start and goal marker state
 */
LINE_STATE LINE_GetLineState();
/**
 * @brief Reset start and goal marker state
 */
void LINE_ResetLineState();
/**
 * @brief Enable line feedback
 */
void LINE_EnableFeedback(const float *lineAngVeloPidGain);
/**
 * @brief Disable line feedback
 */
void LINE_DisableFeedback();
/**
 * @brief Print line value
 */
void LINE_Print();

#endif // LINE_H
