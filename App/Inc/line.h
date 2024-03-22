#ifndef LINE_H

//! Goal and start marker state
typedef enum
{
  STARTGOAL_MARKER_IGNORING,      //!< on the cross line
  STARTGOAL_MARKER_START_WAITING, //!< wait start marker
  STARTGOAL_MARKER_START_PASSING, //!< on the start marker
  STARTGOAL_MARKER_START_PASSED,  //!< started
  STARTGOAL_MARKER_GOAL_WAITING,  //!< wait goal marker
  STARTGOAL_MARKER_GOAL_PASSING,  //!< on the goal marker
  STARTGOAL_MARKER_GOAL_PASSED,   //!< goaled
} STARTGOAL_MARKER;

//! Curvature marker state
typedef enum
{
  CURVATURE_MARKER_IGNORING, //!< on the cross line
  CURVATURE_MARKER_WAITING,  //!< wait curvature marker
  CURVATURE_MARKER_PASSING,  //!< on the curvature marker
  CURVATURE_MARKER_PASSED,   //!< passed
} CURVATURE_MARKER;

/**
 * @brief Update marker state
 */
void LINE_UpdateMarkerState();
/**
 * @brief Get start and goal marker state
 */
STARTGOAL_MARKER LINE_GetStartGoalState();
/**
 * @brief Get curvature marker state
 */
CURVATURE_MARKER LINE_GetCurvatureState();
/**
 * @brief Print line sensor
 */
void LINE_Print();

#endif // LINE_H
