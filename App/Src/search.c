#include "search.h"

// libc
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

// project
#include "line.h"
#include "odometry.h"

//! Search stack
static COURSE_STACK searchStack;
//! Started
static bool searchStarted = false;
//! Section start length
static float searchSectionStartLength = 0.0f;
//! Section start angle
static float searchSectionStartAngle = 0.0f;
//! fast started
static bool fastStarted = false;
//! fast stack index
static uint16_t fastStackIndex = 0;

/**
 * Reset search stack
 */
void SEARCH_Reset()
{
  memset(&searchStack, 0, sizeof(searchStack));
  searchStarted = false;
  searchSectionStartLength = 0.0f;
  searchSectionStartAngle = 0.0f;
}

/**
 *
 */
static void SEARCH_UpdateStack()
{
  float endLength = ODOMETRY_GetCurrent()->length;
  float endAngle = ODOMETRY_GetCurrent()->angle;
  float diffAngle = endAngle - searchSectionStartAngle;
  if (fabs(diffAngle) < PARAMETER_STATIC_STRAIGHT_ANGLE_THRESH)
  {
    searchStack.stack[searchStack.index].pattern = COURSE_PATTERN_STRAIGHT;
  }
  else
  {
    searchStack.stack[searchStack.index].pattern = COURSE_PATTERN_ARC;
  }
  searchStack.stack[searchStack.index].angle = diffAngle;
  searchStack.stack[searchStack.index].length = endLength - searchSectionStartLength;
  searchStack.index++;
  searchSectionStartAngle = endAngle;
  searchSectionStartLength = endLength;
}

/**
 * Update search state
 */
void SEARCH_Update()
{
  STARTGOAL_MARKER_STATE startGoalMarkerState = LINE_GetStartGoalState();
  CURVATURE_MARKER_STATE curvatureMarkerState = LINE_GetCurvatureState();

  if (startGoalMarkerState == STARTGOAL_MARKER_START_PASSED)
  {
    searchSectionStartLength = ODOMETRY_GetCurrent()->length;
    searchSectionStartAngle = ODOMETRY_GetCurrent()->angle;
  }
  else if (curvatureMarkerState == CURVATURE_MARKER_PASSED ||
           startGoalMarkerState == STARTGOAL_MARKER_GOAL_PASSED)
  {
    if (searchStarted)
    {
      SEARCH_UpdateStack();
    }
    if (fastStarted)
    {
      fastStackIndex++;
    }
  }
}

/**
 * @brief Print search stack
 */
void SEARCH_Print()
{
  for (uint16_t i = 0; i < searchStack.index; i++)
  {
    switch (searchStack.stack[i].pattern)
    {
    case COURSE_PATTERN_STRAIGHT:
      printf("%d, STRAIGHT, %d, %d\r\n", i, (int)(searchStack.stack[i].length * 1000.0f), (int)(searchStack.stack[i].angle * 1000.0f));
      break;
    case COURSE_PATTERN_ARC:
      printf("%d, ARC, %d, %d\r\n", i, (int)(searchStack.stack[i].length * 1000.0f), (int)(searchStack.stack[i].angle * 1000.0f));
      break;
    default:
      break;
    }
  }
}

/**
 * @berif start search
 */
void SEARCH_Start()
{
  SEARCH_Reset();
  searchStarted = true;
}

/**
 * @berif stop search
 */
void SEARCH_Stop()
{
  searchStarted = false;
}

/**
 * @brief Get search data
 */
const COURSE *FAST_Get()
{
  if (fastStackIndex >= searchStack.index)
  {
    return NULL;
  }
  return &searchStack.stack[fastStackIndex];
}

/**
 * @brief Start fast
 */
void FAST_Start()
{
  fastStackIndex = 0;
  fastStarted = true;
}

/**
 * @brief Stop fast
 */
void FAST_Stop()
{
  fastStarted = false;
}
