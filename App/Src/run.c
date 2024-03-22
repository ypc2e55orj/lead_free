#include "run.h"

// libc
#include <math.h>

// project
#include "odometry.h"
#include "servo.h"

/**
 * @brief Straight
 */
void RUN_Straight(RUN_DIRECTION dir, float length, float accel, float minVelo, float maxVelo, float endVelo)
{
  ODOMETRY_Reset();
  SERVO_Reset();

  const ODOMETRY *odom = ODOMETRY_GetCurrent();
  const float *tarVelo = SERVO_GetTargetVelocity();

  float sign = dir == RUN_DIRECTION_FORWARD ? 1.0f : -1.0f;
  float accelLength = (maxVelo * maxVelo - *tarVelo * *tarVelo) / (2.0f * accel);
  float decelLength = (maxVelo * maxVelo - endVelo * endVelo) / (2.0f * accel);

  SERVO_SetMaxVelocity(maxVelo);
  SERVO_SetAcceleration(sign * accel);
  while (accelLength > sign * odom->length)
    ;
  while (length - decelLength > sign * odom->length)
    ;
  SERVO_SetAcceleration(-1.0f * sign * accel);
  while (length > sign * odom->length)
  {
    if (sign * *tarVelo < minVelo)
    {
      SERVO_SetAcceleration(0.0f);
      SERVO_SetTargetVelocity(sign * minVelo);
    }
  }
  SERVO_SetAcceleration(0.0f);
  SERVO_SetTargetVelocity(sign * endVelo);
}

/**
 * @brief Turn
 */
void RUN_Turn(RUN_DIRECTION dir, float degree, float angAccel, float minAngVelo, float maxAngVelo, float endAngVelo)
{
  ODOMETRY_Reset();
  SERVO_Reset();

  const ODOMETRY *odom = ODOMETRY_GetCurrent();
  const float *tarAngVelo = SERVO_GetTargetAngularVelocity();

  float sign = dir == RUN_DIRECTION_FORWARD ? 1.0f : -1.0f;
  float angle = degree * M_PI / 180.0f;
  float accelAngle = (maxAngVelo * maxAngVelo - *tarAngVelo * *tarAngVelo) / (2.0f * angAccel);
  float decelAngle = (maxAngVelo * maxAngVelo - endAngVelo * endAngVelo) / (2.0f * angAccel);

  SERVO_SetMaxAngularVelocity(maxAngVelo);
  SERVO_SetAngularAcceleration(sign * angAccel);
  while (accelAngle > sign * odom->angle)
    ;
  while (angle - decelAngle > sign * odom->angle)
    ;
  SERVO_SetAngularAcceleration(-1.0f * sign * angAccel);
  while (angle > sign * odom->angle)
  {
    if (sign * *tarAngVelo < minAngVelo)
    {
      SERVO_SetAngularAcceleration(0.0f);
      SERVO_SetTargetAngularVelocity(sign * minAngVelo);
    }
  }
  SERVO_SetAngularAcceleration(0.0f);
  SERVO_SetTargetAngularVelocity(sign * endAngVelo);
}
