#include "run.h"

// libc
#include <math.h>

// project
#include "odometry.h"
#include "servo.h"

/**
 * @brief Straight
 */
void RUN_Straight(float length, float accel, float minVelo, float maxVelo, float endVelo)
{
  ODOMETRY_Reset();
  SERVO_Reset();
  SERVO_SetAcceleration(accel);

  const ODOMETRY *odom = ODOMETRY_GetCurrent();
  const float *tarVelo = SERVO_GetTargetVelocity();

  float accelLength = (maxVelo * maxVelo - *tarVelo * *tarVelo) / (2.0f * accel);
  float decelLength = (maxVelo * maxVelo - endVelo * endVelo) / (2.0f * accel);

  SERVO_SetAcceleration(accel);
  while (accelLength > odom->length)
    ;
  while (length - decelLength > odom->length)
    ;
  SERVO_SetAcceleration(-1.0f * accel);
  while (length > odom->length)
  {
    if (*tarVelo < minVelo)
    {
      SERVO_SetAcceleration(0.0f);
      SERVO_SetTargetVelocity(minVelo);
    }
  }
  SERVO_SetAcceleration(0.0f);
  SERVO_SetTargetVelocity(endVelo);
}

/**
 * @brief Turn
 */
void RUN_Turn(float degree, float angAccel, float minAngVelo, float maxAngVelo, float endAngVelo)
{
  ODOMETRY_Reset();
  SERVO_Reset();

  const ODOMETRY *odom = ODOMETRY_GetCurrent();
  const float *tarAngVelo = SERVO_GetTargetAngularVelocity();

  float angle = degree * M_PI / 180.0f;
  float accelAngle = (maxAngVelo * maxAngVelo - *tarAngVelo * *tarAngVelo) / (2.0f * angAccel);
  float decelAngle = (maxAngVelo * maxAngVelo - endAngVelo * endAngVelo) / (2.0f * angAccel);

  SERVO_SetAngularAcceleration(angAccel);
  while (accelAngle > odom->angle)
    ;
  while (angle - decelAngle > odom->angle)
    ;
  SERVO_SetAngularAcceleration(-1.0f * angAccel);
  while (angle > odom->angle)
  {
    if (*tarAngVelo < minAngVelo)
    {
      SERVO_SetAngularAcceleration(0.0f);
      SERVO_SetTargetAngularVelocity(minAngVelo);
    }
  }
  SERVO_SetAngularAcceleration(0.0f);
  SERVO_SetTargetAngularVelocity(0.0f);
}
