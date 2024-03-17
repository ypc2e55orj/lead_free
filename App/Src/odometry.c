#include "odometry.h"

// libc
#include <math.h>
#include <float.h>

// project
#include "sensor.h"

#define ODOMETRY_ENCODER_PPR 119.538f //! [Pulses per Revolution]
#define ODOMETRY_TIRE_DIAMETER 20.9f  //! [mm]
#define ODOMETRY_MM_PER_PULSE (((ODOMETRY_TIRE_DIAMETER) * M_PI) / ODOMETRY_ENCODER_PPR)
//! current odometry
static ODOMETRY odometry = {0};
/**
 * @brief Calculate odometry
 */
void ODOMETRY_Calculate()
{
  float veloRight = (float)ENCODER_GetCountRight() * ODOMETRY_MM_PER_PULSE;
  float veloLeft = (float)ENCODER_GetCountLeft() * ODOMETRY_MM_PER_PULSE;
  ENCODER_ResetCount();

  odometry.velocity = (veloRight + veloLeft) / 2.0f;
  odometry.length += odometry.velocity / 1000.0f;
  odometry.angular_velocity = GYRO_GetYaw() * (M_PI / 180.0f);
  float angle = odometry.angle + odometry.angular_velocity / 1000.0f;

  if (fabsf(odometry.angular_velocity) <= FLT_EPSILON)
  {
    float a = odometry.velocity / 1000.0f;
    odometry.x += a * cosf(angle);
    odometry.y += a * sinf(angle);
  }
  else
  {
    float delta = (angle - odometry.angle) / 2.0f;
    float a = 2.0f * odometry.velocity / odometry.angular_velocity * sinf(delta);
    float b = angle + delta;
    odometry.x += a * cosf(b);
    odometry.y += a * sinf(b);
  }
  odometry.angle = angle;
}
/**
 * @brief Reset sums
 */
void ODOMETRY_Reset()
{
  odometry.angle = 0.0f;
  odometry.length = 0.0f;
}
/**
 * @brief Get current
 */
const ODOMETRY *ODOMETRY_GetCurrent()
{
  return &odometry;
}
