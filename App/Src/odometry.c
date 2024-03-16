#include "odometry.h"

// libc
#include <math.h>

// project
#include <sensor.h>

#define ODOMETRY_ENCODER_PPR 135.5f  //! [Pulses per Revolution]
#define ODOMETRY_TIRE_DIAMETER 20.9f //! [mm]
#define ODOMETRY_TIRE_RADIUS ((ODOMETRY_TIRE_DIAMETER) / 2.0f)
#define ODOMETRY_MM_PER_PULSE (((ODOMETRY_TIRE_RADIUS) * M_PI) / ODOMETRY_ENCODER_PPR)

static ODOMETRY odometry = {0};

/**
 * @brief Calculate odometry
 */
void ODOMETRY_Calculate()
{
  float distRight = ENCODER_GetCountRight() * ODOMETRY_MM_PER_PULSE;
  float distLeft = ENCODER_GetCountLeft() * ODOMETRY_MM_PER_PULSE;
  float veloRight = distRight / 1000.0f;
  float veloLeft = distLeft / 1000.0f;
  odometry.velocity = (veloRight + veloLeft) / 2.0f;
  odometry.length += (distRight + distLeft) / 2.0f;
  float yaw = GYRO_GetYaw();
  odometry.angular_velocity = yaw * (M_PI / 180.0f);
  odometry.angle += yaw / 1000.0f;
}

/**
 * @brief Get current
 */
const ODOMETRY *ODOMETRY_GetCurrent()
{
  return &odometry;
}
