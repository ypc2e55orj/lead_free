#include "odometry.h"

// libc
#include <math.h>
#include <float.h>

// project
#include "sensor.h"

#define ODOMETRY_ENCODER_PPR 119.538f //! [Pulses per Revolution]
#define ODOMETRY_TIRE_DIAMETER 20.9f  //! [mm]
#define ODOMETRY_MM_PER_PULSE (((ODOMETRY_TIRE_DIAMETER) * M_PI) / ODOMETRY_ENCODER_PPR)
#define ODOMETRY_NUM_VELOCITY_SAMPLES 8
#define ODOMETRY_MASK_VELOCITY_SAMPLES (ODOMETRY_NUM_VELOCITY_SAMPLES - 1)
//! Current odometry
static ODOMETRY odometry = {0};
//! Velocity moving average
static float odometryVelocitySamples[ODOMETRY_NUM_VELOCITY_SAMPLES] = {0.0f};
//! Velocity moving average index (head)
static uint8_t odometryVelocitySampleHead = 0;
//! Velocity moving average index (tail)
static uint8_t odometryVelocitySampleTail = 0;
//! Vloecity sums
static float odometryVelocitySums = 0.0f;
/**
 * @brief Calculate odometry
 */
void ODOMETRY_Calculate()
{
  float veloRight = (float)ENCODER_GetCountRight() * ODOMETRY_MM_PER_PULSE;
  float veloLeft = (float)ENCODER_GetCountLeft() * ODOMETRY_MM_PER_PULSE;
  ENCODER_ResetCount();
  float velo_mm = (veloRight + veloLeft) / 2.0f;
  float velo_m = velo_mm / 1000.0f;

  odometryVelocitySums -= odometryVelocitySamples[odometryVelocitySampleHead];
  odometryVelocitySampleHead = (odometryVelocitySampleHead + 1) & ODOMETRY_MASK_VELOCITY_SAMPLES;
  odometryVelocitySamples[odometryVelocitySampleTail] = velo_mm; // [mm/ms] == [m/s]
  odometryVelocitySums += odometryVelocitySamples[odometryVelocitySampleTail];
  odometryVelocitySampleTail = (odometryVelocitySampleTail + 1) & ODOMETRY_MASK_VELOCITY_SAMPLES;

  odometry.velocity = odometryVelocitySums / (float)ODOMETRY_NUM_VELOCITY_SAMPLES;
  odometry.length += velo_m;
  odometry.angularVelocity = GYRO_GetYaw() * (M_PI / 180.0f);
  float angle = odometry.angle + odometry.angularVelocity / 1000.0f;

  if (fabsf(odometry.angularVelocity) <= FLT_EPSILON)
  {
    odometry.x += velo_m * cosf(angle);
    odometry.y += velo_m * sinf(angle);
  }
  else
  {
    float delta = (angle - odometry.angle) / 2.0f;
    float a = 2.0f * velo_m / odometry.angularVelocity * sinf(delta);
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
