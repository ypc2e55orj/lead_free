#include "odometry.h"

// libc
#include <math.h>
#include <float.h>

// project
#include "sensor.h"

#define ODOMETRY_ENCODER_PPR 119.538f //! [Pulses per Revolution]
#define ODOMETRY_TIRE_DIAMETER 20.9f  //! [mm]
#define ODOMETRY_MM_PER_PULSE (((ODOMETRY_TIRE_DIAMETER) * M_PI) / ODOMETRY_ENCODER_PPR)
#define ODOMETRY_MAX_LOGSIZE 250

//! current odometry
static ODOMETRY odometry = {0};
//! log
static float odometry_log[ODOMETRY_MAX_LOGSIZE][2] = {};
//! log index
static uint16_t odometry_log_index = 0;
//! log interval count max
static uint16_t odometry_log_count_max = 0;

/**
 * @brief Calculate odometry
 */
void ODOMETRY_Calculate()
{
  static uint16_t odometry_log_count = 0;

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

  if (odometry_log_count_max != 0 && ++odometry_log_count >= odometry_log_count_max)
  {
    odometry_log_count = 0;
    odometry_log[odometry_log_index][0] = odometry.x;
    odometry_log[odometry_log_index][1] = odometry.y;
    if (++odometry_log_index >= ODOMETRY_MAX_LOGSIZE)
    {
      odometry_log_count_max = 0;
    }
  }
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
/**
 * @brief Set log frequency
 */
void ODOMETRY_SetLogFrequency(uint16_t log_freq)
{
  if (log_freq > 1000)
  {
    log_freq = 1000;
  }
  odometry_log_count_max = log_freq == 0 ? 0 : 1000 / log_freq;
}
/**
 * @brief Print log
 */
void ODOMETRY_PrintLog()
{
  printf("x, y\r\n");
  for (uint16_t i = 0; i < odometry_log_index; i++)
  {
    int x_int = (int)odometry_log[i][0];
    int x_frac = (int)(odometry_log[i][0] * 1000) % 1000;
    int y_int = (int)odometry_log[i][1];
    int y_frac = (int)(odometry_log[i][1] * 1000) % 1000;
    printf("%d.%d, %d.%d\r\n", x_int, x_frac, y_int, y_frac);
  }
}
