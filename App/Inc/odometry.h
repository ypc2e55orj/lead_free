#ifndef ODOMETRY_H
#define ODOMETRY_H

// libc
#include <stdint.h>

typedef struct
{
  float velocity;         //!< [m/s]
  float angular_velocity; //!< [rad/s]
  float length;           //!< [m]
  float angle;            //!< [rad]
  float x;                //!< [m]
  float y;                //!< [m]
} ODOMETRY;

/**
 * @brief Calculate odometry
 */
void ODOMETRY_Calculate();
/**
 * @brief Reset sums
 */
void ODOMETRY_Reset();
/**
 * @brief Get current
 */
const ODOMETRY *ODOMETRY_GetCurrent();
/**
 * @brief Set log frequency
 */
void ODOMETRY_SetLogFrequency(uint16_t log_freq);
/**
 * @brief Print log
 */
void ODOMETRY_PrintLog();

#endif // ODOMETRY_H
