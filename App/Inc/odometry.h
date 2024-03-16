#ifndef ODOMETRY_H
#define ODOMETRY_H

typedef struct
{
  float velocity;         //!< [m/s]
  float angular_velocity; //!< [rad/s]
  float length;           //!< [mm]
  float angle;            //!< [rad]
  float wheel_angular_velocity[2];
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

#endif // ODOMETRY_H
