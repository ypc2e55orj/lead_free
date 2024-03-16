#ifndef ODOMETRY_H
#define ODOMETRY_H

typedef struct
{
  float velocity;         //!< [m/s]
  float angular_velocity; //!< [rad/s]
  float length;           //!< [mm]
  float angle;            //!< [rad]
} ODOMETRY;

/**
 * @brief Calculate odometry
 */
void ODOMETRY_Calculate();

/**
 * @brief Get current
 */
const ODOMETRY *ODOMETRY_GetCurrent();

#endif // ODOMETRY_H
