#ifndef ODOMETRY_H
#define ODOMETRY_H

// libc
#include <stdint.h>
#include <stdbool.h>

typedef struct
{
  float velocity;         //!< [m/s]
  float angularVelocity; //!< [rad/s]
  float length;           //!< [m]
  float angle;            //!< [rad]
  float x;                //!< [m]
  float y;                //!< [m]
  bool isApproxStraight;  //!< true if the robot is moving straight
} ODOMETRY;

/**
 * @brief Calculate odometry
 */
void ODOMETRY_CalculateInterval();
/**
 * @brief Reset sums
 */
void ODOMETRY_Reset();
/**
 * @brief Reset position
 */
void ODOMETRY_ResetPos();
/**
 * @brief Get current
 */
const ODOMETRY *ODOMETRY_GetCurrent();

#endif // ODOMETRY_H
