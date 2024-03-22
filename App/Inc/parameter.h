#ifndef PARAMETER_H
#define PARAMETER_H

// libc
#include <stdint.h>
#include <math.h>
// project
#include "pid.h"

#define NUM_PARAMETERS 3

/* Parameter 0 ---------------------------------------------------------------*/
#define PARAMETER_0_MIN_VELOCITY 0.01 // [m/s]
#define PARAMETER_0_MAX_VELOCITY 0.5  // [m/s]
#define PARAMETER_0_ACCELERATION 1.0  // [m/ss]
#define PARAMETER_0_VELOCITY_PID_KP 10.0
#define PARAMETER_0_VELOCITY_PID_KI 0.5
#define PARAMETER_0_VELOCITY_PID_KD 0.0
#define PARAMETER_0_MIN_ANGULAR_VELOCITY (0.01 * M_PI) // [rad/s]
#define PARAMETER_0_MAX_ANGULAR_VELOCITY (2.5 * M_PI)  // [rad/s]
#define PARAMETER_0_ANGULAR_ACCELERATION (40.0 * M_PI) // [rad/ss]
#define PARAMETER_0_ANGULAR_VELOCITY_PID_KP 0.6
#define PARAMETER_0_ANGULAR_VELOCITY_PID_KI 0.05
#define PARAMETER_0_ANGULAR_VELOCITY_PID_KD 0.0

/* Parameter 1 ---------------------------------------------------------------*/
#define PARAMETER_1_MIN_VELOCITY 0.1 // [m/s]
#define PARAMETER_1_MAX_VELOCITY 0.1 // [m/s]
#define PARAMETER_1_ACCELERATION 1.0 // [m/ss]
#define PARAMETER_1_VELOCITY_PID_KP 1.0
#define PARAMETER_1_VELOCITY_PID_KI 0.0
#define PARAMETER_1_VELOCITY_PID_KD 0.0
#define PARAMETER_1_MIN_ANGULAR_VELOCITY 1.0 // [rad/s]
#define PARAMETER_1_MAX_ANGULAR_VELOCITY 1.0 // [rad/s]
#define PARAMETER_1_ANGULAR_ACCELERATION 1.0 // [rad/ss]
#define PARAMETER_1_ANGULAR_VELOCITY_PID_KP 1.0
#define PARAMETER_1_ANGULAR_VELOCITY_PID_KI 0.0
#define PARAMETER_1_ANGULAR_VELOCITY_PID_KD 0.0

/* Parameter 2 ---------------------------------------------------------------*/
#define PARAMETER_2_MIN_VELOCITY 0.3 // [m/s]
#define PARAMETER_2_MAX_VELOCITY 0.3 // [m/s]
#define PARAMETER_2_ACCELERATION 1.5 // [m/ss]
#define PARAMETER_2_VELOCITY_PID_KP 10.0
#define PARAMETER_2_VELOCITY_PID_KI 0.0
#define PARAMETER_2_VELOCITY_PID_KD 0.0
#define PARAMETER_2_MIN_ANGULAR_VELOCITY 10.0 // [rad/s]
#define PARAMETER_2_MAX_ANGULAR_VELOCITY 10.0 // [rad/s]
#define PARAMETER_2_ANGULAR_ACCELERATION 20.0 // [rad/ss]
#define PARAMETER_2_ANGULAR_VELOCITY_PID_KP 0.6
#define PARAMETER_2_ANGULAR_VELOCITY_PID_KI 0.0
#define PARAMETER_2_ANGULAR_VELOCITY_PID_KD 0.0

typedef struct
{
  float minVelocity;
  float maxVelocity;
  float acceleration;
  float velocityPid[3];
  float minAngularVelocity;
  float maxAngularVelocity;
  float angularAcceleration;
  float angularVelocityPid[3];
} PARAMETER;

/**
 * @brief Initialize parameters
 */
void PARAMETER_Init();
/**
 * @brief Set parameter index
 */
void PARAMETER_SetIndex(uint8_t index);
/**
 * @brief Get current parameter
 */
const PARAMETER *PARAMETER_Get();

#endif // PARAMETER_H
