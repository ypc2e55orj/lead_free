#ifndef PARAMETER_H
#define PARAMETER_H

// libc
#include <stdint.h>

// project
#include "pid.h"

#define NUM_PARAMETERS 3

/* Parameter 0 ---------------------------------------------------------------*/
#define PARAMETER_0_MAX_VELOCITY 1.0     // [m/s]
#define PARAMETER_0_MAX_ACCELERATION 1.5 // [m/ss]
#define PARAMETER_0_VELOCITY_PID_KP 10.0
#define PARAMETER_0_VELOCITY_PID_KI 0.0
#define PARAMETER_0_VELOCITY_PID_KD 0.0
#define PARAMETER_0_MAX_ANGULAR_VELOCITY 10.0     // [rad/s]
#define PARAMETER_0_MAX_ANGULAR_ACCELERATION 20.0 // [rad/ss]
#define PARAMETER_0_ANGULAR_VELOCITY_PID_KP 0.6
#define PARAMETER_0_ANGULAR_VELOCITY_PID_KI 0.05
#define PARAMETER_0_ANGULAR_VELOCITY_PID_KD 0.0

/* Parameter 1 ---------------------------------------------------------------*/
#define PARAMETER_1_MAX_VELOCITY 0.1     // [m/s]
#define PARAMETER_1_MAX_ACCELERATION 1.0 // [m/ss]
#define PARAMETER_1_VELOCITY_PID_KP 1.0
#define PARAMETER_1_VELOCITY_PID_KI 0.0
#define PARAMETER_1_VELOCITY_PID_KD 0.0
#define PARAMETER_1_MAX_ANGULAR_VELOCITY 1.0     // [rad/s]
#define PARAMETER_1_MAX_ANGULAR_ACCELERATION 1.0 // [rad/ss]
#define PARAMETER_1_ANGULAR_VELOCITY_PID_KP 1.0
#define PARAMETER_1_ANGULAR_VELOCITY_PID_KI 0.0
#define PARAMETER_1_ANGULAR_VELOCITY_PID_KD 0.0

/* Parameter 2 ---------------------------------------------------------------*/
#define PARAMETER_2_MAX_VELOCITY 0.3     // [m/s]
#define PARAMETER_2_MAX_ACCELERATION 1.5 // [m/ss]
#define PARAMETER_2_VELOCITY_PID_KP 10.0
#define PARAMETER_2_VELOCITY_PID_KI 0.0
#define PARAMETER_2_VELOCITY_PID_KD 0.0
#define PARAMETER_2_MAX_ANGULAR_VELOCITY 10.0     // [rad/s]
#define PARAMETER_2_MAX_ANGULAR_ACCELERATION 20.0 // [rad/ss]
#define PARAMETER_2_ANGULAR_VELOCITY_PID_KP 0.6
#define PARAMETER_2_ANGULAR_VELOCITY_PID_KI 0.0
#define PARAMETER_2_ANGULAR_VELOCITY_PID_KD 0.0

typedef struct
{
  float max_velocity;
  float acceleration;
  float velocity_pid[3];
  float max_angular_velocity;
  float angular_acceleration;
  float angular_velocity_pid[3];
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
