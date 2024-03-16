#ifndef PARAMETER_H
#define PARAMETER_H

// libc
#include <stdint.h>

// project
#include "pid.h"

#define NUM_PARAMETERS 3

/* Parameter 0 ---------------------------------------------------------------*/
#define PARAMETER_0_VELOCITY 0.3     // [m/s]
#define PARAMETER_0_ACCELERATION 1.5 // [m/ss]
#define PARAMETER_0_VELOCITY_PID_KP 20.0
#define PARAMETER_0_VELOCITY_PID_KI 0.0
#define PARAMETER_0_VELOCITY_PID_KD 0.0
#define PARAMETER_0_ANGULAR_VELOCITY 10.0     // [rad/s]
#define PARAMETER_0_ANGULAR_ACCELERATION 20.0 // [rad/ss]
#define PARAMETER_0_ANGULAR_VELOCITY_PID_KP 0.6
#define PARAMETER_0_ANGULAR_VELOCITY_PID_KI 0.0
#define PARAMETER_0_ANGULAR_VELOCITY_PID_KD 0.0

/* Parameter 1 ---------------------------------------------------------------*/
#define PARAMETER_1_VELOCITY 0.3     // [m/s]
#define PARAMETER_1_ACCELERATION 1.5 // [m/ss]
#define PARAMETER_1_VELOCITY_PID_KP 0.1
#define PARAMETER_1_VELOCITY_PID_KI 0.0
#define PARAMETER_1_VELOCITY_PID_KD 0.0
#define PARAMETER_1_ANGULAR_VELOCITY 10.0     // [rad/s]
#define PARAMETER_1_ANGULAR_ACCELERATION 20.0 // [rad/ss]
#define PARAMETER_1_ANGULAR_VELOCITY_PID_KP 0.1
#define PARAMETER_1_ANGULAR_VELOCITY_PID_KI 0.0
#define PARAMETER_1_ANGULAR_VELOCITY_PID_KD 0.0

/* Parameter 2 ---------------------------------------------------------------*/
#define PARAMETER_2_VELOCITY 0.3     // [m/s]
#define PARAMETER_2_ACCELERATION 1.5 // [m/ss]
#define PARAMETER_2_VELOCITY_PID_KP 0.1
#define PARAMETER_2_VELOCITY_PID_KI 0.0
#define PARAMETER_2_VELOCITY_PID_KD 0.0
#define PARAMETER_2_ANGULAR_VELOCITY 10.0     // [rad/s]
#define PARAMETER_2_ANGULAR_ACCELERATION 20.0 // [rad/ss]
#define PARAMETER_2_ANGULAR_VELOCITY_PID_KP 0.1
#define PARAMETER_2_ANGULAR_VELOCITY_PID_KI 0.0
#define PARAMETER_2_ANGULAR_VELOCITY_PID_KD 0.0

typedef struct
{
  float velocity;
  float acceleration;
  PID velocity_pid;
  float angular_velocity;
  float angular_acceleration;
  PID angular_velocity_pid;
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
PARAMETER *PARAMETER_Get();

#endif // PARAMETER_H
