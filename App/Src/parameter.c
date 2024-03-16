#include "parameter.h"

//! parameters
static PARAMETER parameters[NUM_PARAMETERS] = {0};
//! parameter index
static uint8_t parameter_index = 0;

/**
 * @brief Initialize parameters
 */
void PARAMETER_Init()
{
#define SET_PARAMETER(num)                                                               \
  do                                                                                     \
  {                                                                                      \
    parameters[num].velocity = PARAMETER_##num##_VELOCITY;                               \
    parameters[num].acceleration = PARAMETER_##num##_ACCELERATION;                       \
    parameters[num].velocity_pid.kp = PARAMETER_##num##_VELOCITY_PID_KP;                 \
    parameters[num].velocity_pid.ki = PARAMETER_##num##_VELOCITY_PID_KI;                 \
    parameters[num].velocity_pid.kd = PARAMETER_##num##_VELOCITY_PID_KD;                 \
    parameters[num].angular_velocity = PARAMETER_##num##_ANGULAR_VELOCITY;               \
    parameters[num].angular_acceleration = PARAMETER_##num##_ANGULAR_ACCELERATION;       \
    parameters[num].angular_velocity_pid.kp = PARAMETER_##num##_ANGULAR_VELOCITY_PID_KP; \
    parameters[num].angular_velocity_pid.ki = PARAMETER_##num##_ANGULAR_VELOCITY_PID_KI; \
    parameters[num].angular_velocity_pid.kd = PARAMETER_##num##_ANGULAR_VELOCITY_PID_KD; \
  } while (0)

  SET_PARAMETER(0);
  SET_PARAMETER(1);
  SET_PARAMETER(2);

#undef SET_PARAMETER
}

/**
 * @brief Set parameter index
 */
void PARAMETER_SetIndex(uint8_t index)
{
  parameter_index = index;
}
/**
 * @brief Get current parameter
 */
PARAMETER *PARAMETER_Get()
{
  return &parameters[parameter_index];
}
