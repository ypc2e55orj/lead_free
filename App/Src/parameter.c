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
    parameters[num].max_velocity = PARAMETER_##num##_MAX_VELOCITY;                       \
    parameters[num].acceleration = PARAMETER_##num##_MAX_ACCELERATION;                   \
    parameters[num].velocity_pid[0] = PARAMETER_##num##_VELOCITY_PID_KP;                 \
    parameters[num].velocity_pid[1] = PARAMETER_##num##_VELOCITY_PID_KI;                 \
    parameters[num].velocity_pid[2] = PARAMETER_##num##_VELOCITY_PID_KD;                 \
    parameters[num].max_angular_velocity = PARAMETER_##num##_MAX_ANGULAR_VELOCITY;       \
    parameters[num].angular_acceleration = PARAMETER_##num##_MAX_ANGULAR_ACCELERATION;   \
    parameters[num].angular_velocity_pid[0] = PARAMETER_##num##_ANGULAR_VELOCITY_PID_KP; \
    parameters[num].angular_velocity_pid[1] = PARAMETER_##num##_ANGULAR_VELOCITY_PID_KI; \
    parameters[num].angular_velocity_pid[2] = PARAMETER_##num##_ANGULAR_VELOCITY_PID_KD; \
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
const PARAMETER *PARAMETER_Get()
{
  return &parameters[parameter_index];
}
