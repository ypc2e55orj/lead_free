#include "parameter.h"

//! parameters
static PARAMETER parameters[NUM_PARAMETERS] = {0};
//! parameter index
static uint8_t parameterIndex = 0;

/**
 * @brief Initialize parameters
 */
void PARAMETER_Init()
{
#define SET_PARAMETER(num)                                                               \
  do                                                                                     \
  {                                                                                      \
    parameters[num].minVelocity = PARAMETER_##num##_MIN_VELOCITY;                       \
    parameters[num].maxVelocity = PARAMETER_##num##_MAX_VELOCITY;                       \
    parameters[num].acceleration = PARAMETER_##num##_ACCELERATION;                       \
    parameters[num].velocityPid[0] = PARAMETER_##num##_VELOCITY_PID_KP;                 \
    parameters[num].velocityPid[1] = PARAMETER_##num##_VELOCITY_PID_KI;                 \
    parameters[num].velocityPid[2] = PARAMETER_##num##_VELOCITY_PID_KD;                 \
    parameters[num].minAngularVelocity = PARAMETER_##num##_MIN_ANGULAR_VELOCITY;       \
    parameters[num].maxAngularVelocity = PARAMETER_##num##_MAX_ANGULAR_VELOCITY;       \
    parameters[num].angularAcceleration = PARAMETER_##num##_ANGULAR_ACCELERATION;       \
    parameters[num].angularVelocityPid[0] = PARAMETER_##num##_ANGULAR_VELOCITY_PID_KP; \
    parameters[num].angularVelocityPid[1] = PARAMETER_##num##_ANGULAR_VELOCITY_PID_KI; \
    parameters[num].angularVelocityPid[2] = PARAMETER_##num##_ANGULAR_VELOCITY_PID_KD; \
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
  parameterIndex = index;
}
/**
 * @brief Get current parameter
 */
const PARAMETER *PARAMETER_Get()
{
  return &parameters[parameterIndex];
}
