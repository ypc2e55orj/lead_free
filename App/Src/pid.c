#include "pid.h"

/**
 * @brief Reset PID
 */
float PID_Reset(PID *pid)
{
  pid->prev_error = 0.0f;
}
/**
 * @brief Update PID
 */
float PID_Update(PID *pid, float target, float current, float t)
{
  float error = target - current;
  float ret = pid->kp * error + pid->ki * (error + pid->prev_error) * t / 2.0f + pid->kd * (error - pid->prev_error) / t;
  pid->prev_error = error;

  return ret;
}
