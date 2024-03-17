#include "pid.h"

/**
 * @brief Reset PID
 */
void PID_Reset(PID *pid)
{
  pid->prev_error = 0.0f;
  pid->error_sums = 0.0f;
}
/**
 * @brief Update PID
 */
float PID_Update(PID *pid, float target, float current, float t)
{
  float error = target - current;
  pid->error_sums += (error + pid->prev_error) * t / 2.0f;
  float ret = pid->kp * error + pid->ki * pid->error_sums + pid->kd * (error - pid->prev_error) / t;
  pid->prev_error = error;
  return ret;
}
