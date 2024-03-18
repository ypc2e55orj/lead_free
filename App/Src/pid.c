#include "pid.h"

/**
 * @brief Reset PID
 */
void PID_Reset(PID *pid)
{
  pid->prevError = 0.0f;
  pid->errorSums = 0.0f;
}
/**
 * @brief Update PID
 */
float PID_Update(PID *pid, float target, float current, float t)
{
  float error = target - current;
  pid->errorSums += (error + pid->prevError) * t / 2.0f;
  float ret = pid->kp * error + pid->ki * pid->errorSums + pid->kd * (error - pid->prevError) / t;
  pid->prevError = error;
  return ret;
}
