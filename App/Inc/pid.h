#ifndef PID_H
#define PID_H

typedef struct
{
  float kp;
  float ki;
  float kd;

  float prev_error;
} PID;

/**
 * @brief Reset PID
 */
float PID_Reset(PID *pid);
/**
 * @brief Update PID
 */
float PID_Update(PID *pid, float target, float current, float t);

#endif // PID_H
