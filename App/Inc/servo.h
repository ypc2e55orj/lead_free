#ifndef SERVO_H
#define SERVO_H

#define SERVO_PID_P_GAIN 0.0f
#define SERVO_PID_I_GAIN 0.0f
#define SERVO_PID_D_GAIN 0.0f

/**
 * @brief Initialize servos
 */
void SERVO_Init();

/**
 * @brief Set target velocity
 */
void SERVO_SetTarget(float velo, float ang_velo);

/**
 * @brief Update servos
 */
void SERVO_Update();

#endif // SERVO_H
