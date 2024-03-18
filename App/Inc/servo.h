#ifndef SERVO_H
#define SERVO_H

/**
 * @brief Start servos
 */
void SERVO_Start();
/**
 * @brief Reset servos
 */
void SERVO_Reset();
/**
 * @brief Stop servos
 */
void SERVO_Stop();
/**
 * @brief Update servos
 */
void SERVO_Update();
/**
 *  @brief Set target velocity
 */
void SERVO_SetTargetVelocity(float velo);
/**
 *  @brief Set acceleration
 */
void SERVO_SetAcceleration(float accel);
/**
 *  @brief Set target angular velocity
 */
void SERVO_SetTargetAngularVelocity(float angVelo);
/**
 *  @brief Set angular acceleration
 */
void SERVO_SetAngularAcceleration(float angAccel);
/**
 *  @brief Get target velocity
 */
const float *SERVO_GetTargetVelocity();
/**
 *  @brief Get target angular velocity
 */
const float *SERVO_GetTargetAngularVelocity();

#endif // SERVO_H
