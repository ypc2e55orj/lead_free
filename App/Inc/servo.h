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
 * @brief Update servos
 */
void SERVO_Update();
/**
 *  @brief Servo target velocity
 */
void SERVO_TargetVelocity(float velo, float accel);
/**
 *  @brief Servo target angular velocity
 */
void SERVO_TargetAngularVelocity(float ang_velo, float ang_accel);

#endif // SERVO_H
