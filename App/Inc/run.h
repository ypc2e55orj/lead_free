#ifndef RUN_H

typedef enum
{
  RUN_DIRECTION_FORWARD,
  RUN_DIRECTION_BACK,
  RUN_DIRECTION_RIGHT = RUN_DIRECTION_FORWARD,
  RUN_DIRECTION_LEFT = RUN_DIRECTION_BACK,
} RUN_DIRECTION;

/**
 * @brief Line feedback
 */
void RUN_LineFeedback();
/**
 * @brief Calculate accel length
 */
float RUN_CalculateAccelLength(float accel, float maxVelo, float endVelo);
/**
 * @brief Straight
 */
void RUN_Straight(RUN_DIRECTION dir, float length, float accel, float minVelo, float maxVelo, float endVelo);
/**
 * @brief Turn
 */
void RUN_Turn(RUN_DIRECTION dir, float degree, float angAccel, float minAngVelo, float maxAngVelo, float endAngVelo);

#endif // RUN_H
