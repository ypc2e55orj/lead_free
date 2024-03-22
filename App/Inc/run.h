#ifndef RUN_H

typedef enum
{
  RUN_DIRECTION_FORWARD,
  RUN_DIRECTION_BACK,
  RUN_DIRECTION_RIGHT = RUN_DIRECTION_FORWARD,
  RUN_DIRECTION_LEFT = RUN_DIRECTION_BACK,
} RUN_DIRECTION;

/**
 * @brief Straight
 */
void RUN_Straight(float length, float accel, float minVelo, float maxVelo, float endVelo);
/**
 * @brief Turn
 */
void RUN_Turn(float degree, float angAccel, float minAngVelo, float maxAngVelo, float endAngVelo);

#endif // RUN_H
