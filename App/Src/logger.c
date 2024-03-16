#include "logger.h"

// project
#include "odometry.h"
#include "servo.h"

#define LOGGER_BUFFER_SIZE 500
//! log buffer
static int16_t logger_buffer[LOGGER_BUFFER_SIZE][8] = {};
//! log index
static uint16_t logger_buffer_index = 0;
//! log interval count max
static uint16_t logger_freq_count_max = 0;
/**
 * @brief Clear log
 */
void LOGGER_Clear()
{
}
/**
 * @brief Set log frequency
 */
void LOGGER_SetFrequency(uint16_t freq)
{
  if (freq > 1000)
  {
    freq = 1000;
  }
  logger_freq_count_max = freq == 0 ? 0 : 1000 / freq;
}
/**
 * @brief Collect log
 */
void LOGGER_Update()
{
  static uint16_t logger_freq_count = 0;

  const ODOMETRY *odom = ODOMETRY_GetCurrent();

  if (logger_freq_count_max != 0 && ++logger_freq_count >= logger_freq_count_max)
  {
    logger_freq_count = 0;
    logger_buffer[logger_buffer_index][0] = (int16_t)(*SERVO_GetTargetVelocity() * 1000.0f);
    logger_buffer[logger_buffer_index][1] = (int16_t)(*SERVO_GetTargetAngularVelocity() * 1000.0f);
    logger_buffer[logger_buffer_index][2] = (int16_t)(odom->velocity * 1000.0f);
    logger_buffer[logger_buffer_index][3] = (int16_t)(odom->length * 1000.0f);
    logger_buffer[logger_buffer_index][4] = (int16_t)(odom->angular_velocity * 1000.0f);
    logger_buffer[logger_buffer_index][5] = (int16_t)(odom->angle * 1000.0f);
    logger_buffer[logger_buffer_index][6] = (int16_t)(odom->x * 1000.0f);
    logger_buffer[logger_buffer_index][7] = (int16_t)(odom->y * 1000.0f);
    if (++logger_buffer_index >= LOGGER_BUFFER_SIZE)
    {
      logger_freq_count_max = 0;
    }
  }
}
/**
 * @brief Pring log
 */
void LOGGER_Print()
{
  for (uint16_t i = 0; i < logger_buffer_index; i++)
  {
    for (uint16_t j = 0; j < 8; j++)
    {
      printf("%d,", logger_buffer[i][j]);
    }
    printf("\r\n");
  }
}
