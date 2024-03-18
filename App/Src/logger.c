#include "logger.h"

// libc
#include <string.h>
#include <stdio.h>

// project
#include "odometry.h"
#include "servo.h"

#define LOGGER_ENABLED
#ifdef LOGGER_ENABLED
#define LOGGER_BUFFER_SIZE 500
#define LOGGER_ELEM_SIZE 8
//! log buffer
static int16_t loggerBuffer[LOGGER_BUFFER_SIZE * LOGGER_ELEM_SIZE] = {};
//! log index
static uint16_t loggerBufferIndex = 0;
//! log interval count max
static uint16_t loggerFreqCountMax = 0;
//! log mode
static LOGGER_MODE loggerMode = 0;
#endif

/**
 * @brief Collect log (target mode)
 */
static void LOGGER_UpdateTarget()
{
#ifdef LOGGER_ENABLED
  const ODOMETRY *odom = ODOMETRY_GetCurrent();

  loggerBuffer[loggerBufferIndex * LOGGER_ELEM_SIZE + 0] = (int16_t)(*SERVO_GetTargetVelocity() * 1000.0f);
  loggerBuffer[loggerBufferIndex * LOGGER_ELEM_SIZE + 1] = (int16_t)(*SERVO_GetTargetAngularVelocity() * 1000.0f);
  loggerBuffer[loggerBufferIndex * LOGGER_ELEM_SIZE + 2] = (int16_t)(odom->velocity * 1000.0f);
  loggerBuffer[loggerBufferIndex * LOGGER_ELEM_SIZE + 3] = (int16_t)(odom->length * 1000.0f);
  loggerBuffer[loggerBufferIndex * LOGGER_ELEM_SIZE + 4] = (int16_t)(odom->angularVelocity * 1000.0f);
  loggerBuffer[loggerBufferIndex * LOGGER_ELEM_SIZE + 5] = (int16_t)(odom->angle * 1000.0f);
  loggerBuffer[loggerBufferIndex * LOGGER_ELEM_SIZE + 6] = (int16_t)(odom->x * 1000.0f);
  loggerBuffer[loggerBufferIndex * LOGGER_ELEM_SIZE + 7] = (int16_t)(odom->y * 1000.0f);
#endif
}
/**
 * @brief Pring log (target mode)
 */
static void LOGGER_PrintTarget()
{
#ifdef LOGGER_ENABLED
  printf(
      "index,"
      "tar_velo[1000*m/s],"
      "tar_ang_velo[1000*rad/s],"
      "velo[1000*m/s],"
      "len[1000*m],"
      "ang_velo[1000*rad/s],"
      "ang[1000*rad/s],"
      "x[1000*m],"
      "y[1000*m],"
      "\r\n");
  for (uint16_t i = 0; i < loggerBufferIndex; i++)
  {
    printf("%d,", i);
    for (uint16_t j = 0; j < LOGGER_ELEM_SIZE; j++)
    {
      printf("%d,", loggerBuffer[i * LOGGER_ELEM_SIZE + j]);
    }
    printf("\r\n");
  }
#endif
}
/**
 * @brief Clear log
 */
void LOGGER_Clear()
{
#ifdef LOGGER_ENABLED
  memset(loggerBuffer, 0, sizeof(int16_t) * LOGGER_BUFFER_SIZE * LOGGER_ELEM_SIZE);
#endif
}
/**
 * @brief Set log frequency
 */
void LOGGER_Start(uint16_t freq)
{
#ifdef LOGGER_ENABLED
  if (freq > 1000)
  {
    freq = 1000;
  }
  loggerFreqCountMax = freq == 0 ? 0 : 1000 / freq;
#endif
}
/**
 * @brief Stop logger
 */
void LOGGER_Stop()
{
#ifdef LOGGER_ENABLED
  loggerFreqCountMax = 0;
#endif
}
/**
 * @brief Set logger mode
 */
void LOGGER_SetMode(LOGGER_MODE mode)
{
#ifdef LOGGER_ENABLED

#endif
}
/**
 * @brief Collect log
 */
void LOGGER_Update()
{
#ifdef LOGGER_ENABLED
  static uint16_t loggerFreqCount = 0;

  if (loggerFreqCountMax != 0 && ++loggerFreqCount >= loggerFreqCountMax)
  {
    loggerFreqCount = 0;

    // TODO: logger mode
    LOGGER_UpdateTarget();

    if (++loggerBufferIndex >= LOGGER_BUFFER_SIZE)
    {
      loggerFreqCountMax = 0;
    }
  }
#endif
}
/**
 * @brief Pring log
 */
void LOGGER_Print()
{
#ifdef LOGGER_ENABLED

  // TODO: logger mode
  LOGGER_PrintTarget();

#endif
}
