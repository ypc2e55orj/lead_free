#include "logger.h"

// libc
#include <string.h>
#include <stdio.h>

// project
#include "odometry.h"
#include "servo.h"
#include "parameter_static.h"

#if PARAMETER_STATIC_LOGGER_ENABLED != 0
//! log buffer
static int16_t loggerBuffer[PARAMETER_STATIC_LOGGER_BUFFER_SIZE] = {};
//! log index
static uint16_t loggerBufferIndex = 0;
//! log interval count max
static uint16_t loggerFreqCountMax = 0;
//! log mode
static LOGGER_MODE loggerMode = 0;
//! mode element num
static uint16_t loggerModeElmNum[NUM_LOGGER_MODE] = {
    [LOGGER_MODE_TARGET] = 8,
    [LOGGER_MODE_ODOMETRY] = 2,
};
#endif

/**
 * @brief Collect log (target mode)
 */
static void
LOGGER_UpdateTarget()
{
#if PARAMETER_STATIC_LOGGER_ENABLED != 0
  const ODOMETRY *odom = ODOMETRY_GetCurrent();
  loggerBuffer[loggerBufferIndex + 0] = (int16_t)(*SERVO_GetTargetVelocity() * 1000.0f);
  loggerBuffer[loggerBufferIndex + 1] = (int16_t)(*SERVO_GetTargetAngularVelocity() * 1000.0f);
  loggerBuffer[loggerBufferIndex + 2] = (int16_t)(odom->velocity * 1000.0f);
  loggerBuffer[loggerBufferIndex + 3] = (int16_t)(odom->length * 1000.0f);
  loggerBuffer[loggerBufferIndex + 4] = (int16_t)(odom->angularVelocity * 1000.0f);
  loggerBuffer[loggerBufferIndex + 5] = (int16_t)(odom->angle * 1000.0f);
  loggerBuffer[loggerBufferIndex + 6] = (int16_t)(odom->x * 1000.0f);
  loggerBuffer[loggerBufferIndex + 7] = (int16_t)(odom->y * 1000.0f);
#endif
}
/**
 * @brief Pring log (target mode)
 */
static void LOGGER_PrintHeaderTarget()
{
#if PARAMETER_STATIC_LOGGER_ENABLED != 0
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
#endif
}
/**
 * @brief Collect log (odometry mode)
 */
static void LOGGER_UpdateOdometry()
{
#if PARAMETER_STATIC_LOGGER_ENABLED != 0
  const ODOMETRY *odom = ODOMETRY_GetCurrent();
  loggerBuffer[loggerBufferIndex + 0] = (int16_t)(odom->x * 1000.0f);
  loggerBuffer[loggerBufferIndex + 1] = (int16_t)(odom->y * 1000.0f);
#endif
}
/**
 * @brief Pring log (odometry mode)
 */
static void LOGGER_PrintHeaderOdometry()
{
#if PARAMETER_STATIC_LOGGER_ENABLED != 0
  printf(
      "index,"
      "x[1000*m],"
      "y[1000*m],"
      "\r\n");
#endif
}
/**
 * @brief Clear log
 */
void LOGGER_Clear()
{
#if PARAMETER_STATIC_LOGGER_ENABLED != 0
  memset(loggerBuffer, 0, sizeof(int16_t) * PARAMETER_STATIC_LOGGER_BUFFER_SIZE);
  loggerBufferIndex = 0;
#endif
}
/**
 * @brief Set log frequency
 */
void LOGGER_Start(uint16_t freq)
{
#if PARAMETER_STATIC_LOGGER_ENABLED != 0
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
#if PARAMETER_STATIC_LOGGER_ENABLED != 0
  loggerFreqCountMax = 0;
#endif
}
/**
 * @brief Set logger mode
 */
void LOGGER_SetMode(LOGGER_MODE mode)
{
#if PARAMETER_STATIC_LOGGER_ENABLED != 0
  switch (mode)
  {
  case LOGGER_MODE_TARGET:
  case LOGGER_MODE_ODOMETRY:
    loggerMode = mode;
    break;
  default:
    loggerMode = LOGGER_MODE_TARGET;
    break;
  }
#endif
}
/**
 * @brief Collect log
 */
void LOGGER_Update()
{
#if PARAMETER_STATIC_LOGGER_ENABLED != 0
  static uint16_t loggerFreqCount = 0;

  if (loggerFreqCountMax != 0 && ++loggerFreqCount >= loggerFreqCountMax)
  {
    loggerFreqCount = 0;

    switch (loggerMode)
    {
    default:
    case LOGGER_MODE_TARGET:
      LOGGER_UpdateTarget();
      break;
    case LOGGER_MODE_ODOMETRY:
      LOGGER_UpdateOdometry();
      break;
    }

    if ((loggerBufferIndex += loggerModeElmNum[loggerMode]) >= PARAMETER_STATIC_LOGGER_BUFFER_SIZE)
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
#if PARAMETER_STATIC_LOGGER_ENABLED != 0
  switch (loggerMode)
  {
  default:
  case LOGGER_MODE_TARGET:
    LOGGER_PrintHeaderTarget();
    break;
  case LOGGER_MODE_ODOMETRY:
    LOGGER_PrintHeaderOdometry();
    break;
  }
  for (uint16_t i = 0; i < loggerBufferIndex; i += loggerModeElmNum[loggerMode])
  {
    printf("%d,", i);
    for (uint16_t j = 0; j < loggerModeElmNum[loggerMode]; j++)
    {
      printf("%d,", loggerBuffer[i + j]);
    }
    printf("\r\n");
  }
#endif
}
