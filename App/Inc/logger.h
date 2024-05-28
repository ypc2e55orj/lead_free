#ifndef LOGGER_H
#define LOGGER_H

// libc
#include <stdint.h>

typedef enum
{
  LOGGER_MODE_TARGET,
  LOGGER_MODE_ODOMETRY,
  LOGGER_MODE_SENSOR,
  NUM_LOGGER_MODE,
} LOGGER_MODE;

/**
 * @brief Clear log
 */
void LOGGER_Clear();
/**
 * @brief Start logger (Set log frequency)
 */
void LOGGER_Start(uint16_t freq);
/**
 * @brief Stop logger
 */
void LOGGER_Stop();
/**
 * @brief Set logger mode
 */
void LOGGER_SetMode(LOGGER_MODE mode);
/**
 * @brief Collect log
 */
void LOGGER_Update();
/**
 * @brief Pring log
 */
void LOGGER_Print();

#endif // LOGGER_H
