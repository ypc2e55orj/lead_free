#ifndef LOGGER_H
#define LOGGER_H

// libc
#include <stdint.h>

/**
 * @brief Clear log
 */
void LOGGER_Clear();
/**
 * @brief Set log frequency
 */
void LOGGER_SetFrequency(uint16_t freq);
/**
 * @brief Collect log
 */
void LOGGER_Update();
/**
 * @brief Pring log
 */
void LOGGER_Print();

#endif // LOGGER_H
