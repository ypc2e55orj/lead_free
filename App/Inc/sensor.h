#ifndef SENSOR_H
#define SENSOR_H

// libc
#include <stdint.h>

/* ADC -----------------------------------------------------------------------*/
typedef enum
{
  MARKER_SENSOR_RIGHT,
  LINE_SENSOR_RIGHT_OUT,
  LINE_SENSOR_RIGHT_IN,
  LINE_SENSOR_LEFT_IN,
  LINE_SENSOR_LEFT_OUT,
  MARKER_SENSOR_LEFT,
  NUM_SENSOR_POS,
} LINE_SENSOR_POS;
/**
 * @brief Get battery voltage
 * @return battery voltage[mV]
 */
int ADC_GetBatteryVoltage();
/**
 * @brief Get line sensor value
 * @return adc value
 */
uint16_t ADC_GetLineSensorValue(LINE_SENSOR_POS pos);

/* Gyro ---------------------------------------------------------------------*/
/**
 * @brief Update an angular rate.
 */
void GYRO_UpdateYaw();
/**
 * @brief Get current angular rate.
 */
float GYRO_GetYaw();

/* Encoder -------------------------------------------------------------------*/
/**
 * @brief Get a value of right encoder.
 */
int16_t ENCODER_GetCountRight();
/**
 * @brief Get a value of left encoder.
 */
int16_t ENCODER_GetCountLeft();
/**
 * @brief Reset encoder values.
 */
void ENCODER_ResetCount();

/* Sensor --------------------------------------------------------------------*/
void SENSOR_Init();

#endif // SENSOR_H
