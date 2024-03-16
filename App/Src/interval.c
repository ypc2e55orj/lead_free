// STM32CubeMX
#include <main.h>

// Project
#include "sensor.h"
#include "servo.h"
#include "odometry.h"

/* Variables ------------------------------------------------------------------*/
//! TIM6 Handler extern
extern TIM_HandleTypeDef htim6;

/**
 * @brief timer interrupt
 * TIM6: 2kHz
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static enum {
    UPDATE_STATE_GYRO,
    UPDATE_STATE_CALC,
  } updateState = UPDATE_STATE_GYRO;

  if (htim == &htim6)
  {
    switch (updateState)
    {
    default:
    case UPDATE_STATE_GYRO:
      GYRO_UpdateYaw();
      updateState = UPDATE_STATE_CALC;
      break;
    case UPDATE_STATE_CALC:
      ODOMETRY_Calculate();
      SERVO_Update();
      updateState = UPDATE_STATE_GYRO;
      break;
    }
  }
}

/**
 * @brief Start interval process
 */
void INTERVAL_Start()
{
  HAL_TIM_Base_Start_IT(&htim6);
}
