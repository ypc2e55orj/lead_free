// STM32CubeMX
#include <main.h>

// Project
#include "sensor.h"
#include "servo.h"
#include "odometry.h"
#include "button.h"
#include "logger.h"

/* Variables ------------------------------------------------------------------*/
//! TIM6 Handler extern
extern TIM_HandleTypeDef htim6;
//! Buzzer count
static int16_t interval_buzzer_count = 0;
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
    if (interval_buzzer_count > 0)
    {
      interval_buzzer_count--;
      HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);
    }
    else
    {
      HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    }

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
      LOGGER_Update();
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
/**
 * @brief Buzzer
 */
void INTERVAL_Buzzer(int16_t ms)
{
  interval_buzzer_count = ms * 2;
}
