#include "button.h"

// libc
#include <stdint.h>

// STM32CubeMX
#include <main.h>

/**
 * @brief Get button 1
 */
bool BUTTON_GetSw1()
{
  return HAL_GPIO_ReadPin(Switch1_GPIO_Port, Switch1_Pin) == GPIO_PIN_RESET;
}
/**
 * @brief Get button 1
 */
bool BUTTON_GetSw2()
{
  return HAL_GPIO_ReadPin(Switch2_GPIO_Port, Switch2_Pin) == GPIO_PIN_RESET;
}
