#include "app_main.h"

// libc
#include <stdio.h>

// STM32CubeMX
#include <main.h>

// project
#include "interval.h"
#include "sensor.h"
#include "odometry.h"
#include "servo.h"

void app_main(void)
{
  SENSOR_Init();
  INTERVAL_Start();

  while (1)
  {
    const ODOMETRY *current = ODOMETRY_GetCurrent();
    HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    printf("%04d, %04d, %04d, %04d, %04d, %04d, %04d, %04d, %04d, %04d\r\n",
           ADC_GetLineSensorValue(MARKER_SENSOR_LEFT),
           ADC_GetLineSensorValue(LINE_SENSOR_LEFT_OUT),
           ADC_GetLineSensorValue(LINE_SENSOR_LEFT_IN),
           ADC_GetLineSensorValue(LINE_SENSOR_RIGHT_IN),
           ADC_GetLineSensorValue(LINE_SENSOR_RIGHT_OUT),
           ADC_GetLineSensorValue(MARKER_SENSOR_RIGHT),
           ADC_GetBatteryVoltage(),
           ENCODER_GetCountLeft(),
           ENCODER_GetCountRight(),
           (int)(current->angle * 1000));
  }
}
