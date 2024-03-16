#include "sensor.h"

// STM32CubeMX
#include <main.h>

/* ADC -----------------------------------------------------------------------*/
//! ADC1 Handler extern
extern ADC_HandleTypeDef hadc1;
//! ADC2 Handler extern
extern ADC_HandleTypeDef hadc2;
//! TIM3 Handler extern
extern TIM_HandleTypeDef htim3;
//! ADC1 sensors
enum
{
  ADC1_LINE_SENSOR_RIGHT_OUT,
  ADC1_MARKER_SENSOR_RIGHT,
  ADC1_MARKER_SENSOR_LEFT,
  NUM_ADC1,
};
//! ADC2 sensors
enum
{
  ADC2_LINE_SENSOR_RIGHT_IN,
  ADC2_LINE_SENSOR_LEFT_IN,
  ADC2_LINE_SENSOR_LEFT_OUT,
  ADC2_BATTERY_VOLTAGE,
  NUM_ADC2,
};
//! ADC1 buffer
static uint16_t dmaBufferAdc1[NUM_ADC1] = {0};
//! ADC2 buffer
static uint16_t dmaBufferAdc2[NUM_ADC2] = {0};
//! ADC line sensor buffer pos
static uint16_t *dmaBufferPosLineSensor[] = {
    [MARKER_SENSOR_RIGHT] = &dmaBufferAdc1[ADC1_MARKER_SENSOR_RIGHT],
    [LINE_SENSOR_RIGHT_OUT] = &dmaBufferAdc1[ADC1_LINE_SENSOR_RIGHT_OUT],
    [LINE_SENSOR_RIGHT_IN] = &dmaBufferAdc2[ADC2_LINE_SENSOR_RIGHT_IN],
    [LINE_SENSOR_LEFT_IN] = &dmaBufferAdc2[ADC2_LINE_SENSOR_LEFT_IN],
    [LINE_SENSOR_LEFT_OUT] = &dmaBufferAdc2[ADC2_LINE_SENSOR_LEFT_OUT],
    [MARKER_SENSOR_LEFT] = &dmaBufferAdc1[ADC1_MARKER_SENSOR_LEFT],
};
/**
 * @brief Initialize ADC
 */
static void ADC_Init()
{
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dmaBufferAdc1, NUM_ADC1);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)dmaBufferAdc2, NUM_ADC2);
  HAL_TIM_Base_Start(&htim3);
}
/**
 * @brief Get battery voltage
 * @return battery voltage[mV]
 */
int ADC_GetBatteryVoltage()
{
  return dmaBufferAdc2[ADC2_BATTERY_VOLTAGE];
}
/**
 * @brief Get line sensor value
 * @return adc value
 */
uint16_t ADC_GetLineSensorValue(LINE_SENSOR_POS pos)
{
  return *dmaBufferPosLineSensor[pos];
}

/* Gyro ----------------------------------------------------------------------*/
#define BMX055_GYRO_I2C_ADDR 0x69
#define BMX055_GYRO_REG_RANGE 0x0f
#define BMX055_GYRO_REG_BW 0x10
#define BMX055_GYRO_REG_LPM1 0x11
#define BMX055_GYRO_REG_RATE_Z_LSB 0x06
//! I2C1 Handler extern
extern I2C_HandleTypeDef hi2c1;
//! i2c1 dma tx buffer
static uint8_t dmaBufferI2c1Tx[2] = {0};
//! i2c1 dma rx buffer
static uint8_t dmaBufferI2c1Rx[2] = {0};
/**
 * @brief Initialize the gyro.
 */
static void GYRO_Init()
{
  // Set measurement ranges of angular rate.
  dmaBufferI2c1Tx[0] = BMX055_GYRO_REG_RANGE;
  dmaBufferI2c1Tx[1] = 0; // 2000dps
  HAL_I2C_Master_Transmit(&hi2c1, (BMX055_GYRO_I2C_ADDR << 1), dmaBufferI2c1Tx, 2, UINT32_MAX);
  // Set ODR and filter bandwidth.
  dmaBufferI2c1Tx[0] = BMX055_GYRO_REG_BW;
  dmaBufferI2c1Tx[1] = 1; // ODR: 2000Hz, Filter Bandwidth: 230Hz
  HAL_I2C_Master_Transmit(&hi2c1, (BMX055_GYRO_I2C_ADDR << 1), dmaBufferI2c1Tx, 2, UINT32_MAX);
  // Set power mode.
  dmaBufferI2c1Tx[0] = BMX055_GYRO_REG_LPM1;
  dmaBufferI2c1Tx[1] = 0; // NORMAL mode
  HAL_I2C_Master_Transmit(&hi2c1, (BMX055_GYRO_I2C_ADDR << 1), dmaBufferI2c1Tx, 2, UINT32_MAX);
}
/**
 * @brief Update an angular rate.
 */
void GYRO_UpdateYaw()
{
  dmaBufferI2c1Tx[0] = BMX055_GYRO_REG_RATE_Z_LSB;
  HAL_I2C_Master_Transmit_DMA(&hi2c1, (BMX055_GYRO_I2C_ADDR << 1), dmaBufferI2c1Tx, 1);
}
/**
 * @brief Get current angular rate.
 */
float GYRO_GetYaw()
{
  float z = (dmaBufferI2c1Rx[1] << 8) | dmaBufferI2c1Rx[0];
  if (z > 32767)
    z -= 65536;
  return z * 0.061037f; // 2000 / 32767
}
/**
 * @brief I2C DMA completed interrupt
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1)
  {
    // read 2 bytes
    HAL_I2C_Master_Receive_DMA(hi2c, (BMX055_GYRO_I2C_ADDR << 1), dmaBufferI2c1Rx, 2);
  }
}

/* Encoder -------------------------------------------------------------------*/
//! TIM2 Handler extern
extern TIM_HandleTypeDef htim2;
//! left encoder count value
static uint16_t encoderLeftCount;
/**
 * @brief Initialize encoders.
 */
static void ENCODER_Init()
{
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}
/**
 * @brief GPIO external interrupt (left encoder)
 */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  GPIO_PinState a = HAL_GPIO_ReadPin(EncoderLeftA_GPIO_Port, EncoderLeftA_Pin);
  GPIO_PinState b = HAL_GPIO_ReadPin(EncoderLeftB_GPIO_Port, EncoderLeftB_Pin);
  switch (pin)
  {
  case EncoderLeftA_Pin:
    if (a != b)
      encoderLeftCount++;
    else
      encoderLeftCount--;
    break;
  case EncoderLeftB_Pin:
    if (a == b)
      encoderLeftCount++;
    else
      encoderLeftCount--;
    break;
  default:
    break;
  }
}
/**
 * @brief Get a value of right encoder.
 */
uint16_t ENCODER_GetCountRight() { return TIM2->CNT; }
/**
 * @brief Get a value of left encoder.
 */
uint16_t ENCODER_GetCountLeft() { return encoderLeftCount; }
/**
 * @brief Reset encoder values.
 */
void ENCODER_ResetCount()
{
  // Right
  TIM2->CNT = 0;
  // Left
  encoderLeftCount = 0;
}

/* Sensor --------------------------------------------------------------------*/
/**
 * @brief Initialize sensors
 */
void SENSOR_Init()
{
  ADC_Init();
  ENCODER_Init();
  GYRO_Init();
}
