/*
 * abs_p.c
 *
 *  Created on: 17 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

/*
 * Code based on this library: https://github.com/freetronics/BaroSensor/blob/master/BaroSensor.cpp
 */

#include <Sensors/pressure_sensors.h>
#include "stm32f4xx_hal.h"
#include "Misc/Common.h"
#if(SIMULATION == 1)
#include <Misc/SimData.h>
#endif
#include <Misc/rocket_constants.h>

extern I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef* hi2c;
extern osSemaphoreId pressureSensorI2cSemHandle;
extern BARO_data BARO_buffer[];
extern float32_t PITOT_buffer[];
extern volatile float32_t high_range_pressure;

/* i2c address of module */
#define BAROMETER_ADDR 0xEC
#define BAROMETER_OSR 3
#define I2C_TIMEOUT 5 //ms

#define MAX_TEMPERATURE 85
#define MIN_TEMPERATURE -40

#define MAX_PRESSURE 1200
#define MIN_PRESSURE 100

/* delay to wait for sampling to complete, on each OSR level */
const uint8_t SamplingDelayMs[6] =
  { 1, 2, 3, 5, 9, 17 };
uint16_t PROM_DATA[7]; // the barometer contains calibration data (from the factory), and we load them in this array when starting the barometer
uint8_t rxBuffer[5];

/* module commands */
#define BAROMETER_RESET 0x1E
#define CMD_PROM_READ(offset) (0xA0 + (offset << 1)) /* Offset 0-7 */
#define CMD_START_D1(oversample_level) (0x40 + 2*(int)oversample_level)
#define CMD_START_D2(oversample_level) (0x50 + 2*(int)oversample_level)
#define CMD_READ_ADC 0x00

void initBarometer ()
{
  hi2c = &hi2c2;

  HAL_GPIO_WritePin (ABS_P_SENS_ENn_GPIO_Port, ABS_P_SENS_ENn_Pin, RESET);
  HAL_GPIO_WritePin (DIF_P_SENS_ENn_GPIO_Port, DIF_P_SENS_ENn_Pin, RESET);
  HAL_Delay (40);

  uint8_t command = BAROMETER_RESET;
  HAL_I2C_Master_Transmit (hi2c, BAROMETER_ADDR, &command, 1, 5);
  HAL_Delay (5);

  for (int i = 0; i < 7; i++)
    {
      command = CMD_PROM_READ(i);
      HAL_I2C_Master_Transmit (hi2c, BAROMETER_ADDR, &command, 1, 5);
      HAL_I2C_Master_Receive (hi2c, BAROMETER_ADDR, rxBuffer, 2, 5);

      PROM_DATA[i] = rxBuffer[0] << 8 | rxBuffer[1];
    }

  //testAbsPressure ();
}

uint32_t failedReading = 0;

extern int startSimulation;

void TK_fetchBarometer ()
{
#if(SIMULATION == 1)

  while (!startSimulation)
    {
      osDelay (10);
    }

  // Save initialization time to synchronize program clock with data
  float32_t initial_sim_time = SimData[0][SIM_TIMESTAMP] - HAL_GetTick ();
  uint32_t sensorCounter = 0;

  // Populate first sensor structure
  BARO_data* newBaroData = &BARO_buffer[(currentBaroSeqNumber + 1) % CIRC_BUFFER_SIZE];
  newBaroData->temperature = 0;
  newBaroData->pressure = 0;
  newBaroData->altitude = SimData[sensorCounter][SIM_ALTITUDE];

  PITOT_buffer[(currentPitotSeqNumber + 1) % CIRC_BUFFER_SIZE] = 0.0f;
  currentBaroSeqNumber++;
  sensorCounter++;
  currentPitotSeqNumber++;

  for (;;)
    {
      if ((sensorCounter < SIM_TAB_HEIGHT - 1)
          && ((HAL_GetTick () - (SimData[sensorCounter][SIM_TIMESTAMP] - initial_sim_time)) > 0))
        {
          // change sensor data
          // create artificial sensor structure
          BARO_data* newBaroData = &BARO_buffer[(currentBaroSeqNumber + 1) % CIRC_BUFFER_SIZE];
          //populate data
          newBaroData->temperature = 0;
          newBaroData->pressure = SimData[sensorCounter][SIM_PRESSURE];
          newBaroData->altitude = SimData[sensorCounter][SIM_ALTITUDE];

          PITOT_buffer[(currentPitotSeqNumber++) % CIRC_BUFFER_SIZE] = SimData[sensorCounter][SIM_VELOCITYX];
          //increment counters
          currentBaroSeqNumber++;
          currentBaroTimestamp = HAL_GetTick();

          sensorCounter++;
        }
      osDelay (SimData[sensorCounter][SIM_TIMESTAMP] - SimData[sensorCounter - 1][SIM_TIMESTAMP] - 1);
    }

#endif

#if(SIMULATION == 0)
  uint32_t samplingStart, elapsed;
  osSemaphoreWait (pressureSensorI2cSemHandle, 1); // make sure the semaphore is taken at the start of the loop

  for (;;)
    {

      i2cTransmitCommand (CMD_START_D2(BAROMETER_OSR));
      samplingStart = HAL_GetTick ();
      // D2 command started, we need to wait a few milliseconds

      uint8_t pitot_buffer_high_range[2];
      i2cReceive (pitot_buffer_high_range, 2, DIFFERENTIAL_PRESSURE_SENSOR_HIGH_RANGE_ADDRESS);
      uint16_t high_range_output = (pitot_buffer_high_range[0] << 8 | pitot_buffer_high_range[1])
          & DIFF_PRESS_OUTPUT_MASK;

      uint8_t pitot_buffer_low_range[2];
      i2cReceive (pitot_buffer_low_range, 2, DIFFERENTIAL_PRESSURE_SENSOR_LOW_RANGE_ADDRESS);
      uint16_t low_range_output = (pitot_buffer_low_range[0] << 8 | pitot_buffer_low_range[1]) & DIFF_PRESS_OUTPUT_MASK;

      float32_t low_range_pressure = (low_range_output - DIFF_PRESS_OUTPUT_MIN)
          * ( DIFF_PRESS_LOW_RANGE_P_MAX - DIFF_PRESS_LOW_RANGE_P_MIN) / DIFF_PRESS_OUTPUT_RANGE +
      DIFF_PRESS_LOW_RANGE_P_MIN;

      float32_t high_range_pressure = (high_range_output - DIFF_PRESS_OUTPUT_MIN)
          * ( DIFF_PRESS_HIGH_RANGE_P_MAX - DIFF_PRESS_HIGH_RANGE_P_MIN) / DIFF_PRESS_OUTPUT_RANGE +
      DIFF_PRESS_HIGH_RANGE_P_MIN;

      float32_t pitot_pressure = PSI_TO_PASCAL_CONVERSION_FACTOR * low_range_pressure;

      PITOT_buffer[(currentPitotSeqNumber + 1) % CIRC_BUFFER_SIZE] = pitot_pressure;
      currentPitotSeqNumber++;

      if ((elapsed = (HAL_GetTick () - samplingStart)) < SamplingDelayMs[BAROMETER_OSR])
        {
          osDelay (SamplingDelayMs[BAROMETER_OSR] - elapsed);
        }

      i2cTransmitCommand (CMD_READ_ADC);
      delayUs (40);
      i2cReceive (rxBuffer, 3, BAROMETER_ADDR);
      if (rxBuffer[0] == 0)
        {
          continue;
        }
      uint32_t d2 = rxBuffer[0] << 16 | rxBuffer[1] << 8 | rxBuffer[2];

      i2cTransmitCommand (CMD_START_D1(BAROMETER_OSR));
      samplingStart = HAL_GetTick ();
      if ((elapsed = (HAL_GetTick () - samplingStart)) < SamplingDelayMs[BAROMETER_OSR])
        {
          osDelay (SamplingDelayMs[BAROMETER_OSR] - elapsed);
        }

      i2cTransmitCommand (CMD_READ_ADC);
      delayUs (10);
      i2cReceive (rxBuffer, 3, BAROMETER_ADDR);
      if (rxBuffer[0] == 0)
        {
          continue;
        }
      uint32_t d1 = rxBuffer[0] << 16 | rxBuffer[1] << 8 | rxBuffer[2];

      BARO_data* newBaroData = &BARO_buffer[(currentBaroSeqNumber + 1) % CIRC_BUFFER_SIZE];
      if (processD1D2 (d1, d2, newBaroData) == osOK)
        {
          currentBaroSeqNumber++;
          currentBaroTimestamp = HAL_GetTick();
        }
      else
        {
          failedReading++;
          //TODO: reset the barometer
        }

    }

#endif
}

float32_t interpolatePitotReadings (float32_t low_range_pressure, float32_t high_range_pressure)
{
  if (low_range_pressure < 0.95 && high_range_pressure < 1.0)
    {
      return low_range_pressure;
    }
  else
    {
      return high_range_pressure;
    }
}

osStatus processD1D2 (uint32_t d1, uint32_t d2, BARO_data* ret)
{
  int64_t dt = d2 - PROM_DATA[5] * (1L << 8);
  int32_t temp = 2000 + (dt * PROM_DATA[6]) / (1L << 23);

  /* Second order temperature compensation */
  int64_t t2;
  if (temp >= 2000)
    {
      /* High temperature */
      t2 = 5 * (dt * dt) / (1LL << 38);
    }
  else
    {
      /* Low temperature */
      t2 = 3 * (dt * dt) / (1LL << 33);
    }

  ret->temperature = (float32_t) (temp - t2) / 100.0f;

  int64_t off = PROM_DATA[2] * (1LL << 17) + (PROM_DATA[4] * dt) / (1LL << 6);
  int64_t sens = PROM_DATA[1] * (1LL << 16) + (PROM_DATA[3] * dt) / (1LL << 7);

  /* Second order temperature compensation for pressure */
  if (temp < 2000)
    {
      /* Low temperature */
      int32_t tx = temp - 2000;
      tx *= tx;
      int32_t off2 = 61 * tx / (1 << 4);
      int32_t sens2 = 29 * tx / (1 << 4);
      if (temp < -1500)
        {
          /* Very low temperature */
          tx = temp + 1500;
          tx *= tx;
          off2 += 17 * tx;
          sens2 += 9 * tx;
        }
      off -= off2;
      sens -= sens2;
    }

  int32_t p = ((int64_t) d1 * sens / (1LL << 21) - off) / (1LL << 15);
  ret->pressure = (float32_t) p / 100.0; //pressure in mbar (=hPa)

  ret->altitude = altitudeFromPressure (ret->pressure);

  if ((ret->temperature > MAX_TEMPERATURE) | (ret->temperature < MIN_TEMPERATURE) | (ret->pressure > MAX_PRESSURE)
      | (ret->pressure < MIN_PRESSURE))
    {
      return osErrorOS;
    }

  else
    {
      return osOK;
    }

}

inline float altitudeFromPressure (float pressure_hPa)
{
  float altitude = 44330 * (1.0 - pow (pressure_hPa / ADJUSTED_SEA_LEVEL_PRESSURE, 0.1903));

  if (altitude < 100 || altitude > 5000)
    {
      return 0;
    }
  else
    {
      return altitude;
    }
}

osStatus i2cReceive (uint8_t* rxBuffer, uint16_t size, uint8_t device_address)
{
  HAL_I2C_Master_Receive_DMA (hi2c, device_address, rxBuffer, size);
  return osSemaphoreWait (pressureSensorI2cSemHandle, I2C_TIMEOUT);
}

osStatus i2cTransmitCommand (uint8_t command)
{
  HAL_I2C_Master_Transmit_DMA (hi2c, BAROMETER_ADDR, &command, 1);
  return osSemaphoreWait (pressureSensorI2cSemHandle, I2C_TIMEOUT);
}

void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef *hi2c)
{
  osSemaphoreRelease (pressureSensorI2cSemHandle);
}

void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef *hi2c)
{
  osSemaphoreRelease (pressureSensorI2cSemHandle);
}

void HAL_I2C_ErrorCallback (I2C_HandleTypeDef *hi2c)
{
//  osSemaphoreRelease (pressureSensorI2cSemHandle);
}

void HAL_I2C_AbortCpltCallback (I2C_HandleTypeDef *hi2c)
{
  //osSemaphoreRelease (pressureSensorI2cSemHandle);
}

void testAbsPressure ()
{
  uint8_t diff[6];
  for (int i = 0; i < 15; i++)
    {

      HAL_I2C_Master_Receive (hi2c, 0x68, diff, 6, 5);

    }

  taskENTER_CRITICAL();
  uint8_t gage[6];
  HAL_I2C_Master_Receive (hi2c, 0x58, gage, 6, 5);

  HAL_I2C_Master_Transmit (hi2c, 0x28, diff, 1, 5);
  HAL_I2C_Master_Transmit (hi2c, 0x58, diff, 1, 5);

  HAL_I2C_Master_Receive (hi2c, 0x58, gage, 6, 5);

  uint8_t command = CMD_START_D2(5);
  HAL_I2C_Master_Transmit (hi2c, BAROMETER_ADDR, &command, 1, 5);
  HAL_Delay (SamplingDelayMs[5]);

  command = CMD_READ_ADC;
  HAL_I2C_Master_Transmit (hi2c, BAROMETER_ADDR, &command, 1, 5);
  HAL_I2C_Master_Receive (hi2c, BAROMETER_ADDR, rxBuffer, 5, 5);

  uint32_t d2 = rxBuffer[0] << 16 | rxBuffer[1] << 8 | rxBuffer[2];

  int32_t dT = d2 - (PROM_DATA[5] << 8);
  int32_t temp = 2000 + (dT * PROM_DATA[6] >> 23);
}

