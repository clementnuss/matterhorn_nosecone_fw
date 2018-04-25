/*
 * imu.c
 *
 *  Created on: 2 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

#include "stm32f4xx_hal.h"
#include "imu.h"
#if(SIMULATION == 1)
#include <Misc/SimData.h>
#endif
#include <Misc/Common.h>

/*
 * IMU packet format:
 *    function:     PACKET_START    PACKET_COMMAND    [ARGS]      CHECKSUM
 *    size (byte):  1               1                 n           1
 *
 * checksum : sum of all bytes except PACKET_START
 *
 *  ACCELEROMETER RANGE: �6g / �12g / �24g selectable for HH models
 *
 *
 */

SPI_HandleTypeDef* IMU_hspi;
extern SPI_HandleTypeDef hspi1;
extern osSemaphoreId IMU_IntSemHandle;
extern IMU_data IMU_buffer[];

uint32_t SPI_Timeout = 100;
uint32_t SPI_rate_kbpms = 325;

uint8_t TSS_PACKET_START = 0xF6;
uint8_t TSS_SEND_DATA = 0xFF;

uint8_t TSS_SPI_IDLE_STATE = 0x00;
uint8_t TSS_SPI_READY_STATE = 0x01;
uint8_t TSS_SPI_BUSY_STATE = 0x02;
uint8_t TSS_SPI_ACCUM_STATE = 0x04;

uint8_t TSS_ACC_RANGE_ARGS[1] =
  { 1 }; /** ACCELEROMETER RANGE: [0] �6g / [1] �12g / [2] �24g selectable for HH models*/

uint8_t TSS_FILTER_MODE[1] =
  { 1 }; //filter mode: 0 -> disabled; 1 -> Kalman; 2-> QCOMP; 3 QGRAD

uint8_t TSS_PIN_MODE[2] =
  { 1, 0 }; // Pulse mode: 1 - pulse (10us), 2 low until retrieved; pin (0 = TxD pin)

/* Streaming only availble with serial
 uint8_t TSS_STREAMING_TIMING[12] = // interval uint32_t, duration uint32_t, delay (us) uint32_t !!
 { 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xc3, 0x50 };
 uint8_t TSS_STREAMING_SLOTS[8] =
 { TSS_GET_TARED_ORIENTATION_AS_EULER_ANGLES,
 TSS_GET_CORRECTED_ACCELEROMETER_VECTOR, TSS_GET_TEMPERATURE_C, TSS_NULL,
 TSS_NULL, TSS_NULL, TSS_NULL, TSS_NULL };
 uint8_t STREAMING_SIZE = 12 + //Euler angles
 12 + // accelerometer vector
 4; //temperature
 */

#define TSS_COMM_BUFFER_SIZE 100
uint8_t GET_DATA_COMMAND_BUFFER[TSS_COMM_BUFFER_SIZE];
uint8_t rxBuffer[TSS_COMM_BUFFER_SIZE];

/*
 * TODO:
 * add mutex on buffer
 * reset TSS when impossible to communicate
 */

void
TK_IMU (void const * argument)
{

#if(SIMULATION == 1)
  // Save initialization time to synchronize program clock with data
  float32_t initial_sim_time = SimData[0][SIM_TIMESTAMP] - HAL_GetTick ();
  uint32_t sensorCounter = 0;

  IMU_data* newImuDataSet = &IMU_buffer[(currentImuSeqNumber + 1)
      % CIRC_BUFFER_SIZE];
  //populate data
  newImuDataSet->acceleration.x = SimData[sensorCounter][SIM_ACCELX];
  newImuDataSet->acceleration.y = 0;
  newImuDataSet->acceleration.z = 0;
  newImuDataSet->eulerAngles.x = 0;
  newImuDataSet->eulerAngles.y = 0;
  newImuDataSet->eulerAngles.z = 0;
  newImuDataSet->temperatureC = 0;

  currentImuSeqNumber++;
  sensorCounter++;

  for (;;)
    {
      if ((sensorCounter < SIM_TAB_HEIGHT)
	  && ((HAL_GetTick ()
	      - (SimData[sensorCounter][SIM_TIMESTAMP] - initial_sim_time)) > 0))
	{
	  // change sensor data
	  // create artificial sensor structure
	  IMU_data* newImuDataSet = &IMU_buffer[(currentImuSeqNumber + 1)
	      % CIRC_BUFFER_SIZE];
	  //populate data
	  newImuDataSet->acceleration.x = SimData[sensorCounter][SIM_ACCELX];
	  newImuDataSet->acceleration.y = 0;
	  newImuDataSet->acceleration.z = 0;
	  newImuDataSet->eulerAngles.x = 0;
	  newImuDataSet->eulerAngles.y = 0;
	  newImuDataSet->eulerAngles.z = 0;
	  newImuDataSet->temperatureC = 0;

	  //increment counters
	  currentImuSeqNumber++;
	  sensorCounter++;
	}
      osDelay (1);
    }
#endif

#if(SIMULATION == 0)
  IMU_hspi = &hspi1;
  initIMU ();

  //configure interrupt pin
  sendCommand (TSS_SET_PIN_MODE, TSS_PIN_MODE, sizeof(TSS_PIN_MODE), NULL, 0);

  IMU_data* newImuDataSet = &IMU_buffer[0];
  IMU_data empty_data;
  *newImuDataSet = empty_data;
  newImuDataSet->acceleration.x = 14;

  for (;;)
    {
      int32_t ret = osSemaphoreWait (IMU_IntSemHandle, SPI_Timeout);
      if (ret == osOK)
	{
	  IMU_data* newImuDataSet = &IMU_buffer[(currentImuSeqNumber + 1) % CIRC_BUFFER_SIZE];
	  processBuffer (newImuDataSet);

	  currentImuSeqNumber++;
	}
    }
#endif

}

HAL_StatusTypeDef
initIMU ()
{
  IMU_hspi = &hspi1;

  // turn the IMU on.
  HAL_GPIO_WritePin (IMU_ENn_GPIO_Port, IMU_ENn_Pin, RESET);
  HAL_SPI_Init (IMU_hspi);

  osDelay (500);

  sendCommand (TSS_SET_ACCELEROMETER_RANGE, TSS_ACC_RANGE_ARGS,
	       sizeof(TSS_ACC_RANGE_ARGS), NULL, 0);

  /* Takes 45ms to be set. default is already Kalman filter, so no need to set again
   sendCommand (TSS_SET_FILTER_MODE, TSS_FILTER_MODE,
   sizeof(TSS_FILTER_MODE), NULL, 0);
   */

  for (int i = 0; i < sizeof(GET_DATA_COMMAND_BUFFER); i++)
    {
      GET_DATA_COMMAND_BUFFER[i] = TSS_SEND_DATA;
    }

  uint16_t pos = 0;
  insertCommandIntoBuffer (GET_DATA_COMMAND_BUFFER, &pos,
			   TSS_GET_CORRECTED_ACCELEROMETER_VECTOR,
			   3 * sizeof(float) + 10);
  insertCommandIntoBuffer (GET_DATA_COMMAND_BUFFER, &pos,
			   TSS_GET_TARED_ORIENTATION_AS_EULER_ANGLES,
			   3 * sizeof(float) + 35);
  insertCommandIntoBuffer (GET_DATA_COMMAND_BUFFER, &pos, TSS_GET_TEMPERATURE_C,
			   1 * sizeof(float) + 10);

  return osOK;
}

void
HAL_SPI_TxRxCpltCallback (SPI_HandleTypeDef *hspi)
{
  osSemaphoreRelease (IMU_IntSemHandle);
  HAL_SPI_DMAStop (IMU_hspi);
}

HAL_StatusTypeDef
getIMUdataDMA ()
{
  HAL_SPI_DMAResume (IMU_hspi);
  return HAL_SPI_TransmitReceive_DMA (IMU_hspi, GET_DATA_COMMAND_BUFFER,
				      rxBuffer, TSS_COMM_BUFFER_SIZE);
}

void
processBuffer (IMU_data* tss_data)
{
  uint16_t pos = 0;
  while (pos < TSS_COMM_BUFFER_SIZE)
    {
      if (GET_DATA_COMMAND_BUFFER[pos++] == TSS_PACKET_START)
	{
	  uint8_t COMMAND = GET_DATA_COMMAND_BUFFER[pos++];
	  increasePosUntilTssReady (&pos);
	  switch (COMMAND)
	    {
	    case TSS_GET_CORRECTED_ACCELEROMETER_VECTOR:
	      {
		uint8ToFloat ((uint8_t*) &rxBuffer[pos + 0 * sizeof(float)],
			      &(&tss_data->acceleration)->x);
		uint8ToFloat ((uint8_t*) &rxBuffer[pos + 1 * sizeof(float)],
			      &(&tss_data->acceleration)->y);
		uint8ToFloat ((uint8_t*) &rxBuffer[pos + 2 * sizeof(float)],
			      &(&tss_data->acceleration)->z);
	      }
	      break;
	    case TSS_GET_TARED_ORIENTATION_AS_EULER_ANGLES:
	      {
		uint8ToFloat ((uint8_t*) &rxBuffer[pos + 0 * sizeof(float)],
			      &(&(tss_data->eulerAngles))->y);
		uint8ToFloat ((uint8_t*) &rxBuffer[pos + 1 * sizeof(float)],
			      &(&(tss_data->eulerAngles))->x);
		uint8ToFloat ((uint8_t*) &rxBuffer[pos + 2 * sizeof(float)],
			      &(&(tss_data->eulerAngles))->z);
	      }
	    case TSS_GET_TEMPERATURE_C:
	      {
		uint8ToFloat ((uint8_t*) &rxBuffer[pos + 0 * sizeof(float)],
			      &tss_data->temperatureC);
	      }
	    default:
	      break;
	    }
	}
    }
}

void
increasePosUntilTssReady (uint16_t*pos)
{
  while (rxBuffer[(*pos)++] != TSS_SPI_READY_STATE
      && (*pos < TSS_COMM_BUFFER_SIZE))
    ;
}

HAL_StatusTypeDef
getCorrectedAccelerometerVector (float3D* data)
{
  uint8_t accVectors[3 * sizeof(float)];
  sendCommand (TSS_GET_CORRECTED_ACCELEROMETER_VECTOR, NULL, 0, accVectors,
	       sizeof(accVectors));

  uint8ToFloat ((uint8_t*) &accVectors[0 * sizeof(float)], &data->x);
  uint8ToFloat ((uint8_t*) &accVectors[1 * sizeof(float)], &data->y);
  uint8ToFloat ((uint8_t*) &accVectors[2 * sizeof(float)], &data->z);

  return HAL_OK;
}

HAL_StatusTypeDef
getUntaredEulerAngles (float3D* data)
{
  uint8_t accVectors[3 * sizeof(float)];
  sendCommand (TSS_GET_UNTARED_ORIENTATION_AS_EULER_ANGLES, NULL, 0, accVectors,
	       sizeof(accVectors));

  uint8ToFloat ((uint8_t*) &accVectors[0 * sizeof(float)], &data->y);
  uint8ToFloat ((uint8_t*) &accVectors[1 * sizeof(float)], &data->x);
  uint8ToFloat ((uint8_t*) &accVectors[2 * sizeof(float)], &data->z);
  return HAL_OK;
}

/**
 * Wait for the TSS SPI state. Will send "CLEAR_BUFFER" (0x00) if waiting to be IDLE, and "SEND_DATA" (0xFF) otherwise.
 */
HAL_StatusTypeDef
waitTssState (uint32_t timeoutmMs, uint8_t tss_spi_state)
{
  uint32_t tickstart = HAL_GetTick ();
  uint8_t rxByte = 0x14;
  uint8_t txByte = TSS_SEND_DATA;

  while ((rxByte != tss_spi_state)
      && ((HAL_GetTick () - tickstart) < timeoutmMs))
    {
      HAL_SPI_TransmitReceive (IMU_hspi, &txByte, &rxByte, 1, 1);
    }
  return (rxByte == tss_spi_state) ? HAL_OK : HAL_TIMEOUT;
}

HAL_StatusTypeDef
sendCommand (uint8_t command, uint8_t* args, uint8_t argsSize, uint8_t* rxData,
	     uint16_t rxSize)
{

  HAL_SPI_Transmit (IMU_hspi, &TSS_PACKET_START, 1, 1);
  HAL_SPI_Transmit (IMU_hspi, &command, 1, 1);

  if (args != NULL && argsSize != 0)
    {
      uint32_t timeout = (8 * argsSize) / SPI_rate_kbpms + 1;
      HAL_SPI_Transmit (IMU_hspi, args, argsSize, timeout);
    }
  if (waitTssState (SPI_Timeout, TSS_SPI_READY_STATE) != HAL_OK)
    {
      return HAL_ERROR;
    }

  if (rxData != NULL && rxSize != 0)
    {

      /** fill the receive buffer with 0xFF, which will be sent when transmitting the data*/
      for (uint16_t i = 0; i < rxSize; i++)
	{
	  rxData[i] = TSS_SEND_DATA;
	}
      uint32_t timeout = (8 * rxSize) / SPI_rate_kbpms + 1;
      HAL_SPI_Receive (IMU_hspi, rxData, rxSize, timeout);
    }
  return HAL_OK;
}

void
insertCommandIntoBuffer (uint8_t* commandBuffer, uint16_t* pos,
			 uint8_t tss_command, uint16_t tss_send_packet_number)
{

  commandBuffer[(*pos)++] = TSS_PACKET_START;
  commandBuffer[(*pos)++] = tss_command;
  *pos += tss_send_packet_number;
}

