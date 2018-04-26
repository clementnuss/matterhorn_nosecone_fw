/*
 * data_handling.c
 *
 *  Created on: 19 Apr 2018
 *      Author: Clément Nussbaumer
 */

#include "cmsis_os.h"

#include "Misc/Common.h"

#ifdef NOSECONE

#include <Misc/data_handling.h>
#include "Telemetry/telemetry_protocol.h"
#include "Telemetry/simpleCRC.h"

extern IMU_data IMU_buffer[];
extern BARO_data BARO_buffer[];

extern osMessageQId xBeeQueueHandle;

uint32_t lastImuSeqNumber = 0, lastBaroSeqNumber = 0, datagramSeqNumber = 0;

uint16_t currPos = 0;
void* currDatagramPtr;

void TK_data (void const * args)
{
  for (;;)
    {

      if (currentImuSeqNumber > lastImuSeqNumber && currentBaroSeqNumber > lastBaroSeqNumber)
        {

          uint32_t measurement_time = HAL_GetTick ();
          IMU_data* imu_data = &IMU_buffer[currentImuSeqNumber % CIRC_BUFFER_SIZE];
          lastImuSeqNumber = currentImuSeqNumber;

          BARO_data* baro_data = &BARO_buffer[currentBaroSeqNumber % CIRC_BUFFER_SIZE];
          lastBaroSeqNumber = currentBaroSeqNumber;

          void* datagramPtr = createTelemetryDatagram (imu_data, baro_data, measurement_time);
          if (datagramPtr != NULL)
            {
              Telemetry_Message m =
                { .ptr = datagramPtr, .size = SENSOR_PACKET_SIZE };
              osMessagePut (xBeeQueueHandle, &m, 50);
            }
        }

      osDelay (15);
    }
}

void* createTelemetryDatagram (IMU_data* imu_data, BARO_data* baro_data, uint32_t measurement_time)
{

  currPos = 0;
  currDatagramPtr = pvPortMalloc (SENSOR_PACKET_SIZE);

  if (currDatagramPtr == NULL)
    {
      return NULL;
    }
  uint16_t datagramCrc = CRC_16_GENERATOR_POLY.initialValue;

  // Beginning of telemetry packet:
  for (size_t i = 0; i < PREAMBLE_SIZE; ++i)
    {
      write8 (HEADER_PREAMBLE_FLAG);
    }

  //DATAGRAM SEQ NUMBER
  write32u (datagramSeqNumber++);

  //PAYLOAD TYPE
  write8 (TELEMETRY_ERT18);

  for (int16_t i = currPos - 5; i < currPos; i++)
    {
      //Calculate checksum for datagram and payload fields
      datagramCrc = CalculateRemainderFromTable (*((uint8_t*) currDatagramPtr + i), datagramCrc);
    }

  //CONTROL FLAG
  write8 (CONTROL_FLAG);

  // ## Beginning of datagram Payload ##
  // measurement time
  write32u (measurement_time);

  write32f (imu_data->acceleration.x);
  write32f (imu_data->acceleration.y);
  write32f (imu_data->acceleration.z);

  write32f (imu_data->eulerAngles.x);
  write32f (imu_data->eulerAngles.y);
  write32f (imu_data->eulerAngles.z);

  write32f (baro_data->temperature);
  write32f (baro_data->pressure);

  for (int16_t i = (PREAMBLE_SIZE + HEADER_SIZE + CONTROL_FLAG_SIZE); i < currPos; i++)
    {
      //Calculate checksum for datagram and payload fields
      datagramCrc = CalculateRemainderFromTable (*((uint8_t*) currDatagramPtr + i), datagramCrc);
    }

  datagramCrc = FinalizeCRC (datagramCrc);

  write16 (datagramCrc);

  return currDatagramPtr;
}

inline void write8 (uint8_t v)
{
  *(uint8_t*) (currDatagramPtr + currPos++) = v;
}

inline void write16 (uint16_t v)
{
  *(uint16_t*) (currDatagramPtr + currPos) = __bswap16(v);
  currPos += 2;

}

inline void write32f (float32_t v)
{
  *(uint32_t*) (currDatagramPtr + currPos) = __bswap32(*((uint32_t* ) &v));
  currPos += 4;
}

inline void write32u (uint32_t v)
{
  *(uint32_t*) (currDatagramPtr + currPos) = __bswap32(v);
  currPos += 4;
}

#endif
