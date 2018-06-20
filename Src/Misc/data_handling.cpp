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
#include <Misc/datagram_builder.h>

#include "Telemetry/telemetry_protocol.h"
#include "Telemetry/simpleCRC.h"

extern IMU_data IMU_buffer[];
extern BARO_data BARO_buffer[];
extern float32_t PITOT_buffer[];

extern osMessageQId xBeeQueueHandle;

extern int startSimulation;
extern volatile float32_t high_range_pressure;

void TK_data (void const * args)
{

  uint32_t lastImuSeqNumber = 0, lastBaroSeqNumber = 0, telemetrySeqNumber = 0;

  osDelay (800); //Wait for the first values to be written by the IMU and barometer.

  for (;;)
    {
      uint32_t measurement_time = HAL_GetTick ();

      IMU_data* imu_data = &IMU_buffer[currentImuSeqNumber % CIRC_BUFFER_SIZE];
      BARO_data* baro_data = &BARO_buffer[currentBaroSeqNumber % CIRC_BUFFER_SIZE];
      float32_t pitot_press = PITOT_buffer[currentPitotSeqNumber % CIRC_BUFFER_SIZE];

      lastImuSeqNumber = currentImuSeqNumber;
      lastBaroSeqNumber = currentBaroSeqNumber;

      Telemetry_Message m = createTelemetryDatagram (imu_data, baro_data, pitot_press, measurement_time,
                                                     telemetrySeqNumber++);
      osMessagePut (xBeeQueueHandle, (uint32_t) &m, 50);

      osDelay (15 - (HAL_GetTick () - measurement_time));
    }
}

Telemetry_Message createTelemetryDatagram (IMU_data* imu_data, BARO_data* baro_data, float32_t pitot_press,
                                           uint32_t measurement_time, uint32_t telemetrySeqNumber)
{

  DatagramBuilder builder = DatagramBuilder (SENSOR_DATAGRAM_PAYLOAD_SIZE, TELEMETRY_ERT18, telemetrySeqNumber);

  // ## Beginning of datagram Payload ##
  // measurement time
  builder.write32<uint32_t> (measurement_time);

  builder.write32<float32_t> (imu_data->acceleration.x);
  builder.write32<float32_t> (imu_data->acceleration.y);
  builder.write32<float32_t> (imu_data->acceleration.z);

  builder.write32<float32_t> (flight_status);
  builder.write32<float32_t> (airbrakes_angle);
  builder.write32<float32_t> (pitot_press);

  builder.write32<float32_t> (baro_data->temperature);
  builder.write32<float32_t> (baro_data->pressure);

  builder.write32<float32_t> (air_speed_state_estimate);

  return builder.finalizeDatagram ();
}

#endif
