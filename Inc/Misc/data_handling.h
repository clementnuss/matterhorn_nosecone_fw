/*
 * data_handling.h
 *
 *  Created on: 19 Apr 2018
 *      Author: Clément Nussbaumer
 */

#include <Misc/datastructs.h>

#ifndef DATA_HANDLING_H_
#define DATA_HANDLING_H_

#ifdef __cplusplus
extern "C"
{
#endif

  void TK_data (const void* args);

  Telemetry_Message createTelemetryDatagram (IMU_data* imu_data, BARO_data* baro_data, float32_t air_speed,
                                             uint32_t measurement_time, uint32_t telemetrySeqNumber);

#ifdef __cplusplus
}
#endif

#endif //DATA_HANDLING_H_
