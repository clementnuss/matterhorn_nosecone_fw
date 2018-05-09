/*
 * gps.h
 *
 *  Created on: 07 May 2018
 *      Author: Clément Nussbaumer
 */

#ifndef SENSORS_GPS_H_
#define SENSORS_GPS_H_

#include <Misc/datastructs.h>

#ifdef __cplusplus
extern "C"
{
#endif

  void TK_GPS (void const * argument);

  void GPS_RxCpltCallback ();

#ifdef __cplusplus
}
#endif

Telemetry_Message createGPSDatagram (uint32_t seqNumber);

#endif /* SENSORS_PRESSURESENSORS_BAROMETER_H_ */
