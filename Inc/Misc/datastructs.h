/*
 * datastructs.h
 *
 *  Created on: 5 Apr 2018
 *      Author: Clément Nussbaumer
 */

#ifndef INCLUDE_DATASTRUCTS_H_
#define INCLUDE_DATASTRUCTS_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

typedef float float32_t;
typedef double float64_t;

typedef struct
{
  float32_t x, y, z;
} float3D;

typedef struct
{
  float3D acceleration;
  float3D eulerAngles;
  float32_t temperatureC;
} IMU_data;

typedef struct
{
  float32_t temperature;
  float32_t pressure;
  float32_t altitude;
} BARO_data;

typedef struct
{
  void* ptr;
  uint16_t size;
} Telemetry_Message;

#ifdef __cplusplus
 }
#endif

#endif /* INCLUDE_DATASTRUCTS_H_ */
