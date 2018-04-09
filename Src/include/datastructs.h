/*
 * datastructs.h
 *
 *  Created on: 5 Apr 2018
 *      Author: Clément Nussbaumer
 */

#ifndef INCLUDE_DATASTRUCTS_H_
#define INCLUDE_DATASTRUCTS_H_

typedef struct
{
  float x, y, z;
} float3D;

typedef struct
{
  float3D acceleration;
  float3D eulerAngles;
  float temperatureC;
} TSS_data;

#endif /* INCLUDE_DATASTRUCTS_H_ */
