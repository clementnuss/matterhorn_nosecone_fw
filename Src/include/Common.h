/*
 * Common.h
 *
 *  Created on: 4 Apr 2018
 *      Author: Clément Nussbaumer
 */

#ifndef INCLUDE_COMMON_H_
#define INCLUDE_COMMON_H_

#include "datastructs.h"

#define CIRC_BUFFER_SIZE 8

volatile uint32_t currentImuSeqNumber;
TSS_data IMU_buffer[CIRC_BUFFER_SIZE];

inline void uint8ToFloat (uint8_t* uint8Ptr, float* floatPtr)
{
  uint8_t* floatAsUintPtr = (uint8_t*) floatPtr;
  floatAsUintPtr[0] = uint8Ptr[3];
  floatAsUintPtr[1] = uint8Ptr[2];
  floatAsUintPtr[2] = uint8Ptr[1];
  floatAsUintPtr[3] = uint8Ptr[0];
}

inline void floatToUint8 (uint8_t* uint8Ptr, float* floatPtr)
{
  uint8_t* floatAsUintPtr = (uint8_t*) floatPtr;
  uint8Ptr[0] = floatAsUintPtr[3];
  uint8Ptr[1] = floatAsUintPtr[2];
  uint8Ptr[2] = floatAsUintPtr[1];
  uint8Ptr[3] = floatAsUintPtr[0];
}
#endif /* INCLUDE_COMMON_H_ */
