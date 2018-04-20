/*
 * Common.h
 *
 *  Created on: 4 Apr 2018
 *      Author: Clément Nussbaumer
 */

#ifndef INCLUDE_COMMON_H_
#define INCLUDE_COMMON_H_

#include "datastructs.h"
#include <stm32f4xx_hal.h>

#define CIRC_BUFFER_SIZE 8
#define CCMRAM __attribute__((section(".ccmram")))

extern TIM_HandleTypeDef htim7;

/*
 * heap file, line 104, declare the RTOS heap in CCMRAM
 * static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ] __attribute__((section(".ccmram")));
 */

volatile uint32_t currentImuSeqNumber;
volatile uint32_t currentBaroSeqNumber;

static inline void uint8ToFloat (uint8_t* uint8Ptr, float* floatPtr)
{
  uint8_t* floatAsUintPtr = (uint8_t*) floatPtr;
  floatAsUintPtr[0] = uint8Ptr[3];
  floatAsUintPtr[1] = uint8Ptr[2];
  floatAsUintPtr[2] = uint8Ptr[1];
  floatAsUintPtr[3] = uint8Ptr[0];
}

static inline int32_t mod (int32_t x, int32_t n)
{
  int32_t r = x % n;
  return r < 0 ? r + n : r;
}

//TODO: use a timer with a smaller period
static inline void delayUs (uint32_t delay)
{
  uint32_t start = __HAL_TIM_GET_COUNTER(&htim7);
  int32_t elapsed;

  while ((elapsed = mod (((int32_t) __HAL_TIM_GET_COUNTER(&htim7) - start), htim7.Init.Period) < delay))
    ;
}

static inline void floatToUint8 (uint8_t* uint8Ptr, float* floatPtr)
{
  uint8_t* floatAsUintPtr = (uint8_t*) floatPtr;
  uint8Ptr[0] = floatAsUintPtr[3];
  uint8Ptr[1] = floatAsUintPtr[2];
  uint8Ptr[2] = floatAsUintPtr[1];
  uint8Ptr[3] = floatAsUintPtr[0];
}
#endif /* INCLUDE_COMMON_H_ */
