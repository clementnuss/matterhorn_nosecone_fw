/*
 * lower_stage_iface.c
 *
 *  Created on: 22 Apr 2018
 *      Author: Clément Nussbaumer
 */

#include "stm32f4xx_hal.h"


#include <Misc/Common.h>
#include <Misc/lower_stage_iface.h>

#ifdef NOSECONE

extern UART_HandleTypeDef huart4;

extern int startSimulation;

void transmitFirstEvent ()
{

  uint8_t firstEventTrigger[] =
    { 0x55, 0x55, 0x14, 0x14 };

  HAL_UART_Transmit_DMA (&huart4, firstEventTrigger, sizeof(firstEventTrigger));

  startSimulation = 1;
}

void transmitSecondEvent ()
{
  uint8_t firstEventTrigger[] =
    { 0x55, 0x55, 0x22, 0x22 };

  HAL_UART_Transmit_DMA (&huart4, firstEventTrigger, sizeof(firstEventTrigger));
}

#endif
