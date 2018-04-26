/*
 * physical_iface.c
 *
 *  Created on: 25 Apr 2018
 *      Author: Clément Nussbaumer
 */

#include <cmsis_os.h>
#include <stm32f4xx_hal.h>

#include <Misc/Common.h>

#ifdef CENTRALBODY

#include <Misc/physical_iface.h>

uint32_t buzzerStartTime, firstEventStartTime, secondEventStartTime;
uint32_t buzzerDelayMs = 100;

void TK_physical_iface (void const * argument)
{
  uint32_t currentTime;

  for (;;)
    {
      currentTime = HAL_GetTick ();

      if ((buzzerStartTime != 0) && (currentTime - buzzerStartTime) > buzzerDelayMs)
        {
          HAL_GPIO_WritePin (BUZZER_GPIO_Port, BUZZER_Pin, RESET);
          buzzerStartTime = 0;
        }

      if ((firstEventStartTime != 0) && (currentTime - firstEventStartTime) > IGNITION_DURATION)
        {
          HAL_GPIO_WritePin (REC_1ST_EVENT_GPIO_Port, REC_1ST_EVENT_Pin, RESET);
          firstEventStartTime = 0;
        }

      if ((secondEventStartTime != 0) && (currentTime - secondEventStartTime) > IGNITION_DURATION)
        {
          HAL_GPIO_WritePin (REC_2ND_EVENT_GPIO_Port, REC_2ND_EVENT_Pin, RESET);
          secondEventStartTime = 0;
        }

      osDelay (1);
    }

}

void shortBip ()
{
  buzzerStartTime = HAL_GetTick ();
  HAL_GPIO_WritePin (BUZZER_GPIO_Port, BUZZER_Pin, SET);
  buzzerDelayMs = 300;
}

void longBip ()
{
  buzzerStartTime = HAL_GetTick ();
  buzzerDelayMs = 1000;
  HAL_GPIO_WritePin (BUZZER_GPIO_Port, BUZZER_Pin, SET);
}

void triggerFirstEvent ()
{
  firstEventStartTime = HAL_GetTick ();
  HAL_GPIO_WritePin (REC_1ST_EVENT_GPIO_Port, REC_1ST_EVENT_Pin, SET);
}

void triggerSecondEvent ()
{
  secondEventStartTime = HAL_GetTick ();
  HAL_GPIO_WritePin (REC_2ND_EVENT_GPIO_Port, REC_2ND_EVENT_Pin, SET);
}

#endif
