/*
 * airbrakes.c
 *
 *  Created on: 27 Apr 2018
 *      Author: ebrunner
 */

#include "stm32f4xx_hal.h"
#include "Misc/Common.h"
#include <cmsis_os.h>

#include "Airbrakes/controller_functions.h"

extern UART_HandleTypeDef* airbrake_huart;

extern volatile uint32_t flight_status;

osStatus motorDetected = osErrorOS;
uint8_t faulhaber_welcome[9];

void TK_ab_controller (void const * argument)
{
  airbrakes_angle = 0;
  flight_status = 0;
  uint32_t lastAirbrakesUpdate = 0;

#if (SIMULATION == 0)

  for (;;)
    {
      if (LIFTOFF_TIME != 0) {
          aerobrakes_control_init();
          goto control;
          break;
      }

      HAL_StatusTypeDef ret = HAL_UART_Receive (airbrake_huart, faulhaber_welcome, 9, 5000);
      if (ret == HAL_TIMEOUT)
        {
          continue;
        }
      int comparison = strcmp (faulhaber_welcome, "FAULHABER");
      if (comparison == 0)
        {
          motorDetected = osOK;
          flight_status = 5;
          break;
        }
      else
        {
          osDelay (100);
        }

    }
  osDelay (2000);

  aerobrakes_control_init ();

  for (int i = 0; i < 5; ++i)
    {
      aerobrake_helloworld ();
      osDelay (500);
    }

  osDelay (1000);

  /*
   command_aerobrake_controller (1500.0, 100.0); // Should be full open
   command_aerobrake_controller (200.0, 100.0); // Should be full close
   command_aerobrake_controller (238.0, 131.0); // Should be somewhere in between
   command_aerobrake_controller (500.0, 80.0); // Should be full close
   */

#endif

  while (LIFTOFF_TIME == 0)
    {
      osDelay (10);
    }

control:
  for (;;)
    {

      if ((HAL_GetTick () - LIFTOFF_TIME) > 17000)
        {
          full_close ();
          break;
        }

      if (currentState == STATE_COAST)
        {
          command_aerobrake_controller (altitude_estimate, air_speed_state_estimate); // Should be full close
        }
      else if (currentState > STATE_PRIMARY)
        {
          full_close ();
          break;
        }

      int elapsed = HAL_GetTick () - lastAirbrakesUpdate;
      if (elapsed < 100)
        {
          osDelay (100 - elapsed);
        }
      lastAirbrakesUpdate = HAL_GetTick ();
    }

  airbrakes_angle = -1;

  for (;;)
    {
      osDelay(10000);
    }
}

void airbrake_rxCpltCallback ()
{
  motorDetected = osErrorResource;
}
