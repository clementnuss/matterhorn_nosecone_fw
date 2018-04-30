/*
 * airbrakes.c
 *
 *  Created on: 27 Apr 2018
 *      Author: ebrunner
 */

#include "stm32f4xx_hal.h"
#include "Misc/Common.h"
#include <cmsis_os.h>

#ifdef CENTRALBODY

#include "Airbrakes/controller_functions.h"

extern float32_t kalman_air_speed, kalman_altitude;

void TK_ab_controller (void const * argument)
{

  uint32_t lastAirbrakesUpdate = 0;

  aerobrakes_control_init ();
  aerobrake_helloworld ();

  /*
  command_aerobrake_controller (1500.0, 100.0); // Should be full open
  command_aerobrake_controller (200.0, 100.0); // Should be full close
  command_aerobrake_controller (238.0, 131.0); // Should be somewhere in between
  command_aerobrake_controller (500.0, 80.0); // Should be full close
  */

  for (;;)
    {

      if (currentState == STATE_COAST) {
          command_aerobrake_controller (kalman_altitude, kalman_air_speed); // Should be full close
      } else if (currentState == STATE_PRIMARY) {
          full_close();
      }


      int elapsed = HAL_GetTick() - lastAirbrakesUpdate;
      if (100 - elapsed > 0) {
          osDelay(100 - elapsed);
      }
      lastAirbrakesUpdate = HAL_GetTick();
    }
}

#endif
