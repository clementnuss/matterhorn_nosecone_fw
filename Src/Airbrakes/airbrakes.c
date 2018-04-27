/*
 * airbrakes.c
 *
 *  Created on: 27 Apr 2018
 *      Author: ebrunner
 */

#include "stm32f4xx_hal.h"
#include "Misc/Common.h"

#ifdef CENTRALBODY

#include "Airbrakes/controller_functions.h"

void
TK_ab_controller(void const * argument)
{

    aerobrakes_control_init();
    aerobrake_helloworld();

    command_aerobrake_controller(1500.0, 100.0); // Should be full open
    command_aerobrake_controller(200.0, 100.0); // Should be full close
    command_aerobrake_controller(238.0, 131.0); // Should be somewhere in between
    command_aerobrake_controller(500.0, 80.0); // Should be full close

    for(;;)
      {
	osDelay(100);
      }
}
#endif


