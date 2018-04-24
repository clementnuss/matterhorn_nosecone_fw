/*
 * state_machine.c
 *
 *  Created on: 24 Apr 2018
 *      Author: ebrunner
 */

#include "stm32f4xx_hal.h"
#include "Misc/Common.h"
#include "Misc/rocket_constants.h"

// Apogee measurements delay
#define APOGEE_BUFFER_SIZE 5 // Number
#define APOGEE_ALT_DIFF 2

void
TK_state_machine (void const * argument)
{

  // Declare time variable
  uint32_t time_tmp = 0;

  // Declare sensor variables
  IMU_data* imu_data;
  BARO_data* baro_data;

  // Initial calibration commands
  baro_data = getCurrentBARO_data ();
  calib_initial_altitude = baro_data->altitude;

  // Declare apogee detection variables
  uint32_t max_altitude = calib_initial_altitude;
  uint32_t apogee_counter = 0;

  // TODO: Set low package data rate

  // State Machine initialization
  // Hyp: rocket is on rail waiting for liftoff
  currentState = STATE_IDLE;

  // State Macine main task loop
  for (;;)
    {
      // Check state at max period where sensors are updated
      osDelay (8);

      // Update barometer reading
      baro_data = getCurrentBARO_data ();
      // Update accelerometer reading
      imu_data = getCurrentIMU_data ();

      // State Machine
      switch (currentState)
	{

	case STATE_IDLE:
	  {
	    // Compute lift-off triggers for acceleration
	    uint8_t liftoffAccelTrig = (imu_data->acceleration.x
		> ROCKET_CST_LIFTOFF_TRIG_ACCEL);

	    // detect lift-off
	    if (liftoffAccelTrig)
	      {
		currentState = STATE_LIFTOFF; // Switch to lift-off state
		time_tmp = HAL_GetTick (); // Start timer to estimate motor burn out

		// TODO: Set package data rate to High
	      }
	    break;
	  }

	case STATE_LIFTOFF:
	  {
	    // determine motor burn-out based on lift-off detection
	    if ((HAL_GetTick () - time_tmp) > ROCKET_CST_MOTOR_BURNTIME)
	      {
		currentState = STATE_COAST;
	      }
	    break;
	  }

	case STATE_COAST:
	  {

	    // compute apogee triggers for altitude
	    uint8_t minAltTrig = ((baro_data->altitude - calib_initial_altitude)
		> ROCKET_CST_MIN_TRIG_AGL);
	    uint8_t counterAltTrig = 0;
	    uint8_t diffAltTrig = 0;
	    if (max_altitude < baro_data->altitude)
	      {
		max_altitude = baro_data->altitude;
		apogee_counter = 0;
	      }
	    else
	      {
		apogee_counter++;
		if(apogee_counter>APOGEE_BUFFER_SIZE)
		  {
		    counterAltTrig = 1;
		  }
	      }


	    // detect apogee

	  }

	}
    }

}
