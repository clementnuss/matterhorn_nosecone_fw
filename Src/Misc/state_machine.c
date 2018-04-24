/*
 * state_machine.c
 *
 *  Created on: 24 Apr 2018
 *      Author: ebrunner
 */

#include "stm32f4xx_hal.h"
#include "Misc/Common.h"
#include "Misc/rocket_constants.h"

// IMU and Barometer data buffers
extern IMU_data IMU_buffer[];
extern BARO_data BARO_buffer[];

void
TK_state_machine (void const * argument)
{

  // Declare time variables
  uint32_t measurement_time = HAL_GetTick();
  uint32_t last_measurement_time = 0;
  // Declare sensor pointers
  IMU_data* imu_data; BARO_data* baro_data;

  // Initial calibration commands
  baro_data = &BARO_buffer[currentBaroSeqNumber % CIRC_BUFFER_SIZE];
  calib_initial_altitude = baro_data->pressure; //TODO: use altitude function provided by Clement

  // State Machine initialization
  // Hyp: rocket is on rail waiting for liftoff
  currentState = STATE_IDLE;


  // State Macine main task loop
  for (;;)
    {
      // Check state at max period where sensors are updated
      osDelay (8);

      // Update sensor and time variables
      measurement_time = HAL_GetTick ();
      imu_data = &IMU_buffer[currentImuSeqNumber % CIRC_BUFFER_SIZE];
      baro_data = &BARO_buffer[currentBaroSeqNumber % CIRC_BUFFER_SIZE];

      // State
      switch (currentState)
	{
	case STATE_IDLE:
	  {
	    // detect lift-off
	    if(imu_data->acceleration->x > ROCKET_CST_LIFTOFF_TRIG_ACCEL &&
		(baro_data->pressure-calib_initial_altitude) > ROCKET_CST_LIFTOFF_TRIG_AGL) // TODO: use altitude function provided by Clement
	    break;
	  }
	case STATE_LIFTOFF:
	  {
	    break;
	  }

	}
    }

}


