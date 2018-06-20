/*
 * state_estimation.c
 *
 *  Created on: 18 Jun 2018
 *      Author: Clément Nussbaumer
 */

#include <Misc/Common.h>
#include <Misc/rocket_constants.h>

volatile float32_t air_speed_state_estimate, altitude_estimate;
#define ALTITUDE_BUFFER_SIZE 8

void TK_state_estimation ()
{

  float32_t altitude_buffer[ALTITUDE_BUFFER_SIZE][2];
  uint8_t altitude_index = 0;

  uint32_t lastBaroSeqNumber = 0;

  while (LIFTOFF_TIME == 0)
    {
      if (lastBaroSeqNumber < currentBaroSeqNumber)
        {
          altitude_index++;
          altitude_buffer[altitude_index % ALTITUDE_BUFFER_SIZE][0] = BARO_buffer[currentBaroSeqNumber
              % CIRC_BUFFER_SIZE].altitude;
          altitude_buffer[altitude_index % ALTITUDE_BUFFER_SIZE][1] = currentBaroTimestamp;
          lastBaroSeqNumber = currentBaroSeqNumber;
        }

      osDelay (10);
    }

  for (int i = 0; i < ALTITUDE_BUFFER_SIZE; ++i)
    {
      altitude_buffer[i][0] -= calib_initial_altitude;
    }

  for (;;)
    {
      uint32_t work_time = HAL_GetTick ();

      if (lastBaroSeqNumber < currentBaroSeqNumber)
        {
          altitude_index++;
          altitude_buffer[altitude_index % ALTITUDE_BUFFER_SIZE][0] = BARO_buffer[currentBaroSeqNumber
              % CIRC_BUFFER_SIZE].altitude - calib_initial_altitude;
          altitude_buffer[altitude_index % ALTITUDE_BUFFER_SIZE][1] = currentBaroTimestamp;

          lastBaroSeqNumber = currentBaroSeqNumber;
        }

      float32_t d_t = altitude_buffer[altitude_index % ALTITUDE_BUFFER_SIZE][1]
          - altitude_buffer[(altitude_index + 1) % ALTITUDE_BUFFER_SIZE][1];
      d_t /= 1000.0;

      float32_t d_h = altitude_buffer[altitude_index % ALTITUDE_BUFFER_SIZE][0]
          - altitude_buffer[(altitude_index + 1) % ALTITUDE_BUFFER_SIZE][0];

      float32_t sum = 0.0f;
      for (int i = 0; i < ALTITUDE_BUFFER_SIZE; ++i)
        {
          sum += altitude_buffer[i][0];
        }

      altitude_estimate = sum / ALTITUDE_BUFFER_SIZE;
      air_speed_state_estimate = d_h / d_t;

      osDelay (3 - (HAL_GetTick () - work_time));
    }

}
