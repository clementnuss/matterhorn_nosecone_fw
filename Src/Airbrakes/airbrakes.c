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

extern float32_t kalman_air_speed, kalman_altitude;
extern UART_HandleTypeDef* airbrake_huart;

osStatus motorDetected = osErrorOS;
uint8_t faulhaber_welcome[9];

void TK_ab_controller (void const * argument)
{

  uint32_t lastAirbrakesUpdate = 0;

  for (;;)
    {
      HAL_StatusTypeDef ret = HAL_UART_Receive (airbrake_huart, faulhaber_welcome, 9, 5000);
      if (ret == HAL_TIMEOUT)
        {
          continue;
        }
      int comparison = strcmp (faulhaber_welcome, "FAULHABER");
      if (comparison == 0)
        {
          motorDetected = osOK;
          break;
        } else {
            osDelay(100);
        }


    }/*

     HAL_UART_Receive (airbrake_huart, faulhaber_welcome, 9, 5000);

     while (motorDetected != osOK)
     {
     if (motorDetected == osErrorResource)
     {
     int comparison = strcmp (faulhaber_welcome, "FAULHABER");
     if (comparison == 0)
     {
     motorDetected = osOK;
     break;
     }

     for (int i = 0; i < sizeof(faulhaber_welcome); i++)
     {
     faulhaber_welcome[i] = 0;
     }

     osDelay (100);

     HAL_UART_Receive_IT (airbrake_huart, faulhaber_welcome, 9);
     motorDetected = osErrorOS;
     }
     osDelay (50);
     }
     */
  osDelay (3000);

  aerobrakes_control_init ();
  aerobrake_helloworld ();

  osDelay (5000);

  /*
   command_aerobrake_controller (1500.0, 100.0); // Should be full open
   command_aerobrake_controller (200.0, 100.0); // Should be full close
   command_aerobrake_controller (238.0, 131.0); // Should be somewhere in between
   command_aerobrake_controller (500.0, 80.0); // Should be full close
   */

  for (;;)
    {

      if (currentState == STATE_COAST)
        {
          command_aerobrake_controller (kalman_altitude, kalman_air_speed); // Should be full close
        }
      else if (currentState == STATE_PRIMARY)
        {
          full_close ();
        }

      int elapsed = HAL_GetTick () - lastAirbrakesUpdate;
      if (100 - elapsed > 0)
        {
          osDelay (100 - elapsed);
        }
      lastAirbrakesUpdate = HAL_GetTick ();
    }
}

void airbrake_rxCpltCallback ()
{
  motorDetected = osErrorResource;
}
