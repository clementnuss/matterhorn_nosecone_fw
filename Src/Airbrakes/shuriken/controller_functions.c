/*
 * controller_functions.c
 *
 *  Created on: 19 avr. 2018
 *      Author: Maxim
 */

#include <stm32f4xx_hal.h>
#include <string.h>
#include <Misc/Common.h>

#ifdef SHURIKEN

#include <Airbrakes/look_up_tab.h>

#define MAX_OPENING_DEG 172.8
#define Kp_correction_margin 0.3 //Quite random value, to be readapted by the simulator later
//#define Td_correction_margin 0 // No derivator yet, lets keep it simple for now
//#define Ki_correction_margin 0 //No integrator either
#define Central_angle_max_margins 88.15 //To be defined precisely, lets do it with this value for now

extern UART_HandleTypeDef* airbrake_huart;
extern volatile float32_t airbrakes_angle;

//float correction_margin_error_integration = 0;
//float correction_margin_last_error = 0;
char command_string[10];

int tab_deg_to_inc_converter (float degrees_angle)
{
  int inc = -(int) (degrees_angle * 75000 / 360); //3000 inc/evolution, 1:25reductor
  return inc;
}

char* do_string_command (char first, char second, int number)
{
  for (int i = 0; i < sizeof(command_string); i++)
    {
      command_string[i] = 0;
    }
  command_string[0] = first;
  command_string[1] = second;
  char* commandIncStart = (char*) (&command_string[2]);
  sprintf (commandIncStart, "%d", number);
  size_t length = strlen (commandIncStart);
  commandIncStart[length] = '\r';

  return command_string;
}

void motor_goto_position_inc (int position_inc)
{
  do_string_command ('L', 'A', position_inc);
  HAL_UART_Transmit (airbrake_huart, command_string, strlen (command_string), 30);
  uint8_t* command = "M\r";
  HAL_UART_Transmit (airbrake_huart, command, 2, 30);
  // ADD DELAY HERE; THE PERIOD DEPENDS ON KALMAN FILTER FREQUENCY....
  return;
}

void controller_test (void)
{
  uint8_t* command = "HO\r";
  HAL_UART_Transmit (airbrake_huart, command, 3, 30);
  HAL_Delay (100);
  command = "EN\r";
  HAL_UART_Transmit (airbrake_huart, command, 3, 30);
  HAL_Delay (100);
  motor_goto_position_inc (10000);
  command = "M\r";
  HAL_UART_Transmit (airbrake_huart, command, 2, 30);
  return; // SMALL FUNCTION TO TEST IF THE TRANSMISSION IS WORKING... THE MOTOR SHOULD ROTATE
}

void aerobrakes_control_init (void)
{
  // TO CALL AT POWERING ON
//defining home
  uint8_t* command = "HO\r";
  HAL_UART_Transmit (airbrake_huart, command, 3, 30);
//position limits
  command = "LL1\r";
  HAL_UART_Transmit (airbrake_huart, command, 4, 30);
  int max_inc = tab_deg_to_inc_converter (MAX_OPENING_DEG); //" degrees for safety.
  do_string_command ('L', 'L', max_inc);
  HAL_UART_Transmit (airbrake_huart, command_string, 9, 30);
  command = "APL1\r";
  HAL_UART_Transmit (airbrake_huart, command, 5, 30);
// controller properties
//	command = "SP10000\r";							MAXIMUM SPEED in inc/min
//	HAL_UART_Transmit(&huart1, command, 8, 30);
  command = "POR1\r";											//    READAPT PID PARAMS FOR FLAPS
  HAL_UART_Transmit (airbrake_huart, command, 5, 30);
  command = "I1\r";
  HAL_UART_Transmit (airbrake_huart, command, 3, 30);
  command = "PP255\r";
  HAL_UART_Transmit (airbrake_huart, command, 6, 30);
  command = "PD5\r";
  HAL_UART_Transmit (airbrake_huart, command, 4, 30);
  command = "LPC2500\r"; // peak current max, to be redefined
  HAL_UART_Transmit (airbrake_huart, command, 8, 30);
  command = "LCC2000\r"; // continuous current max_to be redefined
  HAL_UART_Transmit (airbrake_huart, command, 8, 30);
  //Enable
  command = "EN\r";
  HAL_UART_Transmit (airbrake_huart, command, 3, 30);
  return;
}

void full_close (void)
{
  int angle_close_inc = tab_deg_to_inc_converter (0.0);
  motor_goto_position_inc (angle_close_inc);
  return;
}

void aerobrake_helloworld (void)
{
  float angle_helloworld = 5.0;
  int angle_helloworld_inc = tab_deg_to_inc_converter (angle_helloworld);
  motor_goto_position_inc (angle_helloworld_inc);
  HAL_Delay (500);
  full_close ();
  return;

}

float angle_tab (float altitude, float speed)
{
  int index_altitude = 0;
  if (altitude < look_up_tab[0][0])
    {
      return 0.0;
    }
  else if (altitude > look_up_tab[TABLE_LENGTH - 1][0])
    {
      return (float) MAX_OPENING_DEG;
    }
  else
    {
      int j;
      float mean_speed_vector[TABLE_DIFF_SPEEDS_SAME_ALTITUDE];
      float mean_angle_vector[TABLE_DIFF_SPEEDS_SAME_ALTITUDE];
      while (look_up_tab[index_altitude][0] < altitude)
        {
          index_altitude += TABLE_DIFF_SPEEDS_SAME_ALTITUDE;
        }
      float phi = (altitude - look_up_tab[index_altitude - TABLE_DIFF_SPEEDS_SAME_ALTITUDE][0])
          / (look_up_tab[index_altitude][0] - look_up_tab[index_altitude - TABLE_DIFF_SPEEDS_SAME_ALTITUDE][0]);
      for (j = 0; j < TABLE_DIFF_SPEEDS_SAME_ALTITUDE; j++)
        {
          mean_speed_vector[j] = phi * look_up_tab[index_altitude - TABLE_DIFF_SPEEDS_SAME_ALTITUDE + j][1]
              + (1 - phi) * look_up_tab[index_altitude + j][1];
          mean_angle_vector[j] = phi * look_up_tab[index_altitude - TABLE_DIFF_SPEEDS_SAME_ALTITUDE + j][2]
              + (1 - phi) * look_up_tab[index_altitude + j][2];
        }

      int index_speed = 0;
      if (speed < mean_speed_vector[0])
        {
          return 0.0;
        }
      else if (speed > mean_speed_vector[TABLE_DIFF_SPEEDS_SAME_ALTITUDE - 1])
        {
          return (float) MAX_OPENING_DEG;
        }
      else
        {
          while (mean_speed_vector[index_speed] < speed)
            {
              index_speed += 1;
            }
          float theta = (speed - mean_speed_vector[index_speed - 1])
              / (mean_speed_vector[index_speed] - mean_speed_vector[index_speed - 1]);
          float mean_angle = theta * mean_angle_vector[index_speed - 1] + (1 - theta) * mean_angle_vector[index_speed];
          return mean_angle;
        }
    }
}

void command_aerobrake_controller (float altitude, float speed)
{
  float opt_act_position_deg = angle_tab (altitude, speed);
  airbrakes_angle = opt_act_position_deg;
//    float opt_act_position_deg = 0.0;
  int opt_act_position_inc = tab_deg_to_inc_converter (opt_act_position_deg);
  int central_inc_max_margins = tab_deg_to_inc_converter (Central_angle_max_margins);
  int full_close_inc = tab_deg_to_inc_converter (0.0);
  int full_open_inc = tab_deg_to_inc_converter (MAX_OPENING_DEG);
  int inc_error = central_inc_max_margins - opt_act_position_inc;
  int command_inc;

  // PAS DE CONTROLE PID si on est en dehors de la bande de controle;
  //pas de accumulation de l'erreur non-plus, pour éviter le wipe-out.
  //On passe en mode controle PID que lorsque l'on est à l'intérieur de la bande de controle
  if (opt_act_position_inc <= full_close_inc || opt_act_position_inc >= full_open_inc)
    {
      command_inc = opt_act_position_inc;
      // DO WE CANCEL the integral of the error when we're out of the band of control or do we leave it like it is? Wipe out?
      //Let's leave it like it this for now
    }
  else
    {
//        correction_margin_error_integration += inc_error;
//        command_inc = opt_act_position_inc - Kp_correction_margin*inc_error - Td_correction_margin*(inc_error-correction_margin_last_error) - Ki_correction_margin*correction_margin_error_integration;
      command_inc = opt_act_position_inc - Kp_correction_margin * inc_error;
      if (command_inc <= full_close_inc)
        command_inc = full_close_inc;
      else if (command_inc >= full_open_inc)
        command_inc = full_open_inc;
    }
//    correction_margin_last_error = inc_error;

  motor_goto_position_inc (command_inc);
  return;
}

/*
 void debug(void)
 {
 float test1 = angle_tab(200, 100); // should be 0
 float test2 = angle_tab(1500, 50); // should be full open
 float test3 = angle_tab(238, 131); // should be somewhewere in between
 float test4 = angle_tab(250, 150); // should be full open
 float test5 = angle_tab(250, 100); // should be 0
 float test6 = angle_tab(446, 110); // should be somewhere in between

 do_string_command('L', 'A', (int)test1);
 HAL_UART_Transmit(&huart1, command_string, strlen(command_string), 30);
 do_string_command('L', 'A', (int)test2);
 HAL_UART_Transmit(&huart1, command_string, strlen(command_string), 30);
 do_string_command('L', 'A', (int)test3);
 HAL_UART_Transmit(&huart1, command_string, strlen(command_string), 30);
 do_string_command('L', 'A', (int)test4);
 HAL_UART_Transmit(&huart1, command_string, strlen(command_string), 30);
 do_string_command('L', 'A', (int)test5);
 HAL_UART_Transmit(&huart1, command_string, strlen(command_string), 30);
 do_string_command('L', 'A', (int)test6);
 HAL_UART_Transmit(&huart1, command_string, strlen(command_string), 30);
 return;
 }
 */

#endif
