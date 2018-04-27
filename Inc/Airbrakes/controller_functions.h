/*
 * controller_functions.h
 *
 *  Created on: 26 avr. 2018
 *      Author: Maxim
 */

#ifndef CONTROLLER_FUNCTIONS_H_
#define CONTROLLER_FUNCTIONS_H_

void controller_test(void);
void aerobrakes_control_init(void);
void full_close(void);
void aerobrake_helloworld(void);
void command_aerobrake_controller(float altitude, float speed);
float angle_tab(float altitude, float speed);
void motor_goto_position_inc(int position_inc); // ADD A - SIGN TO MAKE IT ROTATE ANTICLOCKWISE (as we want)





#endif /* CONTROLLER_FUNCTIONS_H_ */
