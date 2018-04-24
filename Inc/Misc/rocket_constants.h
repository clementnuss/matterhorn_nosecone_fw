/*
 * rocket_constants.h
 *
 *  Created on: 24 Apr 2018
 *      Author: ebrunner
 */

/*
 * Description : This file gathers all rocket specific constants that must be checked each time a
 * a design change occurs such as changing motor, adding mass, launching from a different altitude,...
 * It also includes calibration data such initial altitude.
 *
 * For a mass or motor change, check :
 * 	- ROCKET_CST_LAUNCH_TRIG_ACCEL
 * 	- ROCKET_CST_LIFTOFF_TRIG_AGL
 *
 */

#ifndef MISC_ROCKET_CONSTANTS_H_
#define MISC_ROCKET_CONSTANTS_H_

/*
 * ROCKET PARAMETERS
 */

#define ROCKET_CST_LIFTOFF_TRIG_ACCEL 3 // acceleration lift-off detection trigger [g]
#define ROCKET_CST_LIFTOFF_TRIG_AGL 50 // altitude above ground level lift-off detection trigger [m]
#define ROCKET_CST_MOTOR_BURNTIME 3150 // motor burn time [ms]
/*
 * CALIBRATION DATA
 */

volatile int calib_initial_altitude;

#endif /* MISC_ROCKET_CONSTANTS_H_ */
