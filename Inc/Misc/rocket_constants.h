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
 * 	- ROCKET_CST_MIN_TRIG_AGL
 *
 */

#ifndef MISC_ROCKET_CONSTANTS_H_
#define MISC_ROCKET_CONSTANTS_H_

/*
 * ROCKET PARAMETERS
 */

#define ROCKET_CST_LIFTOFF_TRIG_ACCEL 3 // acceleration lift-off detection trigger [g]
#define ROCKET_CST_MIN_TRIG_AGL 100 // min altitude above ground level to allow apogee detection [m]
#define ROCKET_CST_MOTOR_BURNTIME 1500 // motor burn time [ms]
#define ROCKET_CST_REC_SECONDARY_ALT 100 // altitude of secondary recovery event [m]

/*
 * CALIBRATION DATA
 */

volatile float32_t calib_initial_altitude;


// Values can be found here:
// http://www.meteosuisse.admin.ch/home/meteo/valeurs-de-mesures/valeurs-de-mesures-aux-stations.html?param=airpressure-qfe
/*
static float currentLocationReferenceHPa = 964.9f;
static float currentLocationHeight = 456.0f;
static float currentLocationTemperature = 1.6f;
 */


#define ADJUSTED_SEA_LEVEL_PRESSURE 963.3

/*
static double airSpeedFromPitotPressure(uint16_t pitotPressure) {
    double p_press =
            ((static_cast<float>(pitotPressure)) - 1652) *
            (SensorConstants::PRESSURE_SENSOR2_MAX - SensorConstants::PRESSURE_SENSOR2_MIN) /
            (14745 - 1652) +
            SensorConstants::PRESSURE_SENSOR2_MIN;

    return sqrt(2 * p_press / SensorConstants::AIR_DENSITY);
}*/


#endif /* MISC_ROCKET_CONSTANTS_H_ */
