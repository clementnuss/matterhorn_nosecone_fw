/*
 * abd_p.h
 *
 *  Created on: 17 Apr 2018
 *      Author: Clément Nussbaumer
 */

#ifndef SENSORS_PRESSURESENSORS_BAROMETER_H_
#define SENSORS_PRESSURESENSORS_BAROMETER_H_

#include "cmsis_os.h"
#include "Misc/Common.h"

// 001PD2A3 - Diff pressure sensor -1 to +1 psi - calibration 10-90% - 12 bits
#define DIFFERENTIAL_PRESSURE_SENSOR_LOW_RANGE_ADDRESS 0x28 << 1
#define DIFF_PRESS_LOW_RANGE_P_MIN -1.0
#define DIFF_PRESS_LOW_RANGE_P_MAX 1.0

//015PG6A3 - Gage pressure sensor 0 to 15 psi - calibration 10 - 90 % - 12 bits
#define DIFFERENTIAL_PRESSURE_SENSOR_HIGH_RANGE_ADDRESS 0x68 << 1
#define DIFF_PRESS_HIGH_RANGE_P_MIN 0.0
#define DIFF_PRESS_HIGH_RANGE_P_MAX 15.0

#define DIFF_PRESS_OUTPUT_MIN 0x0666
#define DIFF_PRESS_OUTPUT_MAX 0x3999
#define DIFF_PRESS_OUTPUT_RANGE 13107.0 //Ouput max - output min

#define DIFF_PRESS_OUTPUT_MASK 0x3fff

#define PSI_TO_PASCAL_CONVERSION_FACTOR 6894.76

void initBarometer ();

osStatus i2cTransmitCommand (uint8_t command);

osStatus i2cReceive (uint8_t* rxBuffer, uint16_t size, uint8_t device_address);

osStatus processD1D2 (uint32_t d1, uint32_t d2, BARO_data* ret);

float altitudeFromPressure (float pressure_hPa);

float32_t interpolatePitotReadings (float32_t low_range_pressure, float32_t high_range_pressure);

#endif /* SENSORS_PRESSURESENSORS_BAROMETER_H_ */
