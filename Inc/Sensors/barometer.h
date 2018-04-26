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

void initBarometer ();

osStatus i2cTransmitCommand (uint8_t command);

osStatus i2cReceive (uint8_t* rxBuffer, uint16_t size);

osStatus processD1D2 (uint32_t d1, uint32_t d2, BARO_data* ret);

float altitudeFromPressure (float pressure_hPa);

#endif /* SENSORS_PRESSURESENSORS_BAROMETER_H_ */
