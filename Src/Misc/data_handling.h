/*
 * data_handling.h
 *
 *  Created on: 19 Apr 2018
 *      Author: Clément Nussbaumer
 */

void* createTelemetryDatagram (IMU_data* imu_data, BARO_data* baro_data, uint32_t measurement_time);

inline void write8 (uint8_t v);

inline void write16 (uint16_t v);

inline void write32u (uint32_t v);

inline void write32f (float32_t v);
