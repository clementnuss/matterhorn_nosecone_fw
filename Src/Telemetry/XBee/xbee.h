/*
 * xbee.h
 *
 *  Created on: 9 Apr 2018
 *      Author: Clément Nussbaumer
 */

#ifndef TELEMETRY_XBEE_XBEE_H_
#define TELEMETRY_XBEE_XBEE_H_

#include "cmsis_os.h"

#define XBEE_PAYLOAD_SIZE 214
#define XBEE_PACKET_SEND_TIMEOUT 20

#define XBEE_PERFORMANCE_BPS 80000

void initXbee ();
void TK_xBeeTelemetry (const void* args);

void sendData (uint8_t* txData, uint16_t txDataSize);

void sendXbeeFrame ();

void addToBuffer (uint8_t* txData, uint16_t txDataSize);

uint8_t escapedCharacter (uint8_t byte);

#endif /* TELEMETRY_XBEE_XBEE_H_ */
