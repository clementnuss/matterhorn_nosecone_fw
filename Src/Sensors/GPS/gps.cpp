/*
 * gpsParser.cpp
 *
 *  Created on: 7 May 2018
 *      Author: Clément Nussbaumer
 */

#include <cmsis_os.h>

#include <Sensors/gps.h>

#include <Misc/datagram_builder.h>
#include <Telemetry/telemetry_protocol.h>
#include <Misc/datastructs.h>
#include <lib/TinyGPS++.h>

#include <stm32f4xx_hal.h>

extern UART_HandleTypeDef* gps_huart;
extern osMessageQId xBeeQueueHandle;

#define GPS_RX_BUFFER_SIZE 256

uint32_t lastGpsDmaStreamIndex = 0, endGpsDmaStreamIndex = 0;
uint8_t gpsRxBuffer[GPS_RX_BUFFER_SIZE];

TinyGPSPlus gpsParser;

void TK_GPS (void const * argument)
{

  TinyGPSPlus gps;
  HAL_UART_Receive_DMA (gps_huart, gpsRxBuffer, GPS_RX_BUFFER_SIZE);

  uint32_t gpsSequenceNumber;

  for (;;)
    {
      endGpsDmaStreamIndex = GPS_RX_BUFFER_SIZE - gps_huart->hdmarx->Instance->NDTR;
      while (lastGpsDmaStreamIndex < endGpsDmaStreamIndex)
        {
          gpsParser.encode (gpsRxBuffer[lastGpsDmaStreamIndex++]);
        }

      if (gpsParser.time.isUpdated ())
        {

          Telemetry_Message m = createGPSDatagram (gpsSequenceNumber++);
          osMessagePut (xBeeQueueHandle, (uint32_t) &m, 50);

        }

      osDelay (10);
    }
}

void GPS_RxCpltCallback ()
{
  while (lastGpsDmaStreamIndex < GPS_RX_BUFFER_SIZE)
    {
      gpsParser.encode (gpsRxBuffer[lastGpsDmaStreamIndex++]);
    }

  endGpsDmaStreamIndex = 0;
  lastGpsDmaStreamIndex = 0;
}

Telemetry_Message createGPSDatagram (uint32_t seqNumber)
{
  DatagramBuilder builder = DatagramBuilder (GPS_DATAGRAM_PAYLOAD_SIZE, GPS, seqNumber++);

  uint8_t sats = gpsParser.satellites.isValid () ? static_cast<uint8_t> (gpsParser.satellites.value ()) : 0;

  float32_t hdop = gpsParser.hdop.isValid () ? gpsParser.hdop.hdop () : 0xffffffff;
  float32_t lat = gpsParser.location.isValid () ? gpsParser.location.lat () : 0xffffffff;
  float32_t lon = gpsParser.location.isValid () ? gpsParser.location.lng () : 0xffffffff;
  int32_t altitude = gpsParser.altitude.isValid () ? gpsParser.altitude.value () : 0;

  builder.write32<uint32_t> (HAL_GetTick ());
  builder.write8 (sats);
  builder.write32<float32_t> (hdop);
  builder.write32<float32_t> (lat);
  builder.write32<float32_t> (lon);
  builder.write32<int32_t> (altitude);

  return builder.finalizeDatagram ();
}
