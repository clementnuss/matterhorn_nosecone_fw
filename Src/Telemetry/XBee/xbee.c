/*
 * xbee.c
 *
 *  Created on: 9 Apr 2018
 *      Author: Clément Nussbaumer
 */

#include "xbee.h"

#include "stm32f4xx_hal.h"
#include "Misc/Common.h"

extern osMessageQId xBeeQueueHandle;
osSemaphoreId xBeeTxBufferSemHandle;

extern UART_HandleTypeDef* xBee_huart;

// UART settings
#define XBEE_UART_TIMEOUT 30

// XBee API mode
#define XBEE_START 0x7e
#define XBEE_ESCAPE 0x7d
#define XBEE_TX_FRAME_TYPE 0x10 // Transmit request frame
#define XBEE_FRAME_BEGINNING_SIZE 3 // Start delimiter (0x7E) + uint16_t length of the frame
#define XBEE_FRAME_OPTIONS_SIZE 14
#define XBEE_CHECKSUM_SIZE 1 // checksum size of the XBee packet

static uint8_t XBEE_FRAME_OPTIONS[XBEE_FRAME_OPTIONS_SIZE] =
  {
  XBEE_TX_FRAME_TYPE,  // Frame type
      0x00,           // Frame ID
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff,     // 64 bit dest address
      0xff, 0xfe,           // 16 bits dest address (0xff fe = broadcast)
      0x00,           // Broadcast radius (0 = max)
      0x43 };          // Transmit options (disable ACK and Route discovery)

uint8_t XBEE_FRAME_OPTIONS_CRC = 0;
uint8_t XBEE_SEND_FRAME_TIMEOUT_MS = 40;

uint8_t currentCrc = 0;
uint8_t payloadBuffer[XBEE_PAYLOAD_MAX_SIZE];
uint8_t txDmaBuffer[2 * XBEE_PAYLOAD_MAX_SIZE + XBEE_CHECKSUM_SIZE + XBEE_FRAME_BEGINNING_SIZE];
uint16_t currentPos = 0;

void TK_xBeeTelemetry (const void* args)
{

  initXbee ();

  for (;;)
    {

      osEvent event = osMessageGet (xBeeQueueHandle, XBEE_SEND_FRAME_TIMEOUT_MS);
      if (event.status == osEventMessage)
        {
          Telemetry_Message* m = event.value.p;
          sendData (m->ptr, m->size);
          vPortFree (m->ptr);
        }
      else {
          sendXbeeFrame();
      }
    }
}

void receiveData(){



}

void sendData (uint8_t* txData, uint16_t txDataSize)
{
  if (txDataSize >= XBEE_PAYLOAD_MAX_SIZE)
    {
      return;
    }

  if (currentPos + txDataSize >= XBEE_PAYLOAD_MAX_SIZE)
    {
      sendXbeeFrame ();
    }

  if (currentPos + txDataSize < XBEE_PAYLOAD_MAX_SIZE)
    {
      addToBuffer (txData, txDataSize);
    }

  if (XBEE_PAYLOAD_MAX_SIZE - currentPos < 20) // send the XBee frame if there remains less than 20 bytes available in the txDataBuffer
    {
      sendXbeeFrame ();
    }
}

inline void addToBuffer (uint8_t* txData, uint16_t txDataSize)
{
  for (uint16_t i = 0; i < txDataSize; i++)
    {
      payloadBuffer[currentPos + i] = txData[i];
    }
  currentPos += txDataSize;
}

void sendXbeeFrame ()
{
  if (osSemaphoreWait (xBeeTxBufferSemHandle, XBEE_UART_TIMEOUT) != osOK)
    {
      return;
    }

  uint16_t payloadAndConfigSize = XBEE_FRAME_OPTIONS_SIZE + currentPos;

  uint16_t pos = 0;
  txDmaBuffer[pos++] = XBEE_START;
  txDmaBuffer[pos++] = payloadAndConfigSize >> 8;
  txDmaBuffer[pos++] = payloadAndConfigSize & 0xff;
  for (int i = 0; i < sizeof(XBEE_FRAME_OPTIONS); i++)
    {
      txDmaBuffer[pos++] = XBEE_FRAME_OPTIONS[i];
    }
  currentCrc = XBEE_FRAME_OPTIONS_CRC;

  for (int i = 0; i < currentPos; ++i)
    {
      uint8_t escapedChar;
      if ((escapedChar = escapedCharacter (payloadBuffer[i])))
        {
          txDmaBuffer[pos++] = XBEE_ESCAPE;
          txDmaBuffer[pos++] = escapedChar;
        }
      else
        {
          txDmaBuffer[pos++] = payloadBuffer[i];
        }

      currentCrc += payloadBuffer[i];
    }

  currentCrc = 0xff - currentCrc;
  txDmaBuffer[pos++] = currentCrc;
  HAL_UART_Transmit_DMA (xBee_huart, txDmaBuffer, pos);

  currentPos = 0;
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
{
  osSemaphoreRelease (xBeeTxBufferSemHandle);
}

void initXbee ()
{

  uint packets_per_second = XBEE_PERFORMANCE_BPS / (8 * (XBEE_PAYLOAD_MAX_SIZE + 36));
  XBEE_SEND_FRAME_TIMEOUT_MS = 1000 / packets_per_second;

  uint8_t checksum = 0;
  for (int i = 0; i < sizeof(XBEE_FRAME_OPTIONS); ++i)
    {
      checksum += XBEE_FRAME_OPTIONS[i];
    }
  XBEE_FRAME_OPTIONS_CRC = checksum;
}

inline uint8_t escapedCharacter (uint8_t byte)
{
  switch (byte)
    {
    case 0x7e:
      return 0x5e;
    case 0x7d:
      return 0x5d;
    case 0x11:
      return 0x31;
    case 0x13:
      return 0x33;
    default:
      return 0x00;
    }
}
