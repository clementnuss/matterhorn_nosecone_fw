/*
 * xbeeDownlink.c
 *
 *  Created on: 22 Apr 2018
 *      Author: Clément Nussbaumer
 */

#include "stm32f4xx_hal.h"
#include "Misc/Common.h"

#ifdef CENTRALBODY

#include <Misc/physical_iface.h>

extern UART_HandleTypeDef* nose_huart;

#define XBEE_RX_BUFFER_SIZE 512

#define PREAMBLE_FLAG 0x55
#define RX_PACKET_SIZE 1

uint8_t rxPacketBuffer[RX_PACKET_SIZE];

uint32_t lastDmaStreamIndex = 0, endDmaStreamIndex = 0;
uint8_t rxBuffer[XBEE_RX_BUFFER_SIZE];

uint32_t preambleCnt, packetCnt, currentChecksum;

extern int startSimulation;

enum DECODING_STATE
{
  PARSING_PREAMBLE, PARSING_PACKET, PARSING_CHECKSUM
};

uint8_t currentRxState = PARSING_PREAMBLE;

void TK_nose_communication (const void* args)
{

  HAL_UART_Receive_DMA (nose_huart, rxBuffer, XBEE_RX_BUFFER_SIZE);

  for (;;)
    {
      endDmaStreamIndex = XBEE_RX_BUFFER_SIZE - nose_huart->hdmarx->Instance->NDTR;
      while (lastDmaStreamIndex < endDmaStreamIndex)
        {
          processReceivedByte (rxBuffer[lastDmaStreamIndex++]);
        }

      osDelay (10);
    }
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
  while (lastDmaStreamIndex < XBEE_RX_BUFFER_SIZE)
    {
      processReceivedByte (rxBuffer[lastDmaStreamIndex++]);
    }

  endDmaStreamIndex = 0;
  lastDmaStreamIndex = 0;
}

void resetStateMachine ()
{
  currentRxState = PARSING_PREAMBLE;
  preambleCnt = 0;
  packetCnt = 0;
  currentChecksum = 0;
}

void processReceivedPacket ()
{

  longBip ();
  osDelay (1000);
  startSimulation = 1;

  switch (rxPacketBuffer[0])
    {
    case 0x14:
      {

        break;
      }
    case 0x22:
      {
        break;
      }
    }

}

inline void processReceivedByte (uint8_t rxByte)
{
  switch (currentRxState)
    {
    case PARSING_PREAMBLE:
      {
        if (rxByte == PREAMBLE_FLAG)
          {
            if (++preambleCnt == 2)
              {
                currentRxState = PARSING_PACKET;
              }
          }
        else
          {
            resetStateMachine ();
          }
        break;
      }
    case PARSING_PACKET:
      {
        rxPacketBuffer[packetCnt++] = rxByte;
        currentChecksum += rxByte;

        longBip ();
        osDelay (1000);
        startSimulation = 1;
        if (packetCnt == RX_PACKET_SIZE)
          {
            currentRxState = PARSING_CHECKSUM;
          }
        break;
      }
    case PARSING_CHECKSUM:
      {
        if (currentChecksum == rxByte)
          {
            processReceivedPacket ();
          }
        resetStateMachine ();
        break;
      }
    }
}

#endif
