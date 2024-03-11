/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * peer_to_peer.c - App layer application of simple demonstartion peer to peer
 *  communication. Two crazyflies need this program in order to send and receive.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "app.h"
#include "app_channel.h"

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"

#define DEBUG_MODULE "[P2P => GCS]"
#include "debug.h"

struct SendPacket
{
  uint8_t id;
  uint8_t rssi;
  uint8_t class;
  float confidence;
  float x;
  float y;
  float z;
} __attribute__((packed));

//Receive from p2p radio and send to GCS (computer/python)
void p2pcallbackHandler(P2PPacket* receivePacket)
{
  // Message format
  // [uint8_t id, float rssi_angle, uint8_t class, float confidence, float x, float y, float z]

  // enum CLASS
  // {
  //     NONE = 0,
  //     BALL = 1,
  //     CONE = 2
  // } class;

  static const uint8_t DATA_SIZE = 22;

  if (receivePacket->size != DATA_SIZE)
    return;

  static const uint8_t ID_OFFSET = 6;
  const uint8_t receiveID = receivePacket->data[0] - ID_OFFSET;

  if (receiveID == 0x63)
    return; //GCS command
  else if (receiveID == 0x64)
    return; //SGBA beacon

  static struct SendPacket sendPacket;

  sendPacket.id = receiveID; //Copy ID
  sendPacket.rssi = receivePacket->rssi;

  //Copy detection class
  static const uint8_t FLOAT_SIZE = sizeof(float);
  static const uint8_t DETECTON_DATA_START = sizeof(receiveID) + FLOAT_SIZE; //FLOAT_SIZE is for rssi_angle
  static const uint8_t DETECTION_DATA_SIZE = sizeof(sendPacket.class) + sizeof(sendPacket.confidence);

  //Copy detection
  memcpy(&sendPacket.class, &receivePacket->data[DETECTON_DATA_START], sizeof(sendPacket.class));
  memcpy(&sendPacket.confidence, &receivePacket->data[DETECTON_DATA_START + sizeof(sendPacket.class)], sizeof(float));

  //Copy pos
  memcpy(&sendPacket.x, &receivePacket->data[DETECTON_DATA_START + DETECTION_DATA_SIZE], FLOAT_SIZE);
  memcpy(&sendPacket.y, &receivePacket->data[DETECTON_DATA_START + DETECTION_DATA_SIZE + FLOAT_SIZE], FLOAT_SIZE);
  memcpy(&sendPacket.z, &receivePacket->data[DETECTON_DATA_START + DETECTION_DATA_SIZE + FLOAT_SIZE * 2], FLOAT_SIZE);

  DEBUG_PRINT("ID: %d, RSSI: -%d, Class: %d, Confidence: %.3f, Pos: [%.3f, %.3f, %.3f]\n",
    sendPacket.id,
    sendPacket.rssi,
    sendPacket.class,
    (double)sendPacket.confidence,
    (double)sendPacket.x,
    (double)sendPacket.y,
    (double)sendPacket.z
  );

  //Send to python
  appchannelSendDataPacketBlock(&sendPacket, sizeof(sendPacket));
}

void appMain()
{
    DEBUG_PRINT("P2P => GCS App Layer\n");

    // // Initialize the p2p packet
    // static P2PPacket receivePacket;
    // receivePacket.port=0x00;

    // // Get the current address of the crazyflie and obtain
    // //   the last two digits and send it as the first byte
    // //   of the payload
    // uint64_t address = configblockGetRadioAddress();
    // uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    // receivePacket.data[0]=my_id;

    // //Put a string in the payload
    // char *str="Hello World";
    // memcpy(&receivePacket.data[1], str, sizeof(char)*MESSAGE_LENGHT);

    // // Set the size, which is the amount of bytes the payload with ID and the string
    // receivePacket.size=sizeof(char)*MESSAGE_LENGHT+1;

    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(p2pcallbackHandler);

  while(1) {
    // Send a message every 2 seconds
    //   Note: if they are sending at the exact same time, there will be message collisions,
    //    however since they are sending every 2 seconds, and they are not started up at the same
    //    time and their internal clocks are different, there is not really something to worry about

    vTaskDelay(M2T(2000));
    // radiolinkSendP2PPacketBroadcast(&receivePacket);
  }
}

