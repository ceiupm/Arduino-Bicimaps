/**
 * @file Sensores.cpp
 * @author Felix Galindo
 * @date June 2017
 * @brief Arduino Library for Honeywell's Particle Sensor (HPMA115S0-XXX)
 * @license MIT
 */

#include "Arduino.h"
#include "Sensores.h"

extern "C" {
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
}

/**
 * @brief Constructor for HPMA115S0 class
 * @param  a Stream ({Software/Hardware}Serial) object.
 * @note The serial stream should be already initialized
 * @return  void
 */
HPMA115S0::HPMA115S0(Stream& serial):
  _serial(serial)
{
  _serial.setTimeout(100);
}

/**
 * @brief Function that initializes sensor
 * @return  a String containing sensor response
 */
void HPMA115S0::Init() {
  //Serial.println("PS- Initializing...");
  delay(100);
  StartParticleMeasurement();
  delay(100);
  DisableAutoSend();
}

/**
 * @brief Function that sends serial command to sensor
 * @param  a unsigned char * containing the command
 * @param size of buffer
 * @return  void
 */
void HPMA115S0::SendCmd(const char * cmdBuf, unsigned int cmdSize) {
  //Clear RX
  while (_serial.available())
    _serial.read();

  //Send command
  //Serial.print("PS- Sending cmd: ");
  unsigned int index = 0;
  for (index = 0; index < cmdSize; index++) {
    //Serial.print(cmdBuf[index], HEX);
    //Serial.print(" ");
    _serial.write(cmdBuf[index]);
  }
  //Serial.println("");
  return;
}

/**
 * @brief Function that reads command response from sensor
 * @param Buffer to store data in
 * @param Buffer size
 * @param Expected command type
 * @return  returns number of bytes read from sensor
 */
int HPMA115S0::ReadCmdResp(unsigned char * dataBuf, unsigned int dataBufSize, unsigned int cmdType) {
  static unsigned char respBuf[HPM_MAX_RESP_SIZE];
  static unsigned int respIdx = 0;
  static unsigned int calChecksum = 0;

  //Read response
  respIdx = 0;
  calChecksum = 0;
  memset(respBuf, 0, sizeof(respBuf));
  _serial.setTimeout(100);
  //Serial.println("PS- Waiting for cmd resp...");
  if (_serial.readStringUntil(HPM_CMD_RESP_HEAD)) {
    delay(1); //wait for the rest of the bytes to arrive
    respBuf[HPM_HEAD_IDX] = HPM_CMD_RESP_HEAD;
    respBuf[HPM_LEN_IDX] = _serial.read(); //Read the command length

    //Ensure buffers are big enough
    if (respBuf[HPM_LEN_IDX] && ((respBuf[HPM_LEN_IDX] + 1) <=  sizeof(respBuf) - 2) && (respBuf[HPM_LEN_IDX] - 1) <= dataBufSize ) {
      if (_serial.readBytes(&respBuf[HPM_CMD_IDX], respBuf[HPM_LEN_IDX] + 1) == (respBuf[HPM_LEN_IDX] + 1)) { //read respBuf[HPM_LEN_IDX] num of bytes + calChecksum byte
        if (respBuf[HPM_CMD_IDX] == cmdType) { //check if CMD type matches

          //Calculate and validate checksum
          for (respIdx = 0; respIdx < (2 + respBuf[HPM_LEN_IDX]); respIdx++) {
            calChecksum += respBuf[respIdx];
          }
          calChecksum = (65536 - calChecksum) % 256;
          if (calChecksum == respBuf[2 + respBuf[HPM_LEN_IDX]]) {
            //Serial.println("PS- Received valid data!!!");
            memset(dataBuf, 0, dataBufSize);
            memcpy(dataBuf, &respBuf[HPM_DATA_START_IDX], respBuf[HPM_LEN_IDX] - 1);
            return (respBuf[HPM_LEN_IDX] - 1);
          }
        }
      }
    }
  }
  return false;
}


/**
 * @brief Function that sends a read command to sensor
 * @return  returns true if valid measurements were read from sensor
 */
boolean HPMA115S0::ReadParticleMeasurement(unsigned int * pm2_5, unsigned int * pm10) {
  const char cmdBuf[] = {0x68, 0x01, 0x04, 0x93};
  static unsigned char dataBuf[HPM_READ_PARTICLE_MEASURMENT_LEN - 1];

  //Serial.println("PS- Reading Particle Measurements..." );

  //Send command
  SendCmd(cmdBuf, 4);

  //Read response
  if (ReadCmdResp(dataBuf, sizeof(dataBuf), READ_PARTICLE_MEASURMENT) == (HPM_READ_PARTICLE_MEASURMENT_LEN - 1)) {
    _pm2_5 = dataBuf[0] * 256 + dataBuf[1];
    _pm10 = dataBuf[2] * 256 + dataBuf[3];
    *pm2_5 = _pm2_5;
    *pm10 = _pm10;
    // Serial.println("PS- PM 2.5: " + String(_pm2_5) + " ug/m3" );
    // Serial.println("PS- PM 10: " + String(_pm10) + " ug/m3" );
    return true;
  }
  return false;
}

/**
 * @brief Function that starts sensor measurement
 * @return  void
 */
void HPMA115S0::StartParticleMeasurement() {
  const char cmd[] = {0x68, 0x01, 0x01, 0x96};
  SendCmd(cmd, 4);
}

/**
 * @brief Function that stops sensor measurement
 * @return  void
 */
void HPMA115S0::StopParticleMeasurement() {
  const char cmd[] = {0x68, 0x01, 0x02, 0x95};
  SendCmd(cmd, 4);
}

/**
 * @brief Function that enables auto send
 * @return  void
 */
void HPMA115S0::EnableAutoSend() {
  const char cmd[] = {0x68, 0x01, 0x40, 0x57};
  SendCmd(cmd, 4);
}

/**
 * @brief Function that stops auto send
 * @return  void
 */
void HPMA115S0::DisableAutoSend() {
  const char cmd[] = {0x68, 0x01, 0x20, 0x77};
  SendCmd(cmd, 4);
}

/**
* @brief Function that returns the latest PM 2.5 reading
* @note Sensor reports new reading ~ every 1 sec.
* @return  PM 2.5 reading (unsigned int)
*/
unsigned int HPMA115S0::GetPM2_5() {
  return _pm2_5;
}

/**
* @brief Function that returns the latest PM 10 reading
* @note Sensor reports new reading ~ every 1 sec.
* @return  PM 10 reading (unsigned int)
*/
unsigned int HPMA115S0::GetPM10() {
  return _pm10;
}

//////////////
//AMPHENOL
//////////////
/**
 * @brief Constructor for AMPHENOLSMUART04L class
 * @param  a Stream ({Software/Hardware}Serial) object.
 * @note The serial stream should be already initialized
 * @return  void
 */
AMPHENOLSMUART04L::AMPHENOLSMUART04L(Stream& serial):
  _serial(serial)
{
  _serial.setTimeout(100);
}

/**
 * @brief Function that initializes sensor
 * @return  a String containing sensor response
 */
void AMPHENOLSMUART04L::Init() {
  //Serial.println("PS- Initializing...");
  delay(100);
  StartParticleMeasurement();
  delay(100);
  DisableAutoSend();
}

/**
 * @brief Function that sends serial command to sensor
 * @param  a unsigned char * containing the command
 * @param size of buffer
 * @return  void
 */
void AMPHENOLSMUART04L::SendCmd(const char * cmdBuf, unsigned int cmdSize) {
  //Clear RX
  while (_serial.available())
    _serial.read();

  //Send command
  //Serial.print("PS- Sending cmd: ");
  unsigned int index = 0;
  for (index = 0; index < cmdSize; index++) {
    //Serial.print(cmdBuf[index], HEX);
    //Serial.print(" ");
    _serial.write(cmdBuf[index]);
  }
  //Serial.println("");
  return;
}

/**
 * @brief Function that reads command response from sensor
 * @param Buffer to store data in
 * @param Buffer size
 * @param Expected command type
 * @return  returns number of bytes read from sensor
 */
int AMPHENOLSMUART04L::ReadCmdResp(byte dataBuf[]) {
  byte message[64];
  byte respBuf[32];
  int CRC = 0;
  _serial.readBytes(message, 64);   //read 64 streamed bytes

  for ( byte i = 0 ; i < 32 ; i++ ) {  //look for ox42, 0x4D sequence and load data 	from stream
    if ( message[i] == 0x42 && message[i + 1] == 0x4D ) {
      for ( byte j = 0 ; j < 32 ; j++ ) {
        respBuf[j] = message[i];
        i++;
      }
      break;
    }
  }

  if ((respBuf[30] * 256 + respBuf[31]) == getCS(30, respBuf)) {   //Cyclical Redundancy Check
	memcpy(dataBuf, respBuf, 32);
    return true;
  }
  else {
    return false;
  }
}


/**
 * @brief Function that sends a read command to sensor
 * @return  returns true if valid measurements were read from sensor
 */
boolean AMPHENOLSMUART04L::ReadParticleMeasurement(unsigned int * pm2_5, unsigned int * pm10) {
  const char cmdBuf[] = {0x42, 0x4D, 0xE2, 0x00, 0X00, 0X01, 0X71};
  static byte dataBuf[HPM_READ_PARTICLE_MEASURMENT_LEN_AMP];

  //Serial.println("PS- Reading Particle Measurements..." );

  //Send command
  SendCmd(cmdBuf, 7);

  //Read response
  if (ReadCmdResp(dataBuf)) {
    _pm2_5 = dataBuf[6]*256 + dataBuf[7];
    _pm10 = dataBuf[8] * 256 + dataBuf[9];
    *pm2_5 = _pm2_5;
    *pm10 = _pm10;
    // Serial.println("PS- PM 2.5: " + String(_pm2_5) + " ug/m3" );
    // Serial.println("PS- PM 10: " + String(_pm10) + " ug/m3" );
    return true;
  }
  return false;
}

/**
 * @brief Function that starts sensor measurement
 * @return  void
 */
void AMPHENOLSMUART04L::StartParticleMeasurement() {
  const char cmd[] = {0x42, 0x4D, 0xE4, 0x00, 0X01, 0X01, 0X74};
  SendCmd(cmd, 7);
}

/**
 * @brief Function that stops sensor measurement
 * @return  void
 */
void AMPHENOLSMUART04L::StopParticleMeasurement() {
  const char cmd[] = {0x42, 0x4D, 0xE4, 0x00, 0X00, 0X01, 0X73};
  SendCmd(cmd, 7);
}

/**
 * @brief Function that enables auto send
 * @return  void
 */
void AMPHENOLSMUART04L::EnableAutoSend() {
  const char cmd[] = {0x42, 0x4D, 0xE1, 0x00, 0X01, 0X01, 0X71};
  SendCmd(cmd, 7);
}

/**
 * @brief Function that stops auto send
 * @return  void
 */
void AMPHENOLSMUART04L::DisableAutoSend() {
  const char cmd[] = {0x42, 0x4D, 0xE1, 0x00, 0X00, 0X01, 0X70};
  SendCmd(cmd, 7);
}

/**
* @brief Function that returns the latest PM 2.5 reading
* @note Sensor reports new reading ~ every 1 sec.
* @return  PM 2.5 reading (unsigned int)
*/
unsigned int AMPHENOLSMUART04L::GetPM2_5() {
  return _pm2_5;
}

/**
* @brief Function that returns the latest PM 10 reading
* @note Sensor reports new reading ~ every 1 sec.
* @return  PM 10 reading (unsigned int)
*/
unsigned int AMPHENOLSMUART04L::GetPM10() {
  return _pm10;
}

unsigned int AMPHENOLSMUART04L::getCS(byte len, byte data[])     // Compute the checksum
{
  unsigned int var = 0;
  for (int i = 0; i < len; i++) {
    var += data[i];
  }
  return var;
}


/*
  Copyright (c) 2019 Amphenol Advanced Sensors
  Permission is hereby granted, free of charge, to any person obtaining a copy of
  this software and associated documentation files (the "Software"), to deal in
  the Software without restriction, including without limitation the rights to
  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
  the Software, and to permit persons to whom the Software is furnished to do so,
  subject to the following conditions:
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
  FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
  IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/




