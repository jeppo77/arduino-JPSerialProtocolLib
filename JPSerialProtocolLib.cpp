
#include "JPSerialProtocolLib.h"

//#define SERIAL_PROTOCOL_DEBUG
//#define IGNORE_PARITY

#define WRITE_CHAR (uint8_t)0x5A
#define READ_CHAR (uint8_t)0xA5
#define ANSWER_CHAR (uint8_t)0xAA

JPSerialProtocolLib::JPSerialProtocolLib()
{
  incomingCounter = 0;
}

void JPSerialProtocolLib::begin(long speed, uint8_t newnodeaddress)
{
  Serial.begin(speed);
  nodeaddress = newnodeaddress;
#ifdef SERIAL_PROTOCOL_DEBUG
  Serial.print("starting serial protocol , node 0x");
  Serial.print(newnodeaddress, HEX);
#endif

}

/**
   Return CRC-8 of the data, using x^8 + x^2 + x + 1 polynomial.  A table-based
   algorithm would be faster, but for only a few bytes it isn't worth the code
   size. */
uint8_t Crc8(const uint8_t *data, int len, uint8_t crc_in)
{
  unsigned crc = ((unsigned)crc_in) << 8;
  int i, j;
  for (j = len; j; j--, data++) {
    crc ^= (*data << 8);
    for (i = 8; i; i--) {
      if (crc & 0x8000)
        crc ^= (0x1070 << 3);
      crc <<= 1;
    }
  }
  return (uint8_t)(crc >> 8);
}

void JPSerialProtocolLib::sendWriteCommand(uint8_t destnodeaddress, uint8_t destregaddress, uint8_t *data, uint8_t length)
{
  uint8_t message[SERIAL_WORD_MAX_LENGTH];

  if (length == 0)
    return;
  if (length > SERIAL_DATA_MAX_LENGTH)
    length = SERIAL_DATA_MAX_LENGTH;

  message[0] = WRITE_CHAR;
  message[1] = destnodeaddress;
  message[2] = destregaddress;
  message[3] = nodeaddress;
  message[4] = length;

  for (int i = 0; i < length; i++)
    message[5 + i] = data[i];

  message[6 + length - 1] = Crc8(message, 6 + length - 1, 0);
  message[7 + length - 1] = '\n';

  Serial.write(message, 8 + length - 1);
}

void JPSerialProtocolLib::sendReadCommand(uint8_t destnodeaddress, uint8_t destregaddress)
{
  uint8_t message[SERIAL_WORD_MAX_LENGTH];

  message[0] = READ_CHAR;
  message[1] = destnodeaddress;
  message[2] = destregaddress;
  message[3] = nodeaddress;

  message[4] = Crc8(message, 4, 0);
  message[5] = '\n';

  Serial.write(message, 6);
}

void JPSerialProtocolLib::sendReply(uint8_t *data, uint8_t length)
{
  sendBasicReply(data, length, 0);
}

void JPSerialProtocolLib::sendErrorReply()
{
  uint8_t buf[5] = {'E', 'R', 'R', 'O', 'R'};
  sendBasicReply(buf, 5, 1);
}

void JPSerialProtocolLib::sendBasicReply(uint8_t *data, uint8_t length, boolean error)
{
  uint8_t message[SERIAL_WORD_MAX_LENGTH];

  if (length == 0)
    return;
  if (length > SERIAL_DATA_MAX_LENGTH)
    length = SERIAL_DATA_MAX_LENGTH;

  message[0] = ANSWER_CHAR;
  message[1] = incomingPacket.sourceaddress;
  if (error)
    message[2] = 0xFF;
  else
    message[2] = incomingPacket.destregaddress;
  message[3] = nodeaddress;
  message[4] = length;

  for (int i = 0; i < length; i++)
    message[5 + i] = data[i];

  message[6 + length - 1] = Crc8(message, 6 + length - 1, 0);
  message[7 + length - 1] = '\n';

  Serial.write(message, 8 + length - 1);
}

uint8_t JPSerialProtocolLib::readSerialData()
{
  while (Serial.available())
  {
    inputChar[incomingCounter] = (char)Serial.read();
    if (inputChar[incomingCounter] == WRITE_CHAR ||
         inputChar[incomingCounter] == READ_CHAR ||
         inputChar[incomingCounter] == ANSWER_CHAR ||
          incomingCounter)
    {

    if  (inputChar[incomingCounter] == '\n' )
    {

      uint8_t result = parseSerialData();
      if (result != NO_COMMAND)
      {
        incomingCounter = 0;
        return result;
      }
    }
    if (incomingCounter >= SERIAL_WORD_MAX_LENGTH - 1)
    {
      incomingCounter = 0;
      }
    else
      incomingCounter++;
}
  }
  return NO_COMMAND;
}

uint8_t JPSerialProtocolLib::parseSerialData()
{
  if (inputChar[0] == WRITE_CHAR && incomingCounter == (6 + inputChar[4]) && inputChar[4] > 0 && inputChar[4] <= SERIAL_DATA_MAX_LENGTH)
  {
    //destnodeaddress = inputChar[1];
    //destregaddress = inputChar[2];
    //sourceaddress = inputChar[3];
    //length = inputChar[4];
    //parity = inputChar[6 + inputChar[4]-1];

#ifndef IGNORE_PARITY
    if (Crc8(inputChar, 6 + inputChar[4] - 1, 0) == inputChar[6 + inputChar[4] - 1])
#endif
    {
      if (inputChar[1] == nodeaddress)
      {
        incomingPacket.command = WRITE_COMMAND;
        incomingPacket.destnodeaddress = inputChar[1];
        incomingPacket.destregaddress = inputChar[2];
        incomingPacket.sourceaddress = inputChar[3];
        incomingPacket.length = inputChar[4];
        incomingPacket.parity = inputChar[6 + inputChar[4] - 1];
        for (int i = 0 ; i < inputChar[4] ; i++)
        {
          incomingPacket.data[i] = inputChar[5 + i];
        }
        return WRITE_COMMAND;
      }
      else
      {
        if (nodeaddress != inputChar[3])
          Serial.write(inputChar, incomingCounter + 1);
      }
    }
  }
  else if (inputChar[0] == READ_CHAR && incomingCounter == 5)
  {
    //incomingPacket.destnodeaddress = inputChar[1];
    //incomingPacket.destregaddress = inputChar[2];
    //incomingPacket.sourceaddress = inputChar[3];
    //incomingPacket.parity = inputChar[4];
    //incomingPacket.length = 0;

#ifndef IGNORE_PARITY
    if (Crc8(inputChar, 4, 0) == inputChar[4])
#endif
    {
      if (inputChar[1] == nodeaddress)
      {
        incomingPacket.command = READ_COMMAND;
        incomingPacket.destnodeaddress = inputChar[1];
        incomingPacket.destregaddress = inputChar[2];
        incomingPacket.sourceaddress = inputChar[3];
        incomingPacket.length = 0;
        incomingPacket.parity = inputChar[4];
        for (int i = 0 ; i < SERIAL_DATA_MAX_LENGTH ; i++)
        {
          incomingPacket.data[i] = 0;
        }
        return READ_COMMAND;
      }
      else
      {
        if (nodeaddress != inputChar[3])
          Serial.write(inputChar, incomingCounter + 1);
      }
    }
  }
  else if (inputChar[0] == ANSWER_CHAR && incomingCounter == 6 + inputChar[4] && inputChar[4] > 0 && inputChar[4] <= SERIAL_DATA_MAX_LENGTH)
  {

    //incomingPacket.destnodeaddress = inputChar[1];
    //incomingPacket.destregaddress = inputChar[2];
    //incomingPacket.sourceaddress = inputChar[3];
    //incomingPacket.length = inputChar[4];
    //incomingPacket.parity = inputChar[6+inputChar[4]-1];

#ifndef IGNORE_PARITY
    if (Crc8(inputChar, 6 + inputChar[4] - 1, 0) == inputChar[6 + inputChar[4] - 1])
#endif
    {
      if (inputChar[1] == nodeaddress)
      {

        if (inputChar[2] == 0xFF && inputChar[4] == 5 &&
            inputChar[5] == 'E' && inputChar[6] == 'R' && inputChar[7] == 'R' && inputChar[8] == 'O' && inputChar[9] == 'R')
          incomingPacket.command = REPLY_ERROR;
        else
          incomingPacket.command = REPLY_COMMAND;
        incomingPacket.destnodeaddress = inputChar[1];
        incomingPacket.destregaddress = inputChar[2];
        incomingPacket.sourceaddress = inputChar[3];
        incomingPacket.length = inputChar[4];
        incomingPacket.parity = inputChar[6 + inputChar[4] - 1];
        for (int i = 0 ; i < inputChar[4] ; i++)
        {
          incomingPacket.data[i] = inputChar[5 + i];
        }
        return incomingPacket.command;
      }
      else
      {
        if (nodeaddress != inputChar[3])
          Serial.write(inputChar, incomingCounter + 1);
      }
    }
  }
  return NO_COMMAND;
}

uint8_t JPSerialProtocolLib::getcommand()
{
  return incomingPacket.command;
}

uint8_t JPSerialProtocolLib::getregaddress()
{
  return incomingPacket.destregaddress;
}

uint8_t JPSerialProtocolLib::getsourceaddress()
{
  return incomingPacket.sourceaddress;
}

uint8_t JPSerialProtocolLib::getlength()
{
  return incomingPacket.length;
}

uint8_t JPSerialProtocolLib::getparity()
{
  return incomingPacket.parity;
}

uint8_t JPSerialProtocolLib::getdata(uint8_t *data)
{
  for (int i = 0 ; i < incomingPacket.length ; i++)
  {
    data[i] = incomingPacket.data[i];
  }
  return incomingPacket.length;
}
