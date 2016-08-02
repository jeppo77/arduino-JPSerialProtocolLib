#ifndef JPSerialProtocolLib_h
#define JPSerialProtocolLib_h

#include <Arduino.h>

#define NO_COMMAND 0
#define WRITE_COMMAND 1
#define READ_COMMAND 2
#define REPLY_COMMAND 3
#define REPLY_ERROR 4

#ifndef SERIAL_DATA_MAX_LENGTH
#define SERIAL_DATA_MAX_LENGTH 16
#endif

#define SERIAL_WORD_MAX_LENGTH SERIAL_DATA_MAX_LENGTH + 7

class JPSerialProtocolLib
{
  public:
    JPSerialProtocolLib();

    // IMPORTANT:
    // register address 0xFF is reserved (for error replies)

    void begin(long speed, uint8_t newnodeaddress);

    void sendWriteCommand(uint8_t destnodeaddress, uint8_t destregaddress, uint8_t *data, uint8_t length);
    void sendReadCommand(uint8_t destnodeaddress, uint8_t destregaddress);
    void sendReply(uint8_t *data, uint8_t length);
    void sendErrorReply();

    uint8_t readSerialData();

    uint8_t getcommand();
    uint8_t getregaddress();
    uint8_t getsourceaddress();
    uint8_t getlength();
    uint8_t getdata(uint8_t *data);
    uint8_t getparity();

  private:
  
    typedef struct 
    {
      uint8_t command;
      uint8_t destnodeaddress;
      uint8_t destregaddress;
      uint8_t sourceaddress;
      uint8_t data[SERIAL_DATA_MAX_LENGTH];
      uint8_t parity;
      uint8_t length;
    } incomingPacketType;
    
    incomingPacketType incomingPacket;

    uint8_t inputChar[SERIAL_WORD_MAX_LENGTH];
    uint8_t incomingCounter;
    uint8_t nodeaddress;

    void sendBasicReply(uint8_t *data, uint8_t length, boolean error);

    uint8_t parseSerialData();
};
#endif