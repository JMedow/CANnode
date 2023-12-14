/*
  CANnode.h
  Dec 2023

  Author: Jeremy Medow (jeremy@tungstencustoms.com)
*/

#ifndef CANnode_h
#define CANnode_h

#include <SPI.h>
#include <mcp_can.h>

#define CAN_SPEED CAN_40KBPS

#define DEFAULT_CS_PIN 10

#define ELEMENTS(x) (sizeof(x)/sizeof(x[0]))

// For msgSendForACK try SENDMAX times, waiting ACKTIMEOUT ms each time
#define TXDELAY 5
#define ACKTIMEOUT 30
#define ACKTIMEOUTDATA 5
#define SENDMAX 3
#define READTIMEOUT 30

#define W_NACK 0b00   // Write, no ACK
#define W_ACK 0b10    // Write, ACK expected
#define R_NACK 0b01   // Read, no ACK
#define ACK 0b11      // ACK

#define REG_STATUS 0
#define REG_RES 1
#define REG_TOGGLE 2
#define REG_SENSOR 3
#define REG_TRACK 4
#define REG_PLAY 5
#define REG_VOL 6
#define REG_MULT 7

#define NO_RESPONSE 0xFF

#define BROADCAST_SND 0xFF    // Messages sent by this address are heard by everyone

struct CANmsg{
    uint8_t sndID = 0;
    uint8_t rcvID = 0;
    uint8_t ackRW = 0;
    uint8_t reg = 0;
    uint8_t payload = 0;
};

class CANnode{
public:
    CANnode(int CS_PIN);
    // Initializer
    void initCAN(uint8_t myID, uint8_t mult);
    // Sets _myADDR, starts CAN communication, sets standard filter - if mult>0, do multiple

    void setNoFilterExt(void);
    // Listen to all extended frame messages (i.e. everything)
    void setStandardFilter(uint8_t mult);
    // Listen to anything addressed to _myADDR and any master-sent 0b11xx messages (RESET, BEAT, etc)

    bool msgAvailable(void);
    // Return TRUE if a (filtered) message is ready
    CANmsg getCommand(void);
    // Returns CANmsg, Dumps data and length into _lastData and _lastLen
    uint8_t lastData(uint8_t data[]);
    // Returns the data count of last data sent, fills the data array with the data

    bool updateNode(void);
    // Checks for messages, and deals with them.  Call repeatedly.  Returns true if any registers change.

    bool regWrite(uint8_t rcvADDR, uint8_t reg, uint8_t payload, bool toAck);
    // Writes the payload to the register of rcvADDR
    bool regWriteMult(uint8_t rcvADDR, uint8_t regMask, uint8_t payloads[], bool toAck);
    // Writes the payloads to the registers in the mask of rcvADDR (no ACK)
    bool assignMult(uint8_t regMask, uint8_t data[]);
    // Puts the data in the right registers, returns true if anything changes


    uint8_t myADDR(void);
    // Return _myADDR
    uint8_t whichADDR(void);
    // Which address the last command was sent to
    void setActiveADDR(uint8_t which);
    // Set the active (sending) address

    uint8_t regVal(uint8_t reg);
    // Return the value of a given register
    void setReg(uint8_t reg, uint8_t val);
    // Set the value of a given register

    void sndACK(uint8_t theReg, uint8_t thePayload);
    // Respond with ACK to the sender of the last message

    void sendReadRequestg(uint8_t rcvADDR, uint8_t reg, bool readAll);
    // Send a read request (for response asynch)
    uint8_t regRead(uint8_t rcvADDR, uint8_t reg);
    // Ask for the contents of a register, wait for response (until timeout)
    void regRespond(uint8_t reg);
    // Respond to the last sender with the contents of the register

    bool regReadAll(uint8_t rcvADDR, uint8_t data[]);
    // Puts the contents of all registers into data[]
    void regRespondAll(void);
    // Respond with all register content



private:
    MCP_CAN _theCAN;
    uint8_t _myADDR = 0xFF;
    uint8_t _numMultADDR = 0;
    uint8_t _activeADDR = 0;
    uint8_t _lastLen = 0;
    uint8_t _lastData[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t _lastSnd = 0xFF;
    uint8_t _regs[7] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
};

CANmsg decodeMSG(uint32_t theMSG);
// Break theMSG into CANmsg struct

uint32_t encodeMSG(uint8_t snd_ID, uint8_t rcv_ID, uint8_t ack_rw, uint8_t reg, uint8_t payload);
// Shift everything into the right position

uint8_t pollPins(uint8_t uniquepins[], uint8_t numToRead);
// Poll uniquepins as input pullup to get unique id

//void printMsgHelper(CANmsg theMSG);
// Print message contents to serial

#endif
/*
  END FILE
*/
