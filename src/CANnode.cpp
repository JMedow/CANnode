/*
 CANnode.cpp
 Dec 2023

 Author: Jeremy Medow (jeremy@tungstencustoms.com)
*/

#include "CANnode.h"

// CANnode Class definitions

// Initialize the MCP_CAN object to talk to the MCP2515 IC, with the correct chip select pin
CANnode::CANnode(int CS_PIN):_theCAN(CS_PIN){}

// Initialize MCP2515 chip, set filters to listen for this node's address and global messages
void CANnode::initCAN(uint8_t myID, uint8_t mult = 0){

    _myADDR = myID;
    _activeADDR = 0;

    while (_theCAN.begin(MCP_STDEXT,CAN_SPEED,MCP_8MHZ) != CAN_OK){
        Serial.println("CAN BUS init fail, retrying...");
        delay(100);
    }
    Serial.println("CAN BUS init ok!");

    if(mult>3)
      mult = 3;		// Maximum number of multiple addresses, because of filter size

    _numMultADDR = mult;		// Number of additional addresses

    setStandardFilter(mult);		// Listen to the correct messages

    _theCAN.setMode(MCP_NORMAL);

}

// If you want to listen to everything on the CAN bus, set your mask and filter to zero
void CANnode::setNoFilterExt(void){
    _theCAN.init_Mask(0,1,0);
    _theCAN.init_Filt(0,1,0);
}

// Set filters to listen for this node's address and global messages
void CANnode::setStandardFilter(uint8_t mult = 0){

    Serial.print("Setting filter mult = ");
    Serial.println(mult);

    uint32_t MSGMASK_RCV = encodeMSG(0,0xFF,0,0,0);              // Filter on receiver full id
    uint32_t MSGFILTER_RCV_0 = encodeMSG(0,_myADDR,0,0,0);       // Listen to anyone speaking to _myADDR
    uint32_t MSGFILTER_RCV_1 = encodeMSG(0,_myADDR+1,0,0,0);			// Listen to anyone speaking to _myADDR + 1
    uint32_t MSGFILTER_RCV_2 = encodeMSG(0,_myADDR+2,0,0,0);			// ...
    uint32_t MSGFILTER_RCV_3 = encodeMSG(0,_myADDR+3,0,0,0);			// ...

    uint32_t MSGMASK_SND = encodeMSG(0xFF,0,0,0,0);          // Filter on sender
    uint32_t MSGFILTER_SND = encodeMSG(BROADCAST_SND,0,0,0,0);  // Listen to anything sent _by_ BROADCAST_SND

    // Universal stuff, as above
    _theCAN.init_Mask(0,1,MSGMASK_SND);
    _theCAN.init_Filt(0,1,MSGFILTER_SND);
    _theCAN.init_Filt(1,1,MSGFILTER_SND);

    // Specific stuff, as above
    _theCAN.init_Mask(1,1,MSGMASK_RCV);
    _theCAN.init_Filt(2,1,MSGFILTER_RCV_0);
    _theCAN.init_Filt(3,1,(mult>0)?MSGFILTER_RCV_1:MSGFILTER_RCV_0);		// Only if there's more than one address
    _theCAN.init_Filt(4,1,(mult>1)?MSGFILTER_RCV_2:MSGFILTER_RCV_0);		// Only if there's more than two addresses
    _theCAN.init_Filt(5,1,(mult>2)?MSGFILTER_RCV_3:MSGFILTER_RCV_0);		// Only if there's more than three addresses
}

// Check the MCP2515 to see whether there is a new message in the buffer
bool CANnode::msgAvailable(void){
    return (_theCAN.checkReceive() == CAN_MSGAVAIL);
}

bool CANnode::updateNode(void){
  bool toRet = false;

  if(msgAvailable()){

    CANmsg incoming = getCommand();

    if(incoming.reg == REG_MULT){
      switch(incoming.ackRW){
        case W_ACK:
          toRet = assignMult(incoming.payload,_lastData);
          sndACK(incoming.reg,incoming.payload);
          break;
        case W_NACK:
          toRet = assignMult(incoming.payload,_lastData);
          break;
        case R_NACK:
          regRespondAll();
          break;
      }
    } else {
      switch(incoming.ackRW){
        case W_ACK:
          if(_regs[incoming.reg]!=incoming.payload)   // The register will change
            toRet = true;
          _regs[incoming.reg] = incoming.payload;   // For write with and without ack
          sndACK(incoming.reg,incoming.payload);
          break;
        case W_NACK:
          if(_regs[incoming.reg]!=incoming.payload)   // The register will change
            toRet = true;
          _regs[incoming.reg] = incoming.payload;   // For write with and without ack
          break;
        case R_NACK:
          regRespond(incoming.reg);
          break;
      }
    }
  }

  return toRet;

}

// Put things in place.  Data is always 7 registers.  Only set mask = 1 registers.
bool CANnode::assignMult(uint8_t regMask, uint8_t data[]){
    bool toRet = false;

    for(uint8_t reg = 0; reg < 7; reg++)
      if(bitRead(regMask,reg)){
        if(_regs[reg] != data[reg])     // Something will change
          toRet = true;
        _regs[reg] = data[reg];
      }

    return toRet;
}

// Call this when there is a new message.  It reads the message and dumps the contents into an CANmsg struct and dumps data and len
CANmsg CANnode::getCommand(void){

    uint32_t theMSG;
    _theCAN.readMsgBuf(&theMSG,&_lastLen,_lastData);		// Pull the data from the MCP2515 chip, put data and len into their variables
    CANmsg incoming = decodeMSG(theMSG);				// Break apart the id into a CANmsg for parsing
    setActiveADDR(incoming.rcvID - _myADDR);		// Set the active address (in cases of multiple) based on who was the intended receiver

    _lastSnd = incoming.sndID;						// Keep track of who sent this message for ease of responding

    return incoming;								// Return the CANmsg struct
}

// Send a message to write to a register
bool CANnode::regWrite(uint8_t rcvADDR, uint8_t reg, uint8_t payload, bool toAck = false){

    // Construct the 29-bit id correctly)
    uint32_t msgID = encodeMSG(_myADDR+_activeADDR,rcvADDR,toAck?W_ACK:W_NACK,reg,payload);
    // Send the message to the MCP2515 chip, wait a moment to not overload buffers
    _theCAN.sendMsgBuf(msgID,1,0,_lastData);
    if(TXDELAY)
      delay(TXDELAY);

    if(toAck){
      CANmsg incoming;

      uint8_t numTries = 0;
      uint8_t tempActive = _activeADDR;		// In case of multiple addresses, need to make sure the ACK is going to the right one

      while(numTries < SENDMAX){		// Only try a few times

        uint16_t responsetimer = 0;
        bool response = false;

        while( (!response) && (responsetimer < ACKTIMEOUT) ){	// Until you timeout
          response = msgAvailable();
          if(response){
            incoming = getCommand();
  	        // If an incoming message is from the recipient, is an ACK, and is to the correct sender, then success
            if( (incoming.sndID == rcvADDR) && (incoming.ackRW == ACK) && (_activeADDR == tempActive) )
              return true;
            else
              response = false;
          }
          responsetimer+=5;
          delay(5);
        }

        numTries++;
        if(numTries < SENDMAX){       // So you don't send again after the last wait
          setActiveADDR(tempActive);		// Send as the correct address
          _theCAN.sendMsgBuf(msgID,1,0,_lastData);
          if(TXDELAY)
            delay(TXDELAY);
        }

      }

      return false;   // Didn't get a correct response
    } else
      return true;    // If not waiting for an ACK
}

// Send a message with multiple registers of data
bool CANnode::regWriteMult(uint8_t rcvADDR, uint8_t regMask, uint8_t payloads[], bool toAck = false){

  // Construct the 29-bit id correctly)
  uint32_t msgID = encodeMSG(_myADDR+_activeADDR,rcvADDR,toAck?W_ACK:W_NACK,REG_MULT,regMask);
  // Send the message to the MCP2515 chip, wait a moment to not overload buffers
  _theCAN.sendMsgBuf(msgID,1,7,payloads);
  if(TXDELAY)
    delay(TXDELAY);

  if(toAck){
    CANmsg incoming;

    uint8_t numTries = 0;
    uint8_t tempActive = _activeADDR;		// In case of multiple addresses, need to make sure the ACK is going to the right one

    while(numTries < SENDMAX){		// Only try a few times

      uint16_t responsetimer = 0;
      bool response = false;

      while( (!response) && (responsetimer < ACKTIMEOUT+(21)*ACKTIMEOUTDATA) ){	// Until you timeout
        response = msgAvailable();
        if(response){
          incoming = getCommand();
          // If an incoming message is from the recipient, is an ACK, and is to the correct sender, then success
          if( (incoming.sndID == rcvADDR) && (incoming.ackRW == ACK) && (_activeADDR == tempActive) )
            return true;
          else
            response = false;
        }
        responsetimer+=5;
        delay(5);
      }

      numTries++;
      if(numTries < SENDMAX){       // So you don't send again after the last wait
        setActiveADDR(tempActive);		// Send as the correct address
        _theCAN.sendMsgBuf(msgID,1,7,payloads);
        if(TXDELAY)
          delay(TXDELAY);
      }

    }

    return false;   // Didn't get a correct response
  } else
    return true;    // If not waiting for an ACK
}

// Return which (of multiple) addresses are active
uint8_t CANnode::whichADDR(void){
	return _activeADDR;
}

// Set the active address, subject to a maximum (user error handling)
void CANnode::setActiveADDR(uint8_t which){
	if(which>_numMultADDR)
		which = _numMultADDR;
	_activeADDR = which;
}

// Return the current active address (or the only address if not multiple)
uint8_t CANnode::myADDR(void){
    return _myADDR+_activeADDR;
}

// Fill an array with the data from the last incoming message, and return the length of that array
uint8_t CANnode::lastData(uint8_t data[]){
    for(int i = 0; i < _lastLen; i++)
        data[i] = _lastData[i];
    return _lastLen;
}

// Return the value of a register, subject to error handling
uint8_t CANnode::regVal(uint8_t reg){
  if(reg<8)
    return _regs[reg];
  else
    return 0xFF;
}

// Set the value of a register, subject to error handling
bool CANnode::setReg(uint8_t reg, uint8_t val){
  bool toRet = false;
  if(reg < 8){
    if(_regs[reg] != val)
      toRet = true;
    _regs[reg] = val;
  }
  return toRet;
}

// Send ACK to whichever node last sent you a message
void CANnode::sndACK(uint8_t theReg, uint8_t thePayload){

    uint32_t msgID = encodeMSG(_myADDR+_activeADDR,_lastSnd,ACK,theReg,(theReg == REG_MULT)?thePayload:_regs[theReg]);
    _theCAN.sendMsgBuf(msgID,1,(theReg == REG_MULT)?7:0,_regs);

    if(TXDELAY)
      delay(TXDELAY);
}

// Returns the ID of the last node to send a message
uint8_t lastSender(void){
  return(_lastSnd);
}


// Respond to the last sender with the contents of the register
void CANnode::regRespond(uint8_t reg){

    uint32_t msgID = encodeMSG(_myADDR+_activeADDR,_lastSnd,ACK,reg,_regs[reg]);
    // Send the message to the MCP2515 chip, wait a moment to not overload buffers
    _theCAN.sendMsgBuf(msgID,1,0,_lastData);
    if(TXDELAY)
      delay(TXDELAY);
}

// Send the request to read a register (or mult)
void CANnode::sendReadRequest(uint8_t rcvADDR, uint8_t reg, bool readAll = false){
  uint32_t msgID;
  if(readAll)
    msgID = encodeMSG(_myADDR+_activeADDR,rcvADDR,R_NACK,REG_MULT,0xFF);
  else
    msgID = encodeMSG(_myADDR+_activeADDR,rcvADDR,R_NACK,reg,0xFF);
  // Send the message to the MCP2515 chip, wait a moment to not overload buffers
  _theCAN.sendMsgBuf(msgID,1,0,_lastData);
  if(TXDELAY)
    delay(TXDELAY);
}

// Send a READ message a few times, and return the response subject to a timeout
// Note that other incoming messages during the timeout period will be IGNORED
uint8_t CANnode::regRead(uint8_t rcvADDR, uint8_t reg){


  sendReadRequest(rcvADDR, reg);

  CANmsg incoming;

  uint8_t numTries = 0;
  uint8_t tempActive = _activeADDR;		// In case of multiple addresses, need to make sure the ACK is going to the right one

  while(numTries < SENDMAX){		// Only try a few times

    uint16_t responsetimer = 0;
    bool response = false;

    while( (!response) && (responsetimer < ACKTIMEOUT) ){	// Until you timeout
      response = msgAvailable();
      if(response){
        incoming = getCommand();
        // If an incoming message is from the recipient, is an ACK, and is to the correct sender, then success
        if( (incoming.sndID == rcvADDR) && (incoming.ackRW == ACK) && (_activeADDR == tempActive) )
          return incoming.payload;
        else
          response = false;
      }
      responsetimer+=5;
      delay(5);
    }

    numTries++;
    if(numTries < SENDMAX){       // So you don't send again after the last wait
      setActiveADDR(tempActive);		// Send as the correct address
      sendReadRequest(rcvADDR, reg);
    }

  }

  return NO_RESPONSE;   // Didn't get a correct response

}

bool CANnode::regReadAll(uint8_t rcvADDR, uint8_t data[]){

  sendReadRequest(rcvADDR, REG_MULT, true);

  CANmsg incoming;

  uint8_t numTries = 0;
  uint8_t tempActive = _activeADDR;		// In case of multiple addresses, need to make sure the ACK is going to the right one

  while(numTries < SENDMAX){		// Only try a few times

    uint16_t responsetimer = 0;
    bool response = false;

    while( (!response) && (responsetimer < ACKTIMEOUT) ){	// Until you timeout
      response = msgAvailable();
      if(response){
        incoming = getCommand();
        // If an incoming message is from the recipient, is an ACK, and is to the correct sender, then success
        if( (incoming.sndID == rcvADDR) && (incoming.ackRW == ACK) && (_activeADDR == tempActive) ){
          for(uint8_t reg = 0; reg < 7; reg++)
            data[reg] = _lastData[reg];
          return true;
        }
        else
          response = false;
      }
      responsetimer+=5;
      delay(5);
    }

    numTries++;
    if(numTries < SENDMAX){       // So you don't send again after the last wait
      setActiveADDR(tempActive);		// Send as the correct address
      sendReadRequest(rcvADDR, REG_MULT, true);
    }

  }

  return false;   // Didn't get a correct response

}

void CANnode::regRespondAll(void){
    uint32_t msgID = encodeMSG(_myADDR+_activeADDR,_lastSnd,ACK,REG_MULT,0xFF);
    // Send the message to the MCP2515 chip, wait a moment to not overload buffers
    _theCAN.sendMsgBuf(msgID,1,7,_regs);
    if(TXDELAY)
      delay(TXDELAY);
};


// OTHER STUFF -----------------

// Take a 29-bit extended CAN frame id and break it up into component CANnode parts
CANmsg decodeMSG(uint32_t theID){
    CANmsg dec;

    dec.sndID = (theID >> 21) & 0xFF;
    dec.rcvID = (theID >> 13) & 0xFF;
    dec.ackRW = (theID >> 11) & 0b11;
    dec.reg = (theID >> 8) & 0b111;
    dec.payload = theID & 0xFF;

    return dec;
}

// Basically the opposite of decodeMSG - take an all the parts and make a 29 bit message
uint32_t encodeMSG(uint8_t snd_ID, uint8_t rcv_ID, uint8_t ack_rw, uint8_t reg, uint8_t payload){
  uint8_t maskedReg = reg & 0b111;        // 3 bits only
  uint8_t maskedAckRW = ack_rw & 0b11;    // 2 bits only

  return ((uint32_t)snd_ID << 21) | ((uint32_t)rcv_ID << 13) | ((uint32_t)maskedAckRW << 11) | ((uint32_t)maskedReg << 8) | (uint32_t)payload;
}

// Take an array of pin numbers, digitalRead() them with pullups enabled, and put the output into a byte
// Allows grounding pins to set a unique id in hardware, so software can be identical
uint8_t pollPins(int uniquepins[], int numToRead){
    uint8_t temp = 0;

    for(int i = 0; i < numToRead; i++){
        pinMode(uniquepins[i],INPUT_PULLUP);
        temp |= digitalRead(uniquepins[i])?0:(1<<i);
    }

    return temp;
}

// Take an CANmsg and dump its contents to the serial monitor
/*
void printMsgHelper(CANmsg theMSG){
    Serial.print("SND: ");
    Serial.print(theMSG.sndID,HEX);
    Serial.print("   RCV: ");
    Serial.print(theMSG.rcvID,HEX);
    Serial.print("   RWA: ");
    switch(theMSG.ackRW){
      case W_NACK:
        Serial.print("W");
        break;
      case W_ACK:
        Serial.print("W_ACK");
        break;
      case R_NACK:
        Serial.print("R");
        break;
      case ACK:
        Serial.print("ACK");
        break;
    }
    Serial.print("   Register: ");
    Serial.print(theMSG.reg);
    Serial.print("   Payload: ");
    Serial.println(theMSG.payload);

}*/

/*
  END FILE
*/
