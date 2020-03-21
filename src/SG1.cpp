// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Copyright LowPowerLab LLC 2018, https://www.LowPowerLab.com/contact
// Modified by Daniel Dunn 
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include "SG1.h"
#include "RFM69registers.h"

#include <SPI.h>

#include "utility/rweather/Crypto.h"
#include "utility/rweather/ChaChaPoly.h"

void golay_block_encode(uint8_t * in, uint8_t * out);
bool golay_block_decode(uint8_t * in, uint8_t * out);

ChaChaPoly chachapoly;




//Flags for byte 1 of the header.
#define HEADER_FEC_FIELD 0b11

#define HEADER_FEC_GOLAY 0b01


//If set, the device claims that its time is trusted
#define HEADER_TIME_TRUST_FIELD 0b100

//If set, the device is claiming high precision time as one would get from
//A GPS reciever. Never set this with wose than 100ms if you absolutely must,
//But really it should be sub millisecond.
#define HEADER_TIME_ACCURATE_FIELD 0b1000

//This bit indicates if the packet is a reply
#define HEADER_TYPE_FIELD 0b110000

#define HEADER_TYPE_UNRELIABLE 0b010000
#define HEADER_TYPE_RELIABLE 0b100000
#define HEADER_TYPE_REPLY 0b110000




static uint8_t systemTimeTrust = 0;
static uint64_t lastAccurateTimeSet=0;

static union 
{
  uint8_t entropyPool[20];
  uint64_t entropyRegister;
};

static union
{
//UNIX time in microseconds
int64_t systemTime =-1;
uint8_t systemTimeBytes[8];
};

void RFM69::initSystemTime()
{
 //Better to have a random time, so the encryption using it still works.
  if(systemTime==-1)
  {
    urandom(systemTimeBytes, 8);

    //Make it obviously fake by being negative.
    if(systemTime>0)
    {
      systemTime= -systemTime;
    }
  }
}
static unsigned long systemTimeMicros=0;


// Function to count number of set bits in n
uint8_t bitDiffslt(uint32_t a, uint32_t b,uint8_t m)
{
	// stores the total bits set in n
	int count = 0;
  uint32_t n= a^b;

	for (; n; count++)
  {
		n = n & (n - 1); // clear the least significant bit set
    if(count>=m)
    {
      return 0;
    }
  }

  if(count>=m)
  {
    return 0;
  }

	return true;
}

void doTimestamp()
{
  //Set systemTime to the correct value by adding the micros.
  noInterrupts();
  unsigned long x=micros();
  systemTime += (x-systemTimeMicros);
  systemTimeMicros=x;

  //If it has been ten minutes since accurate setting,
  //The clock has drifted and we aren't accurate anymore.
  
  //TODO: 
  if(systemTimeTrust & TIMETRUST_ACCURATE)
  {
    if(systemTime-lastAccurateTimeSet> 600000LL)
    {
      systemTimeMicros -= TIMETRUST_ACCURATE;
    }
  }
  
  interrupts();
}



//WARNING: Possible bad crypto!! This entropy pool 
//Is entirely about using less code space, that is why it just encrypts the
//Pool with itself as the key.

//However, this should not be capable of ever reducing the entropy,
//On account of the fact we are essentially XORing the state with something
static void mixEntropy()
{
  chachapoly.setKey(entropyPool, 20);
  doTimestamp();
  chachapoly.setIV(systemTimeBytes,8);
  chachapoly.encrypt(entropyPool,entropyPool, 20);
  //We don't use the auth tag
}

uint32_t RFM69::urandomRange(uint32_t f, uint32_t t)
{

  //2^64 is so big the modulo bias shouldn't matter.... Right?
  union 
  {
    uint64_t x;
    uint8_t b[8];
  };
  
  urandom(b,4);
  return (x%(t-f))+f;
}

void RFM69::urandom(uint8_t * target, uint8_t len)
{

  //We are going to use the stream cipher as an RNG.
  //By encrypting the entropy pool with itself as the key
  //That way, no information about the pool itself is ever leaked to
  //the destination.

  chachapoly.clear();

  //Take a reading to further do some mixing.
  entropyRegister+=readTemperature();
  entropyRegister^=readRSSI();

  chachapoly.setKey(entropyPool, 20);
  chachapoly.setIV(systemTimeBytes,8);
  //Copy bytes 32 at a time, re-stirring the pool every time.
  while(len)
  {
    int x = len;
    if (x>20)
    {
      x=20;
    }
   
    chachapoly.encrypt(target, entropyPool, x);
    len-=x;
    target+=x;
  }
  
  //No information should be left about the old state
  mixEntropy();
}


static void RFM69::setTime(int64_t time, uint8_t trust)
{
  noInterrupts();
  //Special value 0 indicates we keep the current time and only set the trust 
  //flags
  if(time)
  {
    systemTime = time;
    systemTimeMicros=micros();
  }
  systemTimeTrust=trust;
  interrupts();
  mixEntropy();
}

uint8_t RFM69::DATA[RF69_MAX_DATA_LEN+1];
uint8_t RFM69::_mode;        // current transceiver state
uint8_t RFM69::DATALEN;
uint8_t RFM69::PAYLOADLEN;
int16_t RFM69::RSSI;          // most accurate RSSI during reception (closest to the reception)
volatile bool RFM69::_haveData;

uint8_t tmpBuffer[128];
uint8_t smallBuffer[128];


int8_t RFM69::getAutoTxPower()
{
  int8_t txPower =13;


  //Detect if we aren't in auto mode
  if(_powerLevel>-127)
  {
    return _powerLevel;
  }
  //If we have a message in the last 3 minutes, that means that we can
  //use TX power control
  if(lastRx>(systemTime-(3L*60L*1000L*1000LL)))
  {
    //Target an RSSI on the other end of -90
    txPower = (-90)+rxPathLoss;
    while(txPower%4)
    {
      txPower++;
    }

    //When we've gone as high as we can go, then we turn on the FEC for an extra boost
    //Pretty sure 12db is legal in most countries, so let's maybe not go above that without
    //manual control.
    if(txPower>12)
    {
      txPower=12;
    }
  }
  else
  {
    txPower=12;
  }
  return txPower;
}

void RFM69::sendSG1(const void* buffer, uint8_t bufferSize, uint8_t * challenge)
{
  doTimestamp();

  int8_t txPower= getAutoTxPower();
  rawSendSG1(buffer, bufferSize, txPower>10, txPower,challenge);

}

void RFM69::rawSendSG1(const void* buffer, uint8_t bufferSize, bool useFEC, int8_t txPower, uint8_t * useChallenge)
{ 
    //The length of everything after the initial 6 byte block
    //8 byte IV, 8 byte MAC, 3 byte channel hint
    int payloadSize=bufferSize+ 8+8+3;

    /*
     * Header Format
     *
     * Length(Actually part of the raw packet framing the RFM69 does, so we don't send it, but it is still the first byte and protected by FEC
     * Flags:
     *    0-1: FEC Type: 0=Only header, 1=Golay code for entire packet
     *    2: Time Trusted

     *
     */
    uint8_t header[3]={0,0,0};
    if(useChallenge==0)
    {
      header[1] |= HEADER_TYPE_UNRELIABLE;
    }

    header[1]|= _headerTimeTrust();

    //If the whole message would be more than 128 bytes we gave to turn off FEC
    if (payloadSize>10)
    {
      useFEC=false;
    }

    //Round up to 4
    while(txPower%4)
      {
        txPower++;
      }

    header[2]=((txPower-(-24))/4);


    rawSetPowerLevel(txPower);
    

    if(systemTimeTrust >= TIMETRUST_CHALLENGERESPONSE)
    {
      header[1] |= HEADER_TIME_TRUST_FIELD;
    }
    //First calculate size, then encrypt, then golay
    if(useFEC)
    {
      //Pad to multiple of 3.
      //But we have to do this by incrementing the actual
      //Buffer size, because the whole packet has the be filled
      //With encrypted data. Otherwise we could not detect which was padding.

      //This means you may recieve more data than you send.
      while(payloadSize%3)
      {
          bufferSize+=1;
          payloadSize=bufferSize+ 8+8+3;
      }

      header[1] |= HEADER_FEC_GOLAY;
      
      
      payloadSize= ((payloadSize)*2);
    }
    else
    {
      payloadSize=bufferSize+ 8+8+3;
    }

    //Add the 6 bytes for the header itself.
    header[0]=payloadSize+6;
    
    chachapoly.clear();
    chachapoly.setKey(channelKey,32);

    doTimestamp();
    

    smallBuffer[0] = ((uint8_t *)&fixedHintSequence)[0];
    smallBuffer[1] = ((uint8_t *)&fixedHintSequence)[1];
    smallBuffer[2] = ((uint8_t *)&fixedHintSequence)[2];

    //Hint comes before the IV, and first IV byte is
    //NodeID so don't put the time in that
    memcpy(smallBuffer+1+3, systemTimeBytes+1,5);

    //First 3 bytes of this are the hint. 4th byte is nodeid part of iv.
    smallBuffer[3]=nodeID;

    //Skip hint sequence to get to the IV
    chachapoly.setIV(smallBuffer+3,8);


    //Keep track of the challenge so we can get replies, but
    //There can't be any replies to a reply, so if this uses a challenge, don't
    if(useChallenge==0)
    {
      memcpy(awaitReplyToIv, smallBuffer+3,8);
    }

    if(useChallenge)
    {
      header[1] |= HEADER_TYPE_REPLY;
    }

    chachapoly.addAuthData(header,3);

    if(useChallenge)
    {
      chachapoly.addAuthData(useChallenge,8);
    }


    //Leave room for hint and IV
    chachapoly.encrypt((uint8_t *)smallBuffer+3+8, (uint8_t *)buffer, bufferSize);
    //memcpy((uint8_t *)smallBuffer+3+8, (uint8_t *)buffer, bufferSize);
    chachapoly.computeTag(smallBuffer+3+8+bufferSize,8);

    debug("Enc tag,b0");
    debug(smallBuffer[3+8+bufferSize]);
    debug(smallBuffer[3+8]);


    if (useFEC)
    {
        for(unsigned char i=0;i<(bufferSize+ 8+8+3);i+=3)
        {
            golay_block_encode(smallBuffer+i, tmpBuffer+6+(i*2));
        }
       
    }
    else {
        memcpy(tmpBuffer+6, smallBuffer, payloadSize);
    }

    golay_block_encode(header,tmpBuffer);



    //+1 skip the length  byte that is already part of the RFM69 raw framing that the lib handles at a lower level.
    send(tmpBuffer+1, header[0]);
}





void RFM69::send(const void* buffer, uint8_t bufferSize)
{
  writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  uint32_t now = millis();
  while (!canSend() && millis() - now < RF69_CSMA_LIMIT_MS) receiveDone();
  sendFrame(buffer, bufferSize);
}

uint8_t rxHeader[8];

void RFM69::recalcBeaconBytes() {
  //Recalc the hint sequences for this 16s period
  doTimestamp();

  //To tolerate misalignment, calculate at +3 and -3 seconds from the current time,
  //And cache until the +3 time changes.

  union{ 
      //Add 3 seconds to push us into the next 
      uint64_t newIntervalNumber = (systemTime+3000000LL)/16777216;
      uint8_t IV[8];
  };

  union{ 
      //Add 3 seconds to push us into the previous 
      uint64_t oldIntervalNumber = (systemTime-3000000LL)/16777216;
      uint8_t oldIV[8];
  };

  uint8_t input[8]={0,0,0,0};
  if (newIntervalNumber==intervalNumber)
  {
    return;
  }

  //Ensure byte 3 is set to zero, it's a padding byte
  privateHintSequence=0;
  privateWakeSequence=0;
  newPrivateHintSequence=0;
  newPrivateWakeSequence=0;

  chachapoly.clear();
  chachapoly.setKey(channelKey,32);
  chachapoly.setIV(IV,8);

  //8 bytes total, first 4 are the hint sequence, next are the wake sequence.
  chachapoly.encrypt((uint8_t*)&newPrivateHintSequence,input,3);
  chachapoly.encrypt((uint8_t*)&newPrivateWakeSequence,input,3);

  //Now calc the old. They may be the same, that's fine.
  chachapoly.clear();
  chachapoly.setKey(channelKey,32);
  chachapoly.setIV(oldIV,8);

  //8 bytes total, first 4 are the hint sequence, next are the wake sequence.
  chachapoly.encrypt((uint8_t*)&privateHintSequence,input,3);
  chachapoly.encrypt((uint8_t*)&privateWakeSequence,input,3);

}



int64_t  RFM69::getPacketTimestamp()
{
  int64_t rxTimestamp=0;
  memcpy( ((uint8_t *)&rxTimestamp)+1, rxIV+1,7);
  return rxTimestamp;
}


void RFM69::doPerPacketTimeFunctions(uint8_t rxTimeTrust)
{


    int64_t rxTimestamp=getPacketTimestamp();
    int64_t diff;


    //Gonna subtract the data time, the preamble, and the sync bytes
    //at 100kbps. We want the start time.

    //This basically find out how far ahead their clock is
    diff=rxTimestamp-(rxTime-((PAYLOADLEN*bitTime)+400LL+400LL));

    //#TODO: This expects that the clock does not get set in between
    //recieve and decode.

    //If we trust this packet more than the clock,
    //Set the clock from this.

    //If time is not set, this allows a replay attack.
    if((rxTimeTrust>=systemTimeTrust) & (rxTimeTrust>=TIMETRUST_SECURE))
    {
      //data+4 is node ID, the rest of the IV is the time.
      if(( (rxTimeTrust & TIMETRUST_ACCURATE) || (abs(diff)>100000LL)))
      {
        systemTime+=diff;
      }
      else
      {
        //For small differences with inaccurate clocks, average
        //Our time and theirs, to hopefully get some kind of consenseus.
        systemTime+= (diff/2LL);
      }

      //Yes, this can go backwards
      channelTimestampHead=getPacketTimestamp();
      
      
      systemTimeTrust=rxTimeTrust;

      if(rxTimeTrust & TIMETRUST_ACCURATE)
      {
        lastAccurateTimeSet=systemTime;
      }
    }


    


}
// internal function - interrupt gets called when a packet is received
void RFM69::interruptHandler() {
    
  if (_mode == RF69_MODE_RX && (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY))
  {
    wakeRequestFlag=0;

    doTimestamp();
    rxTime = systemTime;
    setMode(RF69_MODE_STANDBY);
    select();
    SPI.transfer(REG_FIFO & 0x7F);
    PAYLOADLEN = SPI.transfer(0);
    debug(PAYLOADLEN);

  
    for (uint8_t i = 0; i < (PAYLOADLEN-1); i++)
    {
        DATA[i]=SPI.transfer(0);
    }
    DATALEN = PAYLOADLEN-1;
    
    unselect();
    setMode(RF69_MODE_RX);    
    DATA[DATALEN] = 0; // add null at end of string // add null at end of string  
  }
  RSSI = readRSSI();

}


/*
After recieving a packet, call this to decode it.
Returns 1 if the message was an SG1 packet addressed to 
Our channel.

If True, DATA and DATALEN will be the decoded and decrypted payload.
*/
bool RFM69::decodeSG1()
{
    int8_t _rssi = RSSI;

    doTimestamp();
    //For keeping track of how much we trust the incoming timestamp
    uint8_t rxTimeTrust = 0;

    //First byte is the actual length from the framing, which includes itself
    tmpBuffer[0]=PAYLOADLEN;

    //Both the packet time and the RSSI are unpredictable.
    entropyRegister+=RSSI;
    entropyRegister+= micros();

   //Copy evrything in the header but the length nyte we already have
    memcpy(tmpBuffer+1, DATA, 5);
  
    if(golay_block_decode(tmpBuffer,rxHeader))
    {
        return false;
    }
    //subtract 1 from header, to skip past the implicit length byte,
    //It now represents the length of DATA
    DATALEN=rxHeader[0]-1;
    int8_t txRSSI = ((rxHeader[2]& 0b00001111)*4)-24;

    //Buffer overflow prevention
    if(DATALEN>128)
    {
        DATALEN=128;
    }
    


    //Check what kind of FEC we are supposed to be using.
    if((rxHeader[1]& HEADER_FEC_FIELD))
    {
      debug("fecr");
        //Full packet FEC mode. N
        for (uint8_t i = 0; i < (DATALEN-5); i+=6)
        {
            //Note that we start at 5 to skip header, not 6, because the lenth byte isn't actually in the buffer
            //Decode all the 3 byte blocks into 6 byte blocks.
            if(golay_block_decode(DATA+i+5,tmpBuffer+(i/2)))
            {
                return false;
            }
            
        }
        //Subtract header, div by 2 because of code rate.
        //We are going to use the FEC decoded packet length, not the real one,
        //To allow us to get rid of any extra data after the end caused by a 1 to 0 flip.
        DATALEN=(DATALEN-5)/2;

    }
    else {
        DATALEN=(DATALEN-5);
        //Subtract header, but use the raw packet data.
        memcpy(tmpBuffer, DATA+5,DATALEN);
    }
    
    //3 data bytes indicate a short packet, only using
    //the brief channel hints
    if(DATALEN==3)
      {
        recalcBeaconBytes();
          //Pad with 0, only 3 bytes
          tmpBuffer[3]=0;

          uint32_t rxBeacon = ((uint32_t *)(tmpBuffer))[0];
          if(bitDiffslt(rxBeacon,privateHintSequence,2) |
           bitDiffslt(rxBeacon,privateWakeSequence,2) |
          bitDiffslt(rxBeacon,newPrivateHintSequence,2) |
           bitDiffslt(rxBeacon,newPrivateWakeSequence,2)
          )
          {
            rxPathLoss = txRSSI-_rssi;
            lastRx=systemTime;
          }
          //The wake request flag is used to let them wake us up
          if( bitDiffslt(rxBeacon,privateWakeSequence,2)|  bitDiffslt(rxBeacon,newPrivateWakeSequence,2))
          {
            wakeRequestFlag=1;
          }
          debug("Just a beacon");
          DATALEN=0;
          return false;
      }

    //Back this up here, it's gonna get messed with by the send
    //Function we might use
    uint8_t remainingDataLen = DATALEN;

    if(rxHeader[1]&HEADER_TIME_ACCURATE_FIELD)
    {
      rxTimeTrust+= TIMETRUST_ACCURATE;
    }

    if(rxHeader[1]&HEADER_TIME_TRUST_FIELD)
    {
      rxTimeTrust+= TIMETRUST_CLAIM_TRUST;
    }

    //Let's see if this message is on our channel
    uint32_t rxChannelHint = ((uint32_t *) tmpBuffer)[0];
    //We use a uint32, but it is really 3 bytes
    rxChannelHint &= 0b111111111111111111111111;
    if((rxChannelHint==fixedHintSequence) | 
    (rxChannelHint==privateHintSequence) | 
    (rxChannelHint==privateWakeSequence) |
    (rxChannelHint==newPrivateHintSequence) |
    (rxChannelHint==newPrivateWakeSequence)
    )
    {
      
      

      //NodeID+timestamp is after hint
      memcpy(rxIV,tmpBuffer+3,8);

      //If this is a reply message, we need to authenticate it
      //Based on the timestamp of the previous message.
      //We do that by adding in the authentication data.

      chachapoly.clear();
      chachapoly.setKey(channelKey,32);
      chachapoly.setIV(rxIV,8);

      chachapoly.addAuthData(rxHeader,3);
      
      //If it is not a reply, it has no challenge response properties and therefore
      //we have to check its time
      if(isReply())
      {
        chachapoly.addAuthData((awaitReplyToIv),8);
      }
      else
      {
        //Check We think we have a better clock than them, 
        //lets see if we need to tell them

        //We do this before we check any kind of crypto, because it would be
        //Irrelevant anyway, without a good clock we can't be secure.  

        //We are going to check if we have a local clock. If we do,
        //Then we are going to check if the other node's time is badly inaccurate.
        //If it is we are going to correct them.

        //Don't bother to send stuff if we don't have a valid clock.
        if(systemTimeTrust >= TIMETRUST_CHALLENGERESPONSE)
        {
          //Don't send replies if they actually requested one, leave that to
          //application code.
          if((rxHeader[1]&HEADER_TYPE_FIELD) == HEADER_TYPE_UNRELIABLE)
          {
            int64_t diff=(getPacketTimestamp()-systemTime);
            if (abs(diff) > (250000UL))
            {
              ///the reason we can do this automatically without corrupting state is
              //Replies can't be replied to, so sending this won't overwrite the value that
              //Says what we are watching for, if we were watching for something
              debug("ofset");
              debug((float)diff);
              sendSG1(tmpBuffer, 0, rxIV);
            }
          }
        }

        debug("Clock offset");
        debug((long)((getPacketTimestamp()-systemTime)/1000LL));

        if(getPacketTimestamp()<= channelTimestampHead)
        {
          debug("Rotl");
          DATALEN=0;
          return false;
        }

        //5 seconds max err
        if(getPacketTimestamp()>= (systemTime+5000000LL))
          {

            debug("rtn");


            DATALEN=0;
            return false;
          }

        if(getPacketTimestamp()<= (systemTime-5000000LL))
          {
            debug("Rr2o");
            DATALEN=0;
            return false;
          }
        if(systemTimeTrust<TIMETRUST_CHALLENGERESPONSE)
          {
            debug("rlc");
            DATALEN=0;
            return false;
          }
      }
      
     debug("tg");
     debug(remainingDataLen);

    //By this point, datalen is the FEC encodable par after taking off the header and decoding.

    //Encrypted part starts after the 3 byte hint and 8 byte IV, since we already
    //Took off the header.
    //It also *ends* before the IV, so we subtract 3+8+8
    chachapoly.decrypt(DATA, tmpBuffer+3+8,remainingDataLen-(3+8+8));
  
    
    debug("Ds, b0:");
    debug(tmpBuffer[remainingDataLen-8]);

      if(!chachapoly.checkTag(tmpBuffer+(remainingDataLen-8),8))
      {
        debug("rbc");
        DATALEN=0;
        return false;
      }

      rxPathLoss = txRSSI-_rssi;
      lastRx=systemTime;
      //Remove the hint, IV, and MAC from the length
      //Store in the real data len so the user has it
      DATALEN=remainingDataLen-(3+8+8);

      //Add null terminator that's just kind of always there.
      DATA[DATALEN]=0;

      //other things mess with that register
      RSSI=_rssi;

      //Mark it so we don't accept old packets again.
      channelTimestampHead = getPacketTimestamp();
      //Don't trust if there's no trust flag even if it is authenticated
      if(rxHeader[1]&HEADER_TIME_TRUST_FIELD)
      {
        rxTimeTrust |= TIMETRUST_SECURE;
        if(isReply())
        {
          rxTimeTrust |= TIMETRUST_CHALLENGERESPONSE;
        }
      }
      
      //One reply per message only
      if(isReply())
      {
        memset(awaitReplyToIv,0,8);
      }
      
      doPerPacketTimeFunctions(rxTimeTrust);
      
     return true;
    }
    else
    {
      return false;
    }
}


bool RFM69::isRecieving()
{ 
   noInterrupts();
   //If we did not get the 
   if((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_RXREADY) == 0x00)
   {
       interrupts();
       return false;
   }
   interrupts();
   return true;
}

uint8_t RFM69::_headerTimeTrust()
{
  uint8_t x=0;

  //We only advertise trust if its been set with something
  //Equivalent to challenge response.
  if(systemTimeTrust>=TIMETRUST_CHALLENGERESPONSE)
  {
    x|= HEADER_TIME_TRUST_FIELD;
  }
  if(systemTimeTrust&TIMETRUST_ACCURATE)
  {
    x|=HEADER_TIME_ACCURATE_FIELD;
  }

  return x;
}
void RFM69::_rawSendBeacon(uint8_t power, bool wakeUp)
{
  recalcBeaconBytes();

  while(power%4)
  {
    power++;
  }

  uint8_t header[3]={9,0,0};
  header[2]=(((power-(-24))/4)&0b1111) ;

  header[1]|= _headerTimeTrust();


  golay_block_encode(header,tmpBuffer);
  if(wakeUp)
  {
      memcpy(tmpBuffer+6,&privateWakeSequence,3);
  }
  else
  {
      memcpy(tmpBuffer+6,&privateHintSequence,3);
  }
  
  
  //If we need more than 10dbm, we also probably need to use error correction.
  //This adds an extra 3 bytes
  if(power>10)
  {
    golay_block_encode(tmpBuffer+6,tmpBuffer+6);
    header[1] |= HEADER_FEC_GOLAY;
    header[0]=12;
  }
  //SendFrame does the length byte for you
  sendFrame(tmpBuffer+1, header[0]);
}

bool RFM69::sendBeaconSleep()
{
  while (!canSend())
  {
    /* code */
  }
  _rawSendBeacon(getAutoTxPower(), false);
  //Just long enough for one immediately sent
  delay(3);

  if(receiveDone())
  {
    if(wakeRequestFlag)
    {
      return 1;
    }
    else
    {
      sleep();
    }
  }

  return 0;
}


void RFM69::setChannelKey(unsigned char * key) {
  memcpy(channelKey,key,32);
  recalcBeaconBytes();

  //Get the fixed hint sequence we are going to use
  union{ 
      //Add 3 seconds to push us into the previous 
      uint64_t intervalNumber= 0;
      uint8_t IV[8];
  };

  uint8_t input[8]={0,0,0,0};


  //Ensure byte 3 is set to zero, it's a padding byte
  fixedHintSequence=0;
  
  chachapoly.clear();
  chachapoly.setKey(channelKey,32);
  chachapoly.setIV(IV,8);

  //8 bytes total, first 4 are the hint sequence, next are the wake sequence.
   chachapoly.encrypt((uint8_t*)(&fixedHintSequence),input,3);


}



void RFM69::setBitrate(uint32_t bps)
{
 //Attempt to guess a good filter width.
  setChannelFilter(2*((bps/2) + bps));

  bitTime= 1000000L/bps;

  //Let's go for a modulation index of 1
  if(bps>10000)
  {
    setDeviation(bps);
  }
  else
  {
    //Our minimum FDev is 10khz,
    //There is no reason to go below this,
    //We cannot get channel spacing better than our
    //Crystal accuracy
    setDeviation(15000);
  }
  

  bps = 32000000UL/bps;
  writeReg(REG_BITRATEMSB, bps/256);
  writeReg(REG_BITRATELSB,bps%256);
}

void RFM69::setDeviation(uint32_t hz)
{
  //Round to nearest by adding half the fstep
  hz+=30;

  hz/=61;

  writeReg(REG_FDEVMSB, hz/256);
  writeReg(REG_FDEVLSB,hz%256);
}



//Approximately set the RX bandwidth, rounding up to the next step
//The channel spacing is the filter bandwidth plus 50khz, rounded down
//To the nearest 50KHz

//Possible spacings: 100,150,250,250,500
void RFM69::setChannelFilter(uint32_t hz)
{
  if(hz<=50000)
  {
    channelSpacing = 100;
    writeReg(REG_RXBW,RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_4);
  }
  else if(hz<=100000)
  {
    channelSpacing = 150;
    writeReg(REG_RXBW,RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3);
  }
  else if(hz<=200000)
  {
    channelSpacing = 250;
    writeReg(REG_RXBW,RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_2);
  }
  else if(hz<=330000)
  {
    channelSpacing = 350;
    writeReg(REG_RXBW,RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_1);
  } 
  else
  {
    channelSpacing = 500;
    writeReg(REG_RXBW,RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_1);
  }

  //Recalc the channel number
  setChannelNumber(channelNumber);
}

//Set the RF channel. If the channel is higher than the actual
//Number of channels, wrap around.
void RFM69::setChannelNumber(uint16_t n)
{
  uint32_t minf=0;
  uint32_t maxf=0;

  uint32_t freq = 0;
  channelNumber=n;
  if(freqBand==RF69_915MHZ)
  {
    minf = 902000000UL;
    maxf = 926000000UL;
  }
  else if(freqBand==RF69_868MHZ)
  {
    minf = 863000000UL;
    maxf = 868000000UL;
  }
  else if(freqBand==RF69_433MHZ)
  {
    minf = 433050000UL;
    maxf = 434790000UL;
  }
  
  freq = minf+(((long)channelSpacing*1000L)/2);

  while(n)
  {
    freq += ((long)channelSpacing*1000L);
    if (freq> (maxf- (((long)channelSpacing*1000L)/2)) )
    {
        freq = minf+(((long)channelSpacing*1000L)/2);
    }
    n--;
  }

  setFrequency(freq);

}



void RFM69::getEntropy(int changes) {
  
  //We are looking for 512 changes in the RSSI value 
  //We add every reading together till we see a change, then
  //re-encrypt the entropy pool with itself as the key.

  //This change detection algorithm is very weak due to long strings
  //of values that change every other time. Still, it's probably
  //Usable especially since we are always adding entropy from different sources.


  int x=0;
  int y=0;
  for (int i=0;i<changes;i++)
  {    
    while(1){    //Look for changes in RSSI.
      y =readTemperature(0);
      y += readRSSI();

      //AFC is potentially also a source of randomness
      y+= readReg(REG_FEILSB);
      y+= readReg(REG_AFCLSB);
      
      //It's not really possible for adding uncorrelated values to
      //make something less random, so there's really no reason not to add these.
      entropyRegister+=y;

      //To avoid 
      if((!(y==x)))
        {
          x=y;
          break;
        }
      }
  
    //After every change we detect, mix the entropy.
    //That pool is gonna be SO stirred.
    mixEntropy();
  }

}

bool RFM69::receivedReply()
{
      //We zero it out when we got a reply, so we can use this to check
      return ((uint64_t *)awaitReplyToIv)[0]==0;
 }


bool RFM69::isReply()
{
  if((rxHeader[1]&HEADER_TYPE_FIELD)==HEADER_TYPE_REPLY)
  {
    return 1;
  }
  return 0;
}