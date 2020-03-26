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
#include "utility/Driver_RFM69registers.h"

#include <SPI.h>

#include "utility/rweather/Crypto.h"
#include "utility/rweather/ChaChaPoly.h"

#include "utility/rweather/CryptoLW.h"
#include "utility/rweather/Crypto.h"
#include "utility/rweather/SpeckTiny.h"
#include "utility/rweather/EAX.h"
#include "utility/rweather/Curve25519.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) 

#include "utility/sleepn0m1/Sleep_n0m1.h"
Sleep sleepLib;
void RFM69::sleepMCU(unsigned long x)
{
  sleepLib.pwrDownMode();
  sleepLib.sleepDelay(x);
  addSleepTime(x);
}

#else
//fallback
void RFM69::sleepMCU(unsigned long x)
{
  delay(x);
}
#endif




void golay_block_encode(uint8_t * in, uint8_t * out);
bool golay_block_decode(uint8_t * in, uint8_t * out);

//Leaving all this in, just in case someone wants to make 
//Some crazy ultralight variant. For now, I think ChaCha is the better choice,
//Because it's easier to find implementations
//EAX<SpeckTiny> cipherContext;
ChaChaPoly cipherContext;

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
#define HEADER_TYPE_FIELD            0b1110000

#define HEADER_TYPE_UNRELIABLE       0b0010000
#define HEADER_TYPE_RELIABLE         0b0100000
//Any reply types will always have bit 6 set
#define HEADER_TYPE_REPLY            0b1000000
#define HEADER_TYPE_REPLY_SPECIAL    0b1010000

#define HEADER_TYPE_RELIABLE_SPECIAL 0b0110000




static uint8_t systemTimeTrust = 0;
static uint64_t lastAccurateTimeSet=0;

static unsigned long arduinoMillisOffset = 0;

static union
{
//UNIX time in microseconds
int64_t systemTime =-1;
uint8_t systemTimeBytes[8];
};

unsigned long RFM69::monotonicMillis()
{
  return arduinoMillisOffset+millis();
}

void RFM69::addSleepTime(uint32_t m)
{
  noInterrupts();
  arduinoMillisOffset+=m;
  systemTime+=(m*1000LL);
  interrupts();
}


static union 
{
  uint8_t entropyPool[20];
  uint64_t entropyRegister;

  //BAD CRYPTO ALERT!!!
  //This is shared with a totally insecure RNG
  //This leaks small amounts of information about
  //these 32 bits. This should normally not matter,
  //As we re-encrypt the entire pool both before and after use
  uint32_t entropyRegister32;
};

//The non-secure RNG we use for packet timings
//static uint32_t rngState;
#define rngState entropyRegister32


void RFM69::initSystemTime()
{
 //Better to have a random time, so the encryption using it still works.
  if(systemTime==-1)
  {
    urandom(systemTimeBytes, 8);

    //Make it obviously fake by being negative.
    if(systemTime>0LL)
    {
      systemTime= (-systemTime);
    }
  }
}
static unsigned long systemTimeMicros=0;




void doTimestamp(uint32_t adjustment=0)
{
  //Set systemTime to the correct value by adding the micros.
  //Adjustment adds up to that many micros but will never make the clock go
  //backwards.
  noInterrupts();
  unsigned long x=micros();
  unsigned long x2= x-systemTimeMicros;
  int64_t y = x2-adjustment;
  
  //Clock doesn't go backwards except when actually set.
  
  if(y<0)
  {
    //Don't let the adjustment slow things down
    //to zero, that could cause problems.
    if(x2>0)
    {
      y=1;
    }
    else
    {
      y=0;
    }
  }

  systemTime += y;
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

int64_t RFM69::unixMicros(uint32_t adj)
{
  doTimestamp(adj);
  return systemTime;
}

//WARNING: Possible bad crypto!! This entropy pool 
//Is entirely about using less code space, that is why it just encrypts the
//Pool with itself as the key.

//However, this should not be capable of ever reducing the entropy,
//On account of the fact we are essentially XORing the state with something
static void mixEntropy()
{
  cipherContext.setKey(entropyPool, 20);
  doTimestamp();
  cipherContext.setIV(systemTimeBytes,8);
  cipherContext.encrypt(entropyPool,entropyPool, 20);
  //We don't use the auth tag
}


uint32_t RFM69::xorshift32()
{
	/* Algorithm "xor" from p. 4 of Marsaglia, "Xorshift RNGs" */
  rngState+=micros();
  rngState ^= rngState << 13;
	rngState ^= rngState >> 17;
	rngState ^= rngState << 5;
	return rngState;
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


 //Take a reading to further do some mixing.
  entropyRegister+=readTemperature();
  entropyRegister^=readRSSI();

  //Mix entropy before we actually use it, so that we don't
  //get any reuse with the non-secure uses of the pool.
  mixEntropy();

  cipherContext.clear();
 

  cipherContext.setKey(entropyPool, 20);
  cipherContext.setIV(systemTimeBytes,8);
  //Copy bytes 20 at a time. Note that
  //We never reveal the raw content of the entropy pool for any reason.
  //We just use the encrypted keystream XORed with the pool
  while(len)
  {
    int x = len;
    if (x>20)
    {
      x=20;
    }
   
    cipherContext.encrypt(target, entropyPool, x);
    len-=x;
    target+=x;
  }
  
  //No information should be left about the old state
  mixEntropy();
}

void RFM69::addEntropy(uint32_t x)
{
  entropyRegister32+=x;
}

void RFM69::setTime(int64_t time, uint8_t trust)
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
  int16_t txPower =12;

  int8_t targetRSSI;

  //Higher speed needs more power
  if(bitTime>=9)
  {
    targetRSSI = -90;
  }
  else
  {
    targetRSSI= -85;
  }
  

  //Detect if we aren't in auto mode
  if(_powerLevel>-127)
  {
    return _powerLevel;
  }


  //If we have a message in the last 3 minutes, that means that we can
  //use TX power control
  if(lastSG1Presence>(monotonicMillis()-(3L*60L*1000L)))
  {
    //Target an RSSI on the other end of -90
    txPower = targetRSSI;
    txPower +=rxPathLoss;
    
    
    //For every failed or canceled request that's happened, add 3db of power.
    //txPower+= min(requestedReply*3,12);

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
  return (uint8_t)txPower;
}

void RFM69::sendSG1(const void* buffer, uint8_t bufferSize, uint8_t * challenge, uint8_t * key)
{
  int8_t txPower= getAutoTxPower();
  //Reset the failed request tracking
  requestedReply=0;
  rawSendSG1(buffer, bufferSize, txPower>10, txPower,challenge, key, HEADER_TYPE_UNRELIABLE);
}

void RFM69::sendSG1Request(const void* buffer, uint8_t bufferSize)
{
  int8_t txPower= getAutoTxPower();
  requestedReply+=1;
  rawSendSG1(buffer, bufferSize, txPower>10, txPower,0, 0,HEADER_TYPE_RELIABLE);
}

void RFM69::sendSG1Reply(const void* buffer, uint8_t bufferSize)
{
  int8_t txPower= getAutoTxPower();
  rawSendSG1(buffer, bufferSize, txPower>10, txPower,rxIV, 0, HEADER_TYPE_REPLY);
}

void RFM69::rawSendSG1(const void* buffer, uint8_t bufferSize, bool useFEC, int8_t txPower, 
uint8_t * useChallenge, uint8_t * key, uint8_t packetType)
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
  
    header[1] |= packetType;
    

    header[1]|= _headerTimeTrust();

    //If the whole message would be big we have to turn off FEC
    if (payloadSize>64)
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
    
    cipherContext.clear();
    if(key==0)
    {
      cipherContext.setKey(defaultChannel.channelKey,32);
    }
    else
    {
      cipherContext.setKey(key,32);
    }
    
    doTimestamp();
    

    smallBuffer[0] = ((uint8_t *)&(defaultChannel.fixedHintSequence))[0];
    smallBuffer[1] = ((uint8_t *)&(defaultChannel.fixedHintSequence))[1];
    smallBuffer[2] = ((uint8_t *)&(defaultChannel.fixedHintSequence))[2];

    //Hint comes before the IV, and first IV byte is
    //NodeID so don't put the time in that
    memcpy(smallBuffer+1+3, systemTimeBytes+1,7);

    //First 3 bytes of this are the hint. 4th byte is nodeid part of iv.
    smallBuffer[3]=nodeID;

    //Skip hint sequence to get to the IV
    cipherContext.setIV(smallBuffer+3,8);


    //Keep track of the challenge so we can get replies, but
    //There can't be any replies to a reply, so if this uses a challenge, don't
    if(useChallenge==0)
    {
      memcpy(awaitReplyToIv, smallBuffer+3,8);
    }

    cipherContext.addAuthData(header,3);

    if(useChallenge)
    {
      cipherContext.addAuthData(useChallenge,8);
    }


    //Leave room for hint and IV
    cipherContext.encrypt((uint8_t *)smallBuffer+3+8, (uint8_t *)buffer, bufferSize);
    //memcpy((uint8_t *)smallBuffer+3+8, (uint8_t *)buffer, bufferSize);
    cipherContext.computeTag(smallBuffer+3+8+bufferSize,8);

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
   
    //Similarly subtract one from the number, because send expects the actual data len
    //exculding the length byte itself
    lastSentSG1=monotonicMillis();
    send(tmpBuffer+1, header[0]-1);
}






uint8_t rxHeader[8];

void SG1Channel::recalcBeaconBytes() {
  //Recalc the hint sequences for this 16s period
  doTimestamp();

  //To tolerate misalignment, calculate the current, and also the next closest
  //interval, and accept either.

  union{ 
      //Add 8 seconds to push us into the next period, 
      uint64_t currentIntervalNumber = (systemTime)/16777216LL;
      uint8_t currentIV[8];
  };

  union{ 
      uint64_t altIntervalNumber = (systemTime+8000000L)/16777216LL;
      uint8_t altIV[8];
  };

  //So we tried incrementing, but it didn't change, meaning
  //the actual closest interval is the previous one
  //
  if(currentIntervalNumber==altIntervalNumber)
  {
    altIntervalNumber = (systemTime-8000000L)/16777216LL;
  }

  uint8_t input[8]={0,0,0,0};
  //detect cache
  if (currentIntervalNumber==intervalNumber)
  {
    return;
  }

  //set cache marker
  intervalNumber = currentIntervalNumber;

  //Ensure byte 3 is set to zero, it's a padding byte
  privateHintSequence=0;
  privateWakeSequence=0;
  altPrivateHintSequence=0;
  altPrivateWakeSequence=0;

  cipherContext.clear();
  cipherContext.setKey(channelKey,32);
  cipherContext.setIV(altIV,8);

  //8 bytes total, first 4 are the hint sequence, next are the wake sequence.
  cipherContext.encrypt((uint8_t*)&altPrivateHintSequence,input,3);
  cipherContext.encrypt((uint8_t*)&altPrivateWakeSequence,input,3);

  //If the old and new IVs are the same, just copy,
  //Because that encrypt takes a millisecond.
  if (memcmp(currentIV,altIV,8))
  {
    cipherContext.clear();
    cipherContext.setKey(channelKey,32);
    cipherContext.setIV(currentIV,8);

    //8 bytes total, first 4 are the hint sequence, next are the wake sequence.
    cipherContext.encrypt((uint8_t*)&privateHintSequence,input,3);
    cipherContext.encrypt((uint8_t*)&privateWakeSequence,input,3);
  }
  else
  {
    memcpy((uint8_t*)&privateHintSequence,(uint8_t*)&altPrivateHintSequence,3);
    memcpy((uint8_t*)&privateWakeSequence,(uint8_t*)&altPrivateWakeSequence,3);
  }
  
}



int64_t  RFM69::getPacketTimestamp()
{
  int64_t rxTimestamp=0;
  memcpy( ((uint8_t *)(&rxTimestamp))+1, rxIV+1,7);
  return rxTimestamp;
}


void RFM69::doPerPacketTimeFunctions(uint8_t rxTimeTrust)
{

    int64_t rxTimestamp=0;
    int64_t diff;

    //Gonna subtract the data time, the preamble, and the sync bytes.
    //we want the start time.

    rxTimestamp=getPacketTimestamp();

    //This basically find out how far ahead their clock is
    diff=rxTimestamp-(rxTime-((PAYLOADLEN*bitTime)+400LL+400LL));

    //#TODO: This expects that the clock does not get set in between
    //recieve and decode.

    //If we trust this packet more than the clock,
    //Set the clock from this.

    if((rxTimeTrust>=systemTimeTrust) & (rxTimeTrust>=TIMETRUST_CHALLENGERESPONSE) )
    {
      debug("t");
      //data+4 is node ID, the rest of the IV is the time.
      
      //More than 2s, just jump. Less than that, use the
      //safe version and do slow adjustment without going backwards.
      debug((int32_t)(systemTime/1000000LL));
        debug((int32_t)(diff/1000000LL));
        debug((int32_t)(rxTimestamp/1000000LL));
        debug((int32_t)(rxTime/1000000LL));
      if((abs(diff)>2000000LL))
      {
        debug("ad");
        systemTime+=diff;

        //Yes, this can go backwards. It's somewhat of a nonce reuse hazard.
        //You got a better plan?
        channelTimestampHead=getPacketTimestamp();
      }
      else
      {
        if(rxTimeTrust>systemTimeTrust)
        {
          doTimestamp(diff);
        }
        else
        {
          doTimestamp(diff/2);
        }
      }

          
      systemTimeTrust=rxTimeTrust;

      if(rxTimeTrust & TIMETRUST_ACCURATE)
      {
        lastAccurateTimeSet=systemTime;
      }
    }
    else if ((rxTimeTrust& TIMETRUST_ACCURATE) && (rxTimeTrust&&TIMETRUST_SECURE))
    {
      debug("ta");
      if (abs(diff) <30000L)
      {
        //We can do very small adjustments continually, whenever we get a
        //new packet
        doTimestamp(diff);
      }
    }
}


bool RFM69::decodeSG1Header()
{
    //SG1 packets can't be shorter than this.
    if(DATALEN<8)
    {
      debug("tooshort");
      return false;
    }

    //First byte is the actual length from the framing, which includes itself
    tmpBuffer[0]=PAYLOADLEN;
   //Copy evrything in the header but the length nyte we already have
    memcpy(tmpBuffer+1, DATA, 5);

    if(golay_block_decode(tmpBuffer,rxHeader))
    {
        debug("badheader");
        return false;
    }
    return true;
}

uint32_t RFM69::readHintSequence()
{
  return ((uint32_t *)(DATA+5+6))[0];
}

/*
After recieving a packet, call this to decode it.
Returns 1 if the message was an SG1 packet addressed to 
Our channel.

If True, DATA and DATALEN will be the decoded and decrypted payload.
*/
bool RFM69::decodeSG1(uint8_t * key)
{
    int8_t _rssi = RSSI;

    if(decodeSG1Header()==false)
    {
      return false;
    }

    doTimestamp();
    //For keeping track of how much we trust the incoming timestamp
    uint8_t rxTimeTrust = 0;

    //Set to true later
    wakeRequestFlag=false;
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
                debug("badfec");
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
          defaultChannel.recalcBeaconBytes();
          //Pad with 0, only 3 bytes
          tmpBuffer[3]=0;
          debug("phs");
          debug(defaultChannel.privateHintSequence);
          debug(defaultChannel.privateWakeSequence);

          uint32_t rxBeacon = ((uint32_t *)(tmpBuffer))[0];
          debug(rxBeacon);
          if((rxBeacon==defaultChannel.privateHintSequence) ||
           (rxBeacon==defaultChannel.privateWakeSequence) ||
           (rxBeacon==defaultChannel.altPrivateHintSequence) ||
           (rxBeacon==defaultChannel.altPrivateWakeSequence)
          )
          {
            rxPathLoss = txRSSI-_rssi;
            
            
            lastSG1Presence=systemTime;
            debug("chbeacon");
          
            //The wake request flag is used to let them wake us up
            if( (rxBeacon==defaultChannel.privateWakeSequence)||  (rxBeacon==defaultChannel.altPrivateWakeSequence))
            {            
              debug("chwake");
              wakeRequestFlag=true;
            }
            else
              //It's not one of the wake sequences, just a normal beacon.
              //That means we can send it a message to keep it awake.
              if(keepRemotesAwake)
              {
                sendBeacon(true);
              }
              else
              {
                //Just send a normal beacon, not a wake beacon.
                sendBeacon(false);
              }
              
            {
              /* code */
            }
              debug("relevant beacon");
              DATALEN=0;
              return true;
          }
              debug("irrelevant beacon");
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
    if((rxChannelHint==defaultChannel.fixedHintSequence) | 
    (rxChannelHint==defaultChannel.privateHintSequence) | 
    (rxChannelHint==defaultChannel.privateWakeSequence) |
    (rxChannelHint==defaultChannel.altPrivateHintSequence) |
    (rxChannelHint==defaultChannel.altPrivateWakeSequence)
    )
    {
      
      

      //NodeID+timestamp is after hint
      memcpy(rxIV,tmpBuffer+3,8);

      //If this is a reply message, we need to authenticate it
      //Based on the timestamp of the previous message.
      //We do that by adding in the authentication data.

      cipherContext.clear();
      cipherContext.setKey(defaultChannel.channelKey,32);
      cipherContext.setIV(rxIV,8);

      cipherContext.addAuthData(rxHeader,3);
      
      //If it is not a reply, it has no challenge response properties and therefore
      //we have to check its time
      if(isReply())
      {
        cipherContext.addAuthData((awaitReplyToIv),8);
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
            int64_t diff=(getPacketTimestamp()-((rxTime-((PAYLOADLEN*bitTime)+400LL+400LL))));
            //We are going to try to maintain tight timing
            //100ms is the limit before we tell them.
            //Can't reply to a reply

            //For now, we are just going to automatically reply to all these
            //RELIABLE_SPECIAL with the current time.
            //Todo??
            if (((abs(diff) > (100000LL)) &&(!isReply())) || 
              ((rxHeader[1]&HEADER_TYPE_FIELD) == HEADER_TYPE_RELIABLE_SPECIAL)
            )
            {

              //Don't send replies if they actually requested one, leave that to
              //application code.
              //However, should the packet be more than 5s off, we can't pass this to application
              //code anyway
              if(((rxHeader[1]&HEADER_TYPE_FIELD) == HEADER_TYPE_UNRELIABLE) || 
              (abs(diff)>5000000LL) || 
              ((rxHeader[1]&HEADER_TYPE_FIELD) == HEADER_TYPE_RELIABLE_SPECIAL))
              {
              ///the reason we can do this automatically without corrupting state is
              //Replies can't be replied to, so sending this won't overwrite the value that
              //Says what we are watching for, if we were watching for something
              debug("ofset");
              debug((int32_t)(diff/1000000LL));
              int8_t autoPwr = getAutoTxPower();
              rawSendSG1(tmpBuffer, 0, autoPwr>10, autoPwr,rxIV,0,HEADER_TYPE_REPLY_SPECIAL);
            }
          }
        }


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

      //By this point, datalen is the FEC encodable par after taking off the header and decoding.

      //Encrypted part starts after the 3 byte hint and 8 byte IV, since we already
      //Took off the header.
      //It also *ends* before the IV, so we subtract 3+8+8
      cipherContext.decrypt(DATA, tmpBuffer+3+8,remainingDataLen-(3+8+8));
    
      
      if(!cipherContext.checkTag(tmpBuffer+(remainingDataLen-8),8))
      {
        debug("rbc");
        DATALEN=0;
        return false;
      }

      rxPathLoss = txRSSI-_rssi;
      lastSG1Presence=monotonicMillis();
      lastSG1Message=lastSG1Presence;

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
        requestedReply=0;
      }
      
      doPerPacketTimeFunctions(rxTimeTrust);
      
      if(isSpecialType())
      {
        /*HANDLE SYSTEM RESERVED PACKETS*/
        debug("sp");
        DATALEN=0;
        return false;
      }

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
void RFM69::sendBeacon(bool wakeUp)
{
  defaultChannel.recalcBeaconBytes();
  int8_t power = getAutoTxPower();
  while(power%4)
  {
    power++;
  }
  rawSetPowerLevel(power);

  uint8_t header[3]={9,0,0};
  header[2]=(((power-(-24))/4)&0b1111) ;

  header[1]|= _headerTimeTrust();

  if(power>3)
  {
    header[1] |= HEADER_FEC_GOLAY;
    header[0]=12;
  }
  else
  {
    header[0]=9;
  }
  

  golay_block_encode(header,tmpBuffer);
  if(wakeUp)
  {
      memcpy(tmpBuffer+6,&(defaultChannel.privateWakeSequence),3);
  }
  else
  {
      memcpy(tmpBuffer+6,&(defaultChannel.privateHintSequence),3);
  }
  debug("phs");
  debug(defaultChannel.privateHintSequence);
  
  //If we need more than 3dbm, we also probably need to use error correction.
  //This adds an extra 3 bytes.

  //Beacons are short and uncommon, the 3 extra bytes is worth it most of the time.
  if(power>3)
  {
    memcpy(smallBuffer, tmpBuffer+6,3);
    golay_block_encode(smallBuffer,tmpBuffer+6);
  }


  uint32_t now = millis();
  while (!canSend() && millis() - now < RF69_CSMA_LIMIT_MS)
  {
    //Wake messages have to be sent immediately.
    if(wakeUp==false)
    {
      //8ms seems like a reasonable length to use for CSMA.
      delayMicroseconds(xorshift32()& 8192L);
    }
    _receiveDone();
  }

  lastSentSG1=monotonicMillis();
  //SendFrame does the length byte for you
  sendFrame(tmpBuffer+1, header[0]);
  
}

//Send a beacon and see if anyone tells us to wake up.
bool RFM69::checkBeaconSleep()
{
  //Any recent message indicates we should stay awake.
  if(lastSG1Message>(monotonicMillis()-18000L))
  {
    //But if we haven't sent a message in too long, send one
    //anyway.
    if(lastSentSG1<(monotonicMillis()-1500LL))
    {
      sendBeacon();
    }
    return true;
  }
  debug("cb");
  long waitTime;
  //If our time is marked as accurate, or if we have gotten
  //a packet in the last minute we can use short beacons
  if((systemTimeTrust&TIMETRUST_ACCURATE) || lastSG1Message>(monotonicMillis()-60000L))
  {
    sendBeacon();
    waitTime=(60000L+ (bitTime+500));
  }
  else{
    //Otherwise we have to send a whole packet
    rawSendSG1(0, 0, getAutoTxPower(), getAutoTxPower()>6,0, 0, HEADER_TYPE_RELIABLE_SPECIAL);
    waitTime=(60000L+ (bitTime+1200));
  }

  //Just long enough for them to respond.
  //We are assuming 16ms latency to get a packet and another
  //16 to reply, plus at least 10ms to actually process the crypto.
  //due to USB serial port latency.
  for(long i=0;i<waitTime; i+=1500)
  {
    //Use the real low power sleep mode.
    sleepLib.idleMode();
    sleepLib.sleepDelay(15);
    if(receiveDone())
    {
      debug("br");
      debug(i);
      //Them sending us valid traffic is just as good as a wake
      //message.
      if(decodeSG1())
      {
        return 1;
      }
      if(wakeRequestFlag)
      {
        return 1;
      }
    }
  }
  sleep();
  return 0;
}


void RFM69::setChannelKey(unsigned char * key) {
  defaultChannel.setChannelKey(key);
}

void SG1Channel::setChannelKey(unsigned char * key) {
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
  
  cipherContext.clear();
  cipherContext.setKey(channelKey,32);
  cipherContext.setIV(IV,8);

  //8 bytes total, first 4 are the hint sequence, next are the wake sequence.
   cipherContext.encrypt((uint8_t*)(&fixedHintSequence),input,3);
}


void RFM69::setProfile(uint8_t profile)
{
  switch (profile)
  {
  case RF_PROFILE_GFSK1200:
    setBitrate(1200);
    setDeviation(10000);
    setChannelFilter(25000);
    setChannelSpacing(50000);
    break;

  case RF_PROFILE_GFSK4800:
    setBitrate(4800);
    setDeviation(25000);
    setChannelFilter(55000);
    setChannelSpacing(100000);
    break;

  case RF_PROFILE_GFSK10K:
    setBitrate(10000);
    setDeviation(25000);
    setChannelFilter(55000);
    setChannelSpacing(100000);
    break;

  case RF_PROFILE_GFSK38K:
    setBitrate(38400);
    setDeviation(80000);
    setChannelFilter(200000);
    setChannelSpacing(250000);
    break;

  case RF_PROFILE_GFSK100K:
    setBitrate(100000);
    setDeviation(100000);
    setChannelFilter(200000);
    setChannelSpacing(250000);
    break;

  case RF_PROFILE_GFSK250K:
    setBitrate(250000);
    setDeviation(125000);
    setChannelFilter(500000);
    setChannelSpacing(500000);
    break;

  default:
    break;
  }

}

void RFM69::setChannelSpacing(uint32_t hz)
{
  channelSpacing = hz/1000L;
  //Recalc the channel number
  setChannelNumber(channelNumber);
}


void RFM69::setBitrate(uint32_t bps)
{
  bitTime= 1000000L/bps;
  bps = 32000000UL/bps;
  writeReg(REG_BITRATEMSB, bps/256);
  writeReg(REG_BITRATELSB,bps%256);
}

void RFM69::setDeviation(uint32_t hz)
{
  //probably fine to ditch a preamble byte or two
  //with wide channels
  if(hz>=110000)
  {
    writeReg(REG_PREAMBLELSB,0x02);
  }
  else
  {
      writeReg(REG_PREAMBLELSB,0x04);
  }

  //Round to nearest by adding half the fstep
  hz+=30;
  hz/=61;

  writeReg(REG_FDEVMSB, hz/256);
  writeReg(REG_FDEVLSB,hz%256);

  
  
}

//Approximately set the RX bandwidth, rounding up to the next step
void RFM69::setChannelFilter(uint32_t hz)
{

  if(hz<=12500)
  {
    //12.5khz half spacing.
    //Only 10kkz guard band! I think this will still wodk though.
    writeReg(REG_RXBW,RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_6);
  }

  if(hz<=25000)
  {
    //12.5khz half spacing.
    //Only 10kkz guard band! I think this will still wodk though.
    writeReg(REG_RXBW,RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_5);
  }

  else if(hz<=50000)
  {
    writeReg(REG_RXBW,RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_4);
  }
  else if(hz<=62000)
  {
    writeReg(REG_RXBW,RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_4);
  }
  else if(hz<=100000)
  {
    //100KHz
    writeReg(REG_RXBW,RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3);
  }
  else if(hz<=200000)
  {
    //200KHz
    writeReg(REG_RXBW,RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_2);
  }
  else if(hz<=330000)
  {
    //332KHz
    writeReg(REG_RXBW,RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_1);
  } 
  else
  {
    writeReg(REG_RXBW,RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_1);
  }
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
  if(
    ((rxHeader[1]&HEADER_TYPE_FIELD)==HEADER_TYPE_REPLY) ||
    ((rxHeader[1]&HEADER_TYPE_FIELD)==HEADER_TYPE_REPLY_SPECIAL)
  )
  {
    return 1;
  }
  return 0;
}

bool RFM69::isSpecialType()
{
  if(((rxHeader[1]&HEADER_TYPE_FIELD)==HEADER_TYPE_REPLY_SPECIAL)||
    ((rxHeader[1]&HEADER_TYPE_FIELD)==HEADER_TYPE_RELIABLE_SPECIAL))
  {
    return true;
  }
  return false;
}
bool RFM69::isRequest()
{
  if(((rxHeader[1]&HEADER_TYPE_FIELD)==HEADER_TYPE_RELIABLE)||
  ((rxHeader[1]&HEADER_TYPE_FIELD)==HEADER_TYPE_RELIABLE_SPECIAL))
  {
    return 1;
  }
  return 0;
}
