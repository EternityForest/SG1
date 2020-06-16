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

#ifdef __AVR__
#include <avr/io.h>
#endif

#if defined(E2END) && E2END>0
#include <EEPROM.h>
#else
#warning NO EEPROM, SAVING PAIRING WILL NOT WORK
#endif

//Two tmp buffers sized to hold an entire packet, rounded up to 68
//for overflow protection, and so we can use the very end for misc scratchpad stuff
uint8_t tmpBuffer[68];
uint8_t smallBuffer[68];

void golay_block_encode(uint8_t *in, uint8_t *out);
bool golay_block_decode(uint8_t *in, uint8_t *out);

//Leaving all this in, just in case someone wants to make
//Some crazy ultralight variant. For now, I think ChaCha is the better choice,
//Because it's easier to find implementations
//EAX<SpeckTiny> cipherContext;
ChaChaPoly cipherContext;

#define EEPROM_ENTROPY_OFFSET (32 + 1 + 2 + 1)

#define HINT_SEQUENCE_MASK 0b11111111111111111111UL

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
#define HEADER_TYPE_FIELD 0b1110000

//Same as normal unreliable messages.
#define HEADER_TYPE_SPECIAL 0b0000000

#define HEADER_TYPE_UNRELIABLE 0b0010000
#define HEADER_TYPE_RELIABLE 0b0100000
//Any reply types will always have bit 6 set
#define HEADER_TYPE_REPLY 0b1000000
#define HEADER_TYPE_REPLY_SPECIAL 0b1010000

#define HEADER_TYPE_RELIABLE_SPECIAL 0b0110000

#define SPECIAL_TYPE_PAIRING_REQUEST 16
#define SPECIAL_TYPE_PAIRING_CHALLENGE 17
#define SPECIAL_TYPE_PAIRING_RESPONSE 18
#define SPECIAL_TYPE_PAIRING_CONFIG 18


static uint8_t systemTimeTrust = 0;

uint32_t millisToUnix=0;


//Binary ppm, parts per 2**20
static int32_t systemTimebPPMAdjust = 0;

//Return bytes 3-7 of the time as a uint32_t
static uint32_t timeToIntervalNumber(int64_t time)
{
  uint32_t interval = ((uint64_t)time)>>24;
  return(interval);
}
static union {
  //UNIX time in microseconds
  int64_t systemTime = -1;
  uint8_t systemTimeBytes[8];
  //struct  __attribute__ ((packed)){
  //  uint32_t systemTimeLow;
  //  uint32_t systemTimeHigh;
  //}
};


uint32_t saturatingAbs(int64_t p)
{
  uint32_t x = 0;

  uint8_t target;
  //Topmost bit check ones complement
  if ((((uint8_t *)&p)[7]) & 128)
  {
    target = 255;
  }
  else
  {
    target =0;
  }

  //Check that the top 4 bytes are either 0 or 255 depending on if this is a negative or positive
  //number. If they are anything else it means that the number is using those top
  //bytes.
  for(uint8_t i = 4;i<8;i++)
  {
    if(  (((uint8_t *)&p)[i]) != target)
    {
      return 4000000000UL;
    }
  }

  //Top bit of byte 3 must be 0 for positive or 1 for negative.
  //Otherwise, the number would be too big to fig in 32 once we add the sign bit.
  if (((((uint8_t *)&p)[3]) & 128) != (target&128))
  {
    return 4000000000UL;
  }

  memcpy(&x,&p,4);
  if(target)
  {
    return ~x;
  }
  return x;
}
unsigned long RFM69::approxUnixMillis()
{
  return millis()+millisToUnix;
}

static union {
  uint8_t entropyPool[20];
  //BAD CRYPTO ALERT!!!
  //This is shared with a totally insecure RNG
  //This leaks small amounts of information about
  //these 16 bits. This should normally not matter,
  //As we re-encrypt the entire pool both before and after use
  uint16_t entropyRegister;
};

//The non-secure RNG we use for packet timings
//static uint32_t rngState;
#define rngState entropyRegister

void RFM69::initSystemTime()
{
    //Note: This always randomizes the time no matter what. We just assume it's only ever called early on
    urandom(systemTimeBytes, 8);
    //Make it obviously fake by being negative.
    //Check the raw twos complement top byte that has the sign bit
    //Because 64 bit arithmetic uses too much space
    if (systemTimeBytes[7] < 128)
    {
      systemTimeBytes[7] = (~systemTimeBytes[7]);
    }
}


void RFM69::syncFHSS(uint16_t attempts)
{
  //fhssOffset = xorshift16();

  for (uint16_t i = 0; i < attempts; i++)
  {
    //Try a random FHSS channel and listen for replies,
    //but no matter what  always go back to the normal sequence at the end
    rawSendSG1(0, 0, 0, HEADER_TYPE_RELIABLE_SPECIAL);
    for (int j = 0; j < 100L; j++)
    {
      delay(1);
      if (receiveDone())
      {
        debug("r");
        if (decodeSG1())
        {
          fhssOffset=0;
          debug(j);
          return;
        }
        if (receivedReply())
        {
          fhssOffset=0;
          debug("Sync!");
          debug(j);
          return;
        }
      }
    }
    fhssOffset += 1;
  }
  fhssOffset = 0;
}
static unsigned long systemTimeMicros = 0;



void doTimestamp(int32_t adjustment = 0)
{
  //Set systemTime to the correct value by adding the micros.
  //Adjustment adds up to that many micros but will never make the clock go
  //backwards.
  unsigned long x = micros();
  unsigned long x2 = x - systemTimeMicros;
  //Add a correction factor
  //x2 += (x2 >> 20) * systemTimebPPMAdjust;

  int64_t y = x2 + adjustment;

  //Clock doesn't go backwards except when actually set.

  if (y < 0)
  {
    //Don't let the adjustment slow things down
    //to zero, that could cause problems.
    if (x2 > 0)
    {
      y = 1;
    }
    else
    {
      y = 0;
    }
  }

  systemTime += y;
  systemTimeMicros = x;

  //First compute the system time/1024 for approx millis.
  //Now we subtract millis to get a conversion to turn millis into
  //unix millis.
  millisToUnix = ((RFM69::readUInt32(systemTimeBytes+1)  )>>4)-millis();
  //If it has been ten minutes since accurate setting,
  //The clock has drifted and we aren't accurate anymore.

  //TODO:
  /*if (systemTimeTrust & (TIMETRUST_ACCURATE))
  {
    if (RFM69::approxUnixMillis() - lastAccurateTimeSet > 600000LL)
    {
      systemTimeMicros -= TIMETRUST_ACCURATE;
    }
  }*/
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
  //Pick a fairly random arbitrary position to xor the micros into,
  //so that it's separate from the entropyRegister
  uint8_t m =micros();

  entropyPool[3] ^= m;
  cipherContext.setKey(entropyPool, 20);
  cipherContext.setIV(systemTimeBytes, 8);
  cipherContext.encrypt(entropyPool, entropyPool, 20);
  //We don't use the auth tag
}

static uint16_t applyXorshift(uint16_t rngState)
{
  rngState ^= rngState << 7;
  rngState ^= rngState >> 9;
  rngState ^= rngState << 8;
  return rngState;
}

uint16_t RFM69::xorshift16()
{
  rngState += micros();
  rngState = applyXorshift(rngState);
  return rngState;
}

uint32_t RFM69::urandomRange(uint32_t f, uint32_t t)
{

  //2^64 is so big the modulo bias shouldn't matter.... Right?
  union {
    uint64_t x;
    uint8_t b[8];
  };

  urandom(b, 4);
  return (x % (t - f)) + f;
}

void RFM69::urandom(uint8_t *target, uint8_t len)
{

  //We are going to use the stream cipher as an RNG.
  //By encrypting the entropy pool with itself as the key
  //That way, no information about the pool itself is ever leaked to
  //the destination.

  //Take a reading to further do some mixing.

  //The pool must be seeded for this to work!!!
  //This means that you need to
  //Mix entropy before we actually use it, so that we don't
  //get any reuse with the non-secure uses of the pool.
  mixEntropy();

  

  cipherContext.setKey(entropyPool, 20);
  //Use whatever nonsense is in the tmp buffer at the moment
  cipherContext.setIV(entropyPool, 8);
  //Copy bytes 16 at a time. Note that
  //We never reveal the raw content of the entropy pool for any reason.
  //We just use the encrypted keystream XORed with the tmpBuffer
  //Also, only 16 bytes so we can't possibly reveal the full state

  //Also important is the fact that we do not use the same IV
  //we use for the mixEntropy, as that would mean what we output
  //is the same as the new state of the pool
  while (len)
  {
    uint8_t x = len;
    if (x > 16)
    {
      x = 16;
    }

    cipherContext.encrypt(target, tmpBuffer, x);
    len -= x;
    target += x;
  }
  //No information should be left about the old state
  mixEntropy();
}

void RFM69::addEntropy(uint8_t x)
{
  entropyRegister += x;
}

void RFM69::setTime(int64_t time, uint8_t trust)
{
  //Special value 0 indicates we randomize the time if needed.
  if (time)
  {
    systemTime = time;
    systemTimeMicros = micros();
  }
  else
  {
    initSystemTime();
  }

   systemTimeTrust = trust;


}

volatile bool RFM69::_haveData;

int8_t RFM69::getAutoTxPower()
{
  int16_t txPower = -4;

  int8_t targetRSSI;

  //Higher speed needs more power
  if (bitTime >= 9)
  {
    targetRSSI = -92;
  }
  else
  {
    targetRSSI = -87;
  }

  //Detect if we aren't in auto mode
  if (_powerLevel > -127)
  {
    return _powerLevel;
  }

  //If we have a message in the last 80s, that means that we can
  //use TX power control
  if ((approxUnixMillis() - lastSG1Presence) < (-(80UL * 1000UL)))
  {
    //Target an RSSI on the other end of -90
    txPower = targetRSSI;
    txPower += rxPathLoss;

    //For every failed or canceled request that's happened, add 3db of power.
    //txPower+= min(requestedReply*3,12);

    while (txPower % 4)
    {
      txPower++;
    }

    if (txPower > maxTxPower)
    {
      txPower = maxTxPower;
    }
  }
  else
  {
    //It appears excessively strong signals can actually block things at
    //close range.
    //part of the time, we use low power so as be able to connect close together
    //devices. Once connected auto power takes over.
    if ((xorshift16() & 0b11) == 0)
    {
      txPower = -18;
    }
    else
    {
      txPower = maxTxPower;
    }
  }
  return (uint8_t)txPower;
}

void RFM69::sendSG1(const void *buffer, uint8_t bufferSize)
{
  //Reset the failed request tracking
  requestedReply = 0;
  rawSendSG1(buffer, bufferSize, 0, HEADER_TYPE_UNRELIABLE);
}

void RFM69::sendSG1Request(const void *buffer, uint8_t bufferSize)
{
  requestedReply += 1;
  rawSendSG1(buffer, bufferSize, 0, HEADER_TYPE_RELIABLE);
}

void RFM69::sendSG1Reply(const void *buffer, uint8_t bufferSize)
{
  rawSendSG1(buffer, bufferSize, rxIV, HEADER_TYPE_REPLY);
}

void RFM69::sendSG1RT(const void *buffer, const uint8_t bufferSize)
{
retry:

  defaultChannel.recalcBeaconBytes();

  //It it is important that we use the private hint sequences
  //here not the fixed ones. With fixed, any address collision would be constantly
  //sending bad packets, that would get through fairly reliably ever 2**32 packets.
  
  memcpy(smallBuffer, (uint8_t *)(&(defaultChannel.privateHintSequence)), 3);

  doTimestamp();

  //Copy all 8 of the IV bytes, but we only send 4
  memcpy(smallBuffer + 3, systemTimeBytes, 8);

  //Replace the LSB byte with the node ID.
  smallBuffer[3] = nodeID;

  cipherContext.setKey(defaultChannel.channelKey, 32);
  cipherContext.setIV(smallBuffer + 3, 8);

  //Now we encrypt. Note that we overwrite the top 4 IV bytes,
  //We don't want to send those, the reciever uses the ones from the system clock.
  cipherContext.encrypt(smallBuffer + 3 + 4, (uint8_t *)buffer, bufferSize);
  cipherContext.computeTag(smallBuffer + 3 + 4 + bufferSize, 4);

  //Short range, does not need a long preamble
  writeReg(REG_PREAMBLELSB, 0x02);
  //Note that we don't even try to send these packets if we can't do so
  //in a reasonable time.
  for (int i = 0; i < 3; i += 1)
  {
    if (trySend(smallBuffer, bufferSize + 3 + 4 + 4))
    {
      debug("srt");
      break;
    }
    delayMicroseconds(xorshift16() & 4095L);
    goto retry;
  }
  writeReg(REG_PREAMBLELSB, 0x04);
}

void RFM69::rawSendSG1(const void *buffer, uint8_t bufferSize,
                       uint8_t *useChallenge, uint8_t packetType)
{
  //The length of everything after the initial 6 byte block
  //8 byte IV, 8 byte MAC, 3 byte channel hint
  int payloadSize = bufferSize + 8 + 8 + 3;

  /*
     * Header Format
     *
     * Length(Actually part of the raw packet framing the RFM69 does, so we don't send it, but it is still the first byte and protected by FEC
     * Flags:
     *    0-1: FEC Type: 0=Only header, 1=Golay code for entire packet
     *    2: Time Trusted

     *
     */
  uint8_t header[3] = {0, 0, 0};

  header[1] |= packetType;

  header[1] |= _headerTimeTrust();

  int txPower = getAutoTxPower();
  //Round up to 4
  while (txPower % 4)
  {
    txPower++;
  }
  txPower = max(txPower, -24);
  bool useFEC = false;
  if (txPower >= maxTxPower)
  {
    useFEC = false;
  }
  //If the whole message would be big we have to turn off FEC
  //To get the total under 64 bytes
  if (payloadSize > 29)
  {
    useFEC = false;
  }
  //With header, that would exceed 64 bytes
  if (payloadSize > 58)
  {
    // Print "SZ"
    Serial.write('S');
    Serial.write('Z');
    return;
  }

  header[2] = ((txPower - (-24)) / 4);

  rawSetPowerLevel(txPower);

  if (systemTimeTrust >= TIMETRUST_CHALLENGERESPONSE)
  {
    header[1] |= HEADER_TIME_TRUST_FIELD;
  }
  uint8_t padding = 0;
  //First calculate size, then encrypt, then golay
  if (useFEC)
  {
    //Pad to multiple of 3.
    //But we have to do this by incrementing the actual
    //Buffer size, because the whole packet has the be filled
    //With encrypted data. Otherwise we could not detect which was padding.

    //This means you may recieve more data than you send.
    while (payloadSize % 3)
    {
      bufferSize += 1;
      padding += 1;
      payloadSize = bufferSize + 8 + 8 + 3;
    }

    header[1] |= HEADER_FEC_GOLAY;

    payloadSize = ((payloadSize)*2);
  }
  else
  {
    payloadSize = bufferSize + 8 + 8 + 3;
  }

  //Add the 6 bytes for the header itself.
  header[0] = payloadSize + 6;

  cipherContext.setKey(defaultChannel.channelKey, 32);

retry:
  doTimestamp();

  memcpy(smallBuffer, (uint8_t *)(&(defaultChannel.fixedHintSequence)), 3);

  //First 3 bytes of this are the hint. 4th byte is nodeid part of iv.
  smallBuffer[3] = nodeID;
  //Hint comes before the IV, and first IV byte is
  //NodeID so don't put the time in that
  memcpy(smallBuffer + 3 + 1, systemTimeBytes + 1, 7);

  //Skip hint sequence to get to the IV
  cipherContext.setIV(smallBuffer + 3, 8);

  //Keep track of the challenge so we can get replies, but
  //There can't be any replies to a reply, so if this uses a challenge, don't
  if (useChallenge == 0)
  {
    memcpy(awaitReplyToIv, smallBuffer + 3, 8);
  }

  cipherContext.addAuthData(header, 3);

  if (useChallenge)
  {
    cipherContext.addAuthData(useChallenge, 8);
  }

  //Leave room for hint and IV
  //Don't include padding bytes
  cipherContext.encrypt((uint8_t *)smallBuffer + 3 + 8, (uint8_t *)buffer, bufferSize - padding);

  //This is the zero we will encrypt
  uint8_t padByte = 0;

  //Now encrypt some zeros for padding
  //Only 2 padding bytes can ever be needed.
  if (padding > 0)
  {
    cipherContext.encrypt((uint8_t *)smallBuffer + 3 + 8 + bufferSize - padding, &padByte, 1);
  }
  if (padding == 2)
  {
    cipherContext.encrypt((uint8_t *)smallBuffer + 3 + 8 + (bufferSize - padding) + 1,
                          &padByte, 1);
  }

  cipherContext.computeTag(smallBuffer + 3 + 8 + bufferSize, 8);

  if (useFEC)
  {
    //We are computing this in reverse order, if we went forwared we would overwrite
    //stuff as we went.

    // Suppose we have 21 bytes to begin with.
    // First we do the block starting at position 18(bytes 18,19,and 20)
    // 20 is the last byte because of 0 based.

    //Note that we have to do block 0, so we use i going below 0
    //as the exit condition

    //Destinations always have +6 because we are leaving room for the header
    for (int i = (bufferSize + 8 + 8 + 3); i >= 0; i -= 3)
    {
      golay_block_encode(smallBuffer + i, smallBuffer + 6 + (i * 2));
    }
  }
  else
  {
    //No FEC, just move into place.
    memmove(smallBuffer + 6, smallBuffer, payloadSize);
  }

  golay_block_encode(header, smallBuffer);

  if (channelSpacing > 200 && txPower <= -4)
  {
    //Short range, does not need a long preamble
    writeReg(REG_PREAMBLELSB, 0x02);
  }
  else
  {
    writeReg(REG_PREAMBLELSB, 0x04);
  }

  unsigned long st = millis();
  //More than 3ms, and we do a full recalc.
  //This is neccesary to ensure that we always send precision time.
  while ((millis() - st) < 3)
  {
    //trySend does the length byte for you
    //it also expects just the payload length.
    //Without the len byte

    if (trySend(smallBuffer + 1, header[0] - 1))
    {
      debug("snt");
      debug(header[0] - 1);
      lastSentSG1 = approxUnixMillis();
      return;
    }
    delayMicroseconds(xorshift16() & 0b1111111111L);
  }
  //Back to the default
  writeReg(REG_PREAMBLELSB, 0x04);

  //Time to try again, we weren't able to send
  goto retry;
}

uint8_t rxHeader[8];

uint32_t applyHintSequenceMask(uint32_t x)
{
  return x & HINT_SEQUENCE_MASK;
}


//Encrypt 3 zeros, apply the HINT_SEQUENCE_MASK, then copy the result to the target
void encryptIntoSequence(uint8_t * target)
{
  uint32_t x=0;
  uint32_t in=0;

  cipherContext.encrypt((uint8_t*)&x, (uint8_t *)&in, 3);
  x=applyHintSequenceMask(x);
  memcpy(target,&x,4);
}

void SG1Channel::setupCipher(uint8_t * IV)
{
  cipherContext.setKey(channelKey,32);
  cipherContext.setIV(IV,8);
}

void SG1Channel::recalcBeaconBytes()
{
  //Recalc the hint sequences for this 16s period
  doTimestamp();

  //To tolerate misalignment, calculate the current, and also the next closest
  //interval, and accept either.

  union {
    //Add 8 seconds to push us into the next period,
    uint32_t currentIntervalNumber;
    uint8_t currentIV[8];
  };
  memset(currentIV,0, 8);
  currentIntervalNumber = timeToIntervalNumber(systemTime);

  union {
    uint32_t altIntervalNumber;
    uint8_t altIV[8];
  };
  memset(altIV,0,8);
  altIntervalNumber = timeToIntervalNumber(systemTime + 8000000L);

  //So we tried incrementing, but it didn't change, meaning
  //the actual closest interval is the previous one
  //
  if (currentIntervalNumber == altIntervalNumber)
  {
    altIntervalNumber = timeToIntervalNumber(systemTime - 8000000L);
  }

  //detect cache
  if (altIntervalNumber == intervalNumber)
  {
    return;
  }

  //set cache marker. altIntervalNumber is used because that one changes more often.
  intervalNumber = altIntervalNumber;

  setupCipher(altIV);
  encryptIntoSequence((uint8_t *)& altPrivateHintSequence);
  encryptIntoSequence((uint8_t *)& altPrivateWakeSequence);


  setupCipher(currentIV);
  encryptIntoSequence((uint8_t *)& privateHintSequence);
  encryptIntoSequence((uint8_t *)& privateWakeSequence);

}

int64_t RFM69::getPacketTimestamp()
{
  int64_t rxTimestamp = 0;
  memcpy(((uint8_t *)(&rxTimestamp)) + 1, rxIV + 1, 7);
  return rxTimestamp;
}

const int64_t maxDiffForSmallAdj = 2000000LL;

void RFM69::doPerPacketTimeFunctions(uint8_t rxTimeTrust)
{

  int64_t rxTimestamp = 0;

  int64_t diff;

  int64_t correct = ((((RAWPAYLOADLEN + 1 + 4 + 4) * 8LL) * bitTime) + 500LL);

  //Gonna subtract the data time, the preamble, and the sync bytes.
  //we want the start time.

  rxTimestamp = getPacketTimestamp();

  //This basically find out how far ahead their clock is
  diff = rxTimestamp - rxTime;

  //Add this extra correction to compensate for how far ahead the recieve time is
  //from the packet start
  //add 500us, assume 1ms poll interval or so.
  diff += correct;

  bool canDoSmallAdjust = true;

  //Check if the absolute value is less than 2000000L
  //I could not get this comparision to work any other way.
  //Note we have to mask off the sign bit to get the abs
  //I have no clue what the bug was. 64 bit arithmetic doesn't always seem to work well.

  


  
  if(saturatingAbs(diff)>2000000L)
  {
      canDoSmallAdjust = false;
      debug("s1");
  }
  else
  {
    debug("doadj");
  }

  //canDoSmallAdjust=false;
  

  //#TODO: This expects that the clock does not get set in between
  //recieve and decode.

  //If we trust this packet more than the clock,
  //Set the clock from this.

  if ((systemTimeTrust & TIMETRUST_LOCAL))
  {
    debug("lc");
    //Local time trust absolutely blocks anything else
    return;
  }

  //Packet is secure, and very close to our time, this lets us do small
  //adjustments without challenge response
  if (((rxTimeTrust & TIMETRUST_ACCURATE) || ((systemTimeTrust & TIMETRUST_ACCURATE) == 0)) && (rxTimeTrust >= TIMETRUST_SECURE) && canDoSmallAdjust)
  {
    debug("adj");
    debug(canDoSmallAdjust);
    doTimestamp(diff);
    //Stop the FHSS search when we are actually connected.
  }

  //Otherwise, we have to a jump
  else if ((rxTimeTrust >= systemTimeTrust) && (rxTimeTrust >= TIMETRUST_CHALLENGERESPONSE))
  {
    //data+4 is node ID, the rest of the IV is the time.

    //More than 2s, just jump.
    debug("tj");
    debug(canDoSmallAdjust);
    debug((int32_t)diff);
    debug((int32_t)(diff>>32));
    systemTime = systemTime + diff;

    //Yes, this can go backwards. It's somewhat of a nonce reuse hazard.
    //You got a better plan?
    channelTimestampHead = getPacketTimestamp();

    // Only local stuff can set that header, because we don't have code to really
    // Keep track of when it becomes no longer accurate.
    systemTimeTrust = rxTimeTrust &(~TIMETRUST_ACCURATE);

    //Having them set our timestamp is pretty much the closest thing
    //We have to a "connection", so we send this as kind of an acknowledge
    //for stuff that wants to know if we're there, like FHSS
    rawSendSG1(tmpBuffer, 0, 0, HEADER_TYPE_SPECIAL);
  }
  else
  {
    debug("nj");
    debug(systemTimeTrust);
    debug(rxTimeTrust);
    debug((int32_t)(systemTime >> 32));
    debug((int32_t)(diff >> 32));
    debug((int32_t)(diff));
  }
}

bool RFM69::decodeSG1Header()
{
  //SG1 packets can't be shorter than this.
  //Really 9 bytes, byt the len byte is implied
  if (DATALEN < 8)
  {
    debug("tsht");
    return false;
  }

  //First byte is the actual length from the framing, plus the length byte which is just implied
  tmpBuffer[0] = DATALEN + 1;
  //Copy evrything in the header but the length nyte we already have
  memcpy(tmpBuffer + 1, DATA, 5);

  if (golay_block_decode(tmpBuffer, rxHeader))
  {
    debug("badheader");
    return false;
  }
  //We can't recover data that isn't actually there
  //But we can remove extra bytes that shouldn't be there.
  if (tmpBuffer[0] > (DATALEN + 1))
  {
    debug("MSNG");
    return false;
  }
  return true;
}

uint32_t RFM69::readHintSequence()
{

  uint8_t x[4];
  x[3] = 0;
  //In place decode even though we have to re decode later,
  //We need to be able to read the hint without decoding everything
  if ((rxHeader[1] & HEADER_FEC_FIELD) == HEADER_FEC_GOLAY)
  {
    golay_block_decode(DATA + 5, x);
  }
  else
  {
    memcpy(x, DATA + 5, 3);
  }

  return applyHintSequenceMask(readUInt32(x));
}

void RFM69::loadConnectionFromEEPROM(uint16_t eepromAddr)
{

  #if defined(E2END) && E2END>0

  for (int i = 0; i < (2+32 + 1 + 2 + 1 + 8); i++)
  {
    tmpBuffer[i] = EEPROM.read(eepromAddr + i);
  }
  if(tmpBuffer[0]=='S' && tmpBuffer[1]=='G')
  {
    defaultChannel.setChannelKey(tmpBuffer+2);
    setProfile(tmpBuffer[2+32]);
    setChannelNumber(readUInt16(tmpBuffer + 2+ 32 + 1));
    setNodeID(tmpBuffer[2+ 32 + 1 + 2]);
    //addEntropy(tmpBuffer + 2+ 32 + 1 + 2 + 1, 8);
  }
  #endif

}


/*
EEPROM Layout:

64 bytes total reserved
2 byte magic number: SG
32 byte channel key
1 byte rf profile
2 byte channel number
8 bytes random data for RNG uniqueness.
*/
void RFM69::saveConnectionToEEPROM(uint16_t addr)
{
  #if defined(E2END) && E2END>0
  EEPROM.update(addr, 'S');
  EEPROM.update(addr+1, 'G');
 
  for(uint8_t i=0;i<32;i++)
  {
    EEPROM.update(addr+i+2, defaultChannel.channelKey[i]); 
  }
  EEPROM.update(addr+2+32, rfProfile);
  EEPROM.update(addr+2+32+1, channelNumber %256);
  EEPROM.update(addr+2+32+2, channelNumber/256);

  uint8_t x;

  // Generate 8 bytes of randomness for seeding the RNG
  for(uint8_t i=0;i<8;i++)
  {
     urandom(&x,1);
     EEPROM.update(addr+2+32+2+1+i,x); 
  }
  #endif
}

void RFM69::addEntropy(uint8_t *x, uint8_t len)
{
  for (uint8_t i = 0; i < len; i++)
  {
    entropyPool[i] ^= x[i];
  }
}

/*void RFM69::waitForSendTime()
{
  if(channelNumber>1000)
  { 
  
   if systemTimeBytes
  }
}*/

bool RFM69::listenForPairing(int16_t eepromAddr)
{
  uint16_t oldChannel = channelNumber;
  uint8_t oldProfile = rfProfile;
  uint8_t oldPowerLevel = _powerLevel;


  //The pairing channel is always 3
  setProfile(RF_PROFILE_GFSK38K);
  //We don't really need long range for this
  setPowerLevel(-18);
  setChannelNumber(3);

  uint8_t buf[64];
  uint8_t secret[32];

  unsigned long now = millis();

  unsigned char state = 0;

  //Time out after a bit
  while ((millis() - now) < 15000L)
  {
    if (receiveDone())
    {
      if(RSSI< -58)
      {
        continue;
      }
      debug("GOT");
      debug(RSSI);

      if (state == 0)
      {
        if (DATA[0] == SPECIAL_TYPE_PAIRING_REQUEST)
        {
          //Extend the time a bit
          now = millis();
          debug("PR");
          //We need random data
          urandom(tmpBuffer, 32);
          Curve25519::dh1(buf + 1, secret, tmpBuffer);
          buf[0] = SPECIAL_TYPE_PAIRING_CHALLENGE;
          send(buf, 33);
          debug("st");
          state = 0;
        }
      
        //This packet doesn't do anything besides set up the
        //DH key exchange.
        else if (DATA[0] == SPECIAL_TYPE_PAIRING_RESPONSE)
        {
          debug("PC");
          now = millis();
          //Now we have a shared secret
          if(!Curve25519::dh2(DATA + 1, secret))
          {
            debug("badr");
            state=0;
            continue;
          }
          memcpy(secret, DATA + 1, 32);
          state = 2;
        }
      }

      else if (state == 2)
      {
        if (DATA[0] == SPECIAL_TYPE_PAIRING_CONFIG)
        {
          debug("PCo");
          cipherContext.setKey(secret, 32);
          debug(secret[0]);
          //Reuse this as a buffer we can make all zeros
          memset(secret, 0, 8);

          cipherContext.setIV(secret, 8);

          //In place decrypt 36 bytes
          cipherContext.decrypt(DATA + 1, DATA + 1, DATALEN-(8+1));

          if (cipherContext.checkTag(DATA + (DATALEN-8), 8))
          {
            //Check for a correct setting on the RF profile
            if (DATA[1 + 32] < 8)
            {
              defaultChannel.setChannelKey(DATA + 1);
              setProfile(DATA[1 + 32]);
              setChannelNumber(readUInt16(DATA + 1 + 32+1));
              setNodeID(DATA[1 + 32 + 1 + 2]);
              setPowerLevel(-127);

              if(eepromAddr != -1)
              {
                saveConnectionToEEPROM(eepromAddr);
              }
             
              return 1;
            }
            else
            {
              debug("badrfp");
            }
          }
          else
          {
            debug("bad");
          }
        }
      }
    }
  }
  setProfile(oldProfile);
  setChannelNumber(oldChannel);
  setPowerLevel(oldPowerLevel);
  return 0;
}

#define HINT_TYPE_NORMAL 1
#define HINT_TYPE_WAKE 2
uint8_t RFM69::checkHintSequenceRelevance(const uint32_t hint, const uint8_t privateOnly)
{

  if(hint == defaultChannel.fixedHintSequence)
  {
    if (privateOnly)
    {
      return 0;
    }
    return HINT_TYPE_NORMAL;
  }
  else if (
      (hint == defaultChannel.privateHintSequence) ||
      (hint == defaultChannel.altPrivateHintSequence)
      )
  {
    return HINT_TYPE_NORMAL;
  }

  else if ((hint == defaultChannel.privateWakeSequence) ||
      (hint == defaultChannel.altPrivateWakeSequence))
  {
    return HINT_TYPE_WAKE;
  }

  return false;
}

bool RFM69::pairWithRemote(uint8_t nodeID)
{

  uint16_t oldChannel = channelNumber;
  uint8_t oldProfile = rfProfile;
  int8_t oldPowerLevel = _powerLevel;
  bool alreadyExtended = 0;



  //The pairing channel is always 3
  setProfile(RF_PROFILE_GFSK38K);
  setPowerLevel(-18);
  setChannelNumber(3);

  uint8_t buf[80];
  uint8_t secret[32];

  unsigned long now = millis();

  buf[0] = SPECIAL_TYPE_PAIRING_REQUEST;
  
  while ((millis() - now) < 10000)
  {
    //While we have not yet seen a response,
    //We are going to keep sending this.
    if(alreadyExtended==0)
    {
      send(buf, 1);
      delay(250);
    }

    if (receiveDone())
    {
      if(RSSI< -55)
      {
        continue;
      }
      debug("gp895");
      debug(DATA[0]);

      if (DATA[0] == SPECIAL_TYPE_PAIRING_CHALLENGE)
      {
        debug("pairc");
        if (alreadyExtended==0)
        {
          //Give some extra time
          now=millis()+25000L;
          alreadyExtended=true;
        }
        urandom(buf + 33, 32);

        //Our part of the first step
        Curve25519::dh1(buf + 1, secret, buf + 33);
        buf[0] = SPECIAL_TYPE_PAIRING_RESPONSE;
        //preserve the pubkey
        memcpy(smallBuffer, buf + 1, 32);
        //We alreadyy got their half, we can do this now
        if(!Curve25519::dh2(DATA + 1, secret))
        {
          debug("badc");
          continue;
        }
        //Preserve the shared secret
        memcpy(secret, DATA + 1, 32);

        //need to send them the key
        memcpy(buf + 1, smallBuffer, 32);

        send(buf, 1 + 32);
        debug("str");
        delay(1500);

        //Now we just need to send the actual connection parameters.
        memcpy(buf + 1, defaultChannel.channelKey, 32);
        buf[1 + 32] = oldProfile;
        writeUInt16(&(buf[1 + 32 + 1]),oldChannel);
        buf[1 + 32 + 1 + 2] = nodeID;

        cipherContext.setKey(secret, 32);
        memset(smallBuffer, 0, 8);
        //IV of 0, because key always changes
        cipherContext.setIV(smallBuffer, 8);

        //Encrypt everything, using the shared secret
        cipherContext.encrypt(buf + 1, buf + 1, (32 + 1 + 2+ 1));
        cipherContext.computeTag(buf + 1 + (32 + 1 + 2+1), 8);
        buf[0] = SPECIAL_TYPE_PAIRING_CONFIG;
        //Send the whole thing in a packet.
        send(buf, 1 + (32 + 1 + 2+ 1) + 8);
        debug("stc");
        debug(secret[0]);
      }
    }
   
  }
  //Switch back
  setProfile(oldProfile);
  setChannelNumber(oldChannel);
  setPowerLevel(oldPowerLevel);
}
bool RFM69::checkTimestampReplayAttack(int64_t ts)
{
          int64_t diff = ts - systemTime;

          int32_t diff32 = saturatingAbs(diff);
          debug("chrpa");
          debug(diff32);

    


          //16 seconds max err
          if (diff32>16000000L)
          {

            debug("rtsmp");
            return false;
          }

          if (systemTimeTrust < TIMETRUST_CHALLENGERESPONSE)
          {
            debug("rlc");
            return false;
          }

          return true;
}


  /*
After recieving a packet, call this to decode it.
Returns 1 if the message was an SG1 packet addressed to 
Our channel.

If True, DATA and DATALEN will be the decoded and decrypted payload.
*/
uint8_t RFM69::decodeSG1()
  {
    int8_t _rssi = RSSI;
    gotSpecialPacket = false;
    RAWPAYLOADLEN = DATALEN;

    if (decodeSG1Header() == false)
    {

      return 0;
    }

    doTimestamp();
    //For keeping track of how much we trust the incoming timestamp
    uint8_t rxTimeTrust = 0;

    //Set to true later
    wakeRequestFlag = false;
    //subtract 1 from header, to skip past the implicit length byte,
    //It now represents the length of DATA

    uint8_t tmpDatalen = rxHeader[0] - 1;
    int8_t txRSSI = ((rxHeader[2] & 0b00001111) * 4) - 24;

    //Buffer overflow prevention
    if (tmpDatalen > 80)
    {
      debug("long");
      return 0;
    }

    //Check what kind of FEC we are supposed to be using.
    if ((rxHeader[1] & HEADER_FEC_FIELD) == HEADER_FEC_GOLAY)
    {
      debug("fecr");
      //Full packet FEC mode.
      for (uint8_t i = 0; i < (tmpDatalen - 5); i += 6)
      {
        //Note that we start at 5 to skip header, not 6, because the lenth byte isn't actually in the buffer
        //Decode all the 3 byte blocks into 6 byte blocks.
        if (golay_block_decode(DATA + i + 5, tmpBuffer + (i / 2)))
        {
          debug("badfec");
          return 0;
        }
      }
      //Subtract header, div by 2 because of code rate.
      //We are going to use the FEC decoded packet length, not the real one,
      //To allow us to get rid of any extra data after the end caused by a 1 to 0 flip.
      tmpDatalen = (tmpDatalen - 5) / 2;
    }
    else
    {
      tmpDatalen = (tmpDatalen - 5);
      //Subtract header, but use the raw packet data.
      memcpy(tmpBuffer, DATA + 5, tmpDatalen);
    }

    uint32_t rxHintSequence = readHintSequence(tmpBuffer);

    //3 data bytes indicate a short packet, only using
    //the brief channel hints
    if (tmpDatalen == 3)
    {
      defaultChannel.recalcBeaconBytes();
      
      //Only listen for private hints
      uint8_t relevance = checkHintSequenceRelevance(rxHintSequence,true);
      if (relevance)
      {
        rxPathLoss = txRSSI - _rssi;

        lastSG1Presence = approxUnixMillis();
        debug("chb");

        //The wake request flag is used to let them wake us up
        if (relevance == HINT_TYPE_WAKE)
        {
          debug("chw");
          wakeRequestFlag = true;
        }
        else
        {
            //It's not one of the wake sequences, just a normal beacon.
            //That means we can send it a message to keep it awake.
            if (keepRemotesAwake)
            {
              sendBeacon(true);
            }
            else
            {
              //Just send a normal beacon in replay, not a wake beacon.
              sendBeacon(false);
            }
        }
        debug("rb");
        DATALEN = 0;

        //Since beacons are not secure, make sure someone can't accidentally read the 1st
        //byte as an opcode(Forgetting to check length) and get random values, by always setting it to 0.
        DATA[0]=0;
        return 2;
      }
      debug("irb");
      debug(rxHintSequence);
      DATALEN = 0;
      return false;
    }

    //Back this up here, it's gonna get messed with by the send
    //Function we might use
    uint8_t remainingDataLen = tmpDatalen;

    if (rxHeader[1] & HEADER_TIME_ACCURATE_FIELD)
    {
      rxTimeTrust += TIMETRUST_ACCURATE;
    }

    if (rxHeader[1] & HEADER_TIME_TRUST_FIELD)
    {
      rxTimeTrust += TIMETRUST_CLAIM_TRUST;
    }

    if (checkHintSequenceRelevance(rxHintSequence,false))
    {
      //NodeID+timestamp is after hint
      memcpy(rxIV, tmpBuffer + 3, 8);

      //If this is a reply message, we need to authenticate it
      //Based on the timestamp of the previous message.
      //We do that by adding in the authentication data.

      //If it is not a reply, it has no challenge response properties and therefore
      //we have to check its time
      if (isReply())
      {
        debug("grp");
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
        if (systemTimeTrust >= TIMETRUST_CHALLENGERESPONSE)
        {
          int32_t diff = saturatingAbs(getPacketTimestamp() - (rxTime));
          //We are going to try to maintain tight timing
          //1100ms is the limit before we tell them. We need the tight sync
          //for FHSS, so use 5ms when that is enabled.
          //Can't reply to a reply

          //For now, we are just going to automatically reply to all these
          //RELIABLE_SPECIAL with the current time.
          //Todo??
          debug("ofset");
          debug((int32_t)(diff));


          if (((abs(diff) > (5000LL)) && (!isReply())) ||
              ((rxHeader[1] & HEADER_TYPE_FIELD) == HEADER_TYPE_RELIABLE_SPECIAL))
          {

            //normally, the limit is 1.1 seconds, on accound of the
            //1 second precision limit of a real time clock.
            //We occasionally randomly send packets anyway to allow high precision sync
            //without wasting too much

            //We also *always* reply to reliable special packets.
            if (((abs(diff) > (1100000LL) || (systemTimeBytes[0] < 16))) ||
                (rxHeader[1] & HEADER_TYPE_FIELD) == HEADER_TYPE_RELIABLE_SPECIAL)
            {
              //Don't send replies if they actually requested one, leave that to
              //application code.
              //However, should the packet be too far off, we can't pass this to application
              //code anyway, so there can be no legit reply.  Try to send a little before it gets that bad
              //also.
              if (((rxHeader[1] & HEADER_TYPE_FIELD) == HEADER_TYPE_UNRELIABLE) ||
                  ((rxHeader[1] & HEADER_TYPE_FIELD) == HEADER_TYPE_RELIABLE_SPECIAL) ||
                  
                  //We don't seem to be able to do 64 bit compares, but we can do
                  //do 32 bits, and check byte 4 and 5 manually f
                  (saturatingAbs(diff)>12000000L)
                  )
              {
                ///the reason we can do this automatically without corrupting state is
                //Replies can't be replied to, so sending this won't overwrite the value that
                //Says what we are watching for, if we were watching for something
                debug("sndar");
                int8_t oldpwr = _powerLevel;
                //Autoreplies to very close and not yet conneted devices
                //still need auto tx power control to not overload the reciever,
                //which would prevent connection from happening. 
                if(_rssi > -40)
                {
                  _powerLevel=-18;
                }
                rawSendSG1(smallBuffer, 0, rxIV, HEADER_TYPE_REPLY_SPECIAL);
                _powerLevel=oldpwr;
              }
            }
          }
        }

        if (replayProtection)
        {
          if (getPacketTimestamp() <= channelTimestampHead)
          {
            debug("Rotl");
            return 0;
          }

          if(!checkTimestampReplayAttack(getPacketTimestamp()))
          {
            return 0;
          }
        }
      }

      
      //cipherContext.setKey(defaultChannel.channelKey, 32);
      //cipherContext.setIV(rxIV, 8);
      defaultChannel.setupCipher(rxIV);
      //Mask off the 2 bits used for hop count, as those can be decremented by untrusted
      //nodes.  We don't support repeater operation yet, but these bits are reserved for that
      rxHeader[2] &= 0b00111111;
      cipherContext.addAuthData(rxHeader, 3);

      if (isReply())
      {
        cipherContext.addAuthData((awaitReplyToIv), 8);
      }
      //By this point, datalen is the FEC encodable par after taking off the header and decoding.

      //Encrypted part starts after the 3 byte hint and 8 byte IV, since we already
      //Took off the header.
      //It also *ends* before the IV, so we subtract 3+8+8
      cipherContext.decrypt(tmpBuffer + 3 + 8, tmpBuffer + 3 + 8, remainingDataLen - (3 + 8 + 8));

      if (!cipherContext.checkTag(tmpBuffer + (remainingDataLen - 8), 8))
      {
        debug("rbc");
        debug(DATALEN);
        debug(tmpBuffer[remainingDataLen - 8]);
        //Not 3+8
        debug(tmpBuffer[2 + 8]);
        return 0;
      }


      memcpy(DATA, tmpBuffer + 3 + 8, remainingDataLen - (3 + 8 + 8));

      rxPathLoss = txRSSI - _rssi;

      //Remove the hint, IV, and MAC from the length
      //Store in the real data len so the user has it
      DATALEN = remainingDataLen - (3 + 8 + 8);

      //Add null terminator that's just kind of always there.
      DATA[DATALEN] = 0;

      //other things mess with that register
      RSSI = _rssi;

      if (replayProtection)
      {
        //Mark it so we don't accept old packets again.
        channelTimestampHead = getPacketTimestamp();
      }

      //Don't trust if there's no trust flag even if it is authenticated
      if (rxHeader[1] & HEADER_TIME_TRUST_FIELD)
      {
        rxTimeTrust |= TIMETRUST_SECURE;
        if (isReply())
        {
          rxTimeTrust |= TIMETRUST_CHALLENGERESPONSE;
        }
      }

      if (replayProtection)
      {
        doPerPacketTimeFunctions(rxTimeTrust);
      }

      //One reply per message only, so delete the challenge we were waiting for
      //responses to. This can't come before doPerPacketTimeFunctions,
      //Because that packet can send a message that we really don't have about replies to
      //and we need the lack of data in awaitReplyToIv to accurately reflect if
      //we are still waiting.
      if (isReply())
      {
        memset(awaitReplyToIv, 0, 8);
        requestedReply = 0;
      }
      
      lastSG1Presence = approxUnixMillis();
      if (isSpecialType())
      {
        /*HANDLE SYSTEM RESERVED PACKETS*/

        //These do not set the lastSG1Message flag, because we need to do
        //bckground tasks without triggering a wakeup.
        debug("sp");
        gotSpecialPacket = true;
        return 0;
      }
      debug("rcv");

      //
      lastSG1Message = lastSG1Presence;
      return 1;
    }
    else
    {
      debug("Offchannel");
      debug(rxHintSequence);
      debug(defaultChannel.fixedHintSequence);
      return 0;
    }
  }

  uint32_t RFM69::readHintSequence(uint8_t * x)
  {
    uint32_t rxHintSequence = readUInt32(x);
    rxHintSequence = applyHintSequenceMask (rxHintSequence);
    return rxHintSequence;
  }

  bool RFM69::decodeSG1RT()
  {
    debug("DECODING");

    int8_t _rssi = RSSI;
    gotSpecialPacket = false;
    RAWPAYLOADLEN = DATALEN;

    doTimestamp();
    //We need to use the 4 most significant bytes from the clock to recover the
    //IV, since they weren't explicitly sent.
    memcpy(rxIV, systemTimeBytes, 8);

    //Buffer overflow prevention
    if (DATALEN > 80)
    {
      debug("long");
      debug(DATALEN);
      return false;
    }

    uint32_t rxHintSequence = readHintSequence(DATA);
    defaultChannel.recalcBeaconBytes();

    //Private sequences only here
    if (checkHintSequenceRelevance(rxHintSequence,true))
    {
      debug("oh");
    }
    else
    {
      debug("of");
      debug(rxHintSequence);
      debug(defaultChannel.fixedHintSequence);
      return false;
    }

    //Now we get the 4 bytes of IV that we *do* send, and combine those
    //with the implied 4.
    memcpy(rxIV, DATA + 3, 4);


    //We still do the replay protection for this.
    if (replayProtection)
    {
      uint64_t rxTimestamp;

      //The 0 byte of the IV is actualy the node ID at this point
      //That doesn't matter since we don't need submillisecond accuracy
      memcpy(&rxTimestamp,rxIV,8);
      if(!checkTimestampReplayAttack(rxTimestamp))
      {
        return false;
      }
    }

    
    //cipherContext.setKey(defaultChannel.channelKey, 32);
    //cipherContext.setIV(rxIV, 8);
    defaultChannel.setupCipher(rxIV);
    cipherContext.decrypt(tmpBuffer, DATA + 3 + 4, DATALEN - (3 + 4 + 4));

    if (!cipherContext.checkTag(DATA + (DATALEN - 4), 4))
    {
      debug("rbcrt");
      debug(DATALEN);
      return false;
    }

    memcpy(DATA, tmpBuffer, DATALEN - (3 + 4 + 4));

    lastSG1Presence = approxUnixMillis();
    lastSG1Message = lastSG1Presence;

    //Remove the hint, IV, and MAC from the length
    //Store in the real data len so the user has it
    DATALEN = DATALEN - (3 + 4 + 4);

    //Add null terminator that's just kind of always there.
    DATA[DATALEN] = 0;

    //other things mess with that register
    RSSI = _rssi;

    if (replayProtection)
    {
      //Mark it so we don't accept old packets again.
      channelTimestampHead = getPacketTimestamp();
    }

    //The time they send us has an integer ambiguity.
    //normally this would not matter, because if it was wrong,
    //We would not be able to decode.

    //Due to the short MAC, a few bad packets could slip through, and that could
    //cause a problem.  So we don't even try to do time sync with these.

    debug("rcv");
    return true;
  }

  bool RFM69::isRecieving()
  {
    //If we did not get the
    if ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_RXREADY) == 0x00)
    {
      return false;
    }
    return true;
  }

  uint8_t RFM69::_headerTimeTrust()
  {
    uint8_t x = 0;

    //We only advertise trust if its been set with something
    //Equivalent to challenge response.
    if (systemTimeTrust >= TIMETRUST_CHALLENGERESPONSE)
    {
      x |= HEADER_TIME_TRUST_FIELD;
    }
    if (systemTimeTrust & TIMETRUST_ACCURATE)
    {
      x |= HEADER_TIME_ACCURATE_FIELD;
    }

    return x;
  }
  void RFM69::sendBeacon(bool wakeUp)
  {
    defaultChannel.recalcBeaconBytes();
    int8_t power = getAutoTxPower();
    while (power % 4)
    {
      power++;
    }
    //rawSetPowerLevel is going to limit the power to the actual range
    //Of the module, power could be anything, even impossible values
    rawSetPowerLevel(power);

    uint8_t header[3] = {9, 0, 0};
    //The max is there so that power levels below -24 don't actually get encoded
    if (!_isRFM69HW)
    {
      header[2] = (((max(power, -18) + 24) / 4) & 0b1111);
    }
    else
    {
      //HW modules can't go as low
      header[2] = (((max(power, -2) + 24) / 4) & 0b1111);
    }

    header[1] |= _headerTimeTrust();

    if (power > 3)
    {
      header[1] |= HEADER_FEC_GOLAY;
      header[0] = 12;
    }
    else
    {
      header[0] = 9;
    }

    golay_block_encode(header, tmpBuffer);
    if (wakeUp)
    {
      memcpy(tmpBuffer + 6, &(defaultChannel.privateWakeSequence), 3);
    }
    else
    {
      memcpy(tmpBuffer + 6, &(defaultChannel.privateHintSequence), 3);
    }
    debug("hs");
    debug(defaultChannel.privateHintSequence);

    //If we need more than 3dbm, we also probably need to use error correction.
    //This adds an extra 3 bytes.

    //Beacons are short and uncommon, the 3 extra bytes is worth it most of the time.
    if (power > 3)
    {
      memcpy(smallBuffer, tmpBuffer + 6, 3);
      golay_block_encode(smallBuffer, tmpBuffer + 6);
    }

    uint32_t now = millis();
    while (!canSend() && millis() - now < RF69_CSMA_LIMIT_MS)
    {
      //Wake messages have to be sent immediately.
      if (wakeUp == false)
      {
        //8ms seems like a reasonable length to use for CSMA.
        delayMicroseconds(xorshift16() & 8192L);
      }
      _receiveDone();
    }

    lastSentSG1 = approxUnixMillis();
    //Don't actually send the length byte
    sendFrame(tmpBuffer + 1, header[0] - 1);
  }


  //Send a beacon and see if anyone tells us to wake up.
  bool RFM69::checkBeaconSleep()
  {
    //Any recent message indicates we should stay awake.
    //That only includes real user messages and replies to us,
    //Not backgound time sync
    
    //In case of FHSS
    setChannelNumber(channelNumber);

    uint8_t retVal =0;

    if (lastSG1Message > (approxUnixMillis() - 18000L))
    {
      //But we always send a beacon anyway.
      sendBeacon();
      return true;
    }

    //We can use short beacons only if we are currently "connected",
    // Otherwise we will try to do a full packet for time sync.
    if ((lastSG1Presence > (approxUnixMillis() - 600000)) && lastSG1Presence>0)
    {
      sendBeacon();
    }
    else
    {
      //Randomly hop around in hopes of finding them.   But this
      //could take hours if only trying once per minute so watch out.
      fhssOffset = xorshift16();
      //Otherwise we have to send a whole reliable_special packet,
      //So that they can send us the correct time.
      rawSendSG1(0, 0, 0, HEADER_TYPE_RELIABLE_SPECIAL);
    }

    //Just long enough for them to respond.
    //We are assuming 16ms latency to get a packet and another
    //16 to reply, plus at least 10ms to actually process the crypto.
    //due to USB serial port latency, and the fact that some devices might be running
    // at 4MHz.

    //Add additional time at slow bitrates.

    unsigned long start = micros();
    while ((micros() - start) < (45000UL+(bitTime*600UL)))
    {
      //Use the real low power sleep mode.
      //The millis() timer will wake us up because this isn't deep sleep
      delay(2);

      if (receiveDone())
      {
        //Them sending us valid traffic is just as good as a wake
        //message, but just replying with a beacon isn't because
        //we have to do that in the background anyway.
        if (decodeSG1()==1)
        {
          retVal= 1;
          break;
        }
        if (wakeRequestFlag)
        {
          retVal= 1;
          break;
        }
      }
    }

    fhssOffset=0;
    return retVal;
  }

  void RFM69::setChannelKey(unsigned char *key)
  {
    defaultChannel.setChannelKey(key);
  }
/*
  void SG1Channel::setChannelKeyPrecalculated(unsigned char *key, uint32_t * hints)
  {
    memcpy(channelKey, key, 32);
    fixedHintSequence=hints[0];
    privateHintSequence=hints[0];
    privateWakeSequence=hints[0];
    altPrivateHintSequence=hints[0];
    altPrivateWakeSequence=hints[0];

  }
*/

  void SG1Channel::setChannelKey(unsigned char *key)
  {
    memcpy(channelKey, key, 32);

    // Invalidate the hint sequence cache
    intervalNumber=0;
    recalcBeaconBytes();

    union {
      uint64_t x = 0LL;
      uint8_t IV[8];
    };

    uint8_t input[4] = {0, 0, 0, 0};

    //Ensure byte 3 is set to zero, it's a padding byte
    fixedHintSequence = 0;

    
    cipherContext.setKey(channelKey, 32);
    cipherContext.setIV(IV, 8);
    cipherContext.encrypt((uint8_t *)(&fixedHintSequence), input, 3);

    fixedHintSequence &= HINT_SEQUENCE_MASK;
  }

  void RFM69::setProfile(uint8_t profile)
  {
    rfProfile = profile;
    switch (profile)
    {
    case RF_PROFILE_GFSK600:
      setBitrate(600);
      setDeviation(6000);
      setChannelFilter(12500);
      setChannelSpacing(25000);
      maxTxPower = -4;
      if (channelNumber > 1000)
      {
        maxTxPower = 8;
      }
      break;

    case RF_PROFILE_GFSK1200:
      setBitrate(1200);
      setDeviation(8000);
      setChannelFilter(25000);
      setChannelSpacing(25000);
      maxTxPower = -4;
      if (channelNumber > 1000)
      {
        maxTxPower = 8;
      }
      break;

    case RF_PROFILE_GFSK4800:
      setBitrate(4800);
      setDeviation(177000);
      setChannelFilter(540000);
      setChannelSpacing(750000);
      break;

    case RF_PROFILE_GFSK10K:
      setBitrate(10000);
      setDeviation(177000);
      setChannelFilter(540000);
      setChannelSpacing(750000);
      maxTxPower = 8;
      break;

    case RF_PROFILE_GFSK38K:
      setBitrate(38400);
      setDeviation(177000);
      setChannelFilter(540000);
      setChannelSpacing(750000);
      maxTxPower = 8;
      break;

    case RF_PROFILE_GFSK100K:
      setBitrate(100000);
      setDeviation(177000);
      setChannelFilter(540000);
      setChannelSpacing(750000);
      maxTxPower = 8;
      break;

    case RF_PROFILE_GFSK250K:
      setBitrate(250000);
      setDeviation(177000);
      setChannelFilter(650000);
      setChannelSpacing(750000);
      maxTxPower = 8;
      break;

    default:
      break;
    }
  }

  void RFM69::setChannelSpacing(uint32_t hz)
  {
    channelSpacing = hz / 1000L;
    //Recalc the channel number
    setChannelNumber(channelNumber);
  }

  void RFM69::setBitrate(uint32_t bps)
  {
    bitTime = 1000000L / bps;
    bps = 32000000UL / bps;
    writeReg(REG_BITRATEMSB, bps / 256);
    writeReg(REG_BITRATELSB, bps % 256);
  }

  void RFM69::setDeviation(uint32_t hz)
  {
    //probably fine to ditch a preamble byte or two
    //with wide channels
    if (hz >= 110000)
    {
      writeReg(REG_PREAMBLELSB, 0x02);
    }
    else
    {
      writeReg(REG_PREAMBLELSB, 0x04);
    }

    //Round to nearest by adding half the fstep
    hz += 30;
    hz /= 61;

    writeReg(REG_FDEVMSB, hz / 256);
    writeReg(REG_FDEVLSB, hz % 256);
  }

  //Approximately set the RX bandwidth, rounding up to the next step
  void RFM69::setChannelFilter(uint32_t hz)
  {

    if (hz <= 12500L)
    {
      //12.5khz half spacing.
      //Only 10kkz guard band! I think this will still wodk though.
      writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_6);
    }

    if (hz <= 2500L)
    {
      //12.5khz half spacing.
      //Only 10kkz guard band! I think this will still wodk though.
      writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_5);
    }

    else if (hz <= 50000L)
    {
      writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_4);
    }
    else if (hz <= 62000L)
    {
      writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_4);
    }
    else if (hz <= 100000L)
    {
      //100KHz
      writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3);
    }
    else if (hz <= 200000L)
    {
      //200KHz
      writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_2);
    }
    else if (hz <= 330000L)
    {
      //332KHz
      writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_1);
    }
    else if (hz <= 500000L)
    {
      writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_1);
    }
    else
    {
      writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_0);
    }
  }

  //Set the RF channel. If the channel is higher than the actual
  //Number of channels, wrap around.
  void RFM69::setChannelNumber(uint16_t n)
  {
    uint32_t minf;
    uint32_t maxf;

    if (channelNumber == n)
    {
      if (n <= 1000)
      {
        return;
      }
    }
  
    //Need to recalc the power level limits
    setProfile(rfProfile);
    

    //Back to hz
    uint32_t spacing = channelSpacing * 1000L;

    channelNumber = n;

    if (freqBand == RF69_915MHZ)
    {
      minf = 902500000UL;
      maxf = 922000000UL;
    }
    else if (freqBand == RF69_868MHZ)
    {
      minf = 863500000UL;
      maxf = 868000000UL;
    }
    else if (freqBand == RF69_433MHZ)
    {
      minf = 433050000UL;
      maxf = 434790000UL;
    }
    else
    {
      return;
    }

    //We exclude 1 channel widths from the bottom of the range.
    //This is so that low-numbered channels will naturally not overlap with
    //the same channels in different bandwidths.
    minf += spacing;
    uint16_t totalChannels = ((maxf - minf) / spacing);

    if (channelNumber > 1000L)
    {
      uint16_t prng = n;
      doTimestamp();
      //Get the time in ~60S increments
      uint16_t time = timeToIntervalNumber(systemTime);
      //Now we add the time and the channel, and permute it with the PRNG.
      prng ^= time;
      prng = applyXorshift(prng);
      //Use that as the actual channel.
      //Limit 256 channels of hopping no matter what
      n = (prng + fhssOffset) % 256;
    }

   
    //Min frequency, go up to the center frequency of ch0, then go up to the selected channel
    setFrequency(minf + (spacing / 2) + (spacing * (n % totalChannels)));
  }

  void RFM69::getEntropy(uint8_t changes)
  {

    //We are looking for N changes in the RSSI value
    //We add every reading together till we see a change, then
    //re-encrypt the entropy pool with itself as the key.

    //This change detection algorithm is very weak due to long strings
    //of values that change every other time. Still, it's probably
    //Usable especially since we are always adding entropy from different sources.

    uint8_t x = 0;
    uint8_t y = 0;


    for (int i = 0; i < changes; i++)
    {
      while (1)
      { //Look for changes in RSSI.
        //Don't bother looking at other data, RSSI can be read without
        //changing the mode of the sensor
        y = (uint8_t)readRSSI();

        //It's not really possible for adding uncorrelated values to
        //make something less random, so there's really no reason not to add these.
        entropyRegister += y;

        //To avoid
        if ((!(y == x)))
        {
          x = y;
          break;
        }
      }

      //After every change we detect, mix the entropy.
      //That pool is gonna be SO stirred.
      mixEntropy();
    }

    //Return to RX mode after readTemperature probably put us in standby
    _receiveDone();
  }

  bool RFM69::receivedReply()
  {
    //We zero it out when we got a reply, so we can use this to check
    return memset(awaitReplyToIv,0,8);
  }

  bool RFM69::isReply()
  {
    if (
        ((rxHeader[1] & HEADER_TYPE_FIELD) == HEADER_TYPE_REPLY) ||
        ((rxHeader[1] & HEADER_TYPE_FIELD) == HEADER_TYPE_REPLY_SPECIAL))
    {
      return 1;
    }
    return 0;
  }

  bool RFM69::isSpecialType()
  {
    uint8_t x = (rxHeader[1] & HEADER_TYPE_FIELD); 
    if ((x == HEADER_TYPE_REPLY_SPECIAL) ||
        (x == HEADER_TYPE_RELIABLE_SPECIAL) ||
        (x == HEADER_TYPE_SPECIAL))
    {
      return true;
    }
    return false;
  }
  bool RFM69::isRequest()
  {
    if (((rxHeader[1] & HEADER_TYPE_FIELD) == HEADER_TYPE_RELIABLE) ||
        ((rxHeader[1] & HEADER_TYPE_FIELD) == HEADER_TYPE_RELIABLE_SPECIAL))
    {
      return 1;
    }
    return 0;
  }


unsigned long RFM69::getUnixTime32()
{
  doTimestamp();
  return systemTime/1000000L;
}


void RFM69::setUnixTime32(uint32_t time)
{
  doTimestamp();
  //Watch out, this nonsense takes an entire 
  int64_t systemTimeSeconds = systemTime/1000000L;

  uint32_t time2= time;


  //Only take the bottom 4 bytes. This is because we want to correctly
  //handle 2038 rollover, and the network protocol side of things
  //supports that even thouth the 32 bit seconds do not
  memcpy(&systemTimeSeconds, &time2, 4);

  //Convert back to mcroseconds
  systemTimeSeconds*= 1000000L;

  setTime(systemTimeSeconds);
}



int64_t RFM69::readInt64(void * p)
{
  int64_t rVal;
  memcpy(&rVal,p,8);
  return rVal;
}

void RFM69::writeInt64(void * p, int64_t val)
{
  memcpy(p,&val,8);
}


void RFM69::writeUInt32(void * p, uint32_t val)
{
  memcpy(p,&val,4);
}


uint32_t RFM69::readUInt32(void * p)
{
  uint32_t rVal;
  memcpy(&rVal,p,4);
  return rVal;
}


void RFM69::writeUInt16(void * p, uint16_t val)
{
  memcpy(p,&val,2);
}


uint16_t RFM69::readUInt16(void * p)
{
  uint32_t rVal;
  memcpy(&rVal,p,2);
  return rVal;
}





