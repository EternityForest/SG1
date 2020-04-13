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

void golay_block_encode(uint8_t *in, uint8_t *out);
bool golay_block_decode(uint8_t *in, uint8_t *out);

//Leaving all this in, just in case someone wants to make
//Some crazy ultralight variant. For now, I think ChaCha is the better choice,
//Because it's easier to find implementations
//EAX<SpeckTiny> cipherContext;
ChaChaPoly cipherContext;

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

#define SPECIAL_TIME_VALUE_IGNORE_REPLAY_PROTECTION -1

static uint8_t systemTimeTrust = 0;
//This is in monotonicMillis()
static uint32_t lastAccurateTimeSet = 0;

//Also monotonicMillis, this one just determines how long since
//We synced either with TIMETRUST_ACCURATE, TIMETRUST_LOCAL, or with a remote node.
//basically, it's used for determining if we should be using
//A random FHSS search
//Note this can't start at zero, because micros starts at zero
static uint32_t lastTimeSync = 7998795;
static unsigned long arduinoMillisOffset = 0;

//Binary ppm, parts per 2**20
static int32_t systemTimebPPMAdjust = 0;

//The reference point for when we started measuring the
//time error
static unsigned long systemTimeCorrectionBaseline = 0;

static union {
  //UNIX time in microseconds
  int64_t systemTime = -1;
  uint8_t systemTimeBytes[8];
};

unsigned long RFM69::monotonicMillis()
{
  return arduinoMillisOffset + millis();
}

void RFM69::addSleepTime(uint32_t m)
{
  noInterrupts();
  arduinoMillisOffset += m;
  systemTime += (m * 1000LL);
  //Can't measure time accross sleep modes.
  //Clock isn't accurate
  systemTimeCorrectionBaseline = 0;
  interrupts();
}

static union {
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
  //Assume very small timestamps are just unset
  if (systemTime > -2 && systemTime < 5000000L)
  {
    urandom(systemTimeBytes, 8);

    //Make it obviously fake by being negative.
    if (systemTime > 0LL)
    {
      systemTime = (-systemTime);
    }
  }
}

void RFM69::syncFHSS(uint16_t attempts)
{
  fhssOffset = xorshift32();
  for (uint16_t i = 0; i < attempts; i++)
  {
    //Try a random FHSS channel and listen for replies,
    //but no matter what  always go back to the normal sequence at the end
    fhssOffset += 1;
    rawSendSG1(0, 0, true, getAutoTxPower(), 0, 0, HEADER_TYPE_RELIABLE_SPECIAL);
    for (int j = 0; j < 100L; j++)
    {
      delay(1);
      if (receiveDone())
      {
        debug("r");
        if (decodeSG1())
        {
          debug(j);
          fhssOffset = 0;
          return;
        }
        if (receivedReply())
        {
          fhssOffset = 0;
          debug("Sync!");
          debug(j);
          return;
        }
      }
    }
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
  x2 += (x2 >> 20) * systemTimebPPMAdjust;

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

  //If it has been ten minutes since accurate setting,
  //The clock has drifted and we aren't accurate anymore.

  //TODO:
  if (systemTimeTrust & (TIMETRUST_ACCURATE))
  {
    if (RFM69::monotonicMillis() - lastAccurateTimeSet > 600000LL)
    {
      systemTimeMicros -= TIMETRUST_ACCURATE;
    }
  }
}

void RFM69::sleepPin(int pin, int mode)
{
  sleepLib.sleepPinInterrupt(pin, mode);
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
  entropyRegister += micros();
  cipherContext.setKey(entropyPool, 20);
  cipherContext.setIV(systemTimeBytes, 8);
  cipherContext.encrypt(entropyPool, entropyPool, 20);
  //We don't use the auth tag
}

uint32_t RFM69::xorshift32()
{
  /* Algorithm "xor" from p. 4 of Marsaglia, "Xorshift RNGs" */
  rngState += micros();
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
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
  entropyRegister += readTemperature();
  entropyRegister ^= readRSSI();

  //Mix entropy before we actually use it, so that we don't
  //get any reuse with the non-secure uses of the pool.
  mixEntropy();

  cipherContext.clear();

  cipherContext.setKey(entropyPool, 20);
  cipherContext.setIV(systemTimeBytes, 8);
  //Copy bytes 20 at a time. Note that
  //We never reveal the raw content of the entropy pool for any reason.
  //We just use the encrypted keystream XORed with the pool
  while (len)
  {
    int x = len;
    if (x > 20)
    {
      x = 20;
    }

    cipherContext.encrypt(target, entropyPool, x);
    len -= x;
    target += x;
  }

  //No information should be left about the old state
  mixEntropy();
}

void RFM69::addEntropy(uint32_t x)
{
  entropyRegister32 += x;
}

void RFM69::setTime(int64_t time, uint8_t trust)
{
  noInterrupts();
  //Special value 0 indicates we keep the current time and only set the trust
  //flags
  if (time)
  {
    systemTime = time;
    systemTimeMicros = micros();
  }
  systemTimeTrust = trust;

  if (trust & (TIMETRUST_ACCURATE | TIMETRUST_LOCAL))
  {
    lastTimeSync = monotonicMillis();
  }
  interrupts();
}

volatile bool RFM69::_haveData;

//Two tmp buffers sized to hold an entire packet, rounded up to 80
//for overflow protection, and so we can use the very end for misc scratchpad stuff
uint8_t tmpBuffer[80];
uint8_t smallBuffer[80];

int8_t RFM69::getAutoTxPower()
{
  int16_t txPower = -4;

  int8_t targetRSSI;

  //Higher speed needs more power
  if (bitTime >= 9)
  {
    targetRSSI = -90;
  }
  else
  {
    targetRSSI = -85;
  }

  //Detect if we aren't in auto mode
  if (_powerLevel > -127)
  {
    return _powerLevel;
  }

  //If we have a message in the last 80s, that means that we can
  //use TX power control
  if ((monotonicMillis() - lastSG1Presence) < (-(80L * 1000L)))
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
    if ((xorshift32() & 0b11==00))
    {
      txPower = -8;
    }
    else
    {
      txPower = maxTxPower;
    }
  }
  return (uint8_t)txPower;
}

void RFM69::sendSG1(const void *buffer, uint8_t bufferSize, uint8_t *challenge, uint8_t *key)
{
  int8_t txPower = getAutoTxPower();
  //Reset the failed request tracking
  requestedReply = 0;

  rawSendSG1(buffer, bufferSize, txPower > -5, txPower, challenge, key, HEADER_TYPE_UNRELIABLE);
}

void RFM69::sendSG1Request(const void *buffer, uint8_t bufferSize)
{
  int8_t txPower = getAutoTxPower();
  requestedReply += 1;
  rawSendSG1(buffer, bufferSize, txPower > -5, txPower, 0, 0, HEADER_TYPE_RELIABLE);
}

void RFM69::sendSG1Reply(const void *buffer, uint8_t bufferSize)
{
  int8_t txPower = getAutoTxPower();
  rawSendSG1(buffer, bufferSize, txPower > -5, txPower, rxIV, 0, HEADER_TYPE_REPLY);
}

void RFM69::sendSG1RT(const void * buffer, const uint8_t bufferSize)
{
  retry:
  smallBuffer[0] = ((uint8_t *)&(defaultChannel.fixedHintSequence))[0];
  smallBuffer[1] = ((uint8_t *)&(defaultChannel.fixedHintSequence))[1];
  smallBuffer[2] = ((uint8_t *)&(defaultChannel.fixedHintSequence))[2];

  doTimestamp();
  noInterrupts();

  //Copy all 8 of the IV bytes, but we only send 4 
  memcpy(smallBuffer+3,systemTimeBytes+4,8);
  interrupts();

  //Replace the LSB byte with the node ID.
  smallBuffer[3]=nodeID;
  
  cipherContext.setKey(defaultChannel.channelKey, 32);
  cipherContext.setIV(smallBuffer+3,8);

  //Now we encrypt. Note that we overwrite the top 4 IV bytes,
  //We don't want to send those, the reciever uses the ones from the system clock.
  cipherContext.encrypt(smallBuffer+3+4, (uint8_t *)buffer,bufferSize);
  cipherContext.computeTag(smallBuffer+3+4+bufferSize, 2);

  //Short range, does not need a long preamble
  writeReg(REG_PREAMBLELSB, 0x02);
  //Note that we don't even try to send these packets if we can't do so
  //in a reasonable time.
  for(int i=0;i<3;i+=1)
  {
    if(trySend(smallBuffer,bufferSize+3+2+4))
    {
      debug("srt");
      break;
    }
    delayMicroseconds(xorshift32()&4095L);
    goto retry;
  }
  writeReg(REG_PREAMBLELSB, 0x04);
}

void RFM69::rawSendSG1(const void *buffer, uint8_t bufferSize, bool useFEC, int8_t txPower,
                       uint8_t *useChallenge, uint8_t *key, uint8_t packetType)
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

  //If the whole message would be big we have to turn off FEC
  //To get the total under 64 bytes
  if (payloadSize > 29)
  {
    useFEC = false;
  }
  //With header, that would exceed 64 bytes
  if (payloadSize > 58)
  {
    Serial.println("SZ");
    return;
  }


  //Round up to 4
  while (txPower % 4)
  {
    txPower++;
  }
  txPower=max(txPower,-24);
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

  cipherContext.clear();
  if (key == 0)
  {
    cipherContext.setKey(defaultChannel.channelKey, 32);
  }
  else
  {
    cipherContext.setKey(key, 32);
  }

retry:
  doTimestamp();
  smallBuffer[0] = ((uint8_t *)&(defaultChannel.fixedHintSequence))[0];
  smallBuffer[1] = ((uint8_t *)&(defaultChannel.fixedHintSequence))[1];
  smallBuffer[2] = ((uint8_t *)&(defaultChannel.fixedHintSequence))[2];

  //First 3 bytes of this are the hint. 4th byte is nodeid part of iv.
  smallBuffer[3] = nodeID;
  noInterrupts();
  //Hint comes before the IV, and first IV byte is
  //NodeID so don't put the time in that
  memcpy(smallBuffer + 3 + 1, systemTimeBytes + 1, 7);
  interrupts();

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
  smallBuffer[79] = 0;

  //Now encrypt some zeros for padding
  //Only 2 padding bytes can ever be needed.
  if (padding > 0)
  {
    cipherContext.encrypt((uint8_t *)smallBuffer + 3 + 8 + bufferSize - padding, (uint8_t *)(smallBuffer + 79), 1);
  }
  if (padding == 2)
  {
    cipherContext.encrypt((uint8_t *)smallBuffer + 3 + 8 + (bufferSize - padding) + 1,
                          (uint8_t *)(smallBuffer + 79), 1);
  }

  //memcpy((uint8_t *)smallBuffer+3+8, (uint8_t *)buffer, bufferSize);
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
      lastSentSG1 = monotonicMillis();
      return;
    }
    delayMicroseconds(xorshift32() & 0b1111111111L);
  }

  //Time to try again, we weren't able to send
  goto retry;
}

uint8_t rxHeader[8];

void SG1Channel::recalcBeaconBytes()
{
  //Recalc the hint sequences for this 16s period
  doTimestamp();

  //To tolerate misalignment, calculate the current, and also the next closest
  //interval, and accept either.

  union {
    //Add 8 seconds to push us into the next period,
    uint64_t currentIntervalNumber = (systemTime) >> 26;
    uint8_t currentIV[8];
  };

  union {
    uint64_t altIntervalNumber = (systemTime + 8000000L) >> 26;
    uint8_t altIV[8];
  };

  //So we tried incrementing, but it didn't change, meaning
  //the actual closest interval is the previous one
  //
  if (currentIntervalNumber == altIntervalNumber)
  {
    altIntervalNumber = (systemTime - 8000000L) >> 26;
  }

  uint8_t input[8] = {0, 0, 0, 0};
  //detect cache
  if (currentIntervalNumber == intervalNumber)
  {
    return;
  }

  //set cache marker
  intervalNumber = currentIntervalNumber;

  //Ensure byte 3 is set to zero, it's a padding byte
  privateHintSequence = 0;
  privateWakeSequence = 0;
  altPrivateHintSequence = 0;
  altPrivateWakeSequence = 0;

  cipherContext.clear();
  cipherContext.setKey(channelKey, 32);
  cipherContext.setIV(altIV, 8);

  //8 bytes total, first 4 are the hint sequence, next are the wake sequence.
  cipherContext.encrypt((uint8_t *)&altPrivateHintSequence, input, 3);
  cipherContext.encrypt((uint8_t *)&altPrivateWakeSequence, input, 3);

  //If the old and new IVs are the same, just copy,
  //Because that encrypt takes a millisecond.
  if (memcmp(currentIV, altIV, 8))
  {
    cipherContext.clear();
    cipherContext.setKey(channelKey, 32);
    cipherContext.setIV(currentIV, 8);

    //8 bytes total, first 4 are the hint sequence, next are the wake sequence.
    cipherContext.encrypt((uint8_t *)&privateHintSequence, input, 3);
    cipherContext.encrypt((uint8_t *)&privateWakeSequence, input, 3);
  }
  else
  {
    memcpy((uint8_t *)&privateHintSequence, (uint8_t *)&altPrivateHintSequence, 3);
    memcpy((uint8_t *)&privateWakeSequence, (uint8_t *)&altPrivateWakeSequence, 3);
  }
  privateHintSequence &= HINT_SEQUENCE_MASK;
  privateWakeSequence &= HINT_SEQUENCE_MASK;
  altPrivateHintSequence &= HINT_SEQUENCE_MASK;
  altPrivateWakeSequence &= HINT_SEQUENCE_MASK;
}

int64_t RFM69::getPacketTimestamp()
{
  int64_t rxTimestamp = 0;
  memcpy(((uint8_t *)(&rxTimestamp)) + 1, rxIV + 1, 7);
  return rxTimestamp;
}

void RFM69::doPerPacketTimeFunctions(uint8_t rxTimeTrust)
{

  int64_t rxTimestamp = 0;
  int64_t diff;

  //Gonna subtract the data time, the preamble, and the sync bytes.
  //we want the start time.

  rxTimestamp = getPacketTimestamp();

  //This basically find out how far ahead their clock is
  diff = rxTimestamp - (rxTime - ((((RAWPAYLOADLEN + 1 + 4) * 8 + 40) * bitTime) + 400LL + 32LL + 8LL));

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
  if (((rxTimeTrust & TIMETRUST_ACCURATE) || ((systemTimeTrust & TIMETRUST_ACCURATE) == 0)) && (rxTimeTrust >= TIMETRUST_SECURE) && (abs(diff) < 2000000LL))
  {
    debug("adj");
    doTimestamp((diff >> 1) + (diff >> 2));
    lastTimeSync = monotonicMillis();
    //Stop the FHSS search when we are actually connected.
  }

  //Otherwise, we have to a jump
  else if ((rxTimeTrust >= systemTimeTrust) && (rxTimeTrust >= TIMETRUST_CHALLENGERESPONSE))
  {
    //data+4 is node ID, the rest of the IV is the time.

    //More than 2s, just jump.
    debug("tj");
    systemTime += diff;

    //Yes, this can go backwards. It's somewhat of a nonce reuse hazard.
    //You got a better plan?
    channelTimestampHead = getPacketTimestamp();

    systemTimeTrust = rxTimeTrust;

    if (rxTimeTrust & TIMETRUST_ACCURATE)
    {
      lastAccurateTimeSet = monotonicMillis();
    }
    lastTimeSync = monotonicMillis();
    //Having them set our timestamp is pretty much the closest thing
    //We have to a "connection", so we send this as kind of an acknowledge
    //for stuff that wants to know if we're there, like FHSS
    rawSendSG1(tmpBuffer, 0, true, getAutoTxPower(), 0, 0, HEADER_TYPE_SPECIAL);
  }
}

bool RFM69::decodeSG1Header()
{
  //SG1 packets can't be shorter than this.
  //Really 9 bytes, byt the len byte is implied
  if (DATALEN < 8)
  {
    debug("tooshort");
    debug(DATALEN);
    return false;
  }

  //First byte is the actual length from the framing, plus the length byte which is just implied
  tmpBuffer[0] = DATALEN + 1;
  //Copy evrything in the header but the length nyte we already have
  memcpy(tmpBuffer + 1, DATA, 5);

  if (golay_block_decode(tmpBuffer, rxHeader))
  {
    debug("badheader");
    debug(tmpBuffer[0]);
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
  if (rxHeader[1] & HEADER_FEC_FIELD)
  {
    golay_block_decode(DATA + 5, x);
  }
  else
  {
    memcpy(x, DATA + 5, 3);
  }

  return ((uint32_t *)(x))[0];
}

void RFM69::useEEPROM(uint16_t addr)
{
  eepromAddr = addr;

  for (int i = 0; i < (32 + 1 + 2); i++)
  {
    tmpBuffer[i] = EEPROM.read(eepromAddr + i);
  }
  if (tmpBuffer[32] < 7)
  {
    defaultChannel.setChannelKey(tmpBuffer);
    setProfile(tmpBuffer[32]);
    setChannelNumber(((uint16_t *)(tmpBuffer + 32 + 1))[0]);
  }
}

bool RFM69::listenForPairing(uint8_t deviceClass[16])
{
  uint8_t oldKey[32];
  uint8_t pairingKey[32];

  uint16_t oldChannel = channelNumber;
  uint8_t oldProfile = rfProfile;
  uint8_t oldPowerLevel = _powerLevel;

  memcpy(oldKey, defaultChannel.channelKey, 32);
  memset(pairingKey, 1, 32);

  //The pairing channel is always 3
  setProfile(RF_PROFILE_GFSK38K);
  //Aim for -2db with a monopole, 1mW EIRP
  //We don't really need long range for this
  setPowerLevel(-4);
  setChannelNumber(3);
  setChannelKey(pairingKey);

  uint8_t buf[64];
  uint8_t secret[32];

  unsigned long now = millis();

  unsigned char state = 0;

  bool success = 0;

  while ((millis() - now) < 15000)
  {
    if (receiveDone())
    {
      replayProtection = 0;
      decodeSG1();
      replayProtection = 1;

      if (gotSpecialPacket)
      {
        if (state == 0)
        {
          if (DATA[0] == SPECIAL_TYPE_PAIRING_REQUEST)
          {
            urandom(buf + 33, 32);
            Curve25519::dh1(buf + 1, secret, buf + 33);
            memcpy(buf + 33, deviceClass, 16);
            buf[0] = SPECIAL_TYPE_PAIRING_CHALLENGE;
            rawSendSG1(buf, 48, true, 3, 0, 0, HEADER_TYPE_SPECIAL);
          }
        }
        else if (state == 1)
        {
          //This packet doesn't do anything besides set up the
          //DH key exchange.
          if (DATA[0] == SPECIAL_TYPE_PAIRING_RESPONSE)
          {
            Curve25519::dh2(DATA + 1, secret);
            cipherContext.clear();
            cipherContext.setKey(DATA + 1, 32);
            //Reuse as something we can make all zeros
            memset(secret, 0, 8);
            cipherContext.setIV(secret, 8);
            state = 2;
          }
        }
        else if (state == 2)
        {
          if (DATA[0] == SPECIAL_TYPE_PAIRING_CONFIG)
          {
            cipherContext.decrypt(DATA + 1, DATA + 1, 48);

            if (cipherContext.checkTag(DATA + DATALEN + 1 + 48, 8))
            {
              //Check for a correct setting on the RF profile
              if (DATA[1 + 32] < 8)
              {
                //Write the whole profile to EEPROM
                for (int i = 0; i < (32 + 1 + 2 + 1); i++)
                {
                  EEPROM.update(eepromAddr + i, DATA[1 + i]);
                }

                defaultChannel.setChannelKey(DATA + 1);
                setProfile(DATA[1 + 32]);
                setChannelNumber(((uint16_t *)(DATA + 1 + 32))[0]);
                setNodeID(DATA[1 + 32 + 1 + 2]);

                setPowerLevel(oldPowerLevel);
                debug("Paired!");
                return 1;
              }
              else
              {
                debug("badrfp");
              }
              success = 1;
            }
            else
            {
              debug("bad");
            }
          }
        }
      }
    }
  }
  setProfile(oldProfile);
  setChannelKey(oldKey);
  setChannelNumber(oldChannel);
  setPowerLevel(oldPowerLevel);
}

bool RFM69::pairWithRemote(uint8_t nodeID)
{
  uint8_t oldKey[32];
  uint8_t pairingKey[32];
  uint16_t oldChannel = channelNumber;
  uint8_t oldProfile = rfProfile;
  int8_t oldPowerLevel = _powerLevel;

  memcpy(oldKey, defaultChannel.channelKey, 32);
  memset(pairingKey, 1, 32);

  //The pairing channel is always 3
  setProfile(RF_PROFILE_GFSK38K);
  setPowerLevel(-4);
  setChannelNumber(3);
  setChannelKey(pairingKey);

  uint8_t buf[64];
  uint8_t secret[32];

  unsigned long now = millis();

  unsigned char state = 0;

  buf[0] = SPECIAL_TYPE_PAIRING_REQUEST;
  rawSendSG1(buf, 48, true, 3, 0, 0, HEADER_TYPE_SPECIAL);

  while ((millis() - now) < 15000)
  {
    if (receiveDone())
    {
      debug("gp895");
      replayProtection = 0;
      decodeSG1();
      replayProtection = 1;

      if (gotSpecialPacket)
      {
        if (DATA[0] == SPECIAL_TYPE_PAIRING_CHALLENGE)
        {
          debug("pairc");
          //Generate 32 bytes of randomness
          //First, get an extra byte or two of entropy, just in case,
          //because this is important
          getEntropy(16);
          urandom(buf + 33, 32);

          //Our part of the first step
          Curve25519::dh1(buf + 1, secret, buf + 33);
          buf[0] = SPECIAL_TYPE_PAIRING_RESPONSE;
          //preserve the pubkey
          memcpy(smallBuffer, buf + 1, 32);
          //We alreadyy got their half, we can do this now
          Curve25519::dh2(DATA + 1, secret);
          //Preserve the shared secret
          memcpy(secret, DATA + 1, 32);

          //need to send them the key
          memcpy(buf + 1, smallBuffer, 32);

          rawSendSG1(buf, 1 + 32, true, -4, 0, 0, HEADER_TYPE_SPECIAL);
          delay(1500);

          //Now we just need to send the actual connection parameters.
          memcpy(buf + 1, oldKey, 32);
          buf[1 + 32] += oldProfile;
          ((uint16_t *)&buf[1 + 32 + 1])[0] = oldChannel;
          buf[1 + 32 + 1 + 2] = nodeID;

          cipherContext.setKey(secret, 32);
          memset(smallBuffer, 0, 8);
          //IV of 0, because key always changes
          cipherContext.setIV(smallBuffer, 0);

          //Encrypt everything after the pubkey, using the shared secret
          cipherContext.encrypt(buf + 1, buf + 1, 32 + 1 + 2);
          cipherContext.computeTag(buf + 1 + 32 + 1 + 2, 8);

          //Send the whole thing in a packet.
          //Note that this is going to encrypt everything again.
          //We just accept the slight bit of extra waste, to be able to
          //Have all th packets use the same format.
          rawSendSG1(buf, 1 + 32 + 1 + 2 + 8, true, 3, 0, 0, HEADER_TYPE_SPECIAL);
        }
      }
    }
  }
  //Switch back
  setProfile(oldProfile);
  setChannelKey(oldKey);
  setChannelNumber(oldChannel);
  setPowerLevel(oldPowerLevel);
}

/*
After recieving a packet, call this to decode it.
Returns 1 if the message was an SG1 packet addressed to 
Our channel.

If True, DATA and DATALEN will be the decoded and decrypted payload.
*/
bool RFM69::decodeSG1(uint8_t *key)
{
  int8_t _rssi = RSSI;
  gotSpecialPacket = false;
  RAWPAYLOADLEN = PAYLOADLEN;

  if (decodeSG1Header() == false)
  {
    return false;
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
    debug(tmpDatalen);
    return false;
  }

  //Check what kind of FEC we are supposed to be using.
  if ((rxHeader[1] & HEADER_FEC_FIELD) == HEADER_FEC_GOLAY)
  {
    debug("fecr");
    debug(tmpDatalen);
    //Full packet FEC mode. N
    for (uint8_t i = 0; i < (tmpDatalen - 5); i += 6)
    {
      //Note that we start at 5 to skip header, not 6, because the lenth byte isn't actually in the buffer
      //Decode all the 3 byte blocks into 6 byte blocks.
      if (golay_block_decode(DATA + i + 5, tmpBuffer + (i / 2)))
      {
        debug("badfec");
        debug(tmpDatalen);
        debug(i);
        return false;
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

  uint32_t rxHintSequence = ((uint32_t *)(tmpBuffer))[0];
  rxHintSequence = rxHintSequence & HINT_SEQUENCE_MASK;

  //3 data bytes indicate a short packet, only using
  //the brief channel hints
  if (tmpDatalen == 3)
  {
    defaultChannel.recalcBeaconBytes();
    if ((rxHintSequence == defaultChannel.privateHintSequence) ||
        (rxHintSequence == defaultChannel.privateWakeSequence) ||
        (rxHintSequence == defaultChannel.altPrivateHintSequence) ||
        (rxHintSequence == defaultChannel.altPrivateWakeSequence))
    {
      rxPathLoss = txRSSI - _rssi;

      lastSG1Presence = monotonicMillis();
      debug("chb");

      //The wake request flag is used to let them wake us up
      if ((rxHintSequence == defaultChannel.privateWakeSequence) || (rxHintSequence == defaultChannel.altPrivateWakeSequence))
      {
        debug("chw");
        wakeRequestFlag = true;
      }
      else
          //It's not one of the wake sequences, just a normal beacon.
          //That means we can send it a message to keep it awake.
          if (keepRemotesAwake)
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
      debug("rb");
      DATALEN = 0;
      return true;
    }
    debug("irb");
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

  if ((rxHintSequence == defaultChannel.fixedHintSequence) |
      (rxHintSequence == defaultChannel.privateHintSequence) |
      (rxHintSequence == defaultChannel.privateWakeSequence) |
      (rxHintSequence == defaultChannel.altPrivateHintSequence) |
      (rxHintSequence == defaultChannel.altPrivateWakeSequence))
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
        int64_t diff = getPacketTimestamp() - (rxTime - ((((RAWPAYLOADLEN + 1 + 4) * 8 + 40) * bitTime) + 400LL + 32LL + 8LL));
        //We are going to try to maintain tight timing
        //100ms is the limit before we tell them. We need the tight sync
        //for FHSS, so use 5ms when that is enabled.
        //Can't reply to a reply

        //For now, we are just going to automatically reply to all these
        //RELIABLE_SPECIAL with the current time.
        //Todo??
        if (((abs(diff) > (5000LL)) && (!isReply())) ||
            ((rxHeader[1] & HEADER_TYPE_FIELD) == HEADER_TYPE_RELIABLE_SPECIAL))
        {

          if ((channelNumber > 1000) || (abs(diff) > (100000LL)))
          {
            //Don't send replies if they actually requested one, leave that to
            //application code.
            //However, should the packet be more than 5s off, we can't pass this to application
            //code anyway, so there can be no legit reply.
            if (((rxHeader[1] & HEADER_TYPE_FIELD) == HEADER_TYPE_UNRELIABLE) ||
                ((rxHeader[1] & HEADER_TYPE_FIELD) == HEADER_TYPE_RELIABLE_SPECIAL) ||
                (abs(diff) > (5000000LL)))
            {
              ///the reason we can do this automatically without corrupting state is
              //Replies can't be replied to, so sending this won't overwrite the value that
              //Says what we are watching for, if we were watching for something
              debug("ofset");
              debug((int32_t)(diff));
              int8_t autoPwr = getAutoTxPower();
              rawSendSG1(smallBuffer, 0, autoPwr >= -4, autoPwr, rxIV, 0, HEADER_TYPE_REPLY_SPECIAL);
            }
          }
        }
      }

      if (replayProtection)
      {
        if (getPacketTimestamp() <= channelTimestampHead)
        {
          debug("Rotl");
          return false;
        }

        //5 seconds max err
        if (getPacketTimestamp() >= (systemTime + 5000000LL))
        {

          debug("rtn");
          return false;
        }

        if (getPacketTimestamp() <= (systemTime - 5000000LL))
        {
          debug("Rr2o");
          return false;
        }
        if (systemTimeTrust < TIMETRUST_CHALLENGERESPONSE)
        {
          debug("rlc");
          return false;
        }
      }
    }

    cipherContext.clear();
    cipherContext.setKey(defaultChannel.channelKey, 32);
    cipherContext.setIV(rxIV, 8);

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
      return false;
    }

    memcpy(DATA, tmpBuffer + 3 + 8, remainingDataLen - (3 + 8 + 8));

    rxPathLoss = txRSSI - _rssi;
    lastSG1Presence = monotonicMillis();
    lastSG1Message = lastSG1Presence;

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
    if (isSpecialType())
    {
      /*HANDLE SYSTEM RESERVED PACKETS*/
      debug("sp");
      gotSpecialPacket = true;
      return false;
    }
    debug("rcv");
    return true;
  }
  else
  {
    debug("Offchannel");
    debug(rxHintSequence);
    debug(defaultChannel.fixedHintSequence);
    return false;
  }
}

/*
  Attempt to decode the packet as an SG1RT message.
  These use 9 bytes less data and are much simpler
*/
bool RFM69::decodeSG1RT()
{
  int8_t _rssi = RSSI;
  gotSpecialPacket = false;
  RAWPAYLOADLEN = PAYLOADLEN;

  doTimestamp();
  noInterrupts();
  //We need to use the 4 most significant bytes from the clock to recover the
  //IV, since they weren't explicitly sent.
  memcpy(rxIV, systemTimeBytes,8);
  interrupts();

  //Buffer overflow prevention
  if (DATALEN > 80)
  {
    debug("long");
    debug(DATALEN);
    return false;
  }

  uint32_t rxHintSequence = ((uint32_t *)(DATA))[0];
  rxHintSequence = rxHintSequence & HINT_SEQUENCE_MASK;

  if ((rxHintSequence == defaultChannel.fixedHintSequence) ||
      (rxHintSequence == defaultChannel.privateHintSequence) ||
      (rxHintSequence == defaultChannel.altPrivateHintSequence))
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
    if (getPacketTimestamp() <= channelTimestampHead)
    {
      debug("Rotl");
      return false;
    }

    //5 seconds max err
    if (getPacketTimestamp() >= (systemTime + 5000000LL))
    {

      debug("rtn");
      return false;
    }

    if (getPacketTimestamp() <= (systemTime - 5000000LL))
    {
      debug("Rr2o");
      return false;
    }
    if (systemTimeTrust < TIMETRUST_CHALLENGERESPONSE)
    {
      debug("rlc");
      return false;
    }
  }

  cipherContext.clear();
  cipherContext.setKey(defaultChannel.channelKey, 32);
  cipherContext.setIV(rxIV, 8);

  cipherContext.decrypt(tmpBuffer, DATA + 3 + 4, DATALEN - (3 + 4 + 2));

  if (!cipherContext.checkTag(DATA + (DATALEN - 2), 2))
  {
    debug("rbc");
    debug(DATALEN);
    return false;
  }

  memcpy(DATA, tmpBuffer, DATALEN - (3 + 4 + 2));

  lastSG1Presence = monotonicMillis();
  lastSG1Message = lastSG1Presence;

  //Remove the hint, IV, and MAC from the length
  //Store in the real data len so the user has it
  DATALEN = DATALEN - (3 + 4+2);

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
  noInterrupts();
  //If we did not get the
  if ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_RXREADY) == 0x00)
  {
    interrupts();
    return false;
  }
  interrupts();
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
      delayMicroseconds(xorshift32() & 8192L);
    }
    _receiveDone();
  }

  lastSentSG1 = monotonicMillis();
  //Don't actually send the length byte
  sendFrame(tmpBuffer + 1, header[0] - 1);
}

//Using full power, check the current channel. Future FHSS work.
/*bool RFM69::trySync()
{
  rawSendSG1(0, 0, 12, 1,0, 0, HEADER_TYPE_RELIABLE_SPECIAL);
  delay(120);
  if(recieveDone())
  {
    decodeSG1();
    if(isReply())
    {
      return true;
    }
  }
  return false;
}
*/

//Send a beacon and see if anyone tells us to wake up.
bool RFM69::checkBeaconSleep()
{
  //Any recent message indicates we should stay awake.
  if (lastSG1Message > (monotonicMillis() - 18000L))
  {
    //But if we haven't sent a message in too long, send one
    //anyway.
    if (lastSentSG1 < (monotonicMillis() - 1500LL))
    {
      sendBeacon();
    }
    return true;
  }
  long waitTime;

  //If our time is marked as accurate, or if we have gotten
  //a packet in the last minute we can use short beacons
  if ((systemTimeTrust & TIMETRUST_ACCURATE) || lastSG1Message > (monotonicMillis() - 60000L))
  {
    sendBeacon();
    waitTime = (40000L + (bitTime + 500));
  }
  else
  {
    //Otherwise we have to send a whole packet
    rawSendSG1(0, 0, getAutoTxPower(), getAutoTxPower() > -8, 0, 0, HEADER_TYPE_RELIABLE_SPECIAL);
    waitTime = (40000L + (bitTime + 1200));
  }

  //Just long enough for them to respond.
  //We are assuming 16ms latency to get a packet and another
  //16 to reply, plus at least 10ms to actually process the crypto.
  //due to USB serial port latency.
  unsigned long start = micros();
  while ((micros() - start) < waitTime)
  {
    //Use the real low power sleep mode.
    sleepLib.adcMode();
    sleepLib.sleepDelay(15);
    systemTimeCorrectionBaseline = 0;
    if (receiveDone())
    {
      //Them sending us valid traffic is just as good as a wake
      //message.
      if (decodeSG1())
      {
        return 1;
      }
      if (wakeRequestFlag)
      {
        return 1;
      }
    }
  }
  //Don't actually sleep, user can do that
  //sleep();
  return 0;
}

void RFM69::setChannelKey(unsigned char *key)
{
  defaultChannel.setChannelKey(key);
}

void SG1Channel::setChannelKey(unsigned char *key)
{
  memcpy(channelKey, key, 32);
  recalcBeaconBytes();

  union {
    uint64_t intervalNumber = 0;
    uint8_t IV[8];
  };

  uint8_t input[4] = {0, 0, 0, 0};

  //Ensure byte 3 is set to zero, it's a padding byte
  fixedHintSequence = 0;

  cipherContext.clear();
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
    setChannelSpacing(2500);
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
    setChannelSpacing(640000);
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
  uint32_t minf = 0;
  uint32_t maxf = 0;

  //Back to hz
  uint32_t spacing = channelSpacing * 1000L;

  if (channelNumber == n)
  {
    if (n <= 1000)
    {
      return;
    }
  }
  else
  {
    //Need to recalc the power level limits
    setProfile(rfProfile);
  }

  channelNumber = n;

  if (channelNumber > 1000L)
  {
    uint32_t prng = n;
    doTimestamp();
    //Get the time in ~60ms increments
    //Add the channel number to the time as an offset.
    //This is so the dead time while switching channels isn't always
    //at the same time for every hop sequence, so it isn't wasted.
    uint64_t time = systemTime + (uint64_t)n;

    time = time >> 24;

    //Now we add the time and the channel, and permute it with the PRNG.
    prng ^= time;
    prng ^= prng << 13;
    prng ^= prng >> 17;
    prng ^= prng << 4;

    //Use that as the actual channel.
    //Limit 256 channels of hopping no matter what
    n = (prng + fhssOffset) % 256;
  }

  if (freqBand == RF69_915MHZ)
  {
    minf = 902000000UL;
    maxf = 926000000UL;
  }
  else if (freqBand == RF69_868MHZ)
  {
    minf = 863000000UL;
    maxf = 868000000UL;
  }
  else if (freqBand == RF69_433MHZ)
  {
    minf = 433050000UL;
    maxf = 434790000UL;
  }

  //We exclude 1 channel widths from the bottom of the range.
  //This is so that low-numbered channels will naturally not overlap with
  //the same channels in different bandwidths.
  minf += spacing;

  uint16_t totalChannels = ((maxf - minf) / spacing);

  //Min frequency, go up to the center frequency of ch0, then go up to the selected channel
  setFrequency(minf + (spacing / 2) + (spacing * (n % totalChannels)));
}

void RFM69::getEntropy(int changes)
{

  //We are looking for 512 changes in the RSSI value
  //We add every reading together till we see a change, then
  //re-encrypt the entropy pool with itself as the key.

  //This change detection algorithm is very weak due to long strings
  //of values that change every other time. Still, it's probably
  //Usable especially since we are always adding entropy from different sources.

  int x = 0;
  int y = 0;
  for (int i = 0; i < changes; i++)
  {
    while (1)
    { //Look for changes in RSSI.
      y = readTemperature(0);
      y += readRSSI();

      //AFC is potentially also a source of randomness
      y += readReg(REG_FEILSB);
      y += readReg(REG_AFCLSB);

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
}

bool RFM69::receivedReply()
{
  //We zero it out when we got a reply, so we can use this to check
  return ((uint64_t *)awaitReplyToIv)[0] == 0;
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
  if (((rxHeader[1] & HEADER_TYPE_FIELD) == HEADER_TYPE_REPLY_SPECIAL) ||
      ((rxHeader[1] & HEADER_TYPE_FIELD) == HEADER_TYPE_RELIABLE_SPECIAL) ||
      ((rxHeader[1] & HEADER_TYPE_FIELD) == HEADER_TYPE_SPECIAL))
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
