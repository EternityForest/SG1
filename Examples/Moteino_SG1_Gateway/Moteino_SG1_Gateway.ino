
// RFM69HCW Example Sketch
// Send serial input characters from one RFM69 node to another
// Based on RFM69 library sample code by Felix Rusu
// http://LowPowerLab.com/contact
// Modified for RFM69HCW by Mike Grusin, 4/16

#include <SG1.h>

#include <SPI.h>





#define AWAIT_START 0
#define AWAIT_LEN 1
#define AWAIT_DATA 2
#define AWAIT_END 3


#define START_CODE 42
#define STOP_CODE 43


class NanoframeParser
{
  public:
    uint8_t rxPtr;
    uint8_t rxLen;
    uint8_t state = 0;
    void (*callback)(uint8_t, uint8_t*, uint8_t);

    //Parse a byte using the rx buf, if you get a packet, return the len, else return -1
    void parse(uint8_t b, uint8_t * buf)
    {
      if (state == AWAIT_START)
      {
        if (b == STOP_CODE)
        {
          // Do nothing, allow you to get to this state and stay there
          // With a ver long string of end codes as a flush.
        }
        else if (b == START_CODE)
        {
          state = AWAIT_LEN;
        }
        else
        {
          state = AWAIT_END;
        }
      }

      else if (state == AWAIT_LEN)
      {
        rxLen = b;
        rxPtr = 0;
        state = AWAIT_DATA;
      }

      else if (state == AWAIT_DATA)
      {
        buf[rxPtr] = b;
        rxPtr++;

        if (rxPtr == rxLen)
        {
          state = AWAIT_END;
          if (callback)
          {
            callback(buf[0], buf + 1, rxLen - 1);
          }
          return;
        }
        
        //Don't know how it could ever get into that state, but defensive programing and all.
        if(rxPtr > rxLen)
        {
          state = AWAIT_END;
        }

      }

      else if (state == AWAIT_END)
      {
        if (b == STOP_CODE)
        {
          state = AWAIT_START;
        }
     
       
      }
      else
      {
        //Fallback in case of corruption
        state = AWAIT_END;
      }
      return;
    }
};

NanoframeParser parser;


// RFM69 frequency, uncomment the frequency of your module:

//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ



// Use ACKnowledge when sending messages (or not):




RFM69 radio;




unsigned long last = 0;
uint8_t attempts = 10;

uint8_t tmp[129];

#define MSG_NEWDATA 1
#define MSG_SET_KEY 2
#define MSG_DECODE 3
#define MSG_DECODED 4
#define MSG_FAIL 5
#define MSG_SEND 6
#define MSG_RX 7
#define MSG_SENT 8
#define MSG_RNG 9
#define MSG_CFG 10
#define MSG_TIME 11
#define MSG_RFPOWER 12
#define MSG_DECODEDRT 13
#define MSG_SENDRT 14
#define MSG_PAIR 15


bool listening = 0;


void nfSend(char cmd, uint8_t *d, uint8_t len)
{
  Serial.write(42);
  Serial.write(len + 1);
  Serial.write(cmd);
  Serial.write(d, len);
  Serial.write(43);
}


uint8_t alreadyDecoded = 0;

void callback(byte command, byte *payload, byte length) {
  /* Make use of the payload before sending something, the buffer where payload points to is
     overwritten when a new message is dispatched */

   

  switch (command) {
    case MSG_SET_KEY:
      radio.setChannelKey(payload + 1);
      break;

    case MSG_DECODE:
      //We don't decode one message twice
      if(alreadyDecoded)
      {
        break;
      }
      
      if(payload[1]&1)
      {
        radio.keepRemotesAwake = 1;
      }
      else{
        radio.keepRemotesAwake=0;
      }

      //Same command decodes both RT and non-RT frames

      
      if(radio.decodeSG1RT())
      {
        alreadyDecoded = 1;
        Serial.write(42);
        Serial.write(radio.DATALEN + 32+ 1+1);
        Serial.write(MSG_DECODEDRT);
        Serial.write(radio.RSSI);

        Serial.write(radio.defaultChannel.channelKey, 32);
        Serial.write(radio.DATA, 32);
        Serial.write(43);
      }
      else if (radio.decodeSG1())
      {
        alreadyDecoded = 1;
        Serial.write(42);
        Serial.write(radio.DATALEN + 32+ 1+1);
        Serial.write(MSG_DECODED);
        Serial.write(radio.rxPathLoss);
        Serial.write(radio.RSSI);
        Serial.write(radio.defaultChannel.channelKey, 32);
        Serial.write(radio.DATA, 32);
        Serial.write(43);
      }
   
      listening = 1;
      break;

    case MSG_SEND:
      radio.setPowerLevel((int8_t)(payload[0]));
      radio.sendSG1(payload + 1, length - 1);
      nfSend(MSG_SENT, 0, 0);
      break;

    case MSG_RX:
      listening = 1;
      break;


    //MSG_RNG lets you get a random number.
    case MSG_RNG:
      radio.urandom(tmp, 1);
      nfSend(MSG_RNG, tmp, 1);
      break;

    //Connection parameters string form
    case MSG_CFG:

      radio.setProfile(payload[0]);
      radio.setChannelNumber(((uint16_t *)(payload + 1))[0]);
      radio.setPowerLevel((int8_t)(payload[3]));
      break;

    case MSG_TIME:
      Serial.println("timecmd");
      Serial.println(length);
      if (length)
      {
        radio.setTime( ((uint64_t *)payload)[0], TIMETRUST_SECURE | TIMETRUST_CHALLENGERESPONSE | TIMETRUST_CLAIM_TRUST | TIMETRUST_LOCAL);
      }
      else
      {
        ((uint64_t *)tmp)[0] = radio.unixMicros();
        nfSend(MSG_TIME, tmp, 8);
      }
      break;

    case MSG_SENDRT:
      radio.setPowerLevel((int8_t)(payload[0]));
      radio.sendSG1RT(payload + 1, length - 1);
      nfSend(MSG_SENT, 0, 0);
      break;

    case MSG_PAIR:
      Serial.println("PAIR");
      //Payload contains the node ID we want to give the remote.
      radio.pairWithRemote(payload[1]);
      break;

  }
}

void setup()
{
  Serial.begin(250000);
  parser.callback = &callback;
  radio.initialize(FREQUENCY);
  Serial.println("Init");
  radio.setProfile(RF_PROFILE_GFSK250K);
  Serial.println("prof");
  radio.setChannelNumber(150);
  radio.setPowerLevel(-127);
  radio.setNodeID(NODEID_HUB);
  radio.setTime(null);
  Serial.println("time");
}


byte smallBuf[8];

void loop()
{
  // RECEIVING
  // In this section, we'll check with the RFM69HCW to see
  // if it has received any packets:
  if (listening)
  {
    if (radio.receiveDone()) // Got one!
    {
      //Enable decoding again
      alreadyDecoded = 0;

      //This doesn't disable the actual radio RX, it just keeps us from
      //polling it.
      listening = 0;
      
      memset(smallBuf,0,6);
      
      smallBuf[0] = radio.RSSI;
      

      if (radio.decodeSG1Header())
      {
        Serial.println("Header");
        ((uint32_t *)(smallBuf+4))[0] = radio.readHintSequence();
      }

      
      memcpy(smallBuf+1,radio.DATA,3);

      //Send format: RSSI DecodedHint, RawHint
      //Reciever has to check bothe to see if they make sense
      nfSend(MSG_NEWDATA, smallBuf, 7);
      

      //RF needs to be physically listening, but we don't actually
      //do anything with that data until we're done handling the current packet
      radio._receiveDone();
    }
  }

  while (Serial.available()) {
    parser.parse(Serial.read(), tmp);
  }
}
