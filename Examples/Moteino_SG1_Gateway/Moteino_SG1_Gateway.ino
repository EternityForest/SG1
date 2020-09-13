
// RFM69HCW Example Sketch
// Send serial input characters from one RFM69 node to another
// Based on RFM69 library sample code by Felix Rusu
// http://LowPowerLab.com/contact
// Modified for RFM69HCW by Mike Grusin, 4/16

#include <SG1.h>

#include <SPI.h>



#define SERWRITE(x) Serial.write((uint8_t)(x))
#define SERWRITEN(x,y) Serial.write((uint8_t *)(x),y)




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
      debug("st");
      debug(state);
      debug(b);
      debug(rxPtr);
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
        if(rxLen>128)
        {
          Serial.println("BADLEN");
          Serial.flush();
        }
        rxPtr = 0;
        state = AWAIT_DATA;
      }

      else if (state == AWAIT_DATA)
      {
        buf[rxPtr] = b;
        rxPtr++;

        //Detect and stop overruns, so nobody can flood us with nonsense
        if(rxPtr >= 128)
        {
          state = AWAIT_END;
          return;
        }
        
        if (rxPtr == rxLen)
        {
          state = AWAIT_END;
          if (callback)
          {
            debug("CMD:");
            debug(buf[0]);
            callback(buf[0], buf + 1, rxLen - 1);
            debug("DONE");
          }
          return;
        }

        //Don't know how it could ever get into that state, but defensive programing and all.
        if (rxPtr > rxLen)
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
        else
        {
          Serial.print("OVR");
          Serial.println(buf[0]);
          Serial.flush();
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



#if !defined(__AVR__)
RFM69 radio(8, 3, true);
#else

RFM69 radio;
#endif



const char VERSION = 0;

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

#define MSG_VERSION 16
#define MSG_HW_FAIL 17
#define MSG_DECODEDBEACON 18
#define MSG_LATENCYTEST 19
#define MSG_BGNOISE 20

#define MSG_SENDSPECIAL 25

#define MSG_DECODEDSTRUCTURED 29
#define MSG_SEND_STRUCTURED 30


bool listening = 0;


void nfSend(char cmd, uint8_t *d, uint8_t len)
{
  SERWRITE(42);
  SERWRITE(len + 1);
  SERWRITE(cmd);
  SERWRITEN(d, len);
  SERWRITE(43);
}


uint8_t alreadyDecoded = 0;
byte reqID = 0;
void callback(byte command, byte *payload, byte length) {
  /* Make use of the payload before sending something, the buffer where payload points to is
     overwritten when a new message is dispatched */
  switch (command) {
    case MSG_SET_KEY:
      radio.setChannelKey(payload);
      break;

    case MSG_DECODE:
      //We don't decode one message twice
      if (alreadyDecoded)
      {
        break;
      }

      if (payload[0] & 1)
      {
        radio.keepRemotesAwake = 1;
      }
      else {
        radio.keepRemotesAwake = 0;
      }

      //Same command decodes both RT and non-RT frames


      if (radio.decodeSG1RT())
      {
        alreadyDecoded = 1;
        SERWRITE(42);
        SERWRITE(1 + 1 + 8 + 4 + 32 + radio.DATALEN);
        SERWRITE(MSG_DECODEDRT);
        SERWRITE(radio.RSSI);
        SERWRITEN(radio.rxIV, 8);

        // 4 reserved padding bytes
        SERWRITE(0);
        SERWRITE(0);
        SERWRITE(0);
        SERWRITE(0);

        SERWRITEN(radio.defaultChannel.channelKey, 32);
        SERWRITEN(radio.DATA, (radio.DATALEN));
        SERWRITE(43);
      }
      else
      {

        uint8_t result = radio.decodeSG1();

        if ((result == 1) || (radio.gotSpecialPacket==2))
        {
          bool rp = radio.isReply();

          alreadyDecoded = 1;
          if (rp)
          {
          }
          else
          {
            SERWRITE(42);

            SERWRITE(1 + 1 + 1 + 8 + 4 + 32 + radio.DATALEN);
            
            if (radio.gotSpecialPacket==2)
            {
              SERWRITE(MSG_DECODEDSTRUCTURED);
            }
            else
            {
              SERWRITE(MSG_DECODED);
            }
            
            SERWRITE(radio.rxPathLoss);
            SERWRITE(radio.RSSI);
            SERWRITEN(radio.rxIV, 8);
  
  
            SERWRITE(radio.rxHeader[1]);
            SERWRITE(radio.rxHeader[2]);
            // Reserved padding bytes.
            SERWRITE(0);
            SERWRITE(0);
  
  
            SERWRITEN(radio.defaultChannel.channelKey, 32);
  
          
            SERWRITEN(radio.DATA, (radio.DATALEN));
            SERWRITE(43);
          }
        }
        //Handle beacons separately.
        else if (result == 2)
        {
          alreadyDecoded = 1;
          SERWRITE(42);
          SERWRITE(1 + 1 + 1 + 4 + 32);
          SERWRITE(MSG_DECODEDBEACON);
          SERWRITE(radio.rxPathLoss);
          SERWRITE(radio.RSSI);
          // Reserved padding bytes.
          SERWRITE(0);
          SERWRITE(0);
          SERWRITE(0);
          SERWRITE(0);

          SERWRITEN(radio.defaultChannel.channelKey, 32);
          SERWRITE(43);
        }

      }

      listening = 1;
      //Don't just leave that flag set and waste battery
      radio.keepRemotesAwake = 0;
      break;

    case MSG_SEND:
      debug(payload[4]);
      radio.setPowerLevel((int8_t)(payload[0]));
      reqID =  payload[1];

      if (payload[2]) {
        radio.setNodeID(payload[2]);
      }
      else
      {
        radio.setNodeID(1);
      }
      //1 reserved bytes
      radio.sendSG1(payload + 4, length - 4);

      SERWRITE(42);
      SERWRITE(1 + 1 + 8);
      SERWRITE(MSG_SENT);
      // We can save and restore the IV we are waiting for this way.
      SERWRITE(reqID);
      SERWRITEN(radio.awaitReplyToIv, 8);
      SERWRITE(43);
      break;


    case MSG_SEND_STRUCTURED:
      debug(payload[4]);
      radio.setPowerLevel((int8_t)(payload[0]));
      reqID =  payload[1];

      if (payload[2]) {
        radio.setNodeID(payload[2]);
      }
      else
      {
        radio.setNodeID(1);
      }
      //1 reserved bytes      
      radio.rawSendSG1(payload + 4, length - 4, 0, HEADER_TYPE_STRUCTURED);

      SERWRITE(42);
      SERWRITE(1 + 1 + 8);
      SERWRITE(MSG_SENT);
      // We can save and restore the IV we are waiting for this way.
      SERWRITE(reqID);
      SERWRITEN(radio.awaitReplyToIv, 8);
      SERWRITE(43);
      break;


    case MSG_SENDSPECIAL:
      radio.setPowerLevel((int8_t)(payload[0]));
      reqID =  payload[1];
      if (payload[2]) {
        radio.setNodeID(payload[2]);
      }
      else
      {
        radio.setNodeID(1);
      }
      //1 reserved bytes

      radio.rawSendSG1(payload + 4, length - 4, 0, HEADER_TYPE_SPECIAL);
      SERWRITE(42);
      SERWRITE(1 + 1 + 8);
      SERWRITE(MSG_SENT);
      // We can save and restore the IV we are waiting for this way.
      SERWRITE(reqID);
      SERWRITEN(radio.awaitReplyToIv, 8);
      SERWRITE(43);
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
      radio.setChannelNumber(radio.readUInt16(payload + 1));
      radio.setPowerLevel((int8_t)(payload[3]));
      break;

    case MSG_TIME:
      if (length)
      {
        radio.setTime( radio.readInt64(payload), TIMETRUST_SECURE | TIMETRUST_CHALLENGERESPONSE | TIMETRUST_CLAIM_TRUST | TIMETRUST_LOCAL);
      }
      else
      {
        radio.writeInt64(tmp, radio.unixMicros());
        nfSend(MSG_TIME, tmp, 8);
      }
      break;

    case MSG_SENDRT:
      radio.setPowerLevel((int8_t)(payload[0]));
      if (payload[1]) {
        radio.setNodeID(payload[1]);
      }
      else
      {
        radio.setNodeID(1);
      }
      //3 reserved bytes here
      radio.sendSG1RT(payload + 4, length - 4);
      nfSend(MSG_SENT, 0, 0);
      break;

    case MSG_PAIR:
      //Payload contains the node ID we want to give the remote.
      radio.pairWithRemote(payload[1]);
      break;

    case MSG_LATENCYTEST:
      nfSend(MSG_LATENCYTEST, 0, 0);
      break;

  }
}



void setup()
{
  Serial.begin(250000);
  parser.callback = &callback;
  if (radio.initialize(FREQUENCY) == false)
  {
    Serial.println("RADIO CONNECT FAIL");
    while (1) {
      nfSend(MSG_HW_FAIL, 0, 0);
      Serial.println("FAILURE");
      delay(100);
      while (Serial.available())
      {
        Serial.read();
      }
    }
  }
  Serial.println("SG1 GATEWAY");

  radio.setProfile(RF_PROFILE_GFSK250K);
  radio.setChannelNumber(150);
  radio.setPowerLevel(-127);
  radio.setNodeID(NODEID_HUB);
  radio.setTime(null);
  Serial.println("BOOTED");
}


byte smallBuf[32 + 8];

unsigned long lastSentVersion;


void loop()
{

  if (millis() - lastSentVersion > 1000)
  {
    debug("sv");
    lastSentVersion = millis();
    smallBuf[0] = 'S';
    smallBuf[1] = 'G';
    smallBuf[2] = '1';
    smallBuf[3] = VERSION;
    nfSend(MSG_VERSION, smallBuf, 4);

    smallBuf[0] = radio.readRSSI();
    nfSend(MSG_BGNOISE, smallBuf, 1);
    debug("done");

  }
  // RECEIVING
  // In this section, we'll check with the RFM69HCW to see
  // if it has received any packets:
  if (listening)
  {
    if (radio.receiveDone()) // Got one!
    {
      debug("RF");
      //Enable decoding again
      alreadyDecoded = 0;

      //This doesn't disable the actual radio RX, it just keeps us from
      //polling it.
      listening = 0;

      memset(smallBuf, 0, 7);

      smallBuf[0] = radio.RSSI;

      memcpy(smallBuf + 1, radio.DATA, 3);


      if (radio.decodeSG1Header())
      {
        radio.writeUInt32(smallBuf + 4,radio.readHintSequence());
      }
      debug("HDR");
      



      //Send format: RSSI, RawHint, decodedhint
      //Reciever has to check bothe to see if they make sense

      nfSend(MSG_NEWDATA, smallBuf, 7);
      debug("nfs");


      //RF needs to be physically listening, but we don't actually
      //do anything with that data until we're done handling the current packet

      radio._receiveDone();
      debug("RD");

    }
  }

  while (Serial.available()) {
    parser.parse(Serial.read(), tmp);
  }
}
