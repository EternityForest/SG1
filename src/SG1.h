// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Copyright LowPowerLab LLC 2018, https://www.LowPowerLab.com/contact
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
#ifndef RFM69_h
#define RFM69_h
#include <Arduino.h>            // assumes Arduino IDE v1.0 or greater
#include <SPI.h>

//Levels of time trust. OR the trust attributes together to get a trust level
#define TIMETRUST_ACCURATE 1
#define TIMETRUST_CLAIM_TRUST 2

//Never set these two without CLAIM_TRUST, if they don't even claim trust
//We obviously shouldn't
#define TIMETRUST_SECURE 4
#define TIMETRUST_CHALLENGERESPONSE 8

//////////////////////////////////////////////////////////////////////
//Platform and digitalPinToInterrupt definitions credit to RadioHead//
//////////////////////////////////////////////////////////////////////
// Select platform automatically, if possible
#ifndef RF69_PLATFORM
 #if (MPIDE>=150 && defined(ARDUINO))
  // Using ChipKIT Core on Arduino IDE
  #define RF69_PLATFORM RF69_PLATFORM_CHIPKIT_CORE
 #elif defined(MPIDE)
  // Uno32 under old MPIDE, which has been discontinued:
  #define RF69_PLATFORM RF69_PLATFORM_UNO32
#elif defined(NRF51)
  #define RF69_PLATFORM RF69_PLATFORM_NRF51
#elif defined(NRF52)
  #define RF69_PLATFORM RF69_PLATFORM_NRF52
 #elif defined(ESP8266)
  #define RF69_PLATFORM RF69_PLATFORM_ESP8266
 #elif defined(ESP32)
  #define RF69_PLATFORM RF69_PLATFORM_ESP32
 #elif defined(ARDUINO)
  #define RF69_PLATFORM RF69_PLATFORM_ARDUINO
 #elif defined(__MSP430G2452__) || defined(__MSP430G2553__)
  #define RF69_PLATFORM RF69_PLATFORM_MSP430
 #elif defined(MCU_STM32F103RE)
  #define RF69_PLATFORM RF69_PLATFORM_STM32
 #elif defined(STM32F2XX)
  #define RF69_PLATFORM RF69_PLATFORM_STM32F2
 #elif defined(USE_STDPERIPH_DRIVER)
  #define RF69_PLATFORM RF69_PLATFORM_STM32STD
 #elif defined(RASPBERRY_PI)
  #define RF69_PLATFORM RF69_PLATFORM_RASPI
#elif defined(__unix__) // Linux
  #define RF69_PLATFORM RF69_PLATFORM_UNIX
#elif defined(__APPLE__) // OSX
  #define RF69_PLATFORM RF69_PLATFORM_UNIX
 #else
  #error Platform not defined! 	
 #endif
#endif

#if defined(ESP8266) || defined(ESP32)
  #define ISR_PREFIX ICACHE_RAM_ATTR
#else
  #define ISR_PREFIX
#endif

// digitalPinToInterrupt is not available prior to Arduino 1.5.6 and 1.0.6
// See http://arduino.cc/en/Reference/attachInterrupt
#ifndef NOT_AN_INTERRUPT
 #define NOT_AN_INTERRUPT -1
#endif
#ifndef digitalPinToInterrupt
 #if (RF69_PLATFORM == RF69_PLATFORM_ARDUINO) && !defined(__arm__)
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
   // Arduino Mega, Mega ADK, Mega Pro
   // 2->0, 3->1, 21->2, 20->3, 19->4, 18->5
   #define digitalPinToInterrupt(p) ((p) == 2 ? 0 : ((p) == 3 ? 1 : ((p) >= 18 && (p) <= 21 ? 23 - (p) : NOT_AN_INTERRUPT)))
  #elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) 
   // Arduino 1284 and 1284P - See Maniacbug and Optiboot
   // 10->0, 11->1, 2->2
   #define digitalPinToInterrupt(p) ((p) == 10 ? 0 : ((p) == 11 ? 1 : ((p) == 2 ? 2 : NOT_AN_INTERRUPT)))
  #elif defined(__AVR_ATmega32U4__)
   // Leonardo, Yun, Micro, Pro Micro, Flora, Esplora
   // 3->0, 2->1, 0->2, 1->3, 7->4
   #define digitalPinToInterrupt(p) ((p) == 0 ? 2 : ((p) == 1 ? 3 : ((p) == 2 ? 1 : ((p) == 3 ? 0 : ((p) == 7 ? 4 : NOT_AN_INTERRUPT)))))
  #else
   // All other arduino except Due:
   // Serial Arduino, Extreme, NG, BT, Uno, Diecimila, Duemilanove, Nano, Menta, Pro, Mini 04, Fio, LilyPad, Ethernet etc
   // 2->0, 3->1
   #define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 3 ? 1 : NOT_AN_INTERRUPT))
  #endif
 #elif (RF69_PLATFORM == RF69_PLATFORM_UNO32) || (RF69_PLATFORM == RF69_PLATFORM_CHIPKIT_CORE)
  // Hmmm, this is correct for Uno32, but what about other boards on ChipKIT Core?
  #define digitalPinToInterrupt(p) ((p) == 38 ? 0 : ((p) == 2 ? 1 : ((p) == 7 ? 2 : ((p) == 8 ? 3 : ((p) == 735 ? 4 : NOT_AN_INTERRUPT)))))
 #else
  // Everything else (including Due and Teensy) interrupt number the same as the interrupt pin number
  #define digitalPinToInterrupt(p) (p)
 #endif
#elif defined(__SAMD21__) || defined (__SAMD51__) //Arduino.h in most/all cores wrongly "#define digitalPinToInterrupt(P) (P)" after calling variant.h
  #define digitalPinToInterrupt(P)   (g_APinDescription[P].ulExtInt)
#endif

// On some platforms, attachInterrupt() takes a pin number, not an interrupt number
#if (RF69_PLATFORM == RF69_PLATFORM_ARDUINO) && defined (__arm__) && (defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_SAM_DUE))
 #define RF69_ATTACHINTERRUPT_TAKES_PIN_NUMBER
#endif
////////////////////////////////////////////////////

#define RF69_SPI_CS             SS // SS is the SPI slave select pin, for instance D10 on ATmega328

// INT0 on AVRs should be connected to RFM69's DIO0 (ex on ATmega328 it's D2, on ATmega644/1284 it's D2)
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega88) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)
  #define RF69_IRQ_PIN          2
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
  #define RF69_IRQ_PIN          2
#elif defined(__AVR_ATmega32U4__)
  #define RF69_IRQ_PIN          7
#elif defined(__STM32F1__)
  #define RF69_IRQ_PIN          PA3
#elif defined(MOTEINO_M0)
  #define RF69_IRQ_PIN          9
#elif defined(ARDUINO_SAMD_ZERO) //includes Feather SAMD
  #define RF69_IRQ_PIN          3
#elif defined(ESP8266)
  #define RF69_IRQ_PIN          4
  #define RF69_SPI_CS           15
#else
  #define RF69_IRQ_PIN          2
#endif

#define RF69_MAX_DATA_LEN       128 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE

// available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR   0
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS   1000
#define RF69_FSTEP  61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK   0x80
#define RFM69_CTL_REQACK    0x40

#define RFM69_ACK_TIMEOUT   30  // 30ms roundtrip req for 61byte packets

// You can't have 2 of the same ID on a channel or security goes down.
// These predefined IDs are to reduce the chance of accidentally doing that.
#define NODEID_HUB 1
#define NODEID_SENSOR 2
#define NODEID_SWITCH 3
#define NODEID_LIGHT 4
#define NODEID_HANDHELD 5
#define NODEID_ROBOT 6



class RFM69 {
  public:
    static uint8_t DATA[RF69_MAX_DATA_LEN+1]; // RX/TX payload buffer, including end of string NULL char
    static uint8_t DATALEN;
    static uint8_t PAYLOADLEN;
    static int16_t RSSI; // most accurate RSSI during reception (closest to the reception). RSSI of last packet.
    static uint8_t _mode; // should be protected?

    uint8_t channelKey[32];

    //Used for keeping track of active wake requests.
    uint8_t numWakeRequests=0;
    struct WakeRequest * wakeRequests =0;


    //Thie hint sequences for 3 seconds ago.
    //We have to keep old and new, because the clocks are not to be
    //Expected to stay accurate.

    //Used as the first 4 bytes after header to cause a wakeup
    uint32_t privateWakeSequence;
    //Used as the normal beaconing that devices do to announce that they are on a certain
    //channel
    uint32_t privateHintSequence;

    uint32_t fixedHintSequence;
    uint16_t bitTime=10;



    //The hint sequence for 3 seconds from now
    //Used as the first 4 bytes after header to cause a wakeup
    uint32_t newPrivateWakeSequence;
    //Used as the normal beaconing that devices do to announce that they are on a certain
    //channel
    uint32_t newPrivateHintSequence;


    //Header of the last packet
    uint8_t rxHeader[3];

    //IV, including timestamp
    uint8_t rxIV[8];

    //UNIX timestamp micros
    static int64_t unixMicros();

    bool receivedReply();
    bool isReply();

    bool isRequest();

    uint32_t urandomRange(uint32_t, uint32_t);
    void urandom(uint8_t *, uint8_t);

    static void setTime(int64_t time, uint8_t trust= TIMETRUST_SECURE|TIMETRUST_CHALLENGERESPONSE|TIMETRUST_CLAIM_TRUST|TIMETRUST_ACCURATE);
    //When we last got a correct packet
    int64_t lastRx=0;


    uint8_t nodeID;

    void setNodeID(uint8_t x)
    {
      nodeID=x;
    };

  

    //Cached interval number for the privateHintSequence cache
    uint64_t intervalNumber;

    int8_t rxPathLoss= 127;
    bool wakeRequestFlag =0;

    //Send a beacon and check for the wake message.
    //Return True on wake message, else turn radio off(call recieveDone to wake)
    bool checkBeaconSleep();

    void sendBeacon(bool wakeup=false);

    void encrypt(const char * key);

    RFM69(uint8_t slaveSelectPin=RF69_SPI_CS, uint8_t interruptPin=RF69_IRQ_PIN, bool isRFM69HW=false);

    bool initialize(uint8_t freqBand, uint8_t networkID=1);

    void setNetwork(uint8_t networkID);
    void getEntropy(int changes=128);
    bool decodeSG1(uint8_t * key=0);
    int64_t getPacketTimestamp();
    uint32_t xorshift32();

    int32_t getFEI();

    void setBitrate(uint32_t bps);
    void setChannelFilter(uint32_t bps);
    void setProfile(uint8_t profile);
    void setChannelSpacing(uint32_t hz);

    void setDeviation(uint32_t hz);
    void setChannelNumber(uint16_t ch);

    //Width of a channel in khz includig padding
    uint16_t channelSpacing=350;
    uint16_t channelNumber=1;

    uint8_t freqBand = 0;

    void setChannelKey(uint8_t * key);

    void doBeacon();
    bool canSend();


  

    //Sends a raw RFM69 packet.
    virtual void send(const void* buffer, uint8_t bufferSize);

    //Sends an SG1 protocol packet
    virtual void sendSG1(const void* buffer, uint8_t bufferSize,uint8_t * challenge=0, uint8_t * key=0);
    
    //Default is unreliable type
    virtual void rawSendSG1(const void* buffer, uint8_t bufferSize, bool useFEC, int8_t txPower, uint8_t * useChallenge, uint8_t * key=0,
    uint8_t packetType= 0b0010000);

    virtual void sendSG1Reply(const void* buffer, uint8_t bufferSize);
    virtual void sendSG1Request(const void* buffer, uint8_t bufferSize);

    virtual bool receiveDone();
    bool ACKReceived(uint16_t fromNodeID);
    bool ACKRequested();
    
    uint32_t getFrequency();
    void setFrequency(uint32_t freqHz);

    void setCS(uint8_t newSPISlaveSelect);
    int16_t readRSSI(bool forceTrigger=false); // *current* signal strength indicator; e.g. < -90dBm says the frequency channel is free + ready to transmit
    void spyMode(bool onOff=true);
    void promiscuous(bool onOff=true); //deprecated, replaced with spyMode()
    virtual void setHighPower(bool onOFF=true); // has to be called after initialize() for RFM69HW
    virtual void setPowerLevel(int8_t level); // reduce/increase transmit power level
    virtual void rawSetPowerLevel(int8_t level); // reduce/increase transmit power level

    void sleep();
    uint8_t readTemperature(uint8_t calFactor=0); // get CMOS temperature (8bit)
    void rcCalibration(); // calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]

    // allow hacking registers by making these public
    uint8_t readReg(uint8_t addr);
    void writeReg(uint8_t addr, uint8_t val);
    void readAllRegs();
    void readAllRegsCompact();
    bool isRecieving();

    int64_t rxTime=0;
    int8_t getAutoTxPower();
    
    void addEntropy(uint32_t x);

  protected:
    static void isr0();
    void interruptHandler();
    virtual void interruptHook(uint8_t CTLbyte) {};
    static volatile bool _haveData;
    virtual void sendFrame(const void* buffer, uint8_t size);
    void recalcBeaconBytes();
    void doPerPacketTimeFunctions(uint8_t rxTimeTrust);
    void initSystemTime();
    virtual bool _receiveDone();


    uint8_t _headerTimeTrust();
    bool _recieveDone();
    bool isSpecialType();

    //Lets us do _recieveDone in the internal loop but
    //mark the packet as unhandled to return it to the user later
    //when they call recieveDone
    bool handled=false;

    //Increment when requesting, zero this out when you get a reply
    //Or send a message that doesn't request one.
    //It's used to track failed requests so we can increase power appropriately.
    uint8_t requestedReply=0;


    //Most recent timestamp that re have decoded.
    int64_t channelTimestampHead=0;

    //Are we waiting for a reply
    uint8_t awaitReplyToIv[8];

    uint8_t _slaveSelectPin;
    uint8_t _interruptPin;
    uint8_t _interruptNum;
    uint16_t _address;
    bool _spyMode;
    int8_t _powerLevel;
    bool _isRFM69HW;
#if defined (SPCR) && defined (SPSR)
    uint8_t _SPCR;
    uint8_t _SPSR;
#endif
#ifdef SPI_HAS_TRANSACTION
  SPISettings _settings;
#endif

    virtual void receiveBegin();
    virtual void setMode(uint8_t mode);
    virtual void setHighPowerRegs(bool onOff);
    virtual void select();
    virtual void unselect();

};


#define RF_PROFILE_GFSK1200 1
#define RF_PROFILE_GFSK4800 2
#define RF_PROFILE_GFSK10K 3
#define RF_PROFILE_GFSK38K 4
#define RF_PROFILE_GFSK100K 5
#define RF_PROFILE_GFSK250K 6


uint32_t urandomRange(uint32_t f, uint32_t t);
void urandom(uint8_t * target, uint8_t len);

#endif

//#define debug(x) Serial.println(x);Serial.flush()
//#define REGISTER_DETAIL
#define debug(x)


//WakeRequests keep track of channels that we want to send wake requests
//to. This means we can send messages to channels keys other than our own.
struct WakeRequest
{
  uint8_t trigger[3];
  uint8_t wake[3];
};
