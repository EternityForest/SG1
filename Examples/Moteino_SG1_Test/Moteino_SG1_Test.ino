
// RFM69HCW Example Sketch
// Send serial input characters from one RFM69 node to another
// Based on RFM69 library sample code by Felix Rusu
// http://LowPowerLab.com/contact
// Modified for RFM69HCW by Mike Grusin, 4/16

#include <SG1.h>

#include <SPI.h>



// RFM69 frequency, uncomment the frequency of your module:

//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ



// Use ACKnowledge when sending messages (or not):



// Create a library object for our RFM69HCW module:

RFM69 radio;


char * key = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";
void setup()
{
  // Open a serial port so we can send keystrokes to the module:

  Serial.begin(9600);
  Serial.print("Node ");
  Serial.println(" ready");


  // Initialize the RFM69HCW:
  // radio.setCS(10);  //uncomment this if using Pro Micro
  radio.initialize(FREQUENCY);


  //RX filter bandwidth and deviation are automatically set
  //when you do this. Channel spacing is also calculated automatically.
  radio.setProfile(RF_PROFILE_GFSK250K);

  //This channel number determines the actual RF frequency. Set it to anything you
  //Want, the library knows how to wrap around if you go over the top channel.
  radio.setChannelNumber(150);

  //Manual, use -127 to set automatic.
  radio.setPowerLevel(-127);

  //Show state of module
  //radio.readAllRegs();

  //Every node on a channel needs a unique ID byte
  radio.setNodeID('A');


  //Set a random time. This allows things to work without a clock, but it makes the whole thing subject to replay attacks,
  //should the random time values ever overlap.

  //In practice this would require many reboots of the system to work as an attack,
  // And so is almost certainly fine for home automation, weather stations, and the like,
  //In addition. the "selection" of replayable values would be limited to about a ten second window of matching packets.
  radio.setTime(null);

  //The encryption key used both for actual encryption, and also for message filtering.
  //Many devices can share a channel, the library will ignore messages for other channels.

  radio.setChannelKey((uint8_t *)key);

  Serial.println(radio.readRSSI());

  //RNG demo
  Serial.println(radio.urandomRange(1, 100));

  //Enable the config data feature, 16 bytes, add eeprom 0.
  //Nothing is actually saved to eeprom without an explicit command
  radio.useConfigData(128, 16);

  //Set a config value in RAM
  radio.configData[1]=50;
}

unsigned long last = 0;
uint8_t attempts = 10;


void loop()
{
  // Set up a "buffer" for characters that we'll send:

  const char * pl = "One Ring to rule them all\0\0";
  const char * pl2 = "One Ring to Find Them\0\0";

  //LowPower.idle(SLEEP_15MS, ADC_OFF, TIMER2_ON, TIMER1_ON, TIMER0_ON,
  //                SPI_ON, USART0_ON, TWI_OFF);


  if ( ((millis() - last) > 3000))
  {

    //radio.send((uint8_t *)pl, strlen(pl));



    Serial.println(radio.getAutoTxPower());

    //Read the background noise level.
    Serial.print("BGN:");
    Serial.println(radio.readRSSI());
    Serial.print("System clock: ");
    Serial.println((int32_t)(radio.unixMicros()/1000000LL));
    Serial.println("Path Loss:");
    Serial.println(radio.rxPathLoss);

    Serial.print("Sent, TX pwr:");
    Serial.print(radio.getAutoTxPower());
    Serial.println("\n\n\n");
    
    last = millis();

    radio.sendSG1(pl,strlen(pl));
    radio.sendSG1RT(pl2,strlen(pl2));


    // SG1 defines an application layer called structured messages,
    //they are made of structured records.  The packet type field is different so they will never
    //interfere with raw message applications.

    //Packet types below 192 are reserved for standardization
    radio.writeStructuredRecord(192, "TEST",4);
    radio.flushStructuredMessage();

      Serial.println("Config byte 0 is");
    Serial.println(radio.configData[0]);
    
  }





  // RECEIVING

  // In this section, we'll check with the RFM69HCW to see
  // if it has received any packets:

  while (radio.receiveDone()) // Got one!
  {

    Serial.print("message [");

    // The actual message is contained in the DATA array,
    // and is DATALEN bytes in size:

    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);

    // RSSI is the "Receive Signal Strength Indicator",
    // smaller numbers mean higher power.

    Serial.print("], RSSI ");
    Serial.println(radio.RSSI);

    //This is the FEI, a measure of the of frequency error
    //between sender and reciever
    Serial.print("FEI: ");
    Serial.println(radio.getFEI());

    if (radio.decodeSG1())
    {
      Serial.print("SG1 Decoded: [");

      // The actual message is contained in the DATA array,
      // and is DATALEN bytes in size:

      for (byte i = 0; i < radio.DATALEN; i++)
        Serial.print((char)radio.DATA[i]);

      // RSSI is the "Receive Signal Strength Indicator",
      // smaller numbers mean higher power.

      Serial.println("]");

    }

    
    //After decoding, any structured records are available.
    //Structured messages have a different packet type, so decodeSG1 returns False,
    //so we have to check separately.
    uint8_t * sr;
    while (sr = radio.getStructuredRecord())
    {
      Serial.print("Incoming Record type: ");
      Serial.println(getRecordType(sr));
      Serial.print(" Channel: ");

      Serial.print(getRecordChannel(sr));
      Serial.print(" Len: ");

      Serial.println(getRecordLen(sr));
      Serial.write(getRecordData(sr),getRecordLen(sr));
      
      Serial.println("");
        
    }

    //Flush any system traffic that was queued during the recieving of those messages.
    radio.flushStructuredMessage();
  }
  
  
  
}
