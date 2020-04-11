
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
  radio.setProfile(RF_PROFILE_GFSK100K);

  //This channel number determines the actual RF frequency. Set it to anything you
  //Want, the library knows how to wrap around if you go over the top channel.
  
  //Channels over 1000 automatically enable FHSS, with a unique hop sequence for every number
  radio.setChannelNumber(1599);

  //Manual, use -127 to set automatic.
  radio.setPowerLevel(-8);
  
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
}

unsigned long last = 0;


void loop()
{

  const char * pl = "One Ring\0\0";

  //Every 250 milliseconds, send a message.
  
  //Note that there is a large number of possible channels, and sync is enctirely based on random chance
  //unless you happen to be using millisecond-accurate synchronization like GPS.

  //Sync can't be allowed to drift more than 5 milliseconds, so you'll need a packet about once a minute, at least. And if you lose sync, you may regret it.
  if ( ((millis() - last) > 100))
  {
    radio.sendSG1((uint8_t *)pl, strlen(pl));
    last=millis();
  }




  // RECEIVING

  // In this section, we'll check with the RFM69HCW to see
  // if it has received any packets:

  while (radio.receiveDone()) // Got one!
  {
    Serial.println("PKT");
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
  }
  delay(1);
}
