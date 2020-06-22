
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
  Serial.println(" ready,sleep");


  // Initialize the RFM69HCW:
  // radio.setCS(10);  //uncomment this if using Pro Micro
  radio.initialize(FREQUENCY);

  //RX filter bandwidth and deviation are automatically set
  //when you do this. Channel spacing is also calculated automatically.
  radio.setProfile(RF_PROFILE_GFSK250K);

  //This channel number determines the actual RF frequency. Set it to anything you
  //Want, the library knows how to wrap around if you go over the top channel.
  radio.setChannelNumber(3);

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

  //Send something to try and get them to send us time sync info
  radio.sendSG1("BOOT", 4);

}

unsigned long last = 0;
uint8_t attempts = 10;


void loop()
{

  //This function hansles timesync for you as well.
  if (radio.checkBeaconSleep())
  {
    Serial.println("Got wakeup flag");
    //Do stuff here? Listen for packets for 1ms

    //Send both kinds of message for a more complete test
    radio.sendSG1RT("WAKE", 4);
    Serial.println("Sent RT");

    radio.sendSG1("WAKE", 4);
    Serial.println("Sent SG1");


    for (int i = 0; i < 1000; i++)
    {
      if (radio.receiveDone())
      {
        Serial.println("Got raw data");
        Serial.println(radio.DATALEN);
        //Getting a reply means we can skip this for 10 miniutes
        unsigned long start = millis();
        if (radio.decodeSG1())
        {
          Serial.println(millis() - start);
          Serial.println("decoded");
        }
        last = radio.approxUnixMillis();

      }
      delay(1);
      Serial.flush();
    }

  }
  else
  {
    Serial.println("No wakeup flag, radio sleeping 5s");
    Serial.println((int32_t)(radio.unixMicros() / 1000000LL));
    radio.sleep();

    //Sleep here with true low power sleep instead of delay for real applications
    delay(5000);
    //Wake radio back up
    radio.receiveDone();
  }
}
