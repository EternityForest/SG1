
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


#if !defined(__AVR__)
RFM69 radio(8,3,true);
#else
RFM69 radio;
#endif

char * key = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";
void setup()
{
  // Open a serial port so we can send keystrokes to the module:

  Serial.begin(9600);
  Serial.print("Node ");
  Serial.println(" ready");


  // Initialize the RFM69HCW:
  // radio.setCS(10);  //uncomment this if using Pro Micro
  if(radio.initialize(FREQUENCY)==false)
  {
    while(1)
    {
      Serial.println("RADIO FAIL");
      delay(1000);
    }
  }


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

}

unsigned long last = 0;
unsigned long lastrt = 0;


void loop()
{

  const char * pl = "One ring to rule them all\0\0";

  //Every 1 seconds, send a  message.
  
  if ( ((millis() - last) > 1000L))
  {
    radio.sendSG1((uint8_t *)pl, strlen(pl));
    last = millis();
    Serial.println("Sent full packet");

    //block sending rt packet for a bit, nodes can only handle so many
    lastrt=millis();
    
  }


  //But every 500ms, send a short realtime message that uses less data
  //But is not as secure or reliable.

  //Note that these messages are impossible to decide without synchronized clocks,
  //It won't work if you don't also exchange normal SG1 requests too

  else if ( ((millis() - lastrt) > 500))
  {

    radio.sendSG1RT("RT", 2);
    lastrt = millis();
  }


  // RECEIVING

  while (radio.receiveDone()) // Got one!
  {

    Serial.print("Raw message,  RSSI: ");
    Serial.println(radio.RSSI);

    //The way to know if something is an RT message is to try and decode it
    if (radio.decodeSG1RT())
    {
        Serial.print("SG1 RT Decoded: [");
      for (byte i = 0; i < radio.DATALEN; i++)
      {
        Serial.print((char)radio.DATA[i]);
      }

      Serial.println("]");
      
    }

    //Not an RT message, try decoding as a standard mesage.
     else if (radio.decodeSG1())
    {
      Serial.print("SG1 Decoded: [");

      // The actual message is contained in the DATA array,
      // and is DATALEN bytes in size:
      for (byte i = 0; i < radio.DATALEN; i++)
      {
        Serial.print((char)radio.DATA[i]);
      }
      
      Serial.println("]");

      //Good practice to always handle requests, some nodes rely on
      //them for time sync
      if (radio.isRequest())
      {
        //Send an empty message in response.
        radio.sendSG1Reply(0, 0);
        Serial.println("Sent reply");
      }
    }
  }
}
