
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
}

unsigned long last = 0;
uint8_t attempts = 10;


void loop()
{
  // Set up a "buffer" for characters that we'll send:

  const char * pl = "One Ring\0\0";

  //LowPower.idle(SLEEP_15MS, ADC_OFF, TIMER2_ON, TIMER1_ON, TIMER0_ON,
  //                SPI_ON, USART0_ON, TWI_OFF);


  //Ever 3 seconds, send a request message. Also send every 100 milliseconds if we haven't got a reply yet
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

    radio.sendSG1Request((uint8_t *)pl, strlen(pl));
    Serial.print("Sent, TX pwr:");
    Serial.println("\n\n\n");
    
    last = millis();
  }


  //If we have not gotten a reply, we are going to
  //Retry
  else if (!radio.receivedReply())
  {
    //Allow 500ms, serial printing is slowing everything down
    if ((millis() - last) > 500)
    {
      //Limit 10 attempts to resend
      if (attempts)
      {
        attempts -= 1;
        //radio.send((uint8_t *)pl, strlen(pl));
        radio.sendSG1Request((uint8_t *)pl, strlen(pl));
       
        Serial.println("Retry");
        Serial.print(millis()-last);
           last = millis();
      }
    }
  }

  // now we are "connected" so we can reset the counter and enable retries
  if (radio.receivedReply())
  {
    attempts = 10;
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

      if (radio.isRequest())
      {
        //Send an empty message in response.
        radio.sendSG1Reply(0, 0);
        Serial.println("Sent reply");
      }

      if(radio.isReply())
      {
        Serial.println("Got reply");
      }
    }
  }
}
