/*
 * 
 * Demonstrates SG1 pairing. We enter pairing mode(on the discover channel), and allow a remote device to set up our pairing parameters.
 * Use the pairing master to test this.
 */

#include <SG1.h>

#include <SPI.h>



// RFM69 frequency, uncomment the frequency of your module:

//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ


// Create a library object for our RFM69HCW module:

RFM69 radio;


char * key = "NotTheActualKeyTheMasterIsOn";
void setup()
{
  // Open a serial port so we can send keystrokes to the module:

  Serial.begin(9600);
  Serial.print("Node ...");


  // Initialize the RFM69HCW:
  // radio.setCS(10);  //uncomment this if using Pro Micro
  radio.initialize(FREQUENCY);
  Serial.print("Initialized\n");

  //RX filter bandwidth and deviation are automatically set
  //when you do this. Channel spacing is also calculated automatically.
  radio.setProfile(RF_PROFILE_GFSK250K);

  Serial.println(3);Serial.flush();
  

  //This channel number determines the actual RF frequency. Set it to anything you
  //Want, the library knows how to wrap around if you go over the top channel.
  radio.setChannelNumber(150);
    Serial.println(3);Serial.flush();


  //Manual, use -127 to set automatic.
  radio.setPowerLevel(-127);
  Serial.println(3);Serial.flush();

  //Show state of module
  //radio.readAllRegs();

  //Every node on a channel needs a unique ID byte
  radio.setNodeID('A');
  Serial.println(4);Serial.flush();



  //Set a random time. This allows things to work without a clock, but it makes the whole thing subject to replay attacks,
  //should the random time values ever overlap.

  //In practice this would require many reboots of the system to work as an attack,
  // And so is almost certainly fine for home automation, weather stations, and the like,
  //In addition. the "selection" of replayable values would be limited to about a ten second window of matching packets.
  radio.setTime(null);
  
  Serial.println(3);Serial.flush();

  //The encryption key used both for actual encryption, and also for message filtering.
  //Many devices can share a channel, the library will ignore messages for other channels.

  radio.setChannelKey((uint8_t *)key);
    Serial.println(3);Serial.flush();


}

unsigned long l = 0;
void loop()
{
  if (Serial.available())
  {
    uint8_t cmd =Serial.read();
    if(cmd=='l')
    {
        //Now, we are going to listen for a pairing request from the remote device.
        //For up to 15 seconds.  If we do get a connection, -1 disables saving it to EEPROM, but you
        //could change that to an address.
        Serial.println("PAIRING LISTEN");
        //Reserve 64 bytes at whatever address you choose.
        if(radio.listenForPairing(-1))
        {
          Serial.println("PAIRED");
        }
        else
        {
          Serial.println("FAIL");
        }
    }
    else if(cmd == 's')
    {
      //Save the connection params to eeprom.
      //For practical use, probably better to do this in listenforpairing, to avoid saving nonsense if it fails.
      radio.saveConnectionToEEPROM(64);
    }

     else if(cmd == 'r')
    {
      //Load a connection from address 64
      radio.loadConnectionFromEEPROM(64);
    }
  }
  
  if(millis()-l>1000)
  {
      l=millis();
      radio.sendSG1("This is a test message",strlen("This is a test message"));
  }
  
  if(radio.receiveDone())
  {
    Serial.println("rawmsg");
      Serial.println(9);Serial.flush();

    if(radio.decodeSG1())
    {
        Serial.println(10);Serial.flush();

      Serial.println("Recieved:");
      Serial.println((char *)radio.DATA);
    }
  }
}
