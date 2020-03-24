# RFM69 Library

This is a library for secure-ish(Think cheap Bluetooth lock, not bank grade)
encrypted communications with RFM69.

Work in progress, currently I have Golay code error correction
dynamic TX power control, encryption and authentication,
and a decent API that still lets you transmit and recieve
raw packets if you need to.


## API

### #include <SG1.h>


### RFM69 radio;

Normal constructor

### RFM69(uint8_t slaveSelectPin, uint8_t interruptPin, bool isRFM69HW)

Alternate constructor


### radio.initialize(RF69_915MHZ);
Set frequency band. ONLY THIS ONE IS TESTED! I live in the US and don't know the law for the others!!!

### radio.setProfile(RF_PROFILE_GFSK4800);

setBitrate, setChannelFilter, setDeviation, and setChannelSpacing are available to do this manually. 

Other available profiles:
```
#define RF_PROFILE_GFSK1200 1
#define RF_PROFILE_GFSK4800 2
#define RF_PROFILE_GFSK10K 3
#define RF_PROFILE_GFSK38K 4
#define RF_PROFILE_GFSK100K 5
#define RF_PROFILE_GFSK250K 6
```

### radio.setChannelNumber(41);

SG1 works with channel numbers. The actual maximum number of these depends on the width and frequency
band, but that's OK, we will automatically wrap around. 

NOTE: This library is alpha, THIS CHANNEL MAPPING ALGORITHM MAY CHANGE.

The step size is equal to whatever channel width was set in the profile, so you'll need to be careful as channel 1 may be the same
frequency as channel 2 with a different step size.

You don't need to worry as much about interference with SG1 messages, we just ignore anything that doesn't have the same key as us.


#### Algorithm:

The channel number algorith is as follows:
Start at the bottom of the frequency range, then increment by 500KHz N/100 times, wrapping to the bottom as needed.

Now move up by half a channel width, and increment by channelWidth steps. 
If you get higher than than (half a channel width below the top), wrap to half a channel width above the bottom.

Do this N times.

This gives a measure of predictability when using things with multiple different channel widths in one deployment.

### radio.setPowerLevel(-18);
-127 enables auto TX power, which only works between SG1 nodes. This is real dBM, not an arbitrary 0-31 scale!.


### radio.readAllRegs();
Same as the lowpowerlabs version, debugging purposes, uses a lot of RAM

### radio.setNodeID('A');
Only applies to SG1 encoded messages, not raw ones. Node IDs are not addresses!! They are there for future routing
devices.

### radio.setChannelKey(uint8_t * key)
This has to be a 32 byte random key that defines the channel. Only applies to SG1 messages. We ignore messages not for us.

### radio.receiveDone()
If we are recieving, return true and go to standby  got a packet. If not, start recieving.

### radio.decodeSG1()
Call after getting a packet. Returns true if message was a real SG1 encoded message on our channel key, and decodes the actual
payload into data and datalen.


### int32_t radio.getFEI()
Return the radio's measured frequency offset in Hz. If you have a particular two radios that don't seem to get along, you cat use this.
Note that automatic frequency error correction is enabled, so it should not be a big issue.

### radio.DATA, DATALEN, and RSSI
The data, length, and RSSI of the recieved packet, if any. If you called decodeSG1, it will be the decoded payload, otherwise it is the totally raw message.


### radio.send(uint8_t * data, uint8_t len)
Send a raw message. No additional framing at all gets added here. Length is the length of the data, and does not include the buffer itself.

### radio.sendSG1(uint8_t * data, uint8_t len)
Send an SG1 message, giving you (unproven)encryption, TX power control, and automat FEC as needed. Works exactly like send, you just
have to decode first. Adds 25 bytes to every message, and doubles payload size if the signal is very weak(Due to the error correction used to increase range).

Note that up to 2 extra padding bytes may be added if the system decides Golay encoding is needed.

### radio.sendSG1Reply(uint8_t * data, uint8_t len)

Same as sendSG1, but sends as a reply to the last message recieved. You can reply to anything, except a reply.

### radio.sendSG1Request(uint8_t * data, uint8_t len)

Same as sendSG1, but marks as requesting a reply. There is no automatic replies or retransmission, because we want
to avoid wasting bandwidth on messages that are only acknowledgements, so we let you do your own replies.

### radio.isReply(), radio.isRequest()

Call after decoding to learn what kind of message this is.

Note that replies will only ever be to the most recent non-reply you have sent,
so if you get a reply, you know what it is.

### radio.receivedReply()
Returns true if the most recent non-reply you sent has beed replied to. You can safely send a reply
while listening for one, but you can only be listening for a reply to one packet at a time.

### radio.xorshift32()
Simple non-secure 32 bit random number generator used for internal backoff timings. Reseeded automatically, and fast, 
but not secure.





