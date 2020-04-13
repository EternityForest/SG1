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
#define RF_PROFILE_CUSTOM 0
#define RF_PROFILE_GFSK1200 1
#define RF_PROFILE_GFSK4800 2
#define RF_PROFILE_GFSK10K 3
#define RF_PROFILE_GFSK38K 4
#define RF_PROFILE_GFSK100K 5
#define RF_PROFILE_GFSK250K 6
```

*Be sure to set the profile to "custom" before changing the settings manually,
or they could be overwritten*

### radio.setChannelNumber(41);

SG1 works with channel numbers. The actual maximum number of these depends on the width and frequency band, but that's OK, we will automatically wrap around. 

NOTE: This library is alpha, THIS CHANNEL MAPPING ALGORITHM MAY CHANGE.

The step size is equal to whatever channel width was set in the profile, so you'll need to be careful as channel 1 may be the same frequency as channel 2 with a different step size.

You don't need to worry as much about interference with SG1 messages, we just ignore anything that doesn't have the same key as us.


### radio.setPowerLevel(-18);
-127 enables auto TX power, which only works between SG1 nodes. This is real dBM, not an arbitrary 0-31 scale!.

It is strongly suggested that you use auto TX power control.



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
Call after getting a packet. Returns true if message was a real SG1 encoded message on our channel key, and decodes the actual payload into data and datalen.

Returns false for beacons and system traffic.

Note that this will return true if the message was short beacon frame as well.
In the case of beacons, the data will always be 0.

Note that there are several cases in which this will automatically send replies.
It will reply to any short beacon frames with a beacon or wake mesaage.

It will also send reply messages if the packet is 

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

Same as sendSG1, but sends as a reply to the last message recieved. You can reply to anything, except a reply, but it is not guaranteed to work if you reply to things
other than requests.

This is because messages may have not more than one reply, and the system will occasionally send automatic replies to unreliable packets to correct timing errors.

The system will not do this for request messages, as it knows that the application
will be sending them.

### radio.sendSG1Request(uint8_t * data, uint8_t len)

Same as sendSG1, but marks as requesting a reply. There is no automatic replies or retransmission, because we want to avoid wasting bandwidth on messages that are only acknowledgements, so we let you do your own replies, and include useful data in them.



### radio.isReply(), radio.isRequest()

Call after decoding to learn what kind of message this is.

Note that replies will only ever be to the most recent non-reply you have sent,
so if you get a reply, you know what it is.

### radio.receivedReply()
Returns true if the most recent non-reply you sent has beed replied to. You can safely send a reply while listening for one, but you can only be listening for a reply to one packet at a time.



### radio.sleep()
Turn the radio off, call radio.recieveDone() to wake.

### radio.checkBeaconSleep()

Determine if sending a beacon is necessary, and listens for any replies. If
yyou want to use low power mode, the library is optimized for about 1 beacon per minute.

As this is meant for low power operation, it will use real sleep mode while waiting,
and may disrupt millis() and micros(), so use radio.monotonicMillis().

Note that the accuracy of monotonicMillis() will still be limited by the RC oscillator timer.

If all of the following are true, returns False:

* No recieved data on the channel for 18 seconds
* No response to the beacon within 60ms+500 bit times.

You can then call sleep() to sleep the radio. Call recieveDone() to wake up.



### radio.keepRemotesAwake

If true, the node replies to all beacons on the channel automatically(Assuming you call recieveDone often enough!).

Using this and checkBeaconSleep, you can create very low power devices that periodically send beacons with recieve windows.

Note that it is possible for packet loss to affect a beacon, and thus this can
fail to keep a remote awake.

You will most likely want to detect traffic from the node, and send periodic
keepalives for as long as you want them to stay awake, or until there is no longer a response.



### radio.lastSG1Message
radio.monotonicMillis() time of the last packet. If not using sleep,
it is the same as millis()

### radio.lastSG1Presence
millis() time of the last full packet or beacon we were able to recieve.

### radio.sendBeacon(wakeUp=false)
Send a beacon immediately. If wakeup is true, sends a wake beacon telling
other nodes to wake up.

Beacons on the current channel are recieved as if they are 0 byte packets.


### radio.xorshift32()
Simple non-secure 32 bit random number generator used for internal backoff timings. Reseeded automatically, and fast, but not secure.

### radio.setTime(int64_t uinxMicros)

Set the time to any of 3 different types of value, and mark the time as locally
set and therefore trusted. At least one node must do this!!

If 0, don't change anything. just mark the time. If using a random timestamp,
it should be negative.

Otherwise, it should be a UNIX timestamp in microseconds.

### static radio.unixMicros(adj=0)
Current system timestamp, either a positive real time, or negative random network time not referenced to an epoch. Set adj to something nonzero to add or remove time from it.  

This timestamp can change at any time, including going backwards, as it is synced with SG1.

### static radio.monotonicMillis()
Same as the default millis(), but can by affected by addSleepTime.

### static radio.addSleepTime(uint32_t x)

Adds time to both system and monotonic timers.

In particular, as this timestamp uses mircros(), so when doing some kinds of deep sleep you will want to add amount of time slept to it.


### radio.sleepMCU(x)
Put the MCU into sleep mode for x milliseconds. During this time micros
and millis() may not advance, but radio.monotonicMillis() will correct for this
and should behave as expected. Automatically calls addSleepTime.

### radio.sleepPin(pin, mode)
Sleep until a transition happens on the pin(mode can be RISING, LOW, etc).
