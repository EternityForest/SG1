# RFM69 Library

This is a library for secure-ish(Think cheap Bluetooth lock, not bank grade)
encrypted communications with RFM69 on Atmega328 type chips.

Work in progress, currently I have Golay code error correction
dynamic TX power control, encryption and authentication,
and a decent API that still lets you transmit and recieve
raw packets if you need to.


I also have bluetooth-style pairing using ECDH key exchange. Just type "l" on the pairing device example sketch to listen, then press "p" on the pairing master.

This process takes about 20 seconds, after which you can press "s" on the device to
save the key that the master transmitted to EEPROM, and press "r" to load it again.

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


### radio.listenForPairing(eepromAddr)

Listen for a pairing. This will allow another nearby radio to configure the local device. It will save the recieved config in the eeprom addr, unless it is -1.

You may later load that config from the EEPROM.

Reserve 64 bytes for the config data.

### radio.pairWithRemote(nodeID)

Configure the remote device to the current rf profile, channel, and key, and give it the supplied nodeID. The pairing process uses ECDH key exchange and may take 20 seconds.

Note that unlike the usual idea of pairing, this does not generate a new key, but sets the remote to match the local, so is more flexible and can be thought of as programming rather than pairing.

### radio.loadConnectionFromEEPROM(addr)

Load a connection from eeprom, if one is present at the addr, or else do nothing.

### radio.saveConnectionToEEPROM(addr)

Save a connection to the EEPROM at the given address.


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

The first time you do this, the internal entropy pool gets encrypted with the key.

WARNING: At present, switching channels allows packets from the last 18 seconds to be replayed. This is because different devices may have slightly different clocks,
and we don't want to miss packets due to false replay detection.

The Kaithem gateway software performs host side checking to compensate for this.

### radio.receiveDone()
If we have a buffered message, return True. If we are recieving, return true and go to standby. If not, start recieving.

### uint8_t radio.decodeSG1()
Call after getting a packet. Returns 1 if message was a real SG1 encoded message on our channel key, and decodes the actual payload into DATA and DATALEN.

If the message was a short beacon frame, returns 2. Note that beacon frames are not completely reliable, expect 1 false positive in 10 minutes if someone is hammering random packets of the right length at max rate.

Returns false for beacons and system traffic.

Note that this will return true if the message was short beacon frame as well.
In the case of beacons, the data will always be 0.

Note that there are several cases in which this will automatically send replies.
It will reply to any short beacon frames with a beacon or wake mesaage.

Note that as a side effect, sending may overrwite whatever is currently in radio.DATA, because we listen for new messages while waiting to send.
Copy any data you want to keep before sending anything.

### radio.sendSG1RT(buffer, len)
Sends an SG1RT packet. These have only 128 bits of overhead, but do not include forward error correction.  

They also use a weak 4-byte mac, allowing you to occasionally send forged messages that appear as garbage data(1 in 2**32 fake messages may get through, about 1 a year
if someone is really hammering away.).

You can't use these without also exchanging full data packets to sync the time,
although there is quite a bit of misalignment tolerance.  

In fact, because of the rolling code, there is likely
no easy way to even detect that an RT packet is being sent to you, 
unless you have both the key and synchronized clocks.


Because of the optimization for very low overhead, it is theoretically possible to mis-decode these packets even with perfect signal strength, if the clocks are not aligned, although the rolling code makes this highly unlikely.

Note that as a side effect, sending may overrwite whatever is currently in radio.DATA, because we listen for new messages while waiting to send.


### radio.decodeSG1RT()
Attempt to decode the packet as an SG1 low overhead realtime message. Returns false
and leaves data unchanged if it is not a low overhead message for this channel.

This happens very fast, so you can try decoding a packet as both(And you usually have to, because RT packets don't work without perioding full packets to stay in
sync, unless you disable replay attack protection.)

Note that as a side effect, sending may overrwite whatever is currently in radio.DATA, because we listen for new messages while waiting to send.

Copy any data you want to keep before sending anything.



### int32_t radio.getFEI()
Return the radio's measured frequency offset in Hz. If you have a particular two radios that don't seem to get along, you cat use this.
Note that automatic frequency error correction is enabled, so it should not be a big issue.

### radio.DATA, DATALEN, and RSSI
The data, length, and RSSI of the recieved packet, if any. If you called decodeSG1, it will be the decoded payload, otherwise it is the totally raw message.


### radio.send(uint8_t * data, uint8_t len)
Send a raw message. No additional framing at all gets added here. Length is the length of the data, and does not include the buffer itself.

Note that as a side effect, sending may overrwite whatever is currently in radio.DATA, because we listen for new messages while waiting to send.


### radio.sendSG1(uint8_t * data, uint8_t len)
Send an SG1 message, giving you (unproven)encryption, TX power control, and automat FEC as needed. Works exactly like send, you just
have to decode first. Adds 25 bytes to every message, and doubles payload size if the signal is very weak(Due to the error correction used to increase range).

Note that up to 2 extra padding bytes may be added if the system decides Golay encoding is needed.

Note that as a side effect, sending may overrwite whatever is currently in radio.DATA, because we listen for new messages while waiting to send.


## NOTE: User level Request/Reply has been removed!


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
other nodes to wake up briefly.  decodeSG1 will automatically detect and reply to beacon
frames with another beacon.

Note that beacons are not secure, and wake beacons are only semi-secure.  If someone sends
random beacons constantly, you can expect false positives every ten minutes, which could even
happen accidentally, and as such they can only detect the absence of a device, but not securely confim it's presence.

Wake beacons are harder to forge as devices will typically only listen for brief intervals,
preventing battery attacks.


Beacons on the current channel are recieved as if they are 0 byte packets, and they are also
used to calibrate RSSI. They do not carry any timing information, and due to the rolling code, they cannot be identified if the sender and reciever's clocks are not synchronized to within 16 seconds. 

It is suggested that devices send one periodically if they have no data to send.


### radio.xorshift32()
Simple non-secure 32 bit random number generator used for internal backoff timings. Reseeded automatically, and fast, but not secure.

### radio.urandom(uint8_t * target, uint8_t len)

Generate N bytes of theoretically secure-ish entropy, provided the entropy pool
has sufficient entropy. Small amounts of entropy are added every call, from timing
information and RSSI.

### radio.getEntropy(strength=128)

REMOVED you should never need to call this, the pool is seeded when initializing the radio, and entropy is continually added from packet arrival timings.

### radio.setTime(int64_t uinxMicros,[trust])

Set the time, and mark the time as locally
set and therefore trusted. You must set the clock for things to work!

You can pass the time 0 to set the time to a random timestamp if it is not already set.
Nodes will figure out for themselves how to connect if all clocks are random, but unset
clocks do not work.  

*Note that non-repeating timestamps are very important to making this
secure-ish. Make sure the pool has entropy before calling this!!!!!!!*

Passing 0 may take some time(several seconds), as without a unique counter we need to gather strong entropy.


Otherwise, it should be a UNIX timestamp in microseconds.

If trust is supplied, you should explicitly indicate how trusted the time source is:

```
//XOR all three of these flags for the default trust level for a locally set
//usable clock.
#define TIMETRUST_CLAIM_TRUST 2
#define TIMETRUST_SECURE 4
//At least challenge response or something
//equally trustworthy is needed to recieve packets.
#define TIMETRUST_CHALLENGERESPONSE 8

//ONLY set this if you are confident the time you are serving has 10-50ms accuracy
//or better.
#define TIMETRUST_ACCURATE 1

//Set this flag if you have a local source of real time and don't want the system ////clock to be tampered with by any other node
#define TIMETRUST_LOCAL 128
```


### static radio.unixMicros(adj=0)
Current system timestamp, either a positive real time, or negative random network time not referenced to an epoch. Set adj to something nonzero to add or remove time from it.  

This timestamp can change at any time, including going backwards, as it is synced with SG1.



## Structured Message API

SG1 defines an application layer and provides tools to encode and decode.  Structured messages are limited to 12 bytes each and contain
a set of records, each of which has a 2 byte header and may be 1,2, 4, or 8 bytes long.

Structured messages also have a six bit "channel" used to identify different readings of the same type, like multiple analog inputs, or to identify subcommands.

All message types under 192 are reserved.




### static radio.newStructuredMessage()
Resets the structured message transmit buffer. The structured message TX and RX buffers are separate from radio.DATA, it is safe to send and recieve messages
without fear of affecting the structured buffers.

### uint8_t * static radio.getStructuredRecord()

If there are any records remaining in the current recieved structured message, return one and advance the pointer, otherwise,
return null.  The current structured message is stored in a separate buffer and only updated when recieveing structured messages via decodeSG1().

Therefore, we can safely transmit things in the middle of processing structured data, out buffer will not be overwritten like DATA can be.

Certain commands may have built-in actions.   After you recieve all structured messages, you should flushStructuredMessage to flush and responses generated by those actions.

### static uint8_t radio.getMessageChannel(msg)
Given a pointer to a structured message, return it's channel number

### static uint8_t * radio.getMessageData(msg)
Given a pointer to a structured message, return it's data field.

### static uint8_t radio.getMessageLen(msg)
Given a pointer to a structured message, return it's length.

### static uint8_t radio.getMessageType(msg)
Given a pointer to a structured message, return it's type.

### static float radio.getMessageFloat(msg)
Given a pointer to a structured message, if the message if 4 bytes, interpret it as a float


###  radio.writeStructuredRecord(uint8_t type, void * data, uint8_t len, uint8_t channel=0)

Write a new structured record to the TX buffer.  If it would overflow, the current contents are sent as a full SG1 packet.


### radio.flushStructuredMessage()

Flush TX buffer.

###  radio.writeTroubleCode(uint32_t code, uint8_t flags=TROUBLE_WARNING)
Set the current most recent trouble code, add a MESSAGE_TROUBLE, and immediately flush.

### uint32_t radio.troubleCode
The most recent trouble code.  Use TROUBLE_CODE_MASK to get the actual code without the flags.






### radio.useConfigData(eepromaddr, size)

Enable configuration data.  This is an area that can be read or written by remote devices via structured records.  The data has no magic number or checksum, you do that yourself.

It must be a multiple of 8 bytes.  This function loads whatever is in the eeprom at that address.

Note that config data is only designed to work with two devices per SG1 channel key, as the APIs do not distinguish devices.

The canonical representation of confiig data is to be space-separated groups of 16 hex chars, 4 per line.

### radio.saveConfigData()

Save the active config data as the default in EEPROM, to the configured address. This can also be done via a remote command.  To avoid EEPROM wear this command always checks to see if a write is actually
needed.


### radio.configData

Just a simple pointer to config data.   It may be written locally and requested by a remote device, or the other way around.
