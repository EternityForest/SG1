## SG1 Protocol

SG1 is a semi-secure protocol meant for low bandwidth applications. It passes
blobs of bytes over encrypted channels, optionally using forward error correction to
resist noise.


## Packet Structure


### Header(Always present)

This header is always sent as 6 bytes golay encoded. Golay encoding is always done in 6 byte blocks with 3 input bytes.

#### Length(1 byte)
The length byte. This INCLUDES itself, but is *not* actually sent, it is implied by the actual length of the packet and added before decoding.

Note that most RF modules use a length byte that does *not* include itself, but this is not a relevant detail unless you are decoding with SDR.

#### Flags(1 byte)
Bits:
##### 0-1: FEC level(00=None, 01=GOLAY_ALL)
If 01, the entire rest of the packet must be golay encoded, padding the payload to 3 bytes as needed.

##### 2: TIME_TRUST
True if the sender's time has been set from a trusted source
##### 3: TIME_ACCURATE
True if time is "accurate" by whatever definition the channel uses


##### 4-7: PACKET_TYPE
Reply messages have bit 7 set.

//No reply is asked for
#define HEADER_TYPE_UNRELIABLE    0b0010000
//Reply requested
#define HEADER_TYPE_RELIABLE      0b0100000
//This is a reply
#define HEADER_TYPE_REPLY         0b1000000
//This is a "special" type with reserved system meaning
#define HEADER_TYPE_REPLY_SPECIAL 0b1010000


#### Status(1 byte)
0-3: TX RSSI before antenna gain(4db increments starting at -24db)

### Channel Hint[20 bits ]

This sequence is an approximate "hint" as to what this message is for.
It is used so we don't have to try decrypting against every possible key.

It can be one of:

* The Fixed Hint Sequence(The hash of the channel key)
* The Private Hint Sequence(Which changes every 2**26 microseconds)
* The Private Wake Sequence.

Should it be the wake sequence, it means they are requesting all listening devices on the channel keep listening.

### Hint Sequence Flags[4 bits]:
Reserved, must be all zeros.

### IV(Not present in Beacon messages)
This is the device's Node ID, followed by 7 bytes of the current time, in 256us intervals since the UNIX epoch.


### Calculating the private hint and wake sequences:

One must first divide the system time by 2**26, then use that 8 byte signed number as the IV, with the channel key as the key, to encrypt 6 bytes of zeros.

After masking off the top 4 high order bits in each, the first 3 are the private hint, the second 3 are the private wake sequence.



### Payload(Not present in Beacon messages)
ChaCha20 encrypted using the channel key. The IV is the IV of the message.

The associated data is the 3 bytes of header after FEC decoding, followed by the IV of the message we are responding to, if this is a reply or special reply.



### MAC(Not present in Beacon messages)
This is 8 bytes long, and it is the ChaChaPoly AEAD mac

## Special packets

If a SPECIAL_REPLY packet is empty, it exists entirely to set the clock
on the other device.

In generaal, special packets should not be passed to user code.

The first byte of the special packet is always a packet type, if present.

## Beaconing

We must periodically send a short beacon message. For this the hint must be the private hint sequence, not the fixed hint, as fixed hints can have persistant collisions and there is no MAC to resolve them.

Note that this is not fully authenticated, however it is slightly secure as they look like random data. However, one can falsify the presence of another device
on the channel just by repeating beacons.

If preventing this is important, you wil need to send full authenticated packets.


We must continue listening for at least 45 milliseconds, to give other nodes an opportunity to wake us up with a beacon frame using the Private Wake Sequence.

This is secure, as we only get one try per beacon, and it is completely unpredictable wihtout the key, so falsely sending wake messages is very hard.

Hub devices should send their beacon within 1ms of recieving a beacon from the device. If the reciever is always-on, you can beacon whenever you want.


## Use without a clock

For simple things like RC cars, one device can set it's time to a random negative value(These are reserved for non-wallclock times), and may set it's time trusted and
accurate flags, as it is effectively the master.

This is not really that secure, over a few hundred reboots it's possible that
the clock will at some point have a value that overlaps with some other time.

However this is rather unlikely with things that don't get rebooted a lot.


## Auto TX power

We must use the RSSI and TX power in the header to estimate the path loss to the device. From there we must send a message such that the reciever's RSSI is around -90dbm. Even beacon frames contain this message.


## Packet acceptance

We reject anything more than ten seconds old, or older than the newest packet we have seen on the channel, or that does not decode properly, or more than ten seconds
newer than the current time.

### Disabling replay protection
For unsecured public channels that are only used briefly, disabling all time-based validation is an acceptable mode of operation.

In this case, all automatic setting of the system clock must also be disabled, and packets recieved must not be counted as "the newest packet recieved on the channel" for the purposes of rejecting duplicates, otherwise, this node could move the counter backwards and cause trouble when switching replay protection back on.



## Initial Time Sync

High powered always-on devices should always send a reply to any message with a time that is very inaccurate.

This is the least secure part of the whole setup, because we are temporarily running
in random-clock mode till the master gives us a timestamp.

However, there is still 2**48 possible time values, reuse is unlikely-ish.


## Pairing

A device may listen for pairing messages from other devices.  Pairing is done by jumpting to channel 3 on the GFSK38K profile.

The device must then listen for pairing requests, which are special packets with the type 16.

In response, it then sends a message with type 17, containing the Curve25519 key 
from the first half of ECDH key agreement, followed by a 16 byte UUID representing the type of device.

The other device then sends a message with type 18, again containing the shared secret, followed by a connection parameters string that is encrypted with the shared secret on key(using chachapoly), and signed with an 8 byte MAC.

The connection parameters are as follows:
32 byte new channel key,
8 bit RF profile number(0=1200bps, 6=250kbps)
16 bit new channel number

Upon recieving this, the device must immediately set it's connection parameters to the new settings and jump to the new channel.

Because pairing operates on a public channel where clocks may not be synced,
all pairing must be performed with replay attack protection disabled.


## Device info packets

These are special packets that provide information about a device. The first 
16 bytes must be a UUID representing the device, like a serial number.

The next 16 must be a UUID reprenting the model number.

The following 16 bust represent the device class of the device.

The remainder may be up to 24 UTF8 characters with no null terminator, representing
a friendly name for the device.

These packets should be rarely, if ever sent at any time other than initial setup.




## Connection Parameters Data Format

The canonical representation of the data needed to establish a connection is
the 32 byte channel key, followed by the one byte RF profile, followed by the 
2 byte channel number.


## RF Profiles

Power limit is 8dbm unless specified for narrowband profiles. 
These profiles are meant to comply with FCC 15.247,
which requires a -6db bandwidth of 500Khz to use higher transmission powers.

They should also be usable under Europe's ETSI EN300.220, allowing
863 to 868MHz operation, however they will not be usable for audio of video
due to the ETSI 300KHz limit when used for that purpose.

I am *not* a lawyer! Do your own research, these may not be usable everywhere.

### 1: GFSK600
600 baud, 6.25KHz deviation, 25Khz channel spacing
Power Limit: -4dbm

### 2: GFSK1200
1200 baud, 8KHz deviation, 25Khz channel spacing
Power limit: -4db

### 3: GFSK4800
4800 baud, 177KHz deviation, 600Khz channel spacing

### 4: GFSK10K
10kbaud, 177khz deviation, 600Khz spacing

### 5: GFSK38K
38400 baud, 177khz deviation, 600khz spacing

### 6: GFSK100K:
100kbaud, 177khz deviation, 600khz spacing.

### 7: GFSK250K
250Kbaud, 177khz deviation, 750khz spacing.
Do not use on 433MHz.


## Channel Numbers

To convert a ch number to a frequency, first compute the number of channels that fit in the regional frequency band. Then subtract the bottom channel.

The frequency is then center of the Nth channel, modulo the number of channels should the user select one that is too high.

This always reserves some bandwidth at the bottom of the band for lower bandwidth applications.



Channels above 1000 are actually hop patterns. for FHSS.

## FHSS 

FHSS is currently specified by the reference implementation. Frequencies always hop
every 2**16 microseconds. An offset is used so that different hop patterns do not
change over at the same time.

Only the middle half of any hop slot should be used, for maximum tolerance of misalignment.


## Channel Defaults:

### Channel 3
This should be used for any kind of very low data rate and semi public info,
like time sync services, initial setup, etc. Avoid  using for other things.


## Gateway protocol

### NanoFrame packet framing for serial links

Every packet begins with a start byte that is a 42.
This is followed by a length byte, which only includes the payload.
After this is the payload, and then a single stop byte, a 43.

The first byte of any payload is always the command byte.

This protocol has been designed for extremely low resource usage over
very short serial links with almost nonexistant error rates.

## Other Special Packets
The first byte of a special packet always indicates the subtype.

0, or empty packets are nulls that are only used as keepalives and time syncs.