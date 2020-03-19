
## Packet Structure


### Header(Always present)

This header is always sent as 6 bytes golay encoded. Golay encoding is always done in 6 byte blocks with 3 input bytes.

#### Length(1 byte)
The length byte, also the raw RFM69 length byte

#### Flags(1 byte)
Bits:
##### 0-1: FEC level(00=None, 01=GOLAY_ALL)
If 01, the entire rest of the packet must be golay encoded, padding the payload to 3 bytes as needed.

##### 2: TIME_TRUST
True if the sender's time has been set from a trusted source
##### 3: TIME_ACCURATE
True if time is "accurate" by whatever definition the channel uses
##### 4: IS_REPLY
True if this is a reply to the last message sent on the channel

#### Status(1 byte)
0-3: TX RSSI before antenna gain(4db increments starting at -24db)

### Channel Hint[3 bytes]

This 4 byte sequence is an approximate "hint" as to what this message is for.
It is used so we don't have to try decrypting against every possible key.

It can be one of:

* The Fixed Hint Sequence(The hash of the channel key)
* The Private Hint Sequence(Which changes every 16s)
* The Private Wake Sequence.

Should it be the wake sequence, it means they are requesting all listening devices on the channel keep listening.


### IV(Not present in Beacon messages)
This is the device's Node ID, followed by 7 bytes of the current time, in 256us intervals since the UNIX epoch.




### Payload(Not present in Beacon messages)
ChaCha20 encrypted using the channel key. The IV is the IV of the message.

The associated data is the 3 bytes of header after FEC decoding, followed by the IV of the message we are responding to, if this is
a reply.



### MAC(Not present in Beacon messages)
This is 8 bytes long, and it is the ChaChaPoly AEAD mac


## Beaconing

We must periodically send a short beacon message. For this the hint must be the private hint sequence. Note that this is not authenticated, however it is slightly secure as they look like random data and can't be tracked easily.

We must continue listening for 3 milliseconds, to give other nodes an opportunity to wake us up with a beacon frame using the Private Wake Sequence.

This is secure, as we only get one try per beacon, and it is completely unpredictable wihtout the key, so falsely sending wake messages is very hard.

Hub devices should send their beacon within 1ms of recieving a beacon from the device. If the reciever is always-on, you can beacon whenever you want.



## Auto TX power

We must use the RSSI and TX power in the header to estimate the path loss to the device. From there we must send a message such that the reciever's RSSI is around -90dbm. Even beacon frames contain this message.


## Packet acceptance

We reject anything more than ten seconds old, or older than the newest packet we have seen on the channel, or that does not decode properly.

## Initial Time Sync

High powered devices should always send a reply to any message with a time that is more than 0.5s incorrect. 
