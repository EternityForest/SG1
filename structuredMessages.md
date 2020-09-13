

## Intro

Structured messages are sent with a special type header, and are a stream of records.  Eacch record has a 2 byte header and 1,2,4, or 8 data bytes.

The header contains an 8 bit type, a 2 bit length, and a 6 bit "channel number" which may be used for various purposes depending on the message.

All messages between 128 and 192 are determined by the device's profile.  Messages 192 and up are meant for application specific use.

The rest are standardized here.

## Record Types

### 0: GetInfo

Causes a device to return any info it has about itself, including the profile.

### 1: Profile
Must be a 32 bit number. All messages above 32 have definitions depending on profile. The first 2**24 are reserved.  Devices are not required to implement this, you just assume
they are profile 0 and have no profile specific features.


### 6: Config Data Get
Requests that the device declare it's entire config memory area

### 7: Config Data Write

Tell a device to write the 8 byte payload to the config area memory page indicated by channel. Should the write go beyond the end, just discard those bits.

Must never be saved immediately to nonvolatile memory until actually requested.

### 8: Config Data Save Control
If channel 0, request that devices save config data to nonvolatile memory.  If channel is 1, indicates that hte sender has just saved to nonvolatile memory.

### 9: Config Data Declaration
The data is always an 8-byte "page" of config data, which is the current config data of the device.  A device may have up to 32 such pages.

The channel indicates the page. Channels 32-64 are exactly the same as 0-31, except 0-31 must be sent sequentially starting at 0,
and the reciever should note that a missing packet happened if this is not the case.




### 10: POSIX TZ String

Listening devices must have at least 48 bytes of buffer.  Messages are an 8 byte sequence, written into the block number defined by the channel.
If the string contains no zero bytes, assume there is more to follow, else, the new buffer takes effect immediately.

Devices must use the channel number to ensure that all sectors were written in order and no packets were dropped.

### 10: Time Info Request
Requests that any master or gateway devices that may be listening send any data they may have regarding time and time zones, as defined in other messages.

### 11: Time Info
Message should be 8 bytes, and contain:

16 bit offset, to be added to UTC time to get TAI time, or 0 if unknown
16 bit sunrise time, in minutes past midnight
16 bit sunset time, in minutes past midnight
8 bit "Light" time, in 6 minute intervals past midnight
8 bit "Dark" time, in 6 minute intervals past midnight