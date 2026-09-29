/* ** B
? There is something I don't 100% get and it is writing the uint8_t
Thing because, I know its for bridging Jetson and this board,
but still unsure as to why I am getting a suggestion on doing that.

* HOW THIS WORKS


* Include section
First using prgama once we say that the file is included once per build
And we also have to get stdint.h for the uint8_t defintion types.

* General Code
So first we have to build an enumerable list (A list that is numerical and can't
be changed)
And it goes from 0- the amount of information we are sending / recieving over
ethernet
Which is basically everything.

That being said, I learned that there is no analog to digital converter stuff on
the board, which I believe I previously thought about
And If I am wrong on that I can always just add it into the next highest number.

The IMU board will be wired directly to the Jetson Nano so I don't have to worry
about that.
Same thing for the Time of FLight board that I believe was in the works.

Also very important I feel as if a new CAN .hpp and proto will pop up into this,
but as of right now I am unsure about said things.

* PACKETS

For the first three those are just placeholders mainly to see if things flash
properly over gigabit
Also because I am unsure how we want to handle these things, I know bvro was
previously working on CAN protocols, so I'll just be aware of that

? Note for Power Board

Looking into it, this is purely an analog board, no MCU or "brain" of any kind.
So if we did want to check how accurate voltages are, then maybe we would have
to get some sort of interupter for that.
One big thing is that to get these accurate readings, prolly have to figure some
interperter out yeah.
Also, if the temp is too high or something along that nature, maybe an ERROR
flag would light up.
I suppose that is mainly a problem for Jetson Nano People but important to write
down.
As since this is at gigabit Speed, I don't think we havce to worry about pushing
something to the front of the sending zone.

Honestly unsure on that lowkey. The main one I know is the Footsensor atp.

TODO: I should really look into where the software team was left off, and have
written down shared with Felicity just where everything is.
I can also bring this up at the meeting.

* Ethernet Payload
Basically everything gets put into a union (Allowing for storing different value
types) yet only one can be stored at any given time
Basically the header will run and get all that info, and attached will be what
payload we want to send. And it'll run through each.
Again we don't really have to worry about speed assuming gigabit link works.
If not then we go back down to 100 megabits per seocnd.

* Definitions
Gotten from the straight Datasheet

*/

// * Include Section
#pragma once
#include <stdint.h>

// * General Code
// Unsure what typedef means lowkey
typedef enum {
  MESSAGE_CAN = 0,
  MESSAGE_SPI = 1,
  MESSAGE_USBC = 2,
  MESSAGE_PB = 3,
  MESSAGE_FS = 4
} gigabit_msg_type_e; // Honestly unsure what the _e is added there for.
//? MENTOR NOTE 9/7/26 - _e is just a naming convention (this is an enum, not a
// struct/typedef of
// something else) - has zero effect on the compiled code. Renumbering these
// right now is free since nobody outside this repo has code against a specific
// value yet. The day someone on the Jetson side writes `if (type == 3) ...`,
// these numbers become the wire format and freeze.

typedef struct {
  uint32_t sequence; // Records the sequnced order by packet. It also detects
                     // gaps recieved and not
                     //  There is also a thought of considering the sending and
                     //  attracting part of the code, where we do those two
                     //  things at the same time essentially.
  uint32_t timestamp_ms;
  gigabit_msg_type_e
      type; // Based off of the previously decalred enums, checks which one of
            // those is live and records it as our type of information.
} gigabit_header_t;

// * PACKETS

// * CAN
typedef struct {
  uint8_t length;
  uint8_t data[64];
} can_payload_t;
//? MENTOR NOTE 9/7/26 - good call skipping can_message.hpp's union bug by
// placeholdering here.
// But a real CAN frame is {arbitration, remote, ext, dlc, data[64]} (see
// OLD-GIGE/can_message.hpp can_frame_t) - length+data alone doesn't say WHICH
// CAN id/motor this is about. Does the Jetson need that to route, or does
// routing happen some other way? Worth deciding before this stops being "just a
// placeholder."

// * SPI
typedef struct { // This is just a placeholder because I am honestly unsure
  uint8_t length;
  uint8_t data[64];
} spi_payload_t;

// * USBC
typedef struct {
  uint8_t length;
  uint8_t data[64];
} usbc_payload_t;

// * Power Board
typedef struct {
  uint16_t voltage;
  uint16_t current;
  uint16_t temp;
  uint8_t reserved;
} PB_payload_t;

typedef struct {
  uint16_t force_raw[4]; // Braking the four voltage signal amounts for the four
                         // different legs.
  uint8_t contact_flags; // If we made contact, and if we are touching ground, 0
                         // = not touching
} FS_payload_t;

// * Ethernet Payload

typedef struct {
  gigabit_header_t header; // We get all information from the header we designed
                           // in the first couple lines.
  union packet {
    spi_payload_t SPI;
    usbc_payload_t USBC;
    can_payload_t CAN;
    PB_payload_t PB;
    FS_payload_t FS;
  } eth_payload; // simple name
} global_eth_packet_t;

// * Definitions

static_assert(
    sizeof(global_eth_packet_t) <= 1472,
    "global_eth_packet_t exceeds one UDP payload without IP fragmentation");

// Mechanism (compile-time vs runtime check, why sizeof()
// With the bottom being the error name flag I believe.
