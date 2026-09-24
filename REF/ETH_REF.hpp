/*
hpp is header files in C++
So not .h which is c files
*/
#pragma once // Only includes the file once per build

#include "can_message.hpp" // Calling in CAN to ruse CAN/SPI structs
#include <stdint.h> // allows for the uint8_t types, ixed width spans

// glogbal tx_packet would be from here and or go through here?
// enum = numbers that are readable through names

// using namespace std; Do I need this?
// typdef = Alternative for or alias name for existing data types
// Enum assigns each value from 0
typedef enum { // We want to have each messages recieved align to where they are
               // Lable for everything, saying we have a BLANK message and
               // sending it to where it needs to go
  MESSAGE_CAN = 0,
  MESSAGE_SPI = 1,
  MESSAGE_USB = 2,

  // MESSAGE_ADC = 3, // Analog to Digital Converter | Do we even have this on
  //                  // our board? No right? I don't recall ever making that,
  //                  // and I am not sure why I added it.
  //! COMMENTED OUT 8/21/26 - Confirmed there is no ADC on this board, so this
  //! message type has nothing to describe. Leaving it here in case that changes.

  // MESSAGE_IMU = 4,
  //! COMMENTED OUT 8/21/26 - The IMU board now wires DIRECTLY to the Jetson
  //! Nano, so IMU data never travels through this board. Nothing for us to
  //! relay, so this type is dead for now.
  //! Same story for the Time of Flight board, which is why it never got a type.

  MESSAGE_PB = 5, // Powerboard
  MESSAGE_FS = 6  // Foot Sesnor
} gigabit_msg_type_e;
// Is enumerate class method? Yes

// Payload Structure per 1 per message, with
// Everything is setup in frames, with or data chains and such, so how would we
// run that.

/*
Because we wan to record the last time that something was sent, alon wih anyhing
tha was importan
We can have a typedef struct {} that takes in the variable time of when it was
last ran, and its order in the 7 thing sequence
type as defined by the name, MESSAGE_CAN
*/

typedef struct {
  uint32_t sequence; // Records what order in the sequence, by packet; recieve
                     // detects gaps
  uint32_t timestamp_ms; // When this was genereted, and for latency
                         // measurements betweeen time sent and recieved
  gigabit_msg_type_e
      type; // Records what type of payload is live. So CAN, SPI, etc
} gigabit_header_t;

// /-----------------------
// | PAYLOAD Structs
// | This holds all he thins he poweoad would send so I would probably have to
// | talk to them about that, and same for all the boards. I guess righ tnow
// | just place holders
// |------------------------

// IMU
//! COMMENTED OUT 8/21/26 - IMU board goes DIRECTLY to the Jetson Nano now, so
//! this data path does not exist on our board anymore. Keeping the struct here
//! because the field layout is still correct if it ever routes back through us.
//! Worth noting a union is as big as its largest member, so a dead payload can
//! make EVERY packet bigger, not just the unused ones. In this case it was not
//! the largest one so cutting it did not change the packet size, but the
//! principle still holds for whatever gets added next.
/*
typedef struct { // previously uint, but of course these values acn be negative
                 // for deaccel, x pos, and everything else.
  // X Values
  int16_t x_accel;
  int16_t x_Lvelocity; // Linear
  int16_t x_Avelocity; // Angular
  int16_t x_pos; // If I recall correctly, a bit of information is a decent
                 // amount, so we would only really need crazy numbers if they
                 // are accompanied by strings or something its just long.

  // Y Values
  int16_t y_accel;
  int16_t y_Lvelocity; // Lienar
  int16_t y_Avelocity; // Refer to x, angular
  int16_t y_pos;

  // Z Axis
  int16_t z_accel;
  int16_t z_Lvelocity;
  int16_t z_Avelocity;
  int16_t z_pos;

} imu_payload_t;
*/

// POWER BOARD
typedef struct {
  uint16_t voltage; // How much voltage the board is putting off
  uint16_t current; // Maybe how much current it is sending.
  uint16_t temp;    // I don't know if the board has a temp sensor
  uint8_t reserved; // Just to make things even I believe. With even being
                    // spaced out between everything? even numebrs per thing
} PB_payload_t;

// Foot Sensor
typedef struct {
  //? I would assume the foot sensor works off of pressure, so how much force is
  // on something,
  //* Still live as of 8/21/26 - the magnetic foot sensor DOES route through
  //* this board, unlike the IMU and Time of Flight boards.
  uint16_t force_raw[4]; // The [value] is for breaking things down into x
                         // amounts, so four different legs.
  uint8_t contact_flags; // For when this  contacts, send a bit per foot: 1 =
                         // touching ground, 0 = not touching.
  // Can always add more
} FS_payload_t;

// Analog to digital converter
//! COMMENTED OUT 8/21/26 - Confirmed no ADC on this board, so this was me
//! adding a placeholder for hardware we never had. It was a 65 byte member.
//! Measured it though and cutting it did NOT shrink the packet, because
//! usbc_payload_t is also 65 bytes and is now the largest member. So the
//! union is still sized by usbc. Something to look at if I want it smaller.
/*
typedef struct { // Just like the USB-C stuff that you see below, this is just a
                 // placeholder for things, I can change it as our intell comes
                 // in.
  uint8_t length;
  uint8_t data[64];
} adc_payload_t;
*/

// USB
typedef struct { // for everything that I don't know whats going to happen with,
                 // a raw buffer as a placeholder is my method
  uint8_t length;
  uint8_t data[64];
} usbc_payload_t;

typedef struct {
  gigabit_header_t header; // This is the sequence and header calling, to know
                           // what is what and get timing

  union packet { // Unions these packets together so that I can send it as one
                 // whole packet type. I might have to think about some, inputs
                 // not just transmissions

    // imu_payload_t imu;   //! COMMENTED OUT 8/21/26 - IMU goes direct to Jetson
    FS_payload_t FS;
    PB_payload_t PB;
    // adc_payload_t adc;   //! COMMENTED OUT 8/21/26 - no ADC on this board
    can_tx_packet_t can; // From can message lives here
    spi_tx_packet_t spi; // again from can_message.
    usbc_payload_t usbc;
  } payload; // simple name

} global_eth_packet_t;

// Now I have to make the finalized conection thing.
/*
Teammate's reference

typedef struct {
    ptarget_e peripheral;

    union packet {
        can_tx_packet_t can;
        spi_tx_packet_t spi;
    };

} global_tx_packet_t;

*/

//! M7 and Jetson could disagreeon what fields go in what order
// Meaning that I have to talk with the Jetson nano programmer to either run
// with __attribute__((packed)) or write seralize/deserialize functions
// ** We are working in GMII not RGMII
//? NOTES PER 8/17/26 - Probably should
// There are 6 total interupts, 6, IRQ 202-207
// ETH_PRI_Q_0_IRQn - ETH_PRI_Q_5_IQRn
// I believe all of these are from pic32cz8110ca80208.h which is all in
// MPLab-src-pacls-PIC32CZ8110CA-component-eth.h
// There is already a sercom2 node in pic32cz_ca.dtsi , which recall .dtsi is a
// Node file, actual description of the chips ETH peripheral
/*
?  REFERENCE IN pic32cz_ca.dtsi

sercom2: sercom@45800000 {
    compatible = "microchip,sercom-g1";
    reg = <0x45800000 0x36>;
    interrupts = <69 0>, <70 0>, <71 0>, <72 0>;
    interrupt-names = "error", "rxbrk", "dre", "txc";
    clocks = <&mclkperiph CLOCK_MCHP_MCLKPERIPH_ID_SERCOM2>,
             <&gclkperiph CLOCK_MCHP_GCLKPERIPH_ID_SERCOM2_CORE>;
    clock-names = "mclk", "gclk";
    status = "disabled";
};

Base Ethernet Address: 0x45070000 memory mapped I/O, no RAM involved, writing
directly to the ETH peripherals Network Control Register. Again, base address
inside CPU address uses to reach MAC's control panel.

?  The six Interupts 202-207
Essentially Interupts, the CPU, stop what you're doing, something happened,
instead of constnalty doing a check of something through a while loop or RTOS
task, instead interupts check this without burning cycles. MAC raises an
interrupt frame when something arrives, a transmit comple,ts error occurs, and
CPU jumpts to the handler

Six interupts since six priority queues, each with its own line. I don't have to
do all six, but probably highest prority would be CAN motor controls, so using
the incoming CAN packets, via Gigabit would be sent to my teammate's code.

! How do interupts work without cycles?
Essnetially, if line is high, alert Its a silicon doing the watching, not code


? APB Clock (ID 65) and AXI Clock (ID 64)
Both different internal busses in the chips, ethernet sits on both because it
has two different jobs
- APB (Advanced Peripheral Bus) -- Slow and Nnarrow, for register access. Driver
writes config bti, it goes over APB. Low bandwidth fine when moving 4 bytes at a
time.
- AXI (Advanced eXtensible Interface) -- Fast and Wide, used for bulk data, when
MAC's DMA engine sends a recieved packet to RAM, it goes over AXI, path for
Gigabit..

? GCLK_ID_TX (54) "GIGE/Loopback clock"
Transmits clock, for gigabit GMII, at 125 MHz, its hat clocks data out toward
the PHU.
THIS NEEDS TO LINK PROPERLY, or else the data will be garabage, its also tied to
CTRLB_GBITCLKREQ bit, which requests gigabit clock, and this feeds the GCLK.

? GCLIK_ID_TSU (55)
TSU, Time Stamp Unit for IEEE 1588/PTP (Percision Time Protocol).
Hardware that stamps packets with sub-microsecond-accurate timing.
Allows many devices on a network that agree on shared nanosecond percision. Used
for auomation, audio/video sync, and motion contorl.
Which is pretty much what we need to ensu or Gigabit

So now just making a device tree thing
*/