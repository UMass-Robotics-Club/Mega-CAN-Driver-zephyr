/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Jetson <-> Mega-CAN UDP protocol v3: the C side of udp_protocol/protocol.py.
 *
 * protocol.py is the source of truth. Every struct here mirrors one of its
 * struct.Struct formats field for field ('<' = little-endian, no implicit
 * padding: explicit pad bytes only, 'x' in the Python format). The size and
 * offset asserts below repeat protocol.py's asserts, and app/tests/ packs
 * bytes in Python and decodes them here (and back) to prove it.
 *
 * Pure C, no Zephyr, so the PC tests can include it.
 */

#ifndef MEGA_CAN_PROTO_H_
#define MEGA_CAN_PROTO_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_MAGIC   "MCAN"
#define PROTO_VERSION 3
#define PROTO_PORT    5000

#define PROTO_TYPE_COMMAND 1
#define PROTO_TYPE_STATE   2

#define PROTO_NUM_MOTORS 4
#define PROTO_NUM_BUSES  6
#define PROTO_NUM_FEET   4
#define PROTO_NUM_TOF    4

/* Motor modes (command) */
#define PROTO_MODE_OFF      0
#define PROTO_MODE_RUN      1
#define PROTO_MODE_SET_ZERO 2

/* Motor status (state) */
#define PROTO_MOTOR_OFF      0
#define PROTO_MOTOR_RUNNING  1
#define PROTO_MOTOR_FAULT    2
#define PROTO_MOTOR_NO_REPLY 3

/* Sensor status (feet, ToF) */
#define PROTO_SENSOR_NO_DATA 0
#define PROTO_SENSOR_OK      1
#define PROTO_SENSOR_STALE   2
#define PROTO_SENSOR_FAULT   3

/* State flags */
#define PROTO_FLAG_WATCHDOG_TRIPPED (1U << 0)
#define PROTO_FLAG_BAD_PACKET       (1U << 1)

#define PROTO_PACKED __attribute__((packed))

/* HEADER = "<4sBBHII" */
struct PROTO_PACKED proto_header {
	char magic[4];
	uint8_t version;
	uint8_t type;
	uint16_t count; /* = PROTO_NUM_MOTORS */
	uint32_t seq;
	uint32_t time_us;
};

/* CMD_MOTOR = "<B3xfffff" */
struct PROTO_PACKED proto_cmd_motor {
	uint8_t mode;
	uint8_t pad[3];
	float p_des;
	float v_des;
	float kp;
	float kd;
	float tau_ff;
};

struct PROTO_PACKED proto_command {
	struct proto_header hdr;
	struct proto_cmd_motor motor[PROTO_NUM_MOTORS];
};

/* ST_MOTOR = "<BBBBffff" */
struct PROTO_PACKED proto_st_motor {
	uint8_t status;
	uint8_t bus;
	uint8_t can_id;
	uint8_t faults;
	float position;
	float velocity;
	float torque;
	float temperature_c;
};

/* ST_BUS = "<BBBx" */
struct PROTO_PACKED proto_st_bus {
	uint8_t state; /* 0 ok, 1 warning, 2 error-passive, 3 bus-off */
	uint8_t tx_err;
	uint8_t rx_err;
	uint8_t pad;
};

/* ST_FOOT = "<BxHfI" */
struct PROTO_PACKED proto_st_foot {
	uint8_t status;
	uint8_t pad;
	uint16_t age_ms;
	float force_n;
	uint32_t raw;
};

/* ST_TOF = "<BxHffff" */
struct PROTO_PACKED proto_st_tof {
	uint8_t status;
	uint8_t pad;
	uint16_t age_ms;
	float distance_m;
	float mag_ut[3];
};

/* ST_POWER = "<ffI" */
struct PROTO_PACKED proto_st_power {
	float bus_voltage_v;
	float bus_current_a;
	uint32_t pgood;
};

struct PROTO_PACKED proto_state {
	struct proto_header hdr;
	uint32_t last_cmd_seq; /* ST_EXTRA = "<II" */
	uint32_t flags;
	struct proto_st_motor motor[PROTO_NUM_MOTORS];
	struct proto_st_bus bus[PROTO_NUM_BUSES];
	struct proto_st_foot foot[PROTO_NUM_FEET];
	struct proto_st_tof tof[PROTO_NUM_TOF];
	struct proto_st_power power;
};

/* protocol.py: assert COMMAND_SIZE == 112, STATE_SIZE == 268 */
_Static_assert(sizeof(struct proto_header) == 16, "HEADER");
_Static_assert(sizeof(struct proto_cmd_motor) == 24, "CMD_MOTOR");
_Static_assert(sizeof(struct proto_command) == 112, "COMMAND_SIZE");
_Static_assert(sizeof(struct proto_st_motor) == 20, "ST_MOTOR");
_Static_assert(sizeof(struct proto_st_bus) == 4, "ST_BUS");
_Static_assert(sizeof(struct proto_st_foot) == 12, "ST_FOOT");
_Static_assert(sizeof(struct proto_st_tof) == 20, "ST_TOF");
_Static_assert(sizeof(struct proto_st_power) == 12, "ST_POWER");
_Static_assert(sizeof(struct proto_state) == 268, "STATE_SIZE");
/* block offsets from udp_protocol/README.md */
_Static_assert(offsetof(struct proto_state, motor) == 24, "motors @24");
_Static_assert(offsetof(struct proto_state, bus) == 104, "buses @104");
_Static_assert(offsetof(struct proto_state, foot) == 128, "feet @128");
_Static_assert(offsetof(struct proto_state, tof) == 176, "tof @176");
_Static_assert(offsetof(struct proto_state, power) == 256, "power @256");
_Static_assert(sizeof(float) == 4, "IEEE-754 single");
_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "protocol is little-endian");

#ifdef __cplusplus
}
#endif

#endif /* MEGA_CAN_PROTO_H_ */
