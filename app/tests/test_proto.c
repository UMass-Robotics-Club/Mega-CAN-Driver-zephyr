/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Byte-level check of proto.h against udp_protocol/protocol.py, driven by
 * crosscheck_proto.py:
 *   test_proto encode        -> hex of a state packet with fixed test values
 *   test_proto decode <hex>  -> the fields of a command packet, one per line
 * All test values are exact in binary floating point, so compares are exact.
 */

#include <stdio.h>
#include <string.h>

#include "proto.h"

static void encode(void)
{
	struct proto_state s;
	const uint8_t *b = (const uint8_t *)&s;

	memset(&s, 0, sizeof(s));
	memcpy(s.hdr.magic, PROTO_MAGIC, 4);
	s.hdr.version = PROTO_VERSION;
	s.hdr.type = PROTO_TYPE_STATE;
	s.hdr.count = PROTO_NUM_MOTORS;
	s.hdr.seq = 0x01020304;
	s.hdr.time_us = 0xA0B0C0D0;
	s.last_cmd_seq = 77;
	s.flags = PROTO_FLAG_WATCHDOG_TRIPPED | PROTO_FLAG_BAD_PACKET;
	for (int i = 0; i < PROTO_NUM_MOTORS; i++) {
		s.motor[i].status = (uint8_t)(i % 4);
		s.motor[i].bus = (uint8_t)i;
		s.motor[i].can_id = (uint8_t)(0x10 + i);
		s.motor[i].faults = (uint8_t)(1U << i);
		s.motor[i].position = (float)i + 0.5f;
		s.motor[i].velocity = -((float)i + 0.25f);
		s.motor[i].torque = (float)i * 1.5f;
		s.motor[i].temperature_c = 20.0f + (float)i;
	}
	for (int i = 0; i < PROTO_NUM_BUSES; i++) {
		s.bus[i].state = (uint8_t)(i % 4);
		s.bus[i].tx_err = (uint8_t)(10 + i);
		s.bus[i].rx_err = (uint8_t)(20 + i);
	}
	for (int i = 0; i < PROTO_NUM_FEET; i++) {
		s.foot[i].status = PROTO_SENSOR_OK;
		s.foot[i].age_ms = (uint16_t)(100 + i);
		s.foot[i].force_n = (float)i * 10.0f;
		s.foot[i].raw = 1000U + (uint32_t)i;
	}
	for (int i = 0; i < PROTO_NUM_TOF; i++) {
		s.tof[i].status = PROTO_SENSOR_STALE;
		s.tof[i].age_ms = (uint16_t)(200 + i);
		s.tof[i].distance_m = (float)i * 0.25f;
		s.tof[i].mag_ut[0] = (float)i;
		s.tof[i].mag_ut[1] = -(float)i;
		s.tof[i].mag_ut[2] = (float)i * 2.0f;
	}
	s.power.bus_voltage_v = 24.5f;
	s.power.bus_current_a = 1.25f;
	s.power.pgood = 7;

	for (size_t i = 0; i < sizeof(s); i++) {
		printf("%02x", b[i]);
	}
	printf("\n");
}

static int decode(const char *hex)
{
	struct proto_command c;
	uint8_t *b = (uint8_t *)&c;

	if (strlen(hex) != 2 * sizeof(c)) {
		printf("bad length %zu\n", strlen(hex));
		return 1;
	}
	for (size_t i = 0; i < sizeof(c); i++) {
		unsigned int v;

		sscanf(&hex[2 * i], "%2x", &v);
		b[i] = (uint8_t)v;
	}
	printf("magic %.4s\nversion %u\ntype %u\ncount %u\nseq %u\ntime_us %u\n", c.hdr.magic,
	       c.hdr.version, c.hdr.type, c.hdr.count, c.hdr.seq, c.hdr.time_us);
	for (int i = 0; i < PROTO_NUM_MOTORS; i++) {
		printf("m%d %u %.9g %.9g %.9g %.9g %.9g\n", i, c.motor[i].mode,
		       (double)c.motor[i].p_des, (double)c.motor[i].v_des, (double)c.motor[i].kp,
		       (double)c.motor[i].kd, (double)c.motor[i].tau_ff);
	}
	return 0;
}

int main(int argc, char **argv)
{
	if (argc == 2 && strcmp(argv[1], "encode") == 0) {
		encode();
		return 0;
	}
	if (argc == 3 && strcmp(argv[1], "decode") == 0) {
		return decode(argv[2]);
	}
	fprintf(stderr, "usage: test_proto encode | decode <hex>\n");
	return 2;
}
