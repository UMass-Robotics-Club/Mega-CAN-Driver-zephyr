/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for robstride.c. Frames marked "manual" are copied from worked
 * examples in the ROBSTRIDE 02 user manual, so they check the encoding against
 * Robstride's document rather than against our own reading of it.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "robstride.h"

static int failures;

#define CHECK(cond)                                                                                \
	do {                                                                                       \
		if (!(cond)) {                                                                     \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                     \
			failures++;                                                                \
		}                                                                                  \
	} while (0)

#define CHECK_NEAR(a, b, tol) CHECK(fabsf((a) - (b)) <= (tol))

static void check_frame(const struct rs_frame *f, uint32_t id, const uint8_t data[8],
			const char *what)
{
	if (f->id != id || f->len != 8 || memcmp(f->data, data, 8) != 0) {
		printf("FAIL %s: got id 0x%08x data", what, (unsigned)f->id);
		for (int i = 0; i < 8; i++) {
			printf(" %02x", f->data[i]);
		}
		printf(", want id 0x%08x\n", (unsigned)id);
		failures++;
	}
}

/* manual 3.3.x: type 18 write, host 0xFD, motor 1, index 0x7005 = 1 */
static void test_param_write_manual(void)
{
	struct rs_frame f;
	const uint8_t want[8] = {0x05, 0x70, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};

	rs_build_param_write_u32(&f, 0xFD, 0x01, 0x7005, 1);
	check_frame(&f, 0x1200FD01, want, "param write (manual example)");
}

/* manual 4.1.15: read loc_kp (0x701E) from motor 0x7F, reply = 30.0f */
static void test_param_read_manual(void)
{
	struct rs_frame f;
	const uint8_t want[8] = {0x1E, 0x70, 0, 0, 0, 0, 0, 0};
	const struct rs_frame reply = {
		.id = 0x11007FFD,
		.len = 8,
		.data = {0x1E, 0x70, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x41},
	};
	uint8_t motor;
	uint16_t index;
	uint32_t raw;
	float value;

	rs_build_param_read(&f, 0xFD, 0x7F, 0x701E);
	check_frame(&f, 0x1100FD7F, want, "param read (manual example)");

	CHECK(rs_parse_param_reply(&reply, &motor, &index, &raw) == 0);
	memcpy(&value, &raw, sizeof(value));
	CHECK(motor == 0x7F);
	CHECK(index == 0x701E);
	CHECK(value == 30.0f);
}

static void test_simple_frames(void)
{
	struct rs_frame f;
	const uint8_t zero[8] = {0};
	const uint8_t one[8] = {1, 0, 0, 0, 0, 0, 0, 0};

	rs_build_enable(&f, RS_DEFAULT_HOST_ID, RS_DEFAULT_MOTOR_ID);
	check_frame(&f, 0x0300FD7F, zero, "enable");

	rs_build_stop(&f, RS_DEFAULT_HOST_ID, RS_DEFAULT_MOTOR_ID, false);
	check_frame(&f, 0x0400FD7F, zero, "stop");

	rs_build_stop(&f, RS_DEFAULT_HOST_ID, RS_DEFAULT_MOTOR_ID, true);
	check_frame(&f, 0x0400FD7F, one, "stop + clear faults");

	rs_build_set_zero(&f, RS_DEFAULT_HOST_ID, RS_DEFAULT_MOTOR_ID);
	check_frame(&f, 0x0600FD7F, one, "set zero");

	rs_build_get_id(&f, RS_DEFAULT_HOST_ID, RS_DEFAULT_MOTOR_ID);
	check_frame(&f, 0x0000FD7F, zero, "get id");

	/* CAN_TIMEOUT = 100 ms = 2000 (little-endian in bytes 4..7) */
	const uint8_t tmo[8] = {0x0C, 0x20, 0, 0, 0xD0, 0x07, 0, 0};

	rs_build_param_write_u32(&f, RS_DEFAULT_HOST_ID, RS_DEFAULT_MOTOR_ID, RS_PARAM_CAN_TIMEOUT,
				 100 * RS_CAN_TIMEOUT_PER_MS);
	check_frame(&f, 0x1200FD7F, tmo, "CAN_TIMEOUT write");

	CHECK(rs_frame_type(0x1200FD7F) == RS_TYPE_PARAM_WRITE);
	CHECK(rs_frame_type(0x0280FD7F) == RS_TYPE_FEEDBACK);
}

static void test_control(void)
{
	struct rs_frame f;
	/* all-zero command: p, v, t at mid-scale 0x8000; kp, kd at 0 */
	const struct rs_command hold = {0};
	const uint8_t want[8] = {0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00};

	rs_build_control(&f, 0x7F, &hold);
	/* type 1, torque 0x8000 in bits 23..8, motor 0x7F */
	check_frame(&f, 0x0180007F, want, "control zero");

	/* full scale and clamping */
	const struct rs_command big = {.p = 100.0f, .v = -100.0f, .kp = 1e6f, .kd = -1.0f,
				       .t_ff = 17.0f};
	const uint8_t want_big[8] = {0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00};

	rs_build_control(&f, 0x01, &big);
	check_frame(&f, 0x01FFFF01, want_big, "control clamped");

	const struct rs_command bad = {.p = NAN, .v = 0, .kp = 0, .kd = 0, .t_ff = 0};

	rs_build_control(&f, 0x01, &bad);
	CHECK(f.data[0] == 0 && f.data[1] == 0); /* NaN clamps low, never garbage */
}

static void test_scaling_roundtrip(void)
{
	const float step = 2.0f * RS_P_MAX / 65535.0f;

	for (float x = -RS_P_MAX; x <= RS_P_MAX; x += 0.37f) {
		float back = rs_u16_to_float(rs_float_to_u16(x, -RS_P_MAX, RS_P_MAX), -RS_P_MAX,
					     RS_P_MAX);
		CHECK_NEAR(back, x, step); /* within one step of the 16-bit grid */
	}
	CHECK(rs_float_to_u16(-RS_P_MAX, -RS_P_MAX, RS_P_MAX) == 0);
	CHECK(rs_float_to_u16(RS_P_MAX, -RS_P_MAX, RS_P_MAX) == 65535);
	CHECK(rs_float_to_u16(250.0f, 0.0f, RS_KP_MAX) == 32768);
}

static void test_feedback(void)
{
	struct rs_feedback fb;
	/* type 2 | mode run (2) << 22 | over-temp (bit 18) | motor 0x7F << 8 | host 0xFD */
	const struct rs_frame f = {
		.id = (2U << 24) | (2U << 22) | (1U << 18) | (0x7FU << 8) | 0xFDU,
		.len = 8,
		/* pos 0x8000, vel 0xFFFF, torque 0x0000, temp 27.5 C */
		.data = {0x80, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0x13},
	};
	const struct rs_frame cold = {
		.id = (2U << 24) | (0x01U << 8) | 0xFDU,
		.len = 8,
		.data = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0xFF, 0x9C}, /* -10.0 C */
	};
	const struct rs_frame not_fb = {.id = 0x0300FD7F, .len = 8};

	CHECK(rs_parse_feedback(&f, &fb) == 0);
	CHECK(fb.motor_id == 0x7F);
	CHECK(fb.host_id == 0xFD);
	CHECK(fb.mode == RS_MODE_RUN);
	CHECK(fb.faults == RS_FAULT_OVERTEMP);
	CHECK_NEAR(fb.p, 0.0f, 1e-3f);
	CHECK_NEAR(fb.v, RS_V_MAX, 1e-3f);
	CHECK_NEAR(fb.t, -RS_T_MAX, 1e-3f);
	CHECK_NEAR(fb.temp_c, 27.5f, 1e-4f);

	CHECK(rs_parse_feedback(&cold, &fb) == 0);
	CHECK(fb.faults == 0 && fb.mode == RS_MODE_RESET);
	CHECK_NEAR(fb.temp_c, -10.0f, 1e-4f);

	CHECK(rs_parse_feedback(&not_fb, &fb) == -1);
}

int main(void)
{
	test_param_write_manual();
	test_param_read_manual();
	test_simple_frames();
	test_control();
	test_scaling_roundtrip();
	test_feedback();

	if (failures) {
		printf("%d check(s) FAILED\n", failures);
		return 1;
	}
	printf("robstride: all tests passed\n");
	return 0;
}
