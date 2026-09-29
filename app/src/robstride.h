/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Robstride 02 "private protocol" frames (ROBSTRIDE 02 user manual, section 4).
 *
 * Pure C, no Zephyr: builds on a PC for the unit tests in ../tests/. The
 * firmware copies rs_frame into a Zephyr struct can_frame with the extended
 * ID flag set (CAN_FRAME_IDE) and no FD flags: classic CAN 2.0B, 1 Mbit/s.
 *
 * Every frame has a 29-bit extended ID in three parts, plus 8 data bytes:
 *
 *   bits 28..24   bits 23..8           bits 7..0
 *   [ type ]      [ data area 2 ]      [ target ID ]
 *
 * For most board->motor frames, data area 2 carries the host (board) CAN ID in
 * bits 15..8. Control frames (type 1) put the torque there instead.
 *
 * Byte order is NOT uniform (both from the manual):
 *   - control (1) and feedback (2) values: 16-bit, BIG-endian
 *   - parameter read/write (17/18): index and value LITTLE-endian
 */

#ifndef MEGA_CAN_ROBSTRIDE_H_
#define MEGA_CAN_ROBSTRIDE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Communication types (manual 4.1) */
enum rs_type {
	RS_TYPE_GET_ID = 0,
	RS_TYPE_CONTROL = 1,
	RS_TYPE_FEEDBACK = 2,
	RS_TYPE_ENABLE = 3,
	RS_TYPE_STOP = 4,
	RS_TYPE_SET_ZERO = 6,
	RS_TYPE_SET_CAN_ID = 7,
	RS_TYPE_PARAM_READ = 17,
	RS_TYPE_PARAM_WRITE = 18,
	RS_TYPE_FAULT = 21,
};

/* Control ranges (manual 4.1.2; P_MAX per 4.2.3 is 12.57 = 4*pi) */
#define RS_P_MAX  12.566370614f /* rad */
#define RS_V_MAX  44.0f         /* rad/s */
#define RS_KP_MAX 500.0f
#define RS_KD_MAX 5.0f
#define RS_T_MAX  17.0f         /* N*m */

/* Factory defaults seen in the manual's examples */
#define RS_DEFAULT_MOTOR_ID 0x7F
#define RS_DEFAULT_HOST_ID  0xFD

/* Parameters (manual 3.3 table) */
#define RS_PARAM_CAN_TIMEOUT 0x200C /* uint32, 20000 = 1 s (50 us units), 0 = off */
#define RS_CAN_TIMEOUT_PER_MS 20U

/* Feedback fault bits (type 2 ID bits 16..21, shifted down to bit 0).
 * Same order as FAULT_* in udp_protocol/protocol.py. */
#define BIT_U8(n)             ((uint8_t)(1U << (n)))
#define RS_FAULT_UNDERVOLTAGE BIT_U8(0)
#define RS_FAULT_OVERCURRENT  BIT_U8(1)
#define RS_FAULT_OVERTEMP     BIT_U8(2)
#define RS_FAULT_ENCODER      BIT_U8(3)
#define RS_FAULT_OVERLOAD     BIT_U8(4)
#define RS_FAULT_UNCALIBRATED BIT_U8(5)

/* Mode reported in feedback (type 2 ID bits 22..23) */
enum rs_mode {
	RS_MODE_RESET = 0,
	RS_MODE_CALIBRATION = 1,
	RS_MODE_RUN = 2,
};

/* One classic CAN frame: 29-bit extended ID + up to 8 bytes. */
struct rs_frame {
	uint32_t id;
	uint8_t len;
	uint8_t data[8];
};

/* Operation-control command: t = kp*(p - p_act) + kd*(v - v_act) + t_ff */
struct rs_command {
	float p;    /* target position, rad */
	float v;    /* target velocity, rad/s */
	float kp;
	float kd;
	float t_ff; /* feed-forward torque, N*m */
};

struct rs_feedback {
	uint8_t motor_id;
	uint8_t host_id;
	uint8_t faults; /* RS_FAULT_* */
	uint8_t mode;   /* enum rs_mode */
	float p;        /* rad */
	float v;        /* rad/s */
	float t;        /* N*m */
	float temp_c;   /* winding temperature, deg C */
};

/* Frame type (bits 28..24) of any Robstride ID. */
uint8_t rs_frame_type(uint32_t id);

/* Board -> motor. Values outside the ranges above are clamped. */
void rs_build_control(struct rs_frame *f, uint8_t motor_id, const struct rs_command *cmd);
void rs_build_enable(struct rs_frame *f, uint8_t host_id, uint8_t motor_id);
void rs_build_stop(struct rs_frame *f, uint8_t host_id, uint8_t motor_id, bool clear_faults);
void rs_build_set_zero(struct rs_frame *f, uint8_t host_id, uint8_t motor_id);
void rs_build_get_id(struct rs_frame *f, uint8_t host_id, uint8_t motor_id);
void rs_build_param_read(struct rs_frame *f, uint8_t host_id, uint8_t motor_id, uint16_t index);
void rs_build_param_write_u32(struct rs_frame *f, uint8_t host_id, uint8_t motor_id,
			      uint16_t index, uint32_t value);
void rs_build_param_write_float(struct rs_frame *f, uint8_t host_id, uint8_t motor_id,
				uint16_t index, float value);

/* Motor -> board. Return 0 on success, -1 if the frame isn't that type. */
int rs_parse_feedback(const struct rs_frame *f, struct rs_feedback *out);
int rs_parse_param_reply(const struct rs_frame *f, uint8_t *motor_id, uint16_t *index,
			 uint32_t *raw_value);

/* The 16-bit scaling the protocol uses, exposed for tests. */
uint16_t rs_float_to_u16(float x, float min, float max);
float rs_u16_to_float(uint16_t u, float min, float max);

#ifdef __cplusplus
}
#endif

#endif /* MEGA_CAN_ROBSTRIDE_H_ */
