/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Robstride 02 private-protocol frames. See robstride.h for the frame layout.
 */

#include "robstride.h"

#include <string.h>

#define RS_ID_TYPE_SHIFT  24
#define RS_ID_AREA2_SHIFT 8
#define RS_ID_TYPE_MASK   0x1FU
#define RS_ID_AREA2_MASK  0xFFFFU
#define RS_ID_TARGET_MASK 0xFFU

static uint32_t rs_id(uint8_t type, uint16_t area2, uint8_t target)
{
	return ((uint32_t)(type & RS_ID_TYPE_MASK) << RS_ID_TYPE_SHIFT) |
	       ((uint32_t)area2 << RS_ID_AREA2_SHIFT) | target;
}

static void rs_frame_init(struct rs_frame *f, uint8_t type, uint16_t area2, uint8_t target)
{
	f->id = rs_id(type, area2, target);
	f->len = 8;
	memset(f->data, 0, sizeof(f->data));
}

static void put_be16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v >> 8);
	p[1] = (uint8_t)v;
}

static uint16_t get_be16(const uint8_t *p)
{
	return (uint16_t)((p[0] << 8) | p[1]);
}

static void put_le16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static void put_le32(uint8_t *p, uint32_t v)
{
	for (int i = 0; i < 4; i++) {
		p[i] = (uint8_t)(v >> (8 * i));
	}
}

static uint32_t get_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
	       ((uint32_t)p[3] << 24);
}

uint8_t rs_frame_type(uint32_t id)
{
	return (uint8_t)((id >> RS_ID_TYPE_SHIFT) & RS_ID_TYPE_MASK);
}

/*
 * The protocol maps [min, max] linearly onto 0..65535 (manual 4.1.2).
 * Rounded to the nearest step; out-of-range inputs (NaN included) clamp.
 */
uint16_t rs_float_to_u16(float x, float min, float max)
{
	if (!(x > min)) { /* also catches NaN */
		return 0;
	}
	if (x >= max) {
		return 65535;
	}
	return (uint16_t)((x - min) * 65535.0f / (max - min) + 0.5f);
}

float rs_u16_to_float(uint16_t u, float min, float max)
{
	return min + (float)u * (max - min) / 65535.0f;
}

void rs_build_control(struct rs_frame *f, uint8_t motor_id, const struct rs_command *cmd)
{
	uint16_t torque = rs_float_to_u16(cmd->t_ff, -RS_T_MAX, RS_T_MAX);

	rs_frame_init(f, RS_TYPE_CONTROL, torque, motor_id);
	put_be16(&f->data[0], rs_float_to_u16(cmd->p, -RS_P_MAX, RS_P_MAX));
	put_be16(&f->data[2], rs_float_to_u16(cmd->v, -RS_V_MAX, RS_V_MAX));
	put_be16(&f->data[4], rs_float_to_u16(cmd->kp, 0.0f, RS_KP_MAX));
	put_be16(&f->data[6], rs_float_to_u16(cmd->kd, 0.0f, RS_KD_MAX));
}

void rs_build_enable(struct rs_frame *f, uint8_t host_id, uint8_t motor_id)
{
	rs_frame_init(f, RS_TYPE_ENABLE, host_id, motor_id);
}

void rs_build_stop(struct rs_frame *f, uint8_t host_id, uint8_t motor_id, bool clear_faults)
{
	rs_frame_init(f, RS_TYPE_STOP, host_id, motor_id);
	f->data[0] = clear_faults ? 1U : 0U;
}

void rs_build_set_zero(struct rs_frame *f, uint8_t host_id, uint8_t motor_id)
{
	rs_frame_init(f, RS_TYPE_SET_ZERO, host_id, motor_id);
	f->data[0] = 1U;
}

void rs_build_get_id(struct rs_frame *f, uint8_t host_id, uint8_t motor_id)
{
	rs_frame_init(f, RS_TYPE_GET_ID, host_id, motor_id);
}

void rs_build_param_read(struct rs_frame *f, uint8_t host_id, uint8_t motor_id, uint16_t index)
{
	rs_frame_init(f, RS_TYPE_PARAM_READ, host_id, motor_id);
	put_le16(&f->data[0], index);
}

void rs_build_param_write_u32(struct rs_frame *f, uint8_t host_id, uint8_t motor_id,
			      uint16_t index, uint32_t value)
{
	rs_frame_init(f, RS_TYPE_PARAM_WRITE, host_id, motor_id);
	put_le16(&f->data[0], index);
	put_le32(&f->data[4], value);
}

void rs_build_param_write_float(struct rs_frame *f, uint8_t host_id, uint8_t motor_id,
				uint16_t index, float value)
{
	uint32_t raw;

	memcpy(&raw, &value, sizeof(raw)); /* IEEE-754 bits, sent little-endian */
	rs_build_param_write_u32(f, host_id, motor_id, index, raw);
}

int rs_parse_feedback(const struct rs_frame *f, struct rs_feedback *out)
{
	uint16_t area2 = (uint16_t)((f->id >> RS_ID_AREA2_SHIFT) & RS_ID_AREA2_MASK);

	if (rs_frame_type(f->id) != RS_TYPE_FEEDBACK || f->len < 8) {
		return -1;
	}

	/* data area 2: bits 8..15 motor ID, 16..21 faults, 22..23 mode */
	out->motor_id = (uint8_t)area2;
	out->faults = (uint8_t)((area2 >> 8) & 0x3FU);
	out->mode = (uint8_t)((area2 >> 14) & 0x3U);
	out->host_id = (uint8_t)(f->id & RS_ID_TARGET_MASK);

	out->p = rs_u16_to_float(get_be16(&f->data[0]), -RS_P_MAX, RS_P_MAX);
	out->v = rs_u16_to_float(get_be16(&f->data[2]), -RS_V_MAX, RS_V_MAX);
	out->t = rs_u16_to_float(get_be16(&f->data[4]), -RS_T_MAX, RS_T_MAX);
	out->temp_c = (float)(int16_t)get_be16(&f->data[6]) / 10.0f;
	return 0;
}

int rs_parse_param_reply(const struct rs_frame *f, uint8_t *motor_id, uint16_t *index,
			 uint32_t *raw_value)
{
	if (rs_frame_type(f->id) != RS_TYPE_PARAM_READ || f->len < 8) {
		return -1;
	}

	/* reply: bits 15..8 = motor ID, bits 7..0 = host ID (manual 4.1.15) */
	*motor_id = (uint8_t)(f->id >> RS_ID_AREA2_SHIFT);
	*index = (uint16_t)(f->data[0] | (f->data[1] << 8));
	*raw_value = get_le32(&f->data[4]);
	return 0;
}
