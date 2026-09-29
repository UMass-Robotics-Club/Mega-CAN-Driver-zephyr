/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Robstride motors on can0..can3. See motors.h.
 *
 * Frame per motor per tick (Robstride answers each with type 2 feedback):
 *   OFF       -> stop (type 4). Keeps feedback flowing while off.
 *   RUN       -> first tick: enable (type 3); after that: control (type 1)
 *   SET_ZERO  -> stop until the motor reports it is not running, then one
 *                set-zero (type 6). Never zeroes a running motor: with
 *                position gains active, moving the zero makes it jump.
 */

#include "motors.h"

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>

#include "robstride.h"

LOG_MODULE_REGISTER(motors, LOG_LEVEL_INF);

#define HOST_ID            RS_DEFAULT_HOST_ID
#define MOTOR_CAN_TIMEOUT  100U /* ms: motor resets itself if CAN goes quiet */
#define FEEDBACK_STALE_MS  50
#define INIT_REPLY_TIMEOUT K_MSEC(20)

struct motor {
	const struct device *dev;
	uint8_t bus;
	uint8_t can_id;
	/* tick state (main thread only) */
	bool enabled;
	bool zeroed;
	/* feedback (written in the CAN RX callback) */
	struct rs_feedback fb;
	int64_t fb_time_ms; /* 0 = never */
};

/* Motor i on can<i>, factory ID 0x7F (IDs never changed, per Joa 2026-09-28). */
static struct motor motors[PROTO_NUM_MOTORS] = {
	{.dev = DEVICE_DT_GET(DT_NODELABEL(can0)), .bus = 0, .can_id = RS_DEFAULT_MOTOR_ID},
	{.dev = DEVICE_DT_GET(DT_NODELABEL(can1)), .bus = 1, .can_id = RS_DEFAULT_MOTOR_ID},
	{.dev = DEVICE_DT_GET(DT_NODELABEL(can2)), .bus = 2, .can_id = RS_DEFAULT_MOTOR_ID},
	{.dev = DEVICE_DT_GET(DT_NODELABEL(can3)), .bus = 3, .can_id = RS_DEFAULT_MOTOR_ID},
};

/* Bus health for all six controllers (unused ones report zeros). */
static const struct device *const bus_devs[PROTO_NUM_BUSES] = {
	DEVICE_DT_GET(DT_NODELABEL(can0)), DEVICE_DT_GET(DT_NODELABEL(can1)),
	DEVICE_DT_GET(DT_NODELABEL(can2)), DEVICE_DT_GET(DT_NODELABEL(can3)),
	NULL, NULL,
};

static struct k_spinlock fb_lock;
static atomic_t replied;  /* bit i = motor i answered this tick */
static uint32_t expected; /* bits of motors sent to this tick */
static K_SEM_DEFINE(reply_sem, 0, K_SEM_MAX_LIMIT);

static void rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	struct motor *m = user_data;
	struct rs_frame rf = {.id = frame->id, .len = frame->dlc};
	struct rs_feedback fb;

	ARG_UNUSED(dev);
	memcpy(rf.data, frame->data, sizeof(rf.data));
	if (rs_parse_feedback(&rf, &fb) != 0 || fb.motor_id != m->can_id) {
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&fb_lock);

	m->fb = fb;
	m->fb_time_ms = k_uptime_get();
	k_spin_unlock(&fb_lock, key);

	atomic_set_bit(&replied, m - motors);
	k_sem_give(&reply_sem);
}

static void tx_done(const struct device *dev, int error, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(error);
	ARG_UNUSED(user_data);
}

static int send(struct motor *m, const struct rs_frame *rf)
{
	struct can_frame f = {
		.id = rf->id,
		.dlc = rf->len,
		.flags = CAN_FRAME_IDE, /* classic 2.0B, 29-bit; never FD */
	};

	memcpy(f.data, rf->data, rf->len);
	/* async: returns once queued, so the four buses go out in parallel */
	return can_send(m->dev, &f, K_MSEC(1), tx_done, NULL);
}

int motors_init(void)
{
	const struct can_filter feedback = {
		.id = HOST_ID, /* type 2 feedback: bits 7..0 = host ID */
		.mask = 0xFF,
		.flags = CAN_FILTER_IDE,
	};
	struct rs_frame rf;

	for (int i = 0; i < PROTO_NUM_MOTORS; i++) {
		struct motor *m = &motors[i];

		if (!device_is_ready(m->dev) || can_set_mode(m->dev, CAN_MODE_NORMAL) != 0 ||
		    can_start(m->dev) != 0) {
			LOG_ERR("can%u failed to start", m->bus);
			m->dev = NULL;
			continue;
		}
		if (can_add_rx_filter(m->dev, rx_cb, m, &feedback) < 0) {
			LOG_ERR("can%u: no RX filter", m->bus);
		}

		/* Motor-side watchdog. Lost at power-off, so set on every boot. */
		rs_build_param_write_u32(&rf, HOST_ID, m->can_id, RS_PARAM_CAN_TIMEOUT,
					 MOTOR_CAN_TIMEOUT * RS_CAN_TIMEOUT_PER_MS);
		atomic_clear(&replied);
		send(m, &rf);
		k_sem_take(&reply_sem, INIT_REPLY_TIMEOUT);
		if (atomic_test_bit(&replied, i)) {
			LOG_INF("motor %d (can%u, id 0x%02x): answered, CAN_TIMEOUT %u ms", i,
				m->bus, m->can_id, MOTOR_CAN_TIMEOUT);
		} else {
			LOG_WRN("motor %d (can%u, id 0x%02x): no answer", i, m->bus, m->can_id);
		}
	}
	return 0;
}

void motors_apply(const struct proto_cmd_motor cmd[PROTO_NUM_MOTORS], bool armed)
{
	struct rs_frame rf;

	atomic_clear(&replied);
	k_sem_reset(&reply_sem);
	expected = 0;

	for (int i = 0; i < PROTO_NUM_MOTORS; i++) {
		struct motor *m = &motors[i];
		uint8_t mode = cmd[i].mode;

		if (m->dev == NULL) {
			continue;
		}
		if (mode == PROTO_MODE_RUN && !armed) {
			mode = PROTO_MODE_OFF;
		}
		if (mode != PROTO_MODE_SET_ZERO) {
			m->zeroed = false;
		}

		switch (mode) {
		case PROTO_MODE_RUN:
			if (!m->enabled) {
				rs_build_enable(&rf, HOST_ID, m->can_id);
				m->enabled = true;
			} else {
				const struct rs_command c = {
					.p = cmd[i].p_des,
					.v = cmd[i].v_des,
					.kp = cmd[i].kp,
					.kd = cmd[i].kd,
					.t_ff = cmd[i].tau_ff,
				};

				rs_build_control(&rf, m->can_id, &c); /* clamps to motor limits */
			}
			break;

		case PROTO_MODE_SET_ZERO: {
			k_spinlock_key_t key = k_spin_lock(&fb_lock);
			bool running = m->fb.mode == RS_MODE_RUN;

			k_spin_unlock(&fb_lock, key);
			if (!m->zeroed && !m->enabled && !running) {
				rs_build_set_zero(&rf, HOST_ID, m->can_id);
				m->zeroed = true;
			} else {
				rs_build_stop(&rf, HOST_ID, m->can_id, false);
				m->enabled = false;
			}
			break;
		}

		case PROTO_MODE_OFF:
		default:
			rs_build_stop(&rf, HOST_ID, m->can_id, false);
			m->enabled = false;
			break;
		}

		if (send(m, &rf) == 0) {
			expected |= BIT(i);
		}
	}
}

void motors_all_off(void)
{
	struct rs_frame rf;

	for (int i = 0; i < PROTO_NUM_MOTORS; i++) {
		struct motor *m = &motors[i];

		if (m->dev == NULL) {
			continue;
		}
		rs_build_stop(&rf, HOST_ID, m->can_id, false);
		send(m, &rf);
		m->enabled = false;
	}
}

void motors_wait_feedback(k_timeout_t timeout)
{
	k_timepoint_t end = sys_timepoint_calc(timeout);

	while (((uint32_t)atomic_get(&replied) & expected) != expected) {
		if (k_sem_take(&reply_sem, sys_timepoint_timeout(end)) != 0) {
			break;
		}
	}
}

static uint8_t bus_state(enum can_state s)
{
	switch (s) {
	case CAN_STATE_ERROR_WARNING:
		return 1;
	case CAN_STATE_ERROR_PASSIVE:
		return 2;
	case CAN_STATE_BUS_OFF:
		return 3;
	default:
		return 0;
	}
}

void motors_fill_state(struct proto_st_motor motor[PROTO_NUM_MOTORS],
		       struct proto_st_bus bus[PROTO_NUM_BUSES])
{
	int64_t now = k_uptime_get();

	for (int i = 0; i < PROTO_NUM_MOTORS; i++) {
		struct motor *m = &motors[i];
		struct proto_st_motor *o = &motor[i];
		k_spinlock_key_t key = k_spin_lock(&fb_lock);
		struct rs_feedback fb = m->fb;
		int64_t t = m->fb_time_ms;

		k_spin_unlock(&fb_lock, key);

		o->bus = m->bus;
		o->can_id = m->can_id;
		o->faults = fb.faults;
		o->position = fb.p;
		o->velocity = fb.v;
		o->torque = fb.t;
		o->temperature_c = fb.temp_c;
		if (m->dev == NULL || t == 0 || now - t > FEEDBACK_STALE_MS) {
			o->status = PROTO_MOTOR_NO_REPLY;
		} else if (fb.faults != 0) {
			o->status = PROTO_MOTOR_FAULT;
		} else if (fb.mode == RS_MODE_RUN) {
			o->status = PROTO_MOTOR_RUNNING;
		} else {
			o->status = PROTO_MOTOR_OFF;
		}
	}

	for (int i = 0; i < PROTO_NUM_BUSES; i++) {
		struct can_bus_err_cnt err = {0};
		enum can_state s = CAN_STATE_STOPPED;

		memset(&bus[i], 0, sizeof(bus[i]));
		if (bus_devs[i] != NULL && can_get_state(bus_devs[i], &s, &err) == 0) {
			bus[i].state = bus_state(s);
			bus[i].tx_err = err.tx_err_cnt;
			bus[i].rx_err = err.rx_err_cnt;
		}
	}
}
