/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * mega_can Robstride 02 bench test, one motor per bus on can0..can3.
 *
 * For each bus (classic CAN 2.0B, 1 Mbit/s):
 *   1. type 0  get device ID of motor 0x7F (factory default). No answer ->
 *              report and skip the bus.
 *   2. type 18 CAN_TIMEOUT = 100 ms, then type 17 read it back. If the board
 *              stops talking, the motor drops to reset mode by itself.
 *   3. type 3  enable
 *   4. type 1  3 s of DAMPING MODE at 100 Hz: kp = 0, kd = 1, v = 0, t_ff = 0
 *              (manual 4.3.1's own demo). The motor only resists being
 *              turned by hand; it has no target to move to.
 *   5. type 4  stop
 * Feedback (type 2) is printed 5 times a second.
 *
 * SAFETY: motor shaft free (nothing attached that can be hurt), 120 ohm
 * termination at both ends of each bus, supply current-limited if possible.
 * The can0<->can5 loopback wire used by bringup/ must be removed if a motor
 * is on can0.
 *
 * NOTE on running: the factory boot ROM halts at a BKPT when a debugger is
 * attached; reset with the probe's nRESET (or power-cycle) to run.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/printk.h>

#include <string.h>

#include "robstride.h"

#define MOTOR_ID      RS_DEFAULT_MOTOR_ID
#define HOST_ID       RS_DEFAULT_HOST_ID
#define REPLY_TIMEOUT K_MSEC(20)
#define TIMEOUT_MS    100U
#define DAMP_TICKS    300 /* 3 s at 100 Hz */
#define GET_ID_REPLY  0xFE /* bits 7..0 of a type 0 reply (manual 4.1.1) */

struct bus {
	const char *name;
	const struct device *dev;
};

static const struct bus buses[] = {
	{"can0", DEVICE_DT_GET(DT_NODELABEL(can0))},
	{"can1", DEVICE_DT_GET(DT_NODELABEL(can1))},
	{"can2", DEVICE_DT_GET(DT_NODELABEL(can2))},
	{"can3", DEVICE_DT_GET(DT_NODELABEL(can3))},
};

CAN_MSGQ_DEFINE(rx_q, 16);

static int send(const struct device *dev, const struct rs_frame *rf)
{
	struct can_frame f = {
		.id = rf->id,
		.dlc = rf->len,
		.flags = CAN_FRAME_IDE, /* 29-bit ID; no FD, no BRS */
	};

	memcpy(f.data, rf->data, rf->len);
	return can_send(dev, &f, K_MSEC(10), NULL, NULL);
}

/* Wait for the next frame of `type` (any other frames are dropped). */
static int wait_for(uint8_t type, struct rs_frame *out)
{
	struct can_frame f;
	int64_t deadline = k_uptime_get() + k_ticks_to_ms_ceil64(REPLY_TIMEOUT.ticks);

	while (k_uptime_get() < deadline) {
		if (k_msgq_get(&rx_q, &f, K_MSEC(1)) != 0) {
			continue;
		}
		if (!(f.flags & CAN_FRAME_IDE) || rs_frame_type(f.id) != type) {
			continue;
		}
		out->id = f.id;
		out->len = f.dlc;
		memcpy(out->data, f.data, sizeof(out->data));
		return 0;
	}
	return -ETIMEDOUT;
}

static void print_feedback(const struct rs_feedback *fb)
{
	static const char *const mode[] = {"reset", "calib", "RUN", "?"};

	printk("  id 0x%02x %-5s pos %+7.3f rad  vel %+7.3f rad/s  tq %+6.2f Nm  "
	       "temp %5.1f C  faults 0x%02x\n",
	       fb->motor_id, mode[fb->mode & 3], (double)fb->p, (double)fb->v, (double)fb->t,
	       (double)fb->temp_c, fb->faults);
	if (fb->faults) {
		printk("  faults:%s%s%s%s%s%s\n",
		       (fb->faults & RS_FAULT_UNDERVOLTAGE) ? " undervoltage" : "",
		       (fb->faults & RS_FAULT_OVERCURRENT) ? " overcurrent" : "",
		       (fb->faults & RS_FAULT_OVERTEMP) ? " over-temp" : "",
		       (fb->faults & RS_FAULT_ENCODER) ? " encoder" : "",
		       (fb->faults & RS_FAULT_OVERLOAD) ? " overload" : "",
		       (fb->faults & RS_FAULT_UNCALIBRATED) ? " uncalibrated" : "");
	}
}

/* Send a frame that the motor answers with type 2 feedback. */
static int command(const struct device *dev, const struct rs_frame *rf, struct rs_feedback *fb)
{
	struct rs_frame reply;

	if (send(dev, rf) != 0) {
		return -EIO;
	}
	if (wait_for(RS_TYPE_FEEDBACK, &reply) != 0) {
		return -ETIMEDOUT;
	}
	return rs_parse_feedback(&reply, fb);
}

static void test_bus(const struct bus *b)
{
	struct rs_frame rf, reply;
	struct rs_feedback fb;
	struct can_bus_err_cnt err;
	enum can_state state;
	const struct can_filter all_ext = {.id = 0, .mask = 0, .flags = CAN_FILTER_IDE};
	int filter;

	printk("\n=== %s ===\n", b->name);
	if (!device_is_ready(b->dev)) {
		printk("  device not ready\n");
		return;
	}
	if (can_set_mode(b->dev, CAN_MODE_NORMAL) != 0 || can_start(b->dev) != 0) {
		printk("  can_start failed\n");
		return;
	}
	filter = can_add_rx_filter_msgq(b->dev, &rx_q, &all_ext);
	k_msgq_purge(&rx_q);

	/* 1. who's there? */
	rs_build_get_id(&rf, HOST_ID, MOTOR_ID);
	if (send(b->dev, &rf) != 0 || wait_for(RS_TYPE_GET_ID, &reply) != 0) {
		can_get_state(b->dev, &state, &err);
		printk("  no motor answered at ID 0x%02x (bus state %d, tx_err %u, rx_err %u)\n"
		       "  check: motor power, CANH/CANL, 120R at both ends, motor ID\n",
		       MOTOR_ID, state, err.tx_err_cnt, err.rx_err_cnt);
		goto out;
	}
	printk("  motor 0x%02x answered, unique ID %02x%02x%02x%02x%02x%02x%02x%02x\n",
	       (uint8_t)(reply.id >> 8), reply.data[0], reply.data[1], reply.data[2],
	       reply.data[3], reply.data[4], reply.data[5], reply.data[6], reply.data[7]);
	if ((reply.id & 0xFFU) != GET_ID_REPLY) {
		printk("  (unexpected low byte 0x%02x in reply ID)\n", reply.id & 0xFFU);
	}

	/* 2. motor-side watchdog, then read it back */
	rs_build_param_write_u32(&rf, HOST_ID, MOTOR_ID, RS_PARAM_CAN_TIMEOUT,
				 TIMEOUT_MS * RS_CAN_TIMEOUT_PER_MS);
	if (command(b->dev, &rf, &fb) != 0) {
		printk("  CAN_TIMEOUT write: no feedback\n");
	}
	rs_build_param_read(&rf, HOST_ID, MOTOR_ID, RS_PARAM_CAN_TIMEOUT);
	if (send(b->dev, &rf) == 0 && wait_for(RS_TYPE_PARAM_READ, &reply) == 0) {
		uint8_t id;
		uint16_t index;
		uint32_t value;

		rs_parse_param_reply(&reply, &id, &index, &value);
		printk("  CAN_TIMEOUT = %u (%u ms)%s\n", value, value / RS_CAN_TIMEOUT_PER_MS,
		       value == TIMEOUT_MS * RS_CAN_TIMEOUT_PER_MS ? "" : "  <- NOT what we wrote");
	} else {
		printk("  CAN_TIMEOUT read-back: no reply\n");
	}

	/* 3. enable */
	rs_build_enable(&rf, HOST_ID, MOTOR_ID);
	if (command(b->dev, &rf, &fb) != 0) {
		printk("  enable: no feedback\n");
		goto stop;
	}
	printk("  enabled:\n");
	print_feedback(&fb);

	/* 4. damping only: resists hand-turning, no target */
	{
		const struct rs_command damp = {.p = 0, .v = 0, .kp = 0, .kd = 1.0f, .t_ff = 0};
		int missed = 0;

		printk("  damping mode 3 s (turn the shaft by hand: it should resist)\n");
		for (int i = 0; i < DAMP_TICKS; i++) {
			rs_build_control(&rf, MOTOR_ID, &damp);
			if (command(b->dev, &rf, &fb) != 0) {
				missed++;
			} else if (i % 20 == 0) {
				print_feedback(&fb);
			}
			k_msleep(10);
		}
		printk("  %d/%d commands without feedback\n", missed, DAMP_TICKS);
	}

stop:
	/* 5. stop (always, even after errors) */
	rs_build_stop(&rf, HOST_ID, MOTOR_ID, false);
	if (command(b->dev, &rf, &fb) == 0) {
		printk("  stopped:\n");
		print_feedback(&fb);
	} else {
		printk("  stop: no feedback\n");
	}
out:
	if (filter >= 0) {
		can_remove_rx_filter(b->dev, filter);
	}
	can_stop(b->dev);
}

int main(void)
{
	printk("\nmega_can Robstride test: motor 0x%02x, host 0x%02x, classic CAN 1 Mbit/s\n",
	       MOTOR_ID, HOST_ID);

	for (size_t i = 0; i < ARRAY_SIZE(buses); i++) {
		test_bus(&buses[i]);
	}
	printk("\ndone. Power-cycle to run again.\n");
	return 0;
}
