/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Mega-CAN gateway firmware: Jetson <-> UDP (protocol v3) <-> 4 Robstride motors.
 *
 * Same rules as udp_protocol/fake_board.py, which the software team codes
 * against:
 *   - one state packet back for every valid command, to the sender
 *   - 50 ms without a valid command -> all motors off, watchdog flag set
 *   - after that (and at boot) MODE_RUN is ignored until one all-OFF packet
 *   - packets with seq <= the last one used are dropped; a watchdog trip
 *     resets that, so a restarted Jetson program (seq back at 1) is accepted
 *   - bad packets set FLAG_BAD_PACKET in the next reply
 *
 * Foot sensors, ToF and power are reported as "no data" until their hardware
 * details are known.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>

#include "motors.h"
#include "proto.h"

LOG_MODULE_REGISTER(gateway, LOG_LEVEL_INF);

#define WATCHDOG_MS       50
#define POLL_MS           5    /* watchdog check resolution */
#define FEEDBACK_WAIT_US  1500 /* a CAN command + reply takes ~0.3 ms */

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static struct proto_command cmd;
static struct proto_state st;

static bool armed;
static uint32_t flags;
static uint32_t last_cmd_seq;
static int64_t last_cmd_ms; /* 0 = no command since boot */

static uint32_t now_us(void)
{
	return (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks());
}

static bool header_ok(const struct proto_command *c)
{
	return memcmp(c->hdr.magic, PROTO_MAGIC, 4) == 0 && c->hdr.version == PROTO_VERSION &&
	       c->hdr.type == PROTO_TYPE_COMMAND && c->hdr.count == PROTO_NUM_MOTORS;
}

static bool all_off(const struct proto_command *c)
{
	for (int i = 0; i < PROTO_NUM_MOTORS; i++) {
		if (c->motor[i].mode != PROTO_MODE_OFF) {
			return false;
		}
	}
	return true;
}

static void trip_watchdog(void)
{
	if (armed) {
		LOG_WRN("watchdog: no command for %d ms, motors off", WATCHDOG_MS);
	}
	motors_all_off();
	armed = false;
	flags |= PROTO_FLAG_WATCHDOG_TRIPPED;
	last_cmd_seq = 0; /* accept a restarted client */
}

static void send_state(int sock, const struct sockaddr *to, socklen_t to_len)
{
	static uint32_t state_seq;

	memset(&st, 0, sizeof(st));
	memcpy(st.hdr.magic, PROTO_MAGIC, 4);
	st.hdr.version = PROTO_VERSION;
	st.hdr.type = PROTO_TYPE_STATE;
	st.hdr.count = PROTO_NUM_MOTORS;
	st.hdr.seq = ++state_seq;
	st.hdr.time_us = now_us();
	st.last_cmd_seq = last_cmd_seq;
	st.flags = flags;
	motors_fill_state(st.motor, st.bus);
	/* feet, tof, power: zeros = PROTO_SENSOR_NO_DATA until hardware is known */

	zsock_sendto(sock, &st, sizeof(st), 0, to, to_len);
}

int main(void)
{
	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_port = htons(PROTO_PORT),
		.sin_addr.s_addr = htonl(INADDR_ANY),
	};
	int64_t last_blink = 0;
	int sock;

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	LOG_INF("Mega-CAN gateway, protocol v%d, UDP port %d", PROTO_VERSION, PROTO_PORT);

	motors_init();

	sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0 || zsock_bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		LOG_ERR("UDP socket setup failed: %d", errno);
		return 0;
	}

	while (1) {
		struct zsock_pollfd pfd = {.fd = sock, .events = ZSOCK_POLLIN};
		struct sockaddr_in from;
		socklen_t from_len = sizeof(from);
		int64_t now;
		int len;

		zsock_poll(&pfd, 1, POLL_MS);
		now = k_uptime_get();

		/* LED: slow blink disarmed, fast blink armed */
		if (now - last_blink >= (armed ? 100 : 500)) {
			gpio_pin_toggle_dt(&led);
			last_blink = now;
		}

		if (last_cmd_ms != 0 && now - last_cmd_ms > WATCHDOG_MS &&
		    !(flags & PROTO_FLAG_WATCHDOG_TRIPPED)) {
			trip_watchdog();
		}

		if (!(pfd.revents & ZSOCK_POLLIN)) {
			continue;
		}
		len = zsock_recvfrom(sock, &cmd, sizeof(cmd), ZSOCK_MSG_DONTWAIT,
				     (struct sockaddr *)&from, &from_len);
		if (len < 0) {
			continue;
		}
		if (len != sizeof(cmd) || !header_ok(&cmd)) {
			flags |= PROTO_FLAG_BAD_PACKET;
			continue;
		}
		if (last_cmd_seq != 0 && cmd.hdr.seq <= last_cmd_seq) {
			continue; /* old or duplicate */
		}

		last_cmd_seq = cmd.hdr.seq;
		last_cmd_ms = now;
		if (all_off(&cmd)) {
			if (!armed) {
				LOG_INF("armed (all-off packet received)");
			}
			armed = true;
			flags &= ~PROTO_FLAG_WATCHDOG_TRIPPED;
		}

		motors_apply(cmd.motor, armed);
		motors_wait_feedback(K_USEC(FEEDBACK_WAIT_US));
		send_state(sock, (struct sockaddr *)&from, from_len);
		flags &= ~PROTO_FLAG_BAD_PACKET;
	}
	return 0;
}
