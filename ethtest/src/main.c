/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * mega_can Ethernet bench test, the step after phyprobe/.
 *
 * Brings up the ETH MAC + KSZ9031 with the PIC32CZ driver from ../module, at
 * 192.168.10.2/24. Then:
 *   - answers ping (the IPv4 stack does that by itself)
 *   - echoes every UDP datagram sent to port 7 back to its sender
 *   - LED toggles once a second
 * The driver logs "link up: 1000/100 Mbit/s ..." on every link change.
 *
 * From the Jetson/laptop at 192.168.10.1:
 *   ping 192.168.10.2
 *   echo hello | nc -u -w1 192.168.10.2 7
 * On the board's UART shell (115200):
 *   net iface     link, speed, addresses
 *   net stats     packet and error counters (CRC errors should stay 0)
 *
 * NOTE on running: the factory boot ROM halts at a BKPT when a debugger is
 * attached; reset with the probe's nRESET (or power-cycle) to run.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/printk.h>

#define ECHO_PORT 7

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static void blink(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	while (1) {
		gpio_pin_toggle_dt(&led);
		k_msleep(1000);
	}
}

K_THREAD_DEFINE(blink_tid, 512, blink, NULL, NULL, NULL, 7, 0, 0);

int main(void)
{
	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_port = htons(ECHO_PORT),
		.sin_addr.s_addr = htonl(INADDR_ANY),
	};
	static uint8_t buf[1500];
	int sock;

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	printk("\nmega_can ethtest: 192.168.10.2, UDP echo on port %d\n", ECHO_PORT);

	sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		printk("socket() failed: %d\n", errno);
		return 0;
	}
	if (zsock_bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		printk("bind() failed: %d\n", errno);
		return 0;
	}

	while (1) {
		struct sockaddr_in from;
		socklen_t from_len = sizeof(from);
		int len = zsock_recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&from,
					 &from_len);

		if (len < 0) {
			printk("recvfrom() failed: %d\n", errno);
			k_msleep(100);
			continue;
		}
		zsock_sendto(sock, buf, len, 0, (struct sockaddr *)&from, from_len);
	}
	return 0;
}
