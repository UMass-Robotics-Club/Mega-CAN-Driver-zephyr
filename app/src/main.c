/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * mega_can CAN FD bridge test: can0 and can5 are wired together on the board
 * (CAN0_P/N <-> CAN5_P/N, TCAN1044AV transceivers), so every frame sent on
 * one controller must arrive on the other. Timing comes from the devicetree:
 * 1 Mbit/s nominal, 8 Mbit/s data phase (80 MHz CAN core clock).
 *
 * Each pass sends a 64-byte FD+BRS frame can0 -> can5 and can5 -> can0 and
 * verifies the payload on the receiving side. Periodically a 256-frame burst
 * measures the achieved payload throughput. Results go to the SERCOM0 UART
 * console; the LED toggles on healthy passes.
 *
 * NOTE on running: the factory boot ROM halts at a BKPT when a debugger is
 * attached; reset with the probe's nRESET (or power-cycle) to run.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#include <string.h>

#define TEST_ID_A2B  0x123 /* can0 -> can5 */
#define TEST_ID_B2A  0x321 /* can5 -> can0 */
#define BURST_ID     0x200
#define BURST_COUNT  256
#define PAYLOAD_LEN  64

static const struct device *const can_a = DEVICE_DT_GET(DT_NODELABEL(can0));
static const struct device *const can_b = DEVICE_DT_GET(DT_NODELABEL(can5));
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

struct rx_ctx {
	struct k_sem sem;
	atomic_t rx_count;
	atomic_t bad_payload;
	uint8_t expected_seq;
};

static struct rx_ctx rx_a2b; /* frames arriving on can5 */
static struct rx_ctx rx_b2a; /* frames arriving on can0 */

static struct k_sem burst_sem;
static atomic_t burst_rx;

static void fill_payload(uint8_t *buf, uint8_t seq)
{
	for (int i = 0; i < PAYLOAD_LEN; i++) {
		buf[i] = (uint8_t)(seq + i);
	}
}

static void rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	struct rx_ctx *ctx = user_data;
	uint8_t expected[PAYLOAD_LEN];

	ARG_UNUSED(dev);

	atomic_inc(&ctx->rx_count);

	fill_payload(expected, ctx->expected_seq);
	if (can_dlc_to_bytes(frame->dlc) != PAYLOAD_LEN ||
	    memcmp(frame->data, expected, PAYLOAD_LEN) != 0) {
		atomic_inc(&ctx->bad_payload);
	}

	k_sem_give(&ctx->sem);
}

static void burst_rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(frame);
	ARG_UNUSED(user_data);

	if (atomic_inc(&burst_rx) + 1 >= BURST_COUNT) {
		k_sem_give(&burst_sem);
	}
}

static const char *state_str(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE:
		return "error-active";
	case CAN_STATE_ERROR_WARNING:
		return "error-warning";
	case CAN_STATE_ERROR_PASSIVE:
		return "error-passive";
	case CAN_STATE_BUS_OFF:
		return "bus-off";
	case CAN_STATE_STOPPED:
		return "stopped";
	default:
		return "?";
	}
}

static void print_state(const char *name, const struct device *dev)
{
	struct can_bus_err_cnt err_cnt;
	enum can_state state;

	if (can_get_state(dev, &state, &err_cnt) == 0) {
		printk("  %s: %s (tx_err=%u rx_err=%u)\n", name, state_str(state),
		       err_cnt.tx_err_cnt, err_cnt.rx_err_cnt);
	}
}

static int setup_can(const char *name, const struct device *dev)
{
	int ret;

	if (!device_is_ready(dev)) {
		printk("CAN: %s not ready\n", name);
		return -ENODEV;
	}

	ret = can_set_mode(dev, CAN_MODE_FD);
	if (ret != 0) {
		printk("CAN: %s set FD mode failed (%d)\n", name, ret);
		return ret;
	}

	ret = can_start(dev);
	if (ret != 0) {
		printk("CAN: %s start failed (%d)\n", name, ret);
		return ret;
	}

	return 0;
}

/* TX completion callback: bounded sends, never blocks inside can_send. */
static struct k_sem tx_done;
static atomic_t tx_status;

static void tx_cb(const struct device *dev, int error, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	atomic_set(&tx_status, error);
	k_sem_give(&tx_done);
}

/* Send one 64-byte FD+BRS frame and wait for verified reception. */
static int send_and_verify(const struct device *from, uint32_t id, struct rx_ctx *ctx,
			   uint8_t seq)
{
	struct can_frame frame = {
		.flags = CAN_FRAME_FDF | CAN_FRAME_BRS,
		.id = id,
		.dlc = can_bytes_to_dlc(PAYLOAD_LEN),
	};
	int ret;

	ctx->expected_seq = seq;
	atomic_set(&ctx->bad_payload, 0);
	fill_payload(frame.data, seq);
	k_sem_reset(&ctx->sem);
	k_sem_reset(&tx_done);

	ret = can_send(from, &frame, K_MSEC(100), tx_cb, NULL);
	if (ret != 0) {
		return ret;
	}

	if (k_sem_take(&tx_done, K_MSEC(100)) != 0) {
		/* Never completed on the wire. Purge the queued frame --
		 * otherwise the M_CAN retries it forever and the resulting
		 * error/bus-off interrupt churn starves the CPU. */
		(void)can_stop(from);
		(void)can_start(from);
		return -ETIMEDOUT;
	}
	if (atomic_get(&tx_status) != 0) {
		return (int)atomic_get(&tx_status);
	}

	if (k_sem_take(&ctx->sem, K_MSEC(100)) != 0) {
		return -ETIMEDOUT;
	}

	return atomic_get(&ctx->bad_payload) != 0 ? -EBADMSG : 0;
}

/* Prove the controller + driver + message RAM + IRQ + FD timing work,
 * without involving the external transceivers: M_CAN internal loopback.
 */
static int self_test(const char *name, const struct device *dev, uint32_t id,
		     struct rx_ctx *ctx)
{
	int ret;
	const struct can_filter filter = {
		.id = id,
		.mask = CAN_STD_ID_MASK,
	};
	int filter_id;

	ret = can_stop(dev);
	if (ret != 0 && ret != -EALREADY) {
		printk("selftest %s: stop failed (%d)\n", name, ret);
		return ret;
	}

	ret = can_set_mode(dev, CAN_MODE_FD | CAN_MODE_LOOPBACK);
	if (ret != 0) {
		printk("selftest %s: set loopback mode failed (%d)\n", name, ret);
		return ret;
	}

	ret = can_start(dev);
	if (ret != 0) {
		printk("selftest %s: start failed (%d)\n", name, ret);
		return ret;
	}

	filter_id = can_add_rx_filter(dev, rx_cb, ctx, &filter);
	if (filter_id < 0) {
		printk("selftest %s: add filter failed (%d)\n", name, filter_id);
		return filter_id;
	}

	ret = send_and_verify(dev, id, ctx, 0xA5);
	can_remove_rx_filter(dev, filter_id);

	/* back to normal FD mode for the bridge test */
	(void)can_stop(dev);
	(void)can_set_mode(dev, CAN_MODE_FD);
	(void)can_start(dev);

	printk("selftest %s (internal loopback, FD 1M/8M): %s (%d)\n", name,
	       ret == 0 ? "PASS" : "FAIL", ret);
	return ret;
}

/* Blast BURST_COUNT frames can0 -> can5 and measure payload throughput. */
static void run_burst(uint32_t *kbit_payload, int *lost)
{
	struct can_frame frame = {
		.flags = CAN_FRAME_FDF | CAN_FRAME_BRS,
		.id = BURST_ID,
		.dlc = can_bytes_to_dlc(PAYLOAD_LEN),
	};
	uint32_t start, cycles;
	uint64_t ns;
	int ret;

	fill_payload(frame.data, 0x55);
	atomic_set(&burst_rx, 0);
	k_sem_reset(&burst_sem);

	start = k_cycle_get_32();

	for (int i = 0; i < BURST_COUNT; i++) {
		frame.data[0] = (uint8_t)i;
		k_sem_reset(&tx_done);
		ret = can_send(can_a, &frame, K_MSEC(100), tx_cb, NULL);
		if (ret != 0 || k_sem_take(&tx_done, K_MSEC(100)) != 0 ||
		    atomic_get(&tx_status) != 0) {
			break;
		}
	}

	(void)k_sem_take(&burst_sem, K_MSEC(1000));
	cycles = k_cycle_get_32() - start;
	ns = k_cyc_to_ns_floor64(cycles);

	*lost = BURST_COUNT - (int)atomic_get(&burst_rx);
	if (ns > 0) {
		/* payload bits transferred per second, in kbit/s */
		uint64_t bits = (uint64_t)atomic_get(&burst_rx) * PAYLOAD_LEN * 8U;

		*kbit_payload = (uint32_t)((bits * NSEC_PER_SEC / ns) / 1000U);
	} else {
		*kbit_payload = 0;
	}
}

/* Temporary bring-up diagnostic: drive each transceiver's TXD as GPIO and
 * sample the RXDs a few microseconds later (well inside the TCAN1044AV TXD
 * dominant timeout), with STB driven both low and high, to find out whether
 * the transceivers respond at all and which STB polarity is active. Restores
 * the CAN pinmux (function H) afterwards.
 */
#define PORTBASE(g)   (0x44840000u + (g) * 0x80u)
#define P_DIRSET(g)   (PORTBASE(g) + 0x08u)
#define P_DIRCLR(g)   (PORTBASE(g) + 0x04u)
#define P_OUTSET(g)   (PORTBASE(g) + 0x18u)
#define P_OUTCLR(g)   (PORTBASE(g) + 0x14u)
#define P_IN(g)       (PORTBASE(g) + 0x20u)
#define P_PINCFG(g,p) (PORTBASE(g) + 0x40u + (p))
#define P_PMUX(g,p)   (PORTBASE(g) + 0x30u + (p) / 2u)

#define GRP_A 0
#define GRP_C 2
#define GRP_D 3
#define GRP_B 1
#define GRP_F 5

static void phy_gpio(uint8_t grp, uint8_t pin, bool out)
{
	/* INEN also on outputs, so the pad level can be read back to detect
	 * external contention. */
	sys_write8(0x02, P_PINCFG(grp, pin));
	sys_write32(BIT(pin), out ? P_DIRSET(grp) : P_DIRCLR(grp));
}

static void phy_mux_h(uint8_t grp, uint8_t pin)
{
	uint8_t v = sys_read8(P_PMUX(grp, pin));

	if (pin & 1U) {
		v = (v & 0x0F) | (0x7 << 4);
	} else {
		v = (v & 0xF0) | 0x7;
	}
	sys_write8(v, P_PMUX(grp, pin));
	sys_write8(0x01, P_PINCFG(grp, pin)); /* PMUXEN=1 */
}

static void phy_probe(void)
{
	/* TXD out: PC5 (can0), PA21 (can5); RXD in: PC4, PD23; STB out: PF2, PF7 */
	phy_gpio(GRP_C, 5, true);
	phy_gpio(GRP_A, 21, true);
	phy_gpio(GRP_C, 4, false);
	phy_gpio(GRP_D, 23, false);
	phy_gpio(GRP_B, 11, true); /* CAN0 STB (PCB rework: ball V14) */
	phy_gpio(GRP_A, 24, true); /* CAN5 STB (PCB rework: ball M1) */
	sys_write32(BIT(5), P_OUTSET(GRP_C));
	sys_write32(BIT(21), P_OUTSET(GRP_A));

	for (int stb = 0; stb <= 1; stb++) {
		sys_write32(BIT(11), stb ? P_OUTSET(GRP_B) : P_OUTCLR(GRP_B));
		sys_write32(BIT(24), stb ? P_OUTSET(GRP_A) : P_OUTCLR(GRP_A));
		k_busy_wait(100);

		for (int drv = 0; drv < 2; drv++) {
			/* drv 0: can0 TXD, drv 1: can5 TXD */
			uint8_t grp = drv ? GRP_A : GRP_C;
			uint8_t pin = drv ? 21 : 5;
			uint32_t rx0_h, rx5_h, rx0_l, rx5_l;

			sys_write32(BIT(pin), P_OUTSET(grp));
			k_busy_wait(10);
			rx0_h = (sys_read32(P_IN(GRP_C)) >> 4) & 1U;
			rx5_h = (sys_read32(P_IN(GRP_D)) >> 23) & 1U;

			sys_write32(BIT(pin), P_OUTCLR(grp));
			k_busy_wait(10); /* 10 us << TXD dominant timeout */
			rx0_l = (sys_read32(P_IN(GRP_C)) >> 4) & 1U;
			rx5_l = (sys_read32(P_IN(GRP_D)) >> 23) & 1U;

			/* pad readback: does the driven-low level win the pin? */
			uint32_t pad = (sys_read32(P_IN(grp)) >> pin) & 1U;

			sys_write32(BIT(pin), P_OUTSET(grp));

			printk("phy: STB=%d drive=%s: TXD=1 -> rx0=%u rx5=%u | "
			       "TXD=0 (pad=%u) -> rx0=%u rx5=%u\n",
			       stb, drv ? "can5" : "can0", rx0_h, rx5_h, pad, rx0_l, rx5_l);
		}
	}

	/* Reversed-role probe: if the board swapped TXD/RXD at the
	 * transceivers, the "RX" pins are really the TCAN TXD inputs. Drive
	 * them and watch the "TX" pins (which would be the RXD outputs).
	 */
	sys_write32(BIT(11), P_OUTCLR(GRP_B)); /* STB low, normal */
	sys_write32(BIT(24), P_OUTCLR(GRP_A));
	phy_gpio(GRP_C, 4, true);
	phy_gpio(GRP_D, 23, true);
	phy_gpio(GRP_C, 5, false);
	phy_gpio(GRP_A, 21, false);
	sys_write32(BIT(4), P_OUTSET(GRP_C));
	sys_write32(BIT(23), P_OUTSET(GRP_D));
	k_busy_wait(100);

	for (int drv = 0; drv < 2; drv++) {
		uint8_t grp = drv ? GRP_D : GRP_C;
		uint8_t pin = drv ? 23 : 4;
		uint32_t t0_h, t5_h, t0_l, t5_l;

		sys_write32(BIT(pin), P_OUTSET(grp));
		k_busy_wait(10);
		t0_h = (sys_read32(P_IN(GRP_C)) >> 5) & 1U;
		t5_h = (sys_read32(P_IN(GRP_A)) >> 21) & 1U;

		sys_write32(BIT(pin), P_OUTCLR(grp));
		k_busy_wait(10);
		t0_l = (sys_read32(P_IN(GRP_C)) >> 5) & 1U;
		t5_l = (sys_read32(P_IN(GRP_A)) >> 21) & 1U;
		sys_write32(BIT(pin), P_OUTSET(grp));

		printk("phy-rev: drive=%s(as-txd): 1 -> pc5=%u pa21=%u | 0 -> pc5=%u pa21=%u\n",
		       drv ? "pd23" : "pc4", t0_h, t5_h, t0_l, t5_l);
	}

	/* STB pad readback: drive PF2/PF7 and read the pad level back. If the
	 * pad doesn't follow the drive, something external overpowers the pin
	 * (or the net has a strong pull the driver can't win). */
	for (int lvl = 0; lvl <= 1; lvl++) {
		sys_write32(BIT(11), lvl ? P_OUTSET(GRP_B) : P_OUTCLR(GRP_B));
		sys_write32(BIT(24), lvl ? P_OUTSET(GRP_A) : P_OUTCLR(GRP_A));
		k_busy_wait(50);

		uint32_t inb = sys_read32(P_IN(GRP_B));
		uint32_t ina = sys_read32(P_IN(GRP_A));

		printk("phy-stb: drive=%d -> PB11(can0 stb) pad=%u PA24(can5 stb) pad=%u\n",
		       lvl, (unsigned int)((inb >> 11) & 1U), (unsigned int)((ina >> 24) & 1U));
	}

	/* park STB low (normal mode) and restore the CAN pin muxing. Release
	 * every probed pin to input first: the PORT output driver stays
	 * active behind a muxed *input* function, so a leftover DIR bit puts
	 * the pad in contention with the transceiver's RXD driver. */
	sys_write32(BIT(11), P_OUTCLR(GRP_B));
	sys_write32(BIT(24), P_OUTCLR(GRP_A));
	sys_write32(BIT(5) | BIT(4), P_DIRCLR(GRP_C));
	sys_write32(BIT(21), P_DIRCLR(GRP_A));
	sys_write32(BIT(23), P_DIRCLR(GRP_D));
	phy_mux_h(GRP_C, 5);
	phy_mux_h(GRP_C, 4);
	phy_mux_h(GRP_A, 21);
	phy_mux_h(GRP_D, 23);
}

int main(void)
{
	const struct can_filter filter_a2b = {
		.id = TEST_ID_A2B,
		.mask = CAN_STD_ID_MASK,
	};
	const struct can_filter filter_b2a = {
		.id = TEST_ID_B2A,
		.mask = CAN_STD_ID_MASK,
	};
	const struct can_filter filter_burst = {
		.id = BURST_ID,
		.mask = CAN_STD_ID_MASK,
	};
	uint32_t pass = 0, fail = 0;
	uint8_t seq = 0;
	int ret;

	printk("\n*** mega_can CAN FD bridge test: can0 <-> can5, 1M arb / 8M data ***\n");

	if (!gpio_is_ready_dt(&led) ||
	    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE) < 0) {
		printk("error: LED init failed\n");
	}

	k_sem_init(&rx_a2b.sem, 0, 1);
	k_sem_init(&rx_b2a.sem, 0, 1);
	k_sem_init(&burst_sem, 0, 1);
	k_sem_init(&tx_done, 0, 1);

	phy_probe();

	if (setup_can("can0", can_a) != 0 || setup_can("can5", can_b) != 0) {
		return -ENODEV;
	}

	/* Controller-internal self-test first: validates driver, message RAM,
	 * interrupts and 1M/8M FD timing without the transceivers/bus. */
	self_test("can0", can_a, TEST_ID_A2B, &rx_a2b);
	self_test("can5", can_b, TEST_ID_B2A, &rx_b2a);

	{
		uint32_t core_hz = 0;

		(void)can_get_core_clock(can_a, &core_hz);
		printk("CAN: core clock reported %u Hz\n", core_hz);
	}

	/* Slow-bus bisection: classic CAN at 125 kbit/s over the real bus.
	 * Timing-margin problems disappear at low speed; structural problems
	 * don't. */
	{
		int f0, f5;

		(void)can_stop(can_a);
		(void)can_stop(can_b);
		(void)can_set_mode(can_a, 0);
		(void)can_set_mode(can_b, 0);
		(void)can_set_bitrate(can_a, 125000);
		(void)can_set_bitrate(can_b, 125000);
		(void)can_start(can_a);
		(void)can_start(can_b);

		const struct can_filter f = { .id = 0x155, .mask = CAN_STD_ID_MASK };

		f5 = can_add_rx_filter(can_b, rx_cb, &rx_a2b, &f);

		struct can_frame cf = { .id = 0x155, .dlc = 8 };

		rx_a2b.expected_seq = 0x11;
		for (int i = 0; i < 8; i++) {
			cf.data[i] = (uint8_t)(0x11 + i);
		}
		k_sem_reset(&rx_a2b.sem);
		k_sem_reset(&tx_done);
		f0 = can_send(can_a, &cf, K_MSEC(200), tx_cb, NULL);
		if (f0 == 0 && k_sem_take(&tx_done, K_MSEC(200)) == 0 &&
		    atomic_get(&tx_status) == 0 &&
		    k_sem_take(&rx_a2b.sem, K_MSEC(200)) == 0) {
			printk("CAN: classic 125k can0->can5: PASS\n");
		} else {
			printk("CAN: classic 125k can0->can5: FAIL (send=%d tx_status=%ld)\n",
			       f0, atomic_get(&tx_status));
			print_state("can0", can_a);
			print_state("can5", can_b);
			(void)can_stop(can_a);
			(void)can_start(can_a);
		}
		if (f5 >= 0) {
			can_remove_rx_filter(can_b, f5);
		}

		/* restore FD 1M/8M for the main test */
		(void)can_stop(can_a);
		(void)can_stop(can_b);
		(void)can_set_bitrate(can_a, 1000000);
		(void)can_set_bitrate(can_b, 1000000);
		(void)can_set_mode(can_a, CAN_MODE_FD);
		(void)can_set_mode(can_b, CAN_MODE_FD);
		(void)can_start(can_a);
		(void)can_start(can_b);
	}

	if (can_add_rx_filter(can_b, rx_cb, &rx_a2b, &filter_a2b) < 0 ||
	    can_add_rx_filter(can_a, rx_cb, &rx_b2a, &filter_b2a) < 0 ||
	    can_add_rx_filter(can_b, burst_rx_cb, NULL, &filter_burst) < 0) {
		printk("CAN: adding rx filters failed\n");
		return -EIO;
	}

	printk("CAN: both controllers started (FD mode)\n");

	while (1) {
		bool ok = true;

		/* Bidirectional single-frame verification. */
		ret = send_and_verify(can_a, TEST_ID_A2B, &rx_a2b, seq);
		if (ret != 0) {
			printk("CAN: can0->can5 FAILED (%d)\n", ret);
			ok = false;
		}

		ret = send_and_verify(can_b, TEST_ID_B2A, &rx_b2a, (uint8_t)(seq + 1));
		if (ret != 0) {
			printk("CAN: can5->can0 FAILED (%d)\n", ret);
			ok = false;
		}

		seq += 2;

		if (ok) {
			pass++;
			gpio_pin_toggle_dt(&led);
		} else {
			fail++;
		}

		/* Report + throughput burst every ~64 iterations. */
		if ((pass + fail) % 64 == 1) {
			uint32_t kbit = 0;
			int lost = 0;

			if (ok) {
				run_burst(&kbit, &lost);
			}

			printk("CAN: pass=%u fail=%u rx(a2b)=%ld rx(b2a)=%ld "
			       "burst: %u.%03u Mbit/s payload, %d lost\n",
			       pass, fail, atomic_get(&rx_a2b.rx_count),
			       atomic_get(&rx_b2a.rx_count), kbit / 1000, kbit % 1000, lost);
			print_state("can0", can_a);
			print_state("can5", can_b);
		}

		k_msleep(ok ? 50 : 500);
	}

	return 0;
}
