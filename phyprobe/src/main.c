/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * mega_can KSZ9031 PHY probe. Answers "is the board's Ethernet section alive?"
 * without the GMAC driver: MDIO is bit-banged on GPIOs (see app.overlay).
 *
 * 1. Opens the TX clock switch (SEL low) so PD5 and the PHY's TX_CLK are apart.
 * 2. Resets the PHY (RESET_N low 20 ms, then 100 ms for it to come up),
 *    setting its unresistored strap pins (MODE = GMII/MII, PHYAD2, CLK125_EN)
 *    with MCU pulls during reset.
 * 3. Reads PHYID1/PHYID2 at all 32 MDIO addresses. A KSZ9031 answers
 *    0x0022 / 0x162x at its strapped address only (address 0 is NOT a
 *    broadcast address on this part, KSZ9031 datasheet section 3.11), so expect
 *    exactly one hit, at 3.
 * 4. Prints the PHY's status registers, then reports link changes once a
 *    second: speed, duplex, and how the firmware must set the TX clock path
 *    for that speed.
 *
 * The PHY negotiates on its own; no MAC is needed to get a link. Plug a cable
 * to the Jetson or a laptop to see 1000/100 show up.
 *
 * Register numbers: IEEE 802.3 clause 22 (0-10) and KSZ9031MNX datasheet
 * DS00002096E section 4 (1Fh PHY Control). Results go to the SERCOM0 UART
 * console at 115200; the LED toggles once a second.
 *
 * NOTE on running: the factory boot ROM halts at a BKPT when a debugger is
 * attached; reset with the probe's nRESET (or power-cycle) to run.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/sys/printk.h>

/* Clause 22 registers */
#define REG_BMCR       0x00 /* basic control */
#define REG_BMSR       0x01 /* basic status */
#define REG_PHYID1     0x02
#define REG_PHYID2     0x03
#define REG_ANAR       0x04 /* our advertised abilities */
#define REG_ANLPAR     0x05 /* link partner's 10/100 abilities */
#define REG_GBCR       0x09 /* 1000BASE-T control (our 1000 advertisement) */
#define REG_GBSR       0x0A /* 1000BASE-T status (partner's 1000 abilities) */
/* KSZ9031 vendor registers */
#define REG_KSZ_PHYCTL 0x1F /* PHY Control: final speed/duplex */

#define BMSR_LINK      BIT(2)  /* latches low on link loss: read twice */
#define BMSR_AN_DONE   BIT(5)
#define GBSR_LP_1000FD BIT(11)
#define GBSR_LP_1000HD BIT(10)

#define KSZ_SPD_1000   BIT(6)
#define KSZ_SPD_100    BIT(5)
#define KSZ_SPD_10     BIT(4)
#define KSZ_FULL_DPLX  BIT(3)
#define KSZ_MASTER     BIT(2)

#define KSZ_PHYID1     0x0022
#define KSZ_PHYID2     0x1620 /* low 4 bits = silicon revision */

#define NO_PHY         0xFF

static const struct device *const mdio = DEVICE_DT_GET(DT_NODELABEL(mdio_bb));
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec phy_reset =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), eth_reset_gpios);
static const struct gpio_dt_spec txclk_sel =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), eth_txclk_sel_gpios);
static const struct gpio_dt_spec phy_int =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), eth_int_gpios);

#define STRAP_NODE DT_PATH(zephyr_user)
#define STRAP_SPEC(node, prop, idx) GPIO_DT_SPEC_GET_BY_IDX(node, prop, idx),

/* Board has no strap resistors on these PHY pins; see app.overlay. */
static const struct gpio_dt_spec straps[] = {
	DT_FOREACH_PROP_ELEM(STRAP_NODE, eth_strap_gpios, STRAP_SPEC)
};
static const char *const strap_name[] = {
	"MODE0", "MODE1", "MODE2", "MODE3", "CLK125_EN", "PHYAD2",
};
/* MODE[3:0] = 0001 (GMII/MII), 125 MHz output off, PHYAD2 = 0 */
static const uint8_t strap_level[] = { 1, 0, 0, 0, 0, 0 };
#define EXPECTED_ADDR 3 /* PHYAD2 = 0 (ours) + PHYAD1/0 = 1/1 (LED pull-ups) */

BUILD_ASSERT(ARRAY_SIZE(straps) == ARRAY_SIZE(strap_level));

static uint16_t rd(uint8_t addr, uint8_t reg)
{
	uint16_t v = 0xFFFF;

	if (mdio_read(mdio, addr, reg, &v) != 0) {
		printk("  mdio_read(%u, 0x%02x) failed\n", addr, reg);
	}
	return v;
}

static int pins_init(void)
{
	if (!gpio_is_ready_dt(&led) || !gpio_is_ready_dt(&phy_reset) ||
	    !gpio_is_ready_dt(&txclk_sel) || !gpio_is_ready_dt(&phy_int) ||
	    !device_is_ready(mdio)) {
		printk("GPIO or MDIO device not ready\n");
		return -ENODEV;
	}
	for (size_t i = 0; i < ARRAY_SIZE(straps); i++) {
		if (!gpio_is_ready_dt(&straps[i])) {
			printk("strap GPIO %s not ready\n", strap_name[i]);
			return -ENODEV;
		}
	}

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	/*
	 * Switch open: the PHY's 25 MHz TX_CLK output stays off PD5. PD5 itself
	 * is left as an input (reset state), so nothing drives GTX_CLK either.
	 * This is the safe state; the real driver picks SEL per link speed.
	 */
	gpio_pin_configure_dt(&txclk_sel, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&phy_int, GPIO_INPUT);
	/* Active-low in the devicetree: "active" = RESET_N pulled low. */
	gpio_pin_configure_dt(&phy_reset, GPIO_OUTPUT_ACTIVE);
	return 0;
}

static void phy_hw_reset(void)
{
	/*
	 * KSZ9031 needs reset held >= 10 ms after power is stable (Table 7-7,
	 * tSR); it samples its strap pins (MODE, PHYAD, CLK125_EN) on release.
	 * The board's RC on RESET_N already did a power-on reset, but with the
	 * strap pins floating (no resistors on the board), so the PHY may have
	 * latched a random mode. This reset is the one that counts.
	 *
	 * While RESET_N is low the strap pins are high-impedance inputs, so the
	 * MCU's weak internal pulls are enough to set their level. After release
	 * the PHY drives those pins as outputs; a pull just loses to it, so the
	 * MCU and PHY never fight (driving the pins as outputs instead would).
	 */
	gpio_pin_set_dt(&phy_reset, 1);
	printk("PHY straps while in reset:");
	for (size_t i = 0; i < ARRAY_SIZE(straps); i++) {
		gpio_pin_configure_dt(&straps[i], GPIO_INPUT |
				      (strap_level[i] ? GPIO_PULL_UP : GPIO_PULL_DOWN));
		printk(" %s=%u", strap_name[i], strap_level[i]);
	}
	printk("\n");
	k_msleep(20);

	/* Report what the pins actually read: a stuck pin shows up here. */
	for (size_t i = 0; i < ARRAY_SIZE(straps); i++) {
		int v = gpio_pin_get_dt(&straps[i]);

		if (v != strap_level[i]) {
			printk("  WARNING: %s reads %d, wanted %u (short or external load?)\n",
			       strap_name[i], v, strap_level[i]);
		}
	}

	gpio_pin_set_dt(&phy_reset, 0); /* straps latch on this edge */
	k_msleep(1);
	/* Pulls off; in the real firmware pinctrl hands these pins to the GMAC. */
	for (size_t i = 0; i < ARRAY_SIZE(straps); i++) {
		gpio_pin_configure_dt(&straps[i], GPIO_INPUT);
	}
	k_msleep(100);
}

/* Returns the MDIO address to use, or NO_PHY. */
static uint8_t scan(void)
{
	uint8_t found = NO_PHY;
	int all_ones = 0, all_zeros = 0;

	printk("\nMDIO scan (addresses 0-31):\n");
	for (uint8_t a = 0; a < 32; a++) {
		uint16_t id1 = rd(a, REG_PHYID1);
		uint16_t id2 = rd(a, REG_PHYID2);

		if (id1 == 0xFFFF && id2 == 0xFFFF) {
			all_ones++;
			continue;
		}
		if (id1 == 0x0000 && id2 == 0x0000) {
			all_zeros++;
			continue;
		}

		bool ksz = id1 == KSZ_PHYID1 && (id2 & 0xFFF0) == KSZ_PHYID2;

		printk("  addr %2u: PHYID1=0x%04x PHYID2=0x%04x %s\n", a, id1, id2,
		       ksz ? "<- KSZ9031" : "(unknown PHY)");
		if (ksz) {
			printk("           silicon rev %u\n", id2 & 0xF);
			if (found == NO_PHY) {
				found = a;
			} else {
				printk("           second KSZ9031 answer?! MDIO noise or "
				       "a wiring fault; using address %u\n", found);
			}
		}
	}

	if (found != NO_PHY) {
		printk("PASS: KSZ9031 answers at MDIO address %u\n", found);
		if (found == EXPECTED_ADDR) {
			printk("      = expected: PHYAD2 strap took, LED straps read 1/1\n");
		} else {
			printk("      expected %u: PHYAD2 bit %s, PHYAD1/0 (LED straps) = %u/%u\n"
			       "      -> set reg = <%u> on the PHY node in the board devicetree\n",
			       EXPECTED_ADDR, (found & 4) ? "did NOT take (floating?)" : "ok",
			       (found >> 1) & 1, found & 1, found);
		}
	} else if (all_ones == 32) {
		printk("FAIL: every read was 0xFFFF = nobody drove MDIO.\n"
		       "      Check: PHY power (VCC_ETH_P3V3 + core rails), 25 MHz osc on\n"
		       "      PHY XI (pin 61), RESET_N high (pin 56), MDC/MDIO continuity\n"
		       "      PA3/PA4 -> PHY.\n");
	} else if (all_zeros == 32) {
		printk("FAIL: every read was 0x0000 = MDIO stuck low.\n"
		       "      Check: MDIO short to GND, R58 pull-up, PHY held in reset.\n");
	} else {
		printk("FAIL: answers, but no KSZ9031 ID. Noisy MDIO or wrong part?\n");
	}
	return found;
}

static void dump(uint8_t a)
{
	uint16_t bmsr = rd(a, REG_BMSR);

	bmsr = rd(a, REG_BMSR); /* second read = current link state */

	printk("\nPHY registers (addr %u):\n", a);
	printk("  BMCR   0x%04x  basic control\n", rd(a, REG_BMCR));
	printk("  BMSR   0x%04x  link %s, autoneg %s\n", bmsr,
	       (bmsr & BMSR_LINK) ? "UP" : "down",
	       (bmsr & BMSR_AN_DONE) ? "done" : "not done");
	printk("  ANAR   0x%04x  we advertise (10/100)\n", rd(a, REG_ANAR));
	printk("  GBCR   0x%04x  we advertise (1000)\n", rd(a, REG_GBCR));
	printk("  1Fh    0x%04x  KSZ PHY control\n", rd(a, REG_KSZ_PHYCTL));
	printk("  INT_N  %s\n", gpio_pin_get_dt(&phy_int) ? "asserted" : "idle");
}

/*
 * What the real driver must do for each speed. The TX clock direction flips
 * between GMII and MII, and U4 routes it (see CLAUDE.md, Hardware).
 */
static void report_link(uint8_t a)
{
	uint16_t bmsr;
	uint16_t ctl;

	(void)rd(a, REG_BMSR);
	bmsr = rd(a, REG_BMSR);
	if (!(bmsr & BMSR_LINK)) {
		printk("link DOWN\n");
		return;
	}

	ctl = rd(a, REG_KSZ_PHYCTL);
	const char *dplx = (ctl & KSZ_FULL_DPLX) ? "full" : "half";

	if (ctl & KSZ_SPD_1000) {
		uint16_t gbsr = rd(a, REG_GBSR);

		printk("link UP: 1000BASE-T %s duplex, %s (partner 1000FD=%d)\n"
		       "  -> GMII: SEL (PA22) low, PD5 = GTXCK out (125 MHz, GCLK ch 54)\n",
		       dplx, (ctl & KSZ_MASTER) ? "master" : "slave",
		       !!(gbsr & GBSR_LP_1000FD));
	} else if (ctl & (KSZ_SPD_100 | KSZ_SPD_10)) {
		printk("link UP: %s %s duplex (partner ANLPAR=0x%04x)\n"
		       "  -> MII: PD5 = TXCK in, then SEL (PA22) high\n",
		       (ctl & KSZ_SPD_100) ? "100BASE-TX" : "10BASE-T", dplx,
		       rd(a, REG_ANLPAR));
	} else {
		printk("link UP but no speed bit set yet (1Fh=0x%04x)\n", ctl);
	}
}

int main(void)
{
	uint8_t addr;
	bool last_link = false;
	bool first = true;

	printk("\nmega_can KSZ9031 PHY probe\n");
	if (pins_init() != 0) {
		return 0;
	}

	phy_hw_reset();
	addr = scan();
	if (addr == NO_PHY) {
		/* Keep blinking slowly so a dead PHY isn't mistaken for a dead MCU. */
		while (1) {
			gpio_pin_toggle_dt(&led);
			k_msleep(1000);
		}
	}

	dump(addr);
	printk("\nWatching link (plug/unplug the cable):\n");

	while (1) {
		uint16_t bmsr;
		bool link;

		(void)rd(addr, REG_BMSR);
		bmsr = rd(addr, REG_BMSR);
		link = (bmsr & BMSR_LINK) != 0;
		if (first || link != last_link) {
			report_link(addr);
			last_link = link;
			first = false;
		}

		gpio_pin_toggle_dt(&led);
		k_msleep(1000);
	}
	return 0;
}
