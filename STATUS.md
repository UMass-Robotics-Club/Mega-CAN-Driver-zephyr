# Mega-CAN firmware status

Last updated 2026-09-28, evening (Claude session "GIGE WHOLE"). **Nothing here has run on
the real board yet.** Everything below builds; none of it is bench-tested.

## Where things stand

| Item | State | Where |
| --- | --- | --- |
| `main` (Om's CAN patch) merged into `joa/gige` | done, `84abb51` | git |
| Bring-up test moved `app/` -> `bringup/` | done, byte-identical, builds (38 KB) | `bringup/` |
| UDP protocol v3 for the software team | done, fake board + client tested on PC | `udp_protocol/` |
| PHY probe: straps, reset, MDIO scan, link report | builds (26 KB), **not run on hardware** | `phyprobe/` |
| **Ethernet driver port (GMII + MII)** | **builds, not run on hardware** | `module/` |
| Ethernet test app: ping, UDP echo, net shell | builds (166 KB) | `ethtest/` |
| Board fixes (pinctrl, RAM layout, Ethernet DT) | done, all 3 apps rebuild clean | `boards/microchip/mega_can/` |
| Docs | README, `module/README.md`, `CLAUDE.md` x2, `PHYSICAL_CONCERNS.md` | — |
| Robstride frame code + PC unit tests | done, tests pass (manual's example frames) | `app/src/robstride.*`, `app/tests/` |
| CAN1-CAN4 in the board (pins, transceivers, clocks, message RAM) | done, disabled by default | `mega_can.dts`, `-pinctrl.dtsi` |
| Motor bench test: ID, CAN_TIMEOUT, enable, damping, stop | builds (45 KB), **not run on hardware** | `motortest/` |
| **Gateway firmware: UDP v3 <-> 4 motors, watchdog, arming** | **builds (189 KB), not run on hardware** | `app/` |
| C wire structs proven equal to `protocol.py` (both directions) | done | `app/src/proto.h`, `make -C app/tests` |

Uncommitted, for you to review, commit and push: `.gitignore`, `README.md`, `CLAUDE.md`,
`STATUS.md`, `boards/` (3 files), `app/` (new gateway + tests; the old files moved to
`bringup/`), `bringup/`, `module/`, `phyprobe/`, `ethtest/`, `motortest/`, `udp_protocol/`,
`GIGE.pdf`, `IMPORTED_CONTEXT.md`. (`app/ethernet_message.hpp` is superseded by
`app/src/proto.h`; still there for you to delete or keep.)

## Changed in your board files today (review these first)

| File | Change | Why |
| --- | --- | --- |
| `mega_can-pinctrl.dtsi` | comment: PA22 is the switch select, PA23 is ETH_INT_N | old comment was a transcription error |
| | PD12 `K` -> `L` | K = RMII ref clock; GMII/MII RX clock is L (GRXCK), datasheet Table 6-11 |
| | PD5 out of the default group; new `gmac_txck_mii` / `gmac_txck_gmii` | driver switches it per link speed |
| | PA3/PA4 out of the group | MDIO is bit-banged on them |
| `mega_can-gmac.dtsi` | rewritten: new driver, `reg` 0x2000, PLL1 125 MHz, PHY at 3, straps, switch GPIO, MAC address, bit-bang MDIO | the old draft predated the schematic work |
| `mega_can.dts` | `sram0` = 0x20020000, 872 KiB (was 0x20000000, 1016 KiB) | first 128 KiB is DTCM, which DMA can't reach; room for 6 CAN message RAMs |
| | can1-can4 + transceivers (STB) + clocks, all disabled | motor buses; can0/can5 untouched |

The Ethernet DT file is **not** included by the board, only by `ethtest/`, so
`bringup/` gets no PLL1 and no Ethernet pins.

## Bench plan for tomorrow (in this order)

**1. `bringup/`**: still does its CAN job after the RAM move.
```
.venv/bin/python flash.py bringup/build/zephyr/zephyr.hex    # then power-cycle
```

**2. `phyprobe/`**: is the Ethernet section alive?
```
.venv/bin/python flash.py phyprobe/build/zephyr/zephyr.hex   # power-cycle, UART 115200, plug cable in
```

| You see | Means |
| --- | --- |
| `PASS ... address 3` (exactly one hit) | PHY alive, straps worked |
| PASS at another address | PHY alive; probe says which strap bit didn't take -> change `reg` in `mega_can-gmac.dtsi` |
| all `0xFFFF` | nobody answered: PHY power, 25 MHz osc, RESET_N, MDC/MDIO wiring |
| all `0x0000` | MDIO stuck low |
| `WARNING: MODEx reads ...` | a strap line is shorted or loaded |
| `link UP: 1000BASE-T` | cable + PHY + far end all do gigabit |

**3. `ethtest/`**: only if phyprobe passes. Laptop/Jetson port = 192.168.10.1/24.
```
.venv/bin/python flash.py ethtest/build/zephyr/zephyr.hex    # power-cycle
# on the laptop:
ping 192.168.10.2
echo hello | nc -u -w1 192.168.10.2 7
# on the board's UART shell:
net iface      # link + speed
net stats      # CRC / alignment errors should stay 0
```
**4. `motortest/`**: one Robstride per bus on can0-can3, factory ID 0x7F.
Shaft free, 120 ohm at both ends of each bus, **remove the can0<->can5 loopback
wire** if a motor is on can0.
```
.venv/bin/python flash.py motortest/build/zephyr/zephyr.hex  # power-cycle
```
Per bus it prints: who answered (or "no motor"), CAN_TIMEOUT read-back (should say
100 ms), feedback while enabled, 3 s of damping (turn the shaft: it should resist),
then stop. Faults are decoded by name.

**5. `app/`**: the real gateway. Needs 2-4 passing. Then from the laptop/Jetson:
```
.venv/bin/python flash.py app/build/zephyr/zephyr.hex        # power-cycle
cd udp_protocol && python3 example_client.py --seconds 10    # defaults to 192.168.10.2
```
The example client arms (all-off), then swings every joint in a slow sine with
kp 20 / kd 0.5. **Motors will move**: shafts free, or lower the gains first. The LED
blinks slowly while disarmed, fast while armed. Kill the client and within 50 ms
the watchdog stops every motor (and each motor's own 100 ms CAN_TIMEOUT backs it up).

If 1000 misbehaves (link up but no ping, or CRC errors), force 100 on the laptop
(`ethtool -s <if> speed 100 duplex full autoneg off`) to split "gigabit clock path"
problems from "driver" problems.

Rebuild any app: `PATH=$PWD/.venv/bin:$PATH python -m west build -p always -b mega_can <app> -d <app>/build`

## What the driver port is

`module/drivers/ethernet/eth_mchp_pic32cz.c`, a copy of Zephyr's Microchip GMAC driver
(for the PIC32CX SG) with every change marked `PIC32CZ:`. See `module/README.md`.
In short: a generated name-mapping header (all 71 hardware names), the wrapper enable,
GMII (gigabit mode + 125 MHz clock + safe transmit-clock switching), the strap/reset hook,
and a build error if the D-cache is on (packet buffers aren't cache-maintained yet).

Known gaps, deliberately left for after first light:
- D-cache off for Ethernet builds (costs CPU speed, removes a class of bugs).
- Hardware MDIO not ported (bit-bang instead).
- One queue / one interrupt, like upstream.
- Test RNG instead of a hardware TRNG.

## Waiting on you

1. **Motors**: which bus is each motor on? `motortest` tries can0-can3 and reports.
   **Sensors**: foot sensor and ToF details (list in chat, 2026-09-29).
2. **Bench results** from the three steps above (UART output is enough).

## Plan after the bench

1. Fix whatever the bench shows (straps, address, clock path).
3. `app/`: foot sensors (frame format TBD with the Feather), ToF SPI, power.
4. Driver hardening: D-cache maintenance, hardware MDIO, TRNG; then consider an upstream PR.
5. Watchdog/safety: MCU hardware WDT (the CAN patch removed the `wdt` node), Robstride
   `CAN_TIMEOUT`.
