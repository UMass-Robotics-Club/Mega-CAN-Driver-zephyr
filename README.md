# mega-can-fw — Zephyr workspace for the PIC32CZ CA80 custom board

Custom Zephyr board **`mega_can`** (based on Microchip's `pic32cz_ca80_cult`)
that sets up CAN 0 and 5 and a UART console on **SERCOM0** to allow for bridged testing.

## Layout

```
mega-can-fw/
├── zephyr/  modules/  ...         Zephyr + HAL (west workspace)
├── boards/microchip/mega_can/     the custom board definition
│   └── mega_can-gmac.dtsi         Ethernet (MAC, PHY, clocks); included by Ethernet apps
├── module/                        Zephyr module: drivers Zephyr doesn't have yet
│   └── drivers/ethernet/          PIC32CZ ETH (GMAC) driver, GMII + MII
├── bringup/                       CAN FD bridge hardware test (can0 <-> can5)
├── phyprobe/                      Ethernet PHY test: straps, MDIO scan, link speed
├── ethtest/                       Ethernet test: ping + UDP echo on port 7, net shell
├── app/                           the real gateway firmware (not started yet)
├── udp_protocol/                  Jetson <-> board UDP protocol (Python) + fake board
├── flash.py                       program the image (then power-cycle to run)
└── STATUS.md                      where the work stands
```

## Board: `mega_can`

- SoC: `pic32cz8110ca80208` (Cortex-M7), same as the Curiosity Ultra.
- **LED**: `led0` = **PB31**, active-high (`gpio-leds`).
- **UART console**: **SERCOM0**, `TX = PC0` (PAD0), `RX = PC3` (PAD3), 115200 8N1.
  - Note: SERCOM USART TXD is fixed to PAD0 in hardware, so TX must be PC0.
    (The original request of "TX on C3" is not possible; see the pinctrl file.)
- **Clock**: **external 24 MHz oscillator → DPLL0 → 240 MHz** CPU (verified
  ~241 MHz). The 24 MHz part is an *active* oscillator, so XOSC is in
  external-clock mode (`xosc-xtal-en = 0`).
  - The **SERCOM/UART is clocked from a divided DPLL** (`gclkgen1` =
    `pll0-clkout0 / 5 = 48 MHz`), not the raw 240 MHz `gclk0`. 240 MHz
    over-clocks the SERCOM (garbled output). Using the *already-running* DPLL
    (rather than a separate oscillator) also means the SERCOM clock is ready
    the instant SERCOM inits, which avoids a boot-time fault — see below.
- **Timing**: uses real **`k_msleep()`** — the kernel timer works correctly
  when the app is booted normally (see "How this board boots").

## Setup Workspace

### venv
First make a python virtual env 
```
# Using normal python (Note uv will not work for zephyr!)
python3 -m venv .venv
```

Then activate it and install pyocd along with west (you can just use requirements.txt)
```
source .venv/bin/activate
pip install -r requirements.txt
```

### Zephyr
To setup Zephyr follow the steps in the [Zephyr getting started guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html).

Once you have done that, you will have to apply some diffs to get CAN working for our custom board. Run the following command to apply the diff file to your newly created Zephyr workspace:
```
cd zephyr
git apply ../zephyr-can-patch.diff
```

Now Zephyr should be setup and ready to use!

## Build & run

From the repo root, pick an app and give it its own build directory:
```
west build -p always -b mega_can bringup  -d bringup/build
west build -p always -b mega_can phyprobe -d phyprobe/build
west build -p always -b mega_can ethtest  -d ethtest/build
./flash.py bringup/build/zephyr/zephyr.hex
```

Then **power-cycle the board** to run it (see "How this board boots").

Ethernet apps also load this repo's Zephyr module (`module/`) through
`ZEPHYR_EXTRA_MODULES` in their `CMakeLists.txt`; no west manifest change is
needed. See `module/README.md`.

If `west` says "command not found" after the repo moved folders, the venv's
scripts point at the old path: use `python -m west ...` / `.venv/bin/python
flash.py ...`, or recreate the venv.


## How this board boots — IMPORTANT

The **factory boot ROM halts at a `BKPT` when a debug probe is attached** (a
hot-plug/debug feature: it hands control to the debugger instead of running the
app). So:

- **Programming** the image works normally with the probe attached
  (`./flash.py` / `west flash`) — pyOCD + the Microchip DFP pack.
- **Running** the app requires a **clean power-cycle with the probe NOT holding
  the core**. On a normal power-on (no debugger halting it) the boot ROM
  launches the app in thread mode and everything works — LED blinks at 1 Hz
  via `k_msleep`, UART active.
- Do **not** try to "run" the app by resuming it under the debugger: that
  inherits the boot ROM's halted/exception state, which masks interrupts
  (SysTick never fires) and makes `k_sleep` misbehave. This is a debugger
  artifact, **not** a real firmware bug — power-cycle and it runs correctly.

Workflow:

```
./flash.py            # program (probe attached)
# then power-cycle the board -> app runs
screen /dev/cu.usbmodem102 115200   # (probe UART) watch heartbeats
```

To debug interactively (breakpoints etc.), use `west debug` / gdb, which loads
and runs the app under the debugger's control.

## Notes

- `k_msleep()` / `k_sleep()` work correctly on a normal boot (verified: steady
  1 Hz blink from `k_msleep(500)`). Earlier "k_sleep broken" symptoms were
  entirely the debugger-halts-boot-ROM artifact above.
- Keep peripheral clocks that are ready early: this board clocks the SERCOM
  from the already-running DPLL (`gclkgen1`) so nothing waits on a slow clock
  during init.
