# mega_can Zephyr module

Drivers this board needs that Zephyr doesn't have yet. The layout mirrors
Zephyr's own tree (`drivers/`, `dts/bindings/`), so the files can move into a
Zephyr pull request later with little reshuffling.

## Using it

In an app's `CMakeLists.txt`, before `find_package(Zephyr ...)`:

```cmake
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../module)
```

Zephyr then reads `zephyr/module.yml`, which adds this folder's `CMakeLists.txt`,
`Kconfig` and `dts/bindings`. This workspace uses west's T1 layout (zephyr is the
manifest repo), so the module is not listed in a west manifest; if the workspace
ever moves to its own manifest repo (T2/T3), list this repo there instead and
drop the CMake line.

## Contents

### `drivers/ethernet/eth_mchp_pic32cz.c` (compatible `microchip,pic32cz-eth`)

Port of Zephyr's `drivers/ethernet/eth_mchp_gmac_g1.c` (PIC32CX SG) to the
PIC32CZ CA "ETH" block. Every change is marked `PIC32CZ:` in the source.

- **Registers** (`gmac_pic32cz_compat.h`): the CA's ETH block is a small wrapper
  (CTRLA, CTRLB, EVCTRL, SYNCB, WPCTRL) plus the same Cadence GEM core as the
  SG, at +0x1000. All 71 hardware names the driver uses map to `ETH_*`: 64 one
  to one, 7 field values written out from the datasheet.
- **Wrapper**: software reset, then CTRLB (GMII select, gigabit clock request),
  then CTRLA.ENABLE, waiting on SYNCB.
- **GMII** (new; upstream is MII/RMII only): `NCFGR.GIGE` per link speed, and
  the transmit-clock path switched on every link change. At 1000 the MAC drives
  GTXCK; at 10/100 the PHY drives TX_CLK. Optional `txclk-sel-gpios` +
  `"mii"`/`"gmii"` pinctrl states for boards that share the pin through an
  analog switch. The switching order never lets both sides drive at once.
- **PHY strap hook**: `phy-reset-gpios`, `phy-strap-gpios`, `phy-strap-values`.
  Resets the PHY with MCU pulls on strap pins that have no resistor, before
  MDIO/PHY/ETH start.
- **D-cache**: not handled yet. Upstream targeted a Cortex-M4 without a data
  cache; on the M7 the packet buffers would need flush/invalidate (pattern:
  `eth_sam_gmac.c`). The driver `#error`s unless `CONFIG_DCACHE=n`.

Hardware MDIO (`mdio_mchp_gmac_g1.c`) is not ported yet; mega_can bit-bangs
MDIO with Zephyr's `zephyr,mdio-gpio`.

Status: builds. **Not yet run on hardware.**
