#!/Users/asritha/.local/pipx/venvs/pyocd/bin/python
"""
flash.py -- program the Zephyr image to the PIC32CZ CA80 boot flash via pyOCD.

IMPORTANT -- how this board boots:
  The factory boot ROM HALTS at a BKPT when a debug probe is attached (a
  hot-plug/debug feature). So the application only runs on a clean reset with
  the probe NOT actively halting the core. The workflow is therefore:

    1. ./flash.py            # program the image (probe attached, this halts)
    2. power-cycle the board # boots the app cleanly -> LED blinks, UART, etc.

  (Do NOT expect the app to start running while this script/the probe holds the
  core; that inherits the boot ROM's halted state. Just power-cycle to run.)

Usage:
  ./flash.py                    # program build/zephyr/zephyr.hex
  ./flash.py path/to/other.hex  # program a specific hex
"""

import sys

from pyocd.core.helpers import ConnectHelper
from pyocd.flash.file_programmer import FileProgrammer

TARGET = "pic32cz8110ca80208"
PACK = "/Users/asritha/Downloads/Microchip.PIC32CZ-CA80_DFP.1.7.219.pack"
DEFAULT_HEX = "/Users/asritha/mega-can-fw/build/zephyr/zephyr.hex"


def main():
    hexfile = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_HEX

    opts = {
        "target_override": TARGET,
        "pack": PACK,
        "frequency": 2000000,
        "connect_mode": "halt",
        "resume_on_disconnect": False,
    }
    session = ConnectHelper.session_with_chosen_probe(options=opts)
    if session is None:
        print("no debug probe found", file=sys.stderr)
        return 2

    with session:
        t = session.board.target
        t.reset_and_halt()
        print(f"[flash] programming {hexfile}")
        FileProgrammer(session, chip_erase="sector").program(hexfile)
        print("[flash] done.")

    print("\n>>> Now POWER-CYCLE the board to run the app <<<")
    print("    (the boot ROM halts while the probe is attached; a clean")
    print("     power-on boots the app normally -- LED blinks, UART active).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
