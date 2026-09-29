# Mega-CAN UDP protocol tools

Python tools for the Jetson <-> Mega-CAN UDP link (**protocol v3, draft**).
No dependencies beyond Python 3.8+.

| File | What it is |
| --- | --- |
| `protocol.py` | Packet layout and pack/unpack helpers. **Source of truth for the byte layout**; the firmware must match it. |
| `fake_board.py` | A fake Mega-CAN board: 4 simulated Robstride 02 motors (incl. temperature and over-temp fault), foot pressure, ToF/magnetometer and power, with the real watchdog and re-arm rules. |
| `example_client.py` | A Jetson-side example: arms the board, moves every joint in a sine, optionally logs every state packet to CSV. |

**Status (2026-09-29):** the board firmware (`../app/`) implements this protocol
and builds, but has **not run on the real board yet**. Keep writing and testing
against `fake_board.py`; the real board speaks the same bytes (checked by
`make -C app/tests`, which packs in Python and decodes in the firmware's C structs,
and the other way round). Feet, ToF and power read as "no data" from the real
board until their hardware is wired up.

## Try it on one computer

```
python3 fake_board.py --host 127.0.0.1
python3 example_client.py --board 127.0.0.1 --seconds 10 --csv run.csv
```

On the real network the board is `192.168.10.2`, UDP port `5000` (the client's
defaults). Give the Jetson `192.168.10.1/24` on that interface.

## How the link behaves

- The Jetson sends **one command packet per control tick** (500 Hz in the example).
- The board answers **every valid command with one state packet**, sent back to the
  address and port the command came from.
- **Watchdog:** 50 ms without a valid command -> all motors off, `FLAG_WATCHDOG_TRIPPED` set.
- **Arming:** at boot and after a watchdog trip, `MODE_RUN` is ignored until the board
  receives one packet with every motor `MODE_OFF`.
- **Sequence numbers:** commands with `seq <=` the last accepted one are dropped. A
  watchdog trip resets this, so a restarted Jetson program can start again at 1.
- `MODE_SET_ZERO` acts once, on the tick a motor's slot changes to it.
- The board **clamps** commands to the Robstride 02 limits: position ±4π rad,
  velocity ±44 rad/s, Kp 0-500, Kd 0-5, torque ±17 N·m (`P_MAX`... in `protocol.py`).

## Byte layout (v3)

Everything is **little-endian**, no gaps except the ones marked pad.
Floats are IEEE-754 32-bit.

### Header (16 bytes, both packets)

| Offset | Type | Field |
| --- | --- | --- |
| 0 | char[4] | magic `"MCAN"` |
| 4 | u8 | version = 3 |
| 5 | u8 | type: 1 = command, 2 = state |
| 6 | u16 | motor count = 4 |
| 8 | u32 | seq (sender's own counter) |
| 12 | u32 | time_us (sender's clock) |

### Command packet, Jetson -> board (112 bytes)

Header, then 4 motor slots of 24 bytes at offsets 16, 40, 64, 88:

| +Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | mode: 0 off, 1 run, 2 set zero |
| 1 | 3 pad | |
| 4 | f32 | p_des (rad) |
| 8 | f32 | v_des (rad/s) |
| 12 | f32 | kp |
| 16 | f32 | kd |
| 20 | f32 | tau_ff (N·m) |

Motor torque = kp·(p_des − p) + kd·(v_des − v) + tau_ff, computed inside the motor.

### State packet, board -> Jetson (268 bytes)

| Offset | Size | Block |
| --- | --- | --- |
| 0 | 16 | header |
| 16 | 8 | u32 last_cmd_seq, u32 flags (bit0 watchdog tripped, bit1 bad packet) |
| 24 | 4 × 20 | motors |
| 104 | 6 × 4 | CAN bus health |
| 128 | 4 × 12 | feet |
| 176 | 4 × 20 | ToF / magnetometer |
| 256 | 12 | power |

Motor slot (20 bytes):

| +Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | status: 0 off, 1 running, 2 fault, 3 no reply |
| 1 | u8 | CAN bus (0-5) |
| 2 | u8 | motor CAN ID |
| 3 | u8 | fault bits: 0 undervoltage, 1 overcurrent, 2 over-temp, 3 encoder, 4 overload, 5 uncalibrated |
| 4 | f32 | position (rad) |
| 8 | f32 | velocity (rad/s) |
| 12 | f32 | torque (N·m) |
| 16 | f32 | winding temperature (°C) |

Bus health slot (4 bytes): u8 state (0 ok, 1 warning, 2 error-passive, 3 bus-off),
u8 tx error count, u8 rx error count, 1 pad.

Foot slot (12 bytes): u8 status (0 no data, 1 ok, 2 stale, 3 fault), 1 pad,
u16 age_ms, f32 force (N), u32 raw sensor reading.

ToF slot (20 bytes): u8 status, 1 pad, u16 age_ms, f32 distance (m),
f32 mag x, y, z (µT).

Power (12 bytes): f32 bus voltage (V), f32 bus current (A), u32 power-good bits
(bit0 battery, bit1 motor rail, bit2 logic rail).

## Still placeholders (may change before v3 is final)

- Foot count (4) and units; foot sensors are CAN boards whose frame format is not settled.
- ToF count (4) and fields.
- Power block: not measured by the board yet; reads as zeros from real hardware.
- Motor-to-bus mapping (motor *i* on bus *i*, CAN ID *i*+1). Use the `bus` and `can_id`
  fields rather than hard-coding it.
