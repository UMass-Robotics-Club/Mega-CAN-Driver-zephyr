"""
Mega-CAN <-> Jetson UDP protocol, version 3 (draft).

This file is the single source of truth for the packet layout on the Python
side. The firmware must match it byte for byte.

  Jetson -> board : command packet (type 1), 112 bytes, to UDP port 5000
  board -> Jetson : state packet   (type 2), 268 bytes, back to the sender

All values are little-endian. Units are SI (rad, rad/s, N*m, N, m, uT, V, A, degC).

v3 changes from v2: 12 motors -> 4 (Robstride 02, one per CAN bus); each
motor state slot gains fault bits and winding temperature.
"""

import struct
from dataclasses import dataclass, field
from typing import List

MAGIC = b"MCAN"
VERSION = 3
TYPE_COMMAND = 1
TYPE_STATE = 2

BOARD_PORT = 5000
BOARD_IP = "192.168.10.2"

NUM_MOTORS = 4
NUM_BUSES = 6
NUM_FEET = 4
NUM_TOF = 4

# Robstride 02 command limits (private protocol, manual section 4.1.2).
# The board clamps every command to these before it goes on the CAN bus.
P_MAX = 12.566370614359172  # 4*pi rad
V_MAX = 44.0                # rad/s
KP_MAX = 500.0
KD_MAX = 5.0
T_MAX = 17.0                # N*m

# Motor modes (command)
MODE_OFF = 0
MODE_RUN = 1
MODE_SET_ZERO = 2

# Motor status (state)
MOTOR_OFF = 0
MOTOR_RUNNING = 1
MOTOR_FAULT = 2
MOTOR_NO_REPLY = 3

# Motor fault bits (state). Same order as bits 16..21 of the Robstride
# type-2 feedback frame ID, so the firmware copies them straight across.
FAULT_UNDERVOLTAGE = 1 << 0
FAULT_OVERCURRENT = 1 << 1
FAULT_OVERTEMP = 1 << 2
FAULT_ENCODER = 1 << 3
FAULT_OVERLOAD = 1 << 4
FAULT_UNCALIBRATED = 1 << 5

# Sensor status (feet, ToF)
SENSOR_NO_DATA = 0
SENSOR_OK = 1
SENSOR_STALE = 2
SENSOR_FAULT = 3

# State flags
FLAG_WATCHDOG_TRIPPED = 1 << 0
FLAG_BAD_PACKET = 1 << 1

# Power-good bits
PGOOD_BATTERY = 1 << 0
PGOOD_MOTOR_RAIL = 1 << 1
PGOOD_LOGIC_RAIL = 1 << 2

HEADER = struct.Struct("<4sBBHII")    # magic, version, type, count, seq, time_us
CMD_MOTOR = struct.Struct("<B3xfffff")  # mode, p_des, v_des, kp, kd, tau_ff
ST_EXTRA = struct.Struct("<II")       # last_cmd_seq, flags
ST_MOTOR = struct.Struct("<BBBBffff") # status, bus, can_id, faults, pos, vel, torque, temp_c
ST_BUS = struct.Struct("<BBBx")       # state, tx_err, rx_err
ST_FOOT = struct.Struct("<BxHfI")     # status, age_ms, force_n, raw
ST_TOF = struct.Struct("<BxHffff")    # status, age_ms, distance_m, mag_x/y/z_ut
ST_POWER = struct.Struct("<ffI")      # bus_voltage_v, bus_current_a, pgood

COMMAND_SIZE = HEADER.size + NUM_MOTORS * CMD_MOTOR.size
STATE_SIZE = (HEADER.size + ST_EXTRA.size + NUM_MOTORS * ST_MOTOR.size +
              NUM_BUSES * ST_BUS.size + NUM_FEET * ST_FOOT.size +
              NUM_TOF * ST_TOF.size + ST_POWER.size)

assert COMMAND_SIZE == 112
assert STATE_SIZE == 268


@dataclass
class MotorCommand:
    mode: int = MODE_OFF
    p_des: float = 0.0
    v_des: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    tau_ff: float = 0.0


@dataclass
class MotorState:
    status: int = MOTOR_OFF
    bus: int = 0
    can_id: int = 0
    faults: int = 0
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    temperature_c: float = 0.0


@dataclass
class BusHealth:
    state: int = 0  # 0 ok, 1 warning, 2 error-passive, 3 bus-off
    tx_err: int = 0
    rx_err: int = 0


@dataclass
class FootState:
    status: int = SENSOR_NO_DATA
    age_ms: int = 0
    force_n: float = 0.0
    raw: int = 0


@dataclass
class TofState:
    status: int = SENSOR_NO_DATA
    age_ms: int = 0
    distance_m: float = 0.0
    mag_ut: tuple = (0.0, 0.0, 0.0)


@dataclass
class PowerState:
    bus_voltage_v: float = 0.0
    bus_current_a: float = 0.0
    pgood: int = 0


@dataclass
class State:
    seq: int = 0
    time_us: int = 0
    last_cmd_seq: int = 0
    flags: int = 0
    motors: List[MotorState] = field(
        default_factory=lambda: [MotorState() for _ in range(NUM_MOTORS)])
    buses: List[BusHealth] = field(
        default_factory=lambda: [BusHealth() for _ in range(NUM_BUSES)])
    feet: List[FootState] = field(
        default_factory=lambda: [FootState() for _ in range(NUM_FEET)])
    tof: List[TofState] = field(
        default_factory=lambda: [TofState() for _ in range(NUM_TOF)])
    power: PowerState = field(default_factory=PowerState)


def pack_command(seq, time_us, cmds):
    out = HEADER.pack(MAGIC, VERSION, TYPE_COMMAND, NUM_MOTORS,
                      seq & 0xFFFFFFFF, time_us & 0xFFFFFFFF)
    for c in cmds:
        out += CMD_MOTOR.pack(c.mode, c.p_des, c.v_des, c.kp, c.kd, c.tau_ff)
    return out


def unpack_command(data):
    """Returns (seq, time_us, [MotorCommand]) or raises ValueError."""
    if len(data) != COMMAND_SIZE:
        raise ValueError(f"bad command size {len(data)}")
    magic, ver, typ, count, seq, time_us = HEADER.unpack_from(data, 0)
    if magic != MAGIC or ver != VERSION or typ != TYPE_COMMAND or count != NUM_MOTORS:
        raise ValueError("bad command header")
    cmds = [MotorCommand(*CMD_MOTOR.unpack_from(data, HEADER.size + i * CMD_MOTOR.size))
            for i in range(NUM_MOTORS)]
    return seq, time_us, cmds


def pack_state(s):
    out = HEADER.pack(MAGIC, VERSION, TYPE_STATE, NUM_MOTORS,
                      s.seq & 0xFFFFFFFF, s.time_us & 0xFFFFFFFF)
    out += ST_EXTRA.pack(s.last_cmd_seq & 0xFFFFFFFF, s.flags)
    for m in s.motors:
        out += ST_MOTOR.pack(m.status, m.bus, m.can_id, m.faults, m.position,
                             m.velocity, m.torque, m.temperature_c)
    for b in s.buses:
        out += ST_BUS.pack(b.state, min(b.tx_err, 255), min(b.rx_err, 255))
    for f in s.feet:
        out += ST_FOOT.pack(f.status, min(f.age_ms, 0xFFFF), f.force_n, f.raw)
    for t in s.tof:
        out += ST_TOF.pack(t.status, min(t.age_ms, 0xFFFF), t.distance_m, *t.mag_ut)
    out += ST_POWER.pack(s.power.bus_voltage_v, s.power.bus_current_a, s.power.pgood)
    return out


def unpack_state(data):
    """Returns a State or raises ValueError."""
    if len(data) != STATE_SIZE:
        raise ValueError(f"bad state size {len(data)}")
    magic, ver, typ, count, seq, time_us = HEADER.unpack_from(data, 0)
    if magic != MAGIC or ver != VERSION or typ != TYPE_STATE or count != NUM_MOTORS:
        raise ValueError("bad state header")
    s = State(seq=seq, time_us=time_us)
    off = HEADER.size
    s.last_cmd_seq, s.flags = ST_EXTRA.unpack_from(data, off)
    off += ST_EXTRA.size
    for i in range(NUM_MOTORS):
        s.motors[i] = MotorState(*ST_MOTOR.unpack_from(data, off))
        off += ST_MOTOR.size
    for i in range(NUM_BUSES):
        s.buses[i] = BusHealth(*ST_BUS.unpack_from(data, off))
        off += ST_BUS.size
    for i in range(NUM_FEET):
        s.feet[i] = FootState(*ST_FOOT.unpack_from(data, off))
        off += ST_FOOT.size
    for i in range(NUM_TOF):
        status, age, dist, mx, my, mz = ST_TOF.unpack_from(data, off)
        s.tof[i] = TofState(status, age, dist, (mx, my, mz))
        off += ST_TOF.size
    s.power = PowerState(*ST_POWER.unpack_from(data, off))
    return s
