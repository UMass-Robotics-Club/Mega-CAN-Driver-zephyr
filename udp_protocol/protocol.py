"""
Mega-CAN <-> Jetson UDP protocol, version 4 (draft, MIT motor protocol).

This file is the single source of truth for the packet layout on the Python
side. The firmware must match it byte for byte.

  Jetson -> board : command packet (type 1), 112 bytes, to UDP port 5000
  board -> Jetson : state packet   (type 2), 284 bytes, back to the sender

All values are little-endian. Units are SI (rad, rad/s, N*m, N, m, uT, V, A, degC).

v3 changes from v2: 12 motors -> 4 (Robstride 02, one per CAN bus); each
motor state slot gains fault bits and winding temperature.
v4 changes from v3: motors speak the Robstride MIT protocol; motor state slot
gains op_flags and a 32-bit fault word (+4 bytes per motor).
"""

import struct
from dataclasses import dataclass, field
from typing import List

MAGIC = b"MCAN"
VERSION = 4
TYPE_COMMAND = 1
TYPE_STATE = 2

BOARD_PORT = 5000
BOARD_IP = "192.168.10.2"

NUM_MOTORS = 4
NUM_BUSES = 6
NUM_FEET = 4
NUM_TOF = 4

#---------------------------------------------------------------
# MIT Protocol
#---------------------------------------------------------------

# Robstride 02 command limits (MIT protocol, manual section 6).
# The board clamps every command to these before it goes on the CAN bus.
# The MIT quantizer maps the position range onto 16 bits, so P_MIN/P_MAX must
# be exactly symmetric and identical on the Jetson and in the firmware.
P_MAX = 12.57               # rad
V_MAX = 44.0                # rad/s
KP_MAX = 500.0
KD_MAX = 5.0
T_MAX = 17.0                # N*m

P_MIN = -12.57              # rad  (was -4*pi: asymmetric span skews every position)
V_MIN = -44.0               # rad/s
KP_MIN = 0.0
KD_MIN = 0.0
T_MIN = -17.0               # N*m

# Communication protocol with motor (one-time provisioning, not per-command)
PROTOCOL_PRIV = 0
PROTOCOL_CANOPEN = 1
PROTOCOL_MIT = 2

# Motor modes (command)
MODE_OFF = 0                # board sends Stop    (FF..FD)
MODE_RUN = 1                # board sends Enable (FF..FC) once, then MIT frame every cycle
MODE_SET_ZERO = 2           # FF..FE
MODE_CLEAR_FAULT = 3        # FF..FF FB

# Motor status (state)
MOTOR_OFF = 0
MOTOR_RUNNING = 1
MOTOR_FAULT = 2
MOTOR_NO_REPLY = 3

# Fault word (state, uint32), from the Command 5 response (manual 6.8)
FAULT_OVERTEMP = 1 << 0
FAULT_DRIVERCHIP = 1 << 1
FAULT_UNDERV = 1 << 2
FAULT_OVERV = 1 << 3
FAULT_OVERI_PHASE_B = 1 << 4
FAULT_OVERI_PHASE_C = 1 << 5
FAULT_ENCODER = 1 << 7              # encoder uncalibrated
FAULT_HARDWARE_ID = 1 << 8
FAULT_POSINIT = 1 << 9
FAULT_OVERLOAD_PROTECTION = 1 << 14
FAULT_OVERI_PHASE_A = 1 << 16

# op_flags byte (state), from feedback byte 6
OP_MODE_MASK = 0b11         # 0 reset, 1 cali, 2 motor
OP_WARNING = 1 << 2         # feedback bit 4
OP_FAULT = 1 << 3           # feedback bit 5

# MIT constant frames (8 data bytes)
MIT_ENABLE      = b"\xFF" * 7 + b"\xFC"
MIT_STOP        = b"\xFF" * 7 + b"\xFD"
MIT_ZERO        = b"\xFF" * 7 + b"\xFE"
MIT_CLEAR_FAULT = b"\xFF" * 7 + b"\xFB"      # F_CMD = 0xFF
MIT_READ_FAULT  = b"\xFF" * 6 + b"\x00\xFB"  # F_CMD != 0xFF

#---------------------------------------------------------------
# MIT codec (reference implementation; firmware must match byte for byte)
#---------------------------------------------------------------

def _q(x, lo, hi, bits):
    x = min(max(x, lo), hi)
    return int(round((x - lo) * ((1 << bits) - 1) / (hi - lo)))


def _dq(u, lo, hi, bits):
    return lo + u * (hi - lo) / ((1 << bits) - 1)


def mit_pack(pos, vel, kp, kd, tau):
    """Command -> 8 data bytes (manual 6.5)."""
    a = _q(pos, P_MIN, P_MAX, 16)
    s = _q(vel, V_MIN, V_MAX, 12)
    k = _q(kp, KP_MIN, KP_MAX, 12)
    d = _q(kd, KD_MIN, KD_MAX, 12)
    t = _q(tau, T_MIN, T_MAX, 12)
    return bytes([a >> 8, a & 0xFF, s >> 4, ((s & 0xF) << 4) | (k >> 8),
                  k & 0xFF, d >> 4, ((d & 0xF) << 4) | (t >> 8), t & 0xFF])


def mit_unpack_command(d):
    """8 data bytes -> (pos, vel, kp, kd, tau): what the motor actually receives."""
    a = (d[0] << 8) | d[1]
    s = (d[2] << 4) | (d[3] >> 4)
    k = ((d[3] & 0xF) << 8) | d[4]
    kd = (d[5] << 4) | (d[6] >> 4)
    t = ((d[6] & 0xF) << 8) | d[7]
    return (_dq(a, P_MIN, P_MAX, 16), _dq(s, V_MIN, V_MAX, 12),
            _dq(k, KP_MIN, KP_MAX, 12), _dq(kd, KD_MIN, KD_MAX, 12),
            _dq(t, T_MIN, T_MAX, 12))


def mit_unpack_feedback(d):
    """8 data bytes -> (motor_id, pos, vel, torque, op_flags, temp_c) (manual 6.1)."""
    a = (d[1] << 8) | d[2]
    s = (d[3] << 4) | (d[4] >> 4)
    q = ((d[4] & 0xF) << 8) | d[5]
    flags = (d[6] >> 6) & OP_MODE_MASK
    flags |= OP_WARNING if d[6] & 0x10 else 0
    flags |= OP_FAULT if d[6] & 0x20 else 0
    temp = (((d[6] & 0xF) << 8) | d[7]) / 10.0
    return (d[0], _dq(a, P_MIN, P_MAX, 16), _dq(s, V_MIN, V_MAX, 12),
            _dq(q, T_MIN, T_MAX, 12), flags, temp)


def mit_pack_feedback(motor_id, pos, vel, tau, op_flags, temp_c):
    """Motor-side feedback frame; inverse of mit_unpack_feedback (for simulators)."""
    a = _q(pos, P_MIN, P_MAX, 16)
    s = _q(vel, V_MIN, V_MAX, 12)
    q = _q(tau, T_MIN, T_MAX, 12)
    tt = min(max(int(round(temp_c * 10)), 0), 4095)
    b6 = ((op_flags & OP_MODE_MASK) << 6) | (tt >> 8)
    if op_flags & OP_FAULT:
        b6 |= 0x20
    if op_flags & OP_WARNING:
        b6 |= 0x10
    return bytes([motor_id, a >> 8, a & 0xFF, s >> 4,
                  ((s & 0xF) << 4) | (q >> 8), q & 0xFF, b6, tt & 0xFF])

#-----------------------------------------------------------

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


# Struct packing specifics
HEADER = struct.Struct("<4sBBHII")      # magic, version, type, count, seq, time_us
CMD_MOTOR = struct.Struct("<B3xfffff")  # mode, p_des, v_des, kp, kd, tau_ff
ST_EXTRA = struct.Struct("<II")         # last_cmd_seq, flags
ST_MOTOR = struct.Struct("<BBBBIffff")  # status, bus, can_id, op_flags, faults(u32), pos, vel, torque, temp_c
ST_BUS = struct.Struct("<BBBx")         # state, tx_err, rx_err
ST_FOOT = struct.Struct("<BxHfI")       # status, age_ms, force_n, raw
ST_TOF = struct.Struct("<BxHffff")      # status, age_ms, distance_m, mag_x/y/z_ut
ST_POWER = struct.Struct("<ffI")        # bus_voltage_v, bus_current_a, pgood


COMMAND_SIZE = HEADER.size + NUM_MOTORS * CMD_MOTOR.size
STATE_SIZE = (HEADER.size + ST_EXTRA.size + NUM_MOTORS * ST_MOTOR.size +
              NUM_BUSES * ST_BUS.size + NUM_FEET * ST_FOOT.size +
              NUM_TOF * ST_TOF.size + ST_POWER.size)

assert COMMAND_SIZE == 112
assert STATE_SIZE == 284


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
    op_flags: int = 0
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
        out += ST_MOTOR.pack(m.status, m.bus, m.can_id, m.op_flags,
                             m.faults & 0xFFFFFFFF, m.position,
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