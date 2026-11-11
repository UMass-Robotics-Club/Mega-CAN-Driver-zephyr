#!/usr/bin/env python3
"""
Fake Mega-CAN board: speaks the real UDP protocol with simulated motors and
sensors, so the Jetson/CS code can be written and tested before the hardware
works. Run it on a laptop (or on the Jetson itself) and point the client at it.

    python3 fake_board.py                 # listens on 0.0.0.0:5000
    python3 example_client.py --board 127.0.0.1

It follows the same safety rules the firmware will:
  - 50 ms without a valid command -> all motors off, watchdog flag set
  - after that (and at boot) mode RUN is ignored until one all-OFF packet
  - SET_ZERO acts once, on the tick a slot changes to it
  - packets with seq <= the last one used are dropped; a watchdog trip resets
    that, so a restarted client (seq back at 1) is accepted
"""

import argparse
import math
import random
import socket
import time

import protocol as p

WATCHDOG_S = 0.050
PHYS_DT = 0.0005  # physics sub-step, s

# Rough per-joint model: inertia (kg*m^2) and viscous friction (N*m*s/rad) are
# placeholders; the torque limit is the Robstride 02's.
INERTIA = 0.02
FRICTION = 0.05
TAU_MAX = p.T_MAX

# Rough winding-temperature model: heats with torque^2, cools toward ambient.
# Tuned so a sustained full-torque stall reaches the 135 C fault in ~1 minute.
AMBIENT_C = 25.0
HEAT_PER_NM2 = 0.0069   # degC/s per (N*m)^2
COOL_TAU_S = 60.0
OVERTEMP_C = 135.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def clamp_command(c):
    """Clamp to the Robstride 02 ranges, like the firmware does before CAN."""
    return p.MotorCommand(c.mode, clamp(c.p_des, -p.P_MAX, p.P_MAX),
                          clamp(c.v_des, -p.V_MAX, p.V_MAX),
                          clamp(c.kp, 0.0, p.KP_MAX), clamp(c.kd, 0.0, p.KD_MAX),
                          clamp(c.tau_ff, -p.T_MAX, p.T_MAX))


class FakeBoard:
    def __init__(self):
        self.state = p.State()
        self.armed = False
        self.last_modes = [p.MODE_OFF] * p.NUM_MOTORS
        self.cmds = [p.MotorCommand() for _ in range(p.NUM_MOTORS)]
        self.zero_offset = [0.0] * p.NUM_MOTORS
        self.q = [random.uniform(-0.2, 0.2) for _ in range(p.NUM_MOTORS)]
        self.qd = [0.0] * p.NUM_MOTORS
        self.tau = [0.0] * p.NUM_MOTORS
        self.temp = [AMBIENT_C] * p.NUM_MOTORS
        self.t0 = time.monotonic()
        self.last_cmd_time = None
        for i, m in enumerate(self.state.motors):
            m.bus = i               # one motor per CAN bus (can0..can3)
            m.can_id = i + 1        # motor CAN IDs 1..4

    def now_us(self):
        return int((time.monotonic() - self.t0) * 1e6)

    def trip_watchdog(self):
        if self.armed:
            print("watchdog: no command for 50 ms, motors off")
        self.armed = False
        self.state.flags |= p.FLAG_WATCHDOG_TRIPPED
        self.cmds = [p.MotorCommand() for _ in range(p.NUM_MOTORS)]
        # A restarted Jetson program starts seq over at 1: accept it.
        self.state.last_cmd_seq = 0

    def apply_command(self, seq, cmds):
        if seq <= self.state.last_cmd_seq and self.state.last_cmd_seq != 0:
            return False  # old or duplicate packet
        self.state.last_cmd_seq = seq
        if all(c.mode == p.MODE_OFF for c in cmds):
            if not self.armed:
                print("re-armed (all-off packet received)")
            self.armed = True
            self.state.flags &= ~p.FLAG_WATCHDOG_TRIPPED
        for i, c in enumerate(cmds):
            c = clamp_command(c)
            if c.mode == p.MODE_SET_ZERO and self.last_modes[i] != p.MODE_SET_ZERO:
                self.zero_offset[i] = self.q[i]
            if c.mode == p.MODE_RUN and not self.armed:
                c = p.MotorCommand()  # ignored until re-armed
            self.cmds[i] = c
            self.last_modes[i] = c.mode
        return True

    def step_physics(self, dt):
        n = max(1, int(dt / PHYS_DT))
        h = dt / n
        for _ in range(n):
            for i, c in enumerate(self.cmds):
                pos = self.q[i] - self.zero_offset[i]
                if c.mode == p.MODE_RUN:
                    tau = (c.kp * (c.p_des - pos) + c.kd * (c.v_des - self.qd[i])
                           + c.tau_ff)
                    tau = max(-TAU_MAX, min(TAU_MAX, tau))
                else:
                    tau = 0.0
                acc = (tau - FRICTION * self.qd[i]) / INERTIA
                self.qd[i] += acc * h
                self.q[i] += self.qd[i] * h
                self.tau[i] = tau
                self.temp[i] += (HEAT_PER_NM2 * tau * tau
                                 - (self.temp[i] - AMBIENT_C) / COOL_TAU_S) * h

    def fill_sensors(self):
        t = time.monotonic() - self.t0
        s = self.state
        for i, m in enumerate(s.motors):
            m.faults = p.FAULT_OVERTEMP if self.temp[i] > OVERTEMP_C else 0
            if m.faults:
                m.status = p.MOTOR_FAULT
                self.cmds[i] = p.MotorCommand()  # a faulted Robstride drops to reset mode
            elif self.cmds[i].mode == p.MODE_RUN:
                m.status = p.MOTOR_RUNNING
            else:
                m.status = p.MOTOR_OFF
            m.temperature_c = self.temp[i] + random.gauss(0, 0.05)
            m.position = self.q[i] - self.zero_offset[i] + random.gauss(0, 1e-4)
            m.velocity = self.qd[i] + random.gauss(0, 1e-3)
            m.torque = self.tau[i] + random.gauss(0, 0.01)
        # Feet: trot-like pattern, diagonal pairs alternate stance/swing at 2 Hz.
        for i, f in enumerate(s.feet):
            phase = 2 * math.pi * 2.0 * t + (math.pi if i in (1, 2) else 0.0)
            stance = max(0.0, math.sin(phase))
            f.status = p.SENSOR_OK
            f.age_ms = random.randint(0, 2)
            f.force_n = 60.0 * stance + random.gauss(0, 0.5)
            f.raw = int(2048 + 30 * f.force_n)  # fake hall ADC counts
        # ToF boards: distance + magnetometer (Earth field ~50 uT).
        for i, tof in enumerate(s.tof):
            tof.status = p.SENSOR_OK
            tof.age_ms = random.randint(0, 10)
            tof.distance_m = 0.30 + 0.05 * math.sin(t + i) + random.gauss(0, 0.002)
            tof.mag_ut = (20.0 + random.gauss(0, 0.3), random.gauss(0, 0.3),
                          -45.0 + random.gauss(0, 0.3))
        current = 0.5 + 0.4 * sum(abs(x) for x in self.tau)
        s.power.bus_current_a = current
        s.power.bus_voltage_v = 24.0 - 0.05 * current
        s.power.pgood = p.PGOOD_BATTERY | p.PGOOD_MOTOR_RAIL | p.PGOOD_LOGIC_RAIL

    def run(self, host, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((host, port))
        sock.settimeout(0.005)
        print(f"fake Mega-CAN listening on {host}:{port} "
              f"(command {p.COMMAND_SIZE} B, state {p.STATE_SIZE} B)")
        last = time.monotonic()
        self.trip_watchdog()
        self.state.flags &= ~p.FLAG_WATCHDOG_TRIPPED  # boot: not a trip
        while True:
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                data, addr = None, None
            now = time.monotonic()
            self.step_physics(now - last)
            last = now
            if (self.last_cmd_time is not None and now - self.last_cmd_time > WATCHDOG_S
                    and not self.state.flags & p.FLAG_WATCHDOG_TRIPPED):
                self.trip_watchdog()
            if data is None:
                continue
            try:
                seq, _, cmds = p.unpack_command(data)
            except ValueError:
                self.state.flags |= p.FLAG_BAD_PACKET
                continue
            if not self.apply_command(seq, cmds):
                continue
            self.last_cmd_time = now
            self.fill_sensors()
            self.state.seq += 1
            self.state.time_us = self.now_us()
            sock.sendto(p.pack_state(self.state), addr)
            self.state.flags &= ~p.FLAG_BAD_PACKET


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=p.BOARD_PORT)
    args = ap.parse_args()
    FakeBoard().run(args.host, args.port)


if __name__ == "__main__":
    main()
