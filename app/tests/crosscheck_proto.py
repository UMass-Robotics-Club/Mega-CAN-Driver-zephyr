#!/usr/bin/env python3
"""
Proves app/src/proto.h and udp_protocol/protocol.py agree byte for byte:
Python packs a command -> C decodes it; C encodes a state -> Python unpacks it.
Run by `make -C app/tests`.
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "udp_protocol"))
import protocol as p  # noqa: E402

EXE = os.path.join(HERE, "test_proto")
fail = 0


def check(cond, what):
    global fail
    if not cond:
        print("FAIL", what)
        fail += 1


# Python -> C: command packet
cmds = [p.MotorCommand(mode=i % 3, p_des=i + 0.5, v_des=-(i + 0.25), kp=10.0 * (i + 1),
                       kd=0.125 * (i + 1), tau_ff=-1.5 * i) for i in range(p.NUM_MOTORS)]
pkt = p.pack_command(0xDEADBEEF, 123456, cmds)
out = subprocess.run([EXE, "decode", pkt.hex()], capture_output=True, text=True,
                     check=True).stdout.split("\n")
fields = dict(line.split(" ", 1) for line in out if line)
check(fields["magic"] == "MCAN", "magic")
check(int(fields["version"]) == p.VERSION, "version")
check(int(fields["type"]) == p.TYPE_COMMAND, "type")
check(int(fields["count"]) == p.NUM_MOTORS, "count")
check(int(fields["seq"]) == 0xDEADBEEF, "seq")
check(int(fields["time_us"]) == 123456, "time_us")
for i, c in enumerate(cmds):
    got = fields[f"m{i}"].split()
    check(int(got[0]) == c.mode, f"m{i} mode")
    want = [c.p_des, c.v_des, c.kp, c.kd, c.tau_ff]
    check([float(x) for x in got[1:]] == want, f"m{i} floats {got[1:]} != {want}")

# C -> Python: state packet
raw = bytes.fromhex(subprocess.run([EXE, "encode"], capture_output=True, text=True,
                                   check=True).stdout.strip())
check(len(raw) == p.STATE_SIZE, f"state size {len(raw)}")
s = p.unpack_state(raw)
check(s.seq == 0x01020304 and s.time_us == 0xA0B0C0D0, "state header")
check(s.last_cmd_seq == 77 and s.flags == 3, "extra")
for i, m in enumerate(s.motors):
    check((m.status, m.bus, m.can_id, m.faults) == (i % 4, i, 0x10 + i, 1 << i), f"motor{i} bytes")
    check((m.position, m.velocity, m.torque, m.temperature_c) ==
          (i + 0.5, -(i + 0.25), i * 1.5, 20.0 + i), f"motor{i} floats")
for i, b in enumerate(s.buses):
    check((b.state, b.tx_err, b.rx_err) == (i % 4, 10 + i, 20 + i), f"bus{i}")
for i, f in enumerate(s.feet):
    check((f.status, f.age_ms, f.force_n, f.raw) == (1, 100 + i, i * 10.0, 1000 + i), f"foot{i}")
for i, t in enumerate(s.tof):
    check((t.status, t.age_ms, t.distance_m, t.mag_ut) ==
          (2, 200 + i, i * 0.25, (float(i), -float(i), i * 2.0)), f"tof{i}")
check((s.power.bus_voltage_v, s.power.bus_current_a, s.power.pgood) == (24.5, 1.25, 7), "power")

if fail:
    print(f"proto: {fail} check(s) FAILED")
    sys.exit(1)
print("proto: C structs match protocol.py byte for byte (both directions)")
