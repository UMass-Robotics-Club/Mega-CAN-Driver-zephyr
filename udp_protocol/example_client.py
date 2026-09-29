#!/usr/bin/env python3
"""
Example Jetson-side client: arms the board, swings every joint in a slow sine,
prints a summary once a second, and optionally logs every state packet to CSV.

    python3 example_client.py --board 127.0.0.1 --csv run.csv --seconds 10
"""

import argparse
import csv
import math
import socket
import time

import protocol as p


def csv_header():
    cols = ["time_s", "seq", "last_cmd_seq", "flags"]
    for i in range(p.NUM_MOTORS):
        cols += [f"m{i}_status", f"m{i}_faults", f"m{i}_pos", f"m{i}_vel", f"m{i}_tau",
                 f"m{i}_temp_c"]
    for i in range(p.NUM_FEET):
        cols += [f"foot{i}_status", f"foot{i}_force_n"]
    for i in range(p.NUM_TOF):
        cols += [f"tof{i}_status", f"tof{i}_dist_m", f"tof{i}_mx", f"tof{i}_my", f"tof{i}_mz"]
    cols += ["bus_v", "bus_a", "pgood"]
    return cols


def csv_row(t, s):
    row = [f"{t:.4f}", s.seq, s.last_cmd_seq, s.flags]
    for m in s.motors:
        row += [m.status, m.faults, f"{m.position:.5f}", f"{m.velocity:.4f}",
                f"{m.torque:.4f}", f"{m.temperature_c:.1f}"]
    for f in s.feet:
        row += [f.status, f"{f.force_n:.2f}"]
    for tof in s.tof:
        row += [tof.status, f"{tof.distance_m:.4f}", *(f"{x:.2f}" for x in tof.mag_ut)]
    row += [f"{s.power.bus_voltage_v:.3f}", f"{s.power.bus_current_a:.3f}", s.power.pgood]
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--board", default=p.BOARD_IP)
    ap.add_argument("--port", type=int, default=p.BOARD_PORT)
    ap.add_argument("--rate", type=float, default=500.0, help="packets per second")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--csv", help="log every state packet to this file")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.02)
    board = (args.board, args.port)
    seq = 0
    t0 = time.monotonic()
    writer = None
    if args.csv:
        fh = open(args.csv, "w", newline="")
        writer = csv.writer(fh)
        writer.writerow(csv_header())

    period = 1.0 / args.rate
    next_tick = time.monotonic()
    last_print = 0.0
    lost = 0
    while True:
        t = time.monotonic() - t0
        if t > args.seconds:
            break
        seq += 1
        if t < 0.1:
            # Safe start: all off (also re-arms after a watchdog trip).
            cmds = [p.MotorCommand() for _ in range(p.NUM_MOTORS)]
        else:
            cmds = [p.MotorCommand(mode=p.MODE_RUN,
                                   p_des=0.5 * math.sin(2 * math.pi * 0.5 * t + i),
                                   kp=20.0, kd=0.5)
                    for i in range(p.NUM_MOTORS)]
        sock.sendto(p.pack_command(seq, int(t * 1e6), cmds), board)
        try:
            data, _ = sock.recvfrom(2048)
            s = p.unpack_state(data)
        except (socket.timeout, ValueError):
            lost += 1
            s = None

        if s is not None:
            if writer:
                writer.writerow(csv_row(t, s))
            if s.flags & p.FLAG_WATCHDOG_TRIPPED:
                print("board reports watchdog trip")
            bad = [i for i, m in enumerate(s.motors)
                   if m.status in (p.MOTOR_FAULT, p.MOTOR_NO_REPLY)]
            if bad:
                print("motors faulted / not answering -> stopping: " +
                      ", ".join(f"m{i} status={s.motors[i].status} "
                                f"faults=0x{s.motors[i].faults:02x}" for i in bad))
                break
            if t - last_print >= 1.0:
                last_print = t
                print(f"t={t:5.1f}s seq={s.seq} m0 pos={s.motors[0].position:+.3f} rad "
                      f"temp={s.motors[0].temperature_c:.1f} C "
                      f"feet={[round(f.force_n) for f in s.feet]} N "
                      f"bus={s.power.bus_voltage_v:.1f} V lost={lost}")

        next_tick += period
        delay = next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)

    # Leave the motors off on exit.
    seq += 1
    sock.sendto(p.pack_command(seq, 0, [p.MotorCommand() for _ in range(p.NUM_MOTORS)]),
                board)
    if writer:
        fh.close()
        print(f"wrote {args.csv}")


if __name__ == "__main__":
    main()
