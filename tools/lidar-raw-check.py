#!/usr/bin/env python3
"""Read the LD19 straight from its serial port (no driver) and say where scans get lost.

    python3 tools/lidar-raw-check.py [--port /dev/ldlidar] [--seconds 30] [--csv out.csv]

Stop the car's stack first (the port is exclusive): bash tools/car-clean.sh.

The LD19 sends 47-byte packets of 12 points (0x54 0x2C header, CRC-8 poly 0x4D at the end), ~450 per second at 10 Hz.
The ldlidar_stl_ros2 driver drops a packet with a bad CRC silently, and a turn whose 0 deg packet is lost is not closed
(it throws 1-2 turns away). Its ROS loop also runs at a fixed 10 Hz (WallRate) and keeps only the last turn, so two
turns ending inside one loop cycle lose one. This script counts each separately, per second:

  crc   packets with a bad CRC (noise on the line: motor, cable, power)
  gap   packets missing between two good ones (angle jump), and how many fell on the 0 deg crossing
  junk  bytes thrown away hunting for a header
  hz    turns per second of the LD19 itself (its own speed field and the turns seen)
  loop  turns the driver's 10 Hz loop would overwrite with a perfect line (simulated)
"""
import argparse
import csv
import struct
import time

import serial

HEADER, VER_LEN, PACKET = 0x54, 0x2C, 47
CRC_POLY = 0x4D


def _crc_table():
    table = []
    for byte in range(256):
        crc = byte
        for _ in range(8):
            crc = ((crc << 1) ^ CRC_POLY) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return table


CRC_TABLE = _crc_table()


def crc8(data):
    crc = 0
    for b in data:
        crc = CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


class DriverLoop:
    """The driver's ROS loop: rclcpp::WallRate(10) around 'take the latest finished turn'. Returns turns lost."""

    def __init__(self, period=0.1):
        self.period, self.cycle, self.pending, self.lost, self.published = period, None, 0, 0, 0

    def turn(self, t):
        if self.cycle is None:
            self.cycle = t
        # Loop iterations up to t: each takes what is pending (only the newest survives).
        while self.cycle + self.period <= t:
            self.cycle += self.period
            if self.pending:
                self.lost += self.pending - 1
                self.published += 1
                self.pending = 0
        self.pending += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ldlidar')
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--csv', help='one row per packet (host time, ok, speed, start, end, lidar ms)')
    args = parser.parse_args()
    port = serial.Serial(args.port, 230400, timeout=0.05)
    port.reset_input_buffer()
    writer = csv.writer(open(args.csv, 'w')) if args.csv else None
    if writer:
        writer.writerow(['t', 'crc_ok', 'speed_dps', 'start_deg', 'end_deg', 'lidar_ms'])
    buf = bytearray()
    start = last_print = time.monotonic()
    sec = dict(ok=0, crc=0, gap=0, gap0=0, junk=0, turns=0)
    total = dict(sec)
    prev_end, prev_start, step, speed = None, None, None, 0
    loop = DriverLoop()
    lost_printed = 0
    print('   s  packets  crc  gap(0deg)  junk  hz(speed/turns)  loop_lost')
    while time.monotonic() - start < args.seconds:
        buf += port.read(port.in_waiting or 1)
        now = time.monotonic()
        while len(buf) >= PACKET:
            if buf[0] != HEADER or buf[1] != VER_LEN:
                del buf[0]
                sec['junk'] += 1
                continue
            pkt = bytes(buf[:PACKET])
            ok = crc8(pkt[:-1]) == pkt[-1]
            speed_dps, start_angle = struct.unpack_from('<HH', pkt, 2)
            end_angle, lidar_ms = struct.unpack_from('<HH', pkt, 42)
            if writer:
                writer.writerow([f'{now - start:.4f}', int(ok), speed_dps, start_angle / 100, end_angle / 100, lidar_ms])
            if not ok:
                sec['crc'] += 1
                del buf[:PACKET]    # like the driver: the whole packet is dropped
                continue
            del buf[:PACKET]
            sec['ok'] += 1
            speed = speed_dps
            s, e = start_angle / 100.0, end_angle / 100.0
            span = (e - s) % 360.0
            if span < 30.0:
                step = span / 11.0 if step is None else 0.9 * step + 0.1 * span / 11.0
            if prev_end is not None and step:
                jump = (s - prev_end) % 360.0
                if jump > 2.5 * step and jump < 300.0:
                    missing = round(jump / (span + step)) or 1
                    sec['gap'] += missing
                    if s < prev_end:      # the hole spans 0 deg: the driver cannot close the turn
                        sec['gap0'] += 1
            if prev_start is not None and s < prev_start:
                sec['turns'] += 1
                loop.turn(now)
            prev_end, prev_start = e, s
        if now - last_print >= 1.0:
            lost = loop.lost - lost_printed
            lost_printed = loop.lost
            print(f'{now - start:4.0f}  {sec["ok"]:7d}  {sec["crc"]:3d}  {sec["gap"]:3d}({sec["gap0"]})     '
                  f'{sec["junk"]:4d}  {speed / 360:5.2f}/{sec["turns"]:2d}         {lost:3d}', flush=True)
            for k in total:
                total[k] += sec[k]
                sec[k] = 0
            last_print = now
    elapsed = time.monotonic() - start
    print(f'\ntotal {elapsed:.0f} s: {total["ok"]} good packets, {total["crc"]} bad CRC, {total["gap"]} missing '
          f'({total["gap0"]} at 0 deg), {total["junk"]} junk bytes, {total["turns"]} turns '
          f'({total["turns"] / elapsed:.2f} Hz); the 10 Hz driver loop would lose {loop.lost} turns '
          f'({loop.lost / elapsed:.2f}/s) with this line')


if __name__ == '__main__':
    main()
