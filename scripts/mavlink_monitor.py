#!/usr/bin/env python3
"""MAVLink sidecar for the SAR demo (pymavlink).

Connects to PX4 SITL's onboard MAVLink endpoint (PX4 sends to UDP 14540),
logs GLOBAL_POSITION_INT / SYS_STATUS / STATUSTEXT to output/mavlink_log.csv,
demonstrates a write command (requests AUTOPILOT_VERSION), and on exit
cross-checks the MAVLink GPS track against output/findings.csv geotags.

Usage: python3 scripts/mavlink_monitor.py [--timeout 240]
"""
import argparse
import csv
import math
import os
import time

from pymavlink import mavutil

EARTH_RADIUS = 6378137.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--conn', default='udpin:0.0.0.0:14540')
    ap.add_argument('--out', default='output/mavlink_log.csv')
    ap.add_argument('--findings', default='output/findings.csv')
    ap.add_argument('--timeout', type=float, default=0,
                    help='seconds to run; 0 = until interrupted')
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    print(f'connecting {args.conn} ...')
    conn = mavutil.mavlink_connection(args.conn)
    conn.wait_heartbeat(timeout=30)
    print(f'heartbeat from sys {conn.target_system} comp {conn.target_component}')

    # write command demo: request AUTOPILOT_VERSION
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
        0, 0, 0, 0, 0, 0)

    # request position + sys_status streams at 5 Hz
    for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                   mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS):
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, 200000, 0, 0, 0, 0, 0)

    rows = []
    t0 = time.time()
    print('logging (Ctrl-C to stop) ...')
    try:
        while True:
            msg = conn.recv_match(
                type=['GLOBAL_POSITION_INT', 'SYS_STATUS', 'STATUSTEXT',
                      'AUTOPILOT_VERSION'],
                blocking=True, timeout=2)
            if args.timeout and time.time() - t0 > args.timeout:
                break
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == 'GLOBAL_POSITION_INT':
                rows.append(('gps', msg.lat / 1e7, msg.lon / 1e7,
                             msg.alt / 1e3, ''))
            elif mtype == 'SYS_STATUS':
                rows.append(('sys', msg.voltage_battery / 1e3,
                             msg.current_battery / 100.0, msg.battery_remaining, ''))
            elif mtype == 'STATUSTEXT':
                rows.append(('text', '', '', '', msg.text))
                print(f'  STATUSTEXT: {msg.text}')
            elif mtype == 'AUTOPILOT_VERSION':
                caps = msg.capabilities
                rows.append(('version', caps, '', '', 'autopilot_version'))
                print(f'  AUTOPILOT_VERSION caps={caps}')
    except KeyboardInterrupt:
        pass

    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['type', 'a', 'b', 'c', 'text'])
        w.writerows(rows)
    print(f'wrote {len(rows)} rows to {args.out}')

    # cross-check: mavlink GPS track vs DDS-derived findings
    gps = [(r[1], r[2]) for r in rows if r[0] == 'gps']
    if gps and os.path.exists(args.findings):
        with open(args.findings) as f:
            findings = list(csv.DictReader(f))
        errs = []
        for r in findings:
            flat, flon = float(r['latitude']), float(r['longitude'])
            best = min(
                math.hypot((flat - g[0]) * math.radians(EARTH_RADIUS),
                           (flon - g[1]) * math.radians(EARTH_RADIUS) *
                           math.cos(math.radians(flat)))
                for g in gps)
            errs.append((r['label'], best))
        print('mavlink-vs-findings agreement (distance to nearest mavlink GPS point):')
        for label, e in errs:
            print(f'  {label:<16} {e:6.2f} m')


if __name__ == '__main__':
    main()
