#!/usr/bin/env python3
"""Calibrate the camera->body yaw offset from a flight's sightings.

Replays output/sightings.csv against config/targets.yaml for each 90-deg
yaw candidate about the body down axis and reports per-candidate geotag
accuracy. The best candidate goes into geotag_node's DEFAULT_CAM_TO_BODY.
"""
import argparse
import csv
import math

import numpy as np
import yaml

EARTH_RADIUS = 6378137.0
FX, FY, CX, CY = 539.9, 539.9, 640.0, 480.0  # mono_cam_down 1280x960


def quat_to_matrix(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rz(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0],
                     [math.sin(a), math.cos(a), 0],
                     [0, 0, 1.0]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sightings', default='output/sightings.csv')
    ap.add_argument('--targets', default='config/targets.yaml')
    args = ap.parse_args()

    with open(args.targets) as f:
        spec = yaml.safe_load(f)
    truth = {}
    for t in spec['targets']:
        key = t['name'] if t['type'] == 'aruco' else t['color']
        truth[key] = (t['enu'][1], t['enu'][0])  # (north, east)

    sightings = []
    with open(args.sightings) as f:
        for r in csv.DictReader(f):
            if r['label'] in truth:
                sightings.append(r)
    print(f'{len(sightings)} sightings of known targets')

    for yaw_deg in (0, 90, 180, 270):
        R_c2b = rz(yaw_deg)
        errs = []
        for r in sightings:
            ray_cam = np.array([(float(r['u']) - CX) / FX,
                                (float(r['v']) - CY) / FY, 1.0])
            R_b2n = quat_to_matrix(float(r['qw']), float(r['qx']),
                                   float(r['qy']), float(r['qz']))
            ray_ned = R_b2n @ (R_c2b @ ray_cam)
            if ray_ned[2] < 0.05:
                continue
            drone = np.array([float(r['drone_n']), float(r['drone_e']),
                              float(r['drone_z'])])
            t = -drone[2] / ray_ned[2]
            pred = drone + t * ray_ned
            tn, te = truth[r['label']]
            errs.append(math.hypot(pred[0] - tn, pred[1] - te))
        if errs:
            errs.sort()
            med = errs[len(errs) // 2]
            print(f'yaw {yaw_deg:>3} deg: n={len(errs):<4} median {med:6.2f} m  '
                  f'mean {sum(errs)/len(errs):6.2f} m  max {max(errs):6.2f} m')


if __name__ == '__main__':
    main()
