#!/usr/bin/env python3
"""Score a mission: compare output/findings.csv against config/targets.yaml.

Usage: python3 scripts/score_mission.py [--findings output/findings.csv]
                                        [--targets config/targets.yaml]
"""
import argparse
import csv
import math
import yaml

EARTH_RADIUS = 6378137.0


def enu_to_lla(origin, enu):
    lat = origin['latitude_deg'] + math.degrees(enu[1] / EARTH_RADIUS)
    lon = origin['longitude_deg'] + math.degrees(
        enu[0] / (EARTH_RADIUS * math.cos(math.radians(origin['latitude_deg']))))
    return lat, lon


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--findings', default='output/findings.csv')
    ap.add_argument('--targets', default='config/targets.yaml')
    ap.add_argument('--match_radius_m', type=float, default=5.0)
    args = ap.parse_args()

    with open(args.targets) as f:
        spec = yaml.safe_load(f)
    origin = spec['origin']
    targets = spec['targets']

    findings = []
    with open(args.findings) as f:
        for row in csv.DictReader(f):
            findings.append(row)

    print(f'{"target":<16} {"detected":<9} {"sightings":<10} {"pos error (m)":<14}')
    errors, detected = [], 0
    for t in targets:
        name = t['name']
        truth_n = t['enu'][1]   # ENU y -> NED north
        truth_e = t['enu'][0]   # ENU x -> NED east
        best = None
        for r in findings:
            # aruco labels equal target names (aruco_0); color labels are the
            # color word and match the target name suffix (victim_orange)
            if r['label'] != name and not name.endswith(r['label']):
                continue
            d = math.hypot(float(r['north_m']) - truth_n, float(r['east_m']) - truth_e)
            if best is None or d < best[0]:
                best = (d, r)
        if best and best[0] <= args.match_radius_m:
            detected += 1
            errors.append(best[0])
            print(f'{name:<16} {"yes":<9} {best[1]["sightings"]:<10} {best[0]:<14.2f}')
        elif best:
            print(f'{name:<16} {"MISLOC":<9} {best[1]["sightings"]:<10} {best[0]:<14.2f}')
        else:
            print(f'{name:<16} {"no":<9} {"-":<10} {"-":<14}')

    total = len(targets)
    print('-' * 55)
    print(f'detected: {detected}/{total} ({100.0 * detected / total:.0f}%)')
    if errors:
        errors.sort()
        med = errors[len(errors) // 2]
        print(f'geotag error: median {med:.2f} m, max {max(errors):.2f} m')
    unmatched = [r['label'] for r in findings
                 if not any(r['label'] == t['name'] or t['name'].endswith(r['label'])
                            for t in targets)]
    if unmatched:
        print(f'false positives: {sorted(set(unmatched))}')


if __name__ == '__main__':
    main()
