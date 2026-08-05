"""Telemetry + map visualization node for the SAR demo.

Publishes:
  /sar/flown_path     nav_msgs/Path (ENU map frame) - actual track
  /sar/planned_path_enu nav_msgs/Path (ENU map frame) - planned grid
  /sar/map_image      sensor_msgs/Image - top-down 2D map (path, grid,
                      findings, drone, HUD) for RViz and the demo recorder
"""
import csv
import math
import os
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from px4_msgs.msg import BatteryStatus, VehicleLocalPosition, VehicleStatus

# map window in ENU metres (covers AOI + home)
X_MIN, X_MAX = -8.0, 68.0   # east
Y_MIN, Y_MAX = -26.0, 26.0  # north
PX_PER_M = 12
HUD_H = 64


class VizNode(Node):
    def __init__(self):
        super().__init__('telemetry_viz_node')
        self.declare_parameter('local_position_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('vehicle_status_topic', '/fmu/out/vehicle_status_v4')
        self.declare_parameter('battery_topic', '/fmu/out/battery_status')
        self.declare_parameter('findings_csv', 'output/findings.csv')
        self.declare_parameter('map_rate_hz', 4.0)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(
            VehicleLocalPosition,
            self.get_parameter('local_position_topic').value,
            self.on_position, sensor_qos)
        self.create_subscription(
            VehicleStatus,
            self.get_parameter('vehicle_status_topic').value,
            lambda m: setattr(self, 'nav_state', m.nav_state), sensor_qos)
        self.create_subscription(
            BatteryStatus,
            self.get_parameter('battery_topic').value,
            lambda m: setattr(self, 'battery',
                              m.remaining if m.remaining == m.remaining else m.voltage_v / 16.8),
            sensor_qos)
        self.create_subscription(
            Path, '/sar/planned_path', self.on_planned, 10)

        self.flown_pub = self.create_publisher(Path, '/sar/flown_path', 10)
        self.planned_pub = self.create_publisher(Path, '/sar/planned_path_enu', 10)
        self.map_pub = self.create_publisher(Image, '/sar/map_image', 1)

        self.bridge = CvBridge()
        self.pos = None        # (east, north, alt, heading)
        self.nav_state = -1
        self.battery = float('nan')
        self.trail = []        # ENU points
        self.planned_enu = []
        self.findings = []
        self.findings_mtime = 0.0
        self.lock = threading.Lock()

        rate = self.get_parameter('map_rate_hz').value
        self.create_timer(1.0 / rate, self.render_map)
        self.create_timer(2.0, self.reload_findings)
        self.get_logger().info('viz node up')

    # ---------------- data ----------------
    def on_position(self, msg):
        if not (msg.xy_valid and msg.z_valid):
            return
        # NED -> ENU
        e, n, alt = msg.y, msg.x, -msg.z
        heading = msg.heading
        with self.lock:
            self.pos = (e, n, alt, heading)
            if not self.trail or math.hypot(e - self.trail[-1][0],
                                            n - self.trail[-1][1]) > 0.15:
                self.trail.append((e, n))
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for pe, pn in self.trail:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = pe
            p.pose.position.y = pn
            path.poses.append(p)
        self.flown_pub.publish(path)

    def on_planned(self, msg):
        with self.lock:
            self.planned_enu = [(p.pose.position.y, p.pose.position.x)
                                for p in msg.poses]  # NED->ENU
        out = Path()
        out.header.frame_id = 'map'
        out.header.stamp = self.get_clock().now().to_msg()
        for e, n in self.planned_enu:
            p = PoseStamped()
            p.header = out.header
            p.pose.position.x = e
            p.pose.position.y = n
            out.poses.append(p)
        self.planned_pub.publish(out)

    def reload_findings(self):
        path = self.get_parameter('findings_csv').value
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return
        if mtime == self.findings_mtime:
            return
        self.findings_mtime = mtime
        rows = []
        with open(path) as f:
            for r in csv.DictReader(f):
                rows.append({'label': r['label'],
                             'e': float(r['east_m']), 'n': float(r['north_m']),
                             'type': int(r['type']), 'count': r['sightings']})
        with self.lock:
            self.findings = rows

    # ---------------- map rendering ----------------
    def to_px(self, e, n, w, h):
        x = int((e - X_MIN) * PX_PER_M)
        y = int(h - HUD_H - (n - Y_MIN) * PX_PER_M)
        return x, y

    def render_map(self):
        w = int((X_MAX - X_MIN) * PX_PER_M)
        h = int((Y_MAX - Y_MIN) * PX_PER_M) + HUD_H
        img = np.full((h, w, 3), (32, 32, 32), np.uint8)
        cv2.rectangle(img, (0, h - HUD_H), (w, h), (20, 20, 20), -1)

        with self.lock:
            pos = self.pos
            trail = list(self.trail)
            planned = list(self.planned_enu)
            findings = list(self.findings)
            nav_state, battery = self.nav_state, self.battery

        # AOI rectangle (mission.yaml values)
        aoi = [(10, -20), (10, 20), (60, 20), (60, -20)]
        cv2.polylines(img, [np.array([self.to_px(e, n, w, h) for e, n in aoi])],
                      True, (0, 200, 200), 2)
        # planned grid
        if len(planned) > 1:
            cv2.polylines(img, [np.array([self.to_px(e, n, w, h) for e, n in planned])],
                          False, (200, 150, 60), 1)
        # flown path
        if len(trail) > 1:
            cv2.polylines(img, [np.array([self.to_px(e, n, w, h) for e, n in trail])],
                          False, (60, 220, 60), 2)
        # home
        hx, hy = self.to_px(0, 0, w, h)
        cv2.drawMarker(img, (hx, hy), (255, 255, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
        cv2.putText(img, 'H', (hx + 6, hy - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1)
        # findings
        for f in findings:
            x, y = self.to_px(f['e'], f['n'], w, h)
            color = (60, 220, 60) if f['type'] == 0 else (60, 120, 255)
            cv2.circle(img, (x, y), 7, color, -1)
            cv2.putText(img, f"{f['label']} x{f['count']}", (x + 9, y + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        # drone
        if pos is not None:
            e, n, alt, heading = pos
            x, y = self.to_px(e, n, w, h)
            # ENU heading: NED yaw -> ENU angle = 90deg - yaw
            ang = math.radians(90.0) - heading
            tip = (int(x + 14 * math.cos(ang)), int(y - 14 * math.sin(ang)))
            cv2.arrowedLine(img, (x, y), tip, (255, 255, 255), 2, tipLength=0.4)
            cv2.circle(img, (x, y), 5, (255, 255, 255), -1)

        # HUD
        mode = {14: 'OFFBOARD', 5: 'RTL', 18: 'LAND', 3: 'MISSION'}.get(nav_state, str(nav_state))
        hud = f'mode {mode}   findings {len(findings)}   trail {len(trail)} pts'
        if battery == battery:  # SITL may not publish battery
            hud += f'   batt {battery*100:.0f}%'
        if pos is not None:
            hud += f'   alt {pos[2]:.1f} m'
        cv2.putText(img, hud, (10, h - HUD_H + 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1)
        cv2.putText(img, 'SAR grid explorer - map (ENU, m)', (10, h - HUD_H + 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

        msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        self.map_pub.publish(msg)


def main():
    rclpy.init()
    node = VizNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
