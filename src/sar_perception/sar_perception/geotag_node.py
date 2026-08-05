"""Geotag node: fuse camera detections with PX4 pose to WGS-84 findings.

Pipeline: pixel -> camera ray -> body frame -> NED (vehicle attitude) ->
ground-plane intersection -> lat/lon. Deduplicates repeat sightings into
clusters, writes findings.csv / findings.geojson, publishes RViz markers.
"""
import csv
import json
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from px4_msgs.msg import VehicleAttitude, VehicleGlobalPosition, VehicleLocalPosition

from sar_msgs.msg import Detection

EARTH_RADIUS = 6378137.0

# camera optical frame -> body FRD for the x500_sar mono_cam_down mount.
# Nadir view, 90 deg yaw about the down axis. CALIBRATED via
# scripts/calibrate_cam.py on flight data (median 1.86 m vs 14+ m others).
DEFAULT_CAM_TO_BODY = [0.0, -1.0, 0.0,
                       1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0]


def quat_to_matrix(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class GeotagNode(Node):
    def __init__(self):
        super().__init__('geotag_node')
        self.declare_parameter('detections_topic', '/sar/detections')
        self.declare_parameter('local_position_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('attitude_topic', '/fmu/out/vehicle_attitude')
        self.declare_parameter('global_position_topic', '/fmu/out/vehicle_global_position')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('dedupe_radius_m', 5.0)
        self.declare_parameter('min_ray_down', 0.05)
        self.declare_parameter('center_gate', 0.8)  # fraction of frame half-dim; 1.0 = off
        self.declare_parameter('output_dir', 'output')
        self.declare_parameter('cam_to_body', DEFAULT_CAM_TO_BODY)

        self.cam_to_body = np.array(
            self.get_parameter('cam_to_body').value, dtype=float).reshape(3, 3)
        self.output_dir = self.get_parameter('output_dir').value
        os.makedirs(self.output_dir, exist_ok=True)

        self.K = None            # camera intrinsics
        self.ned = None          # [x, y, z] local NED
        self.R_body_ned = None   # body -> NED rotation
        self._last_q = (1.0, 0.0, 0.0, 0.0)
        self.global_pos = None   # (lat, lon, alt_amsl)
        self.clusters = []       # persistent findings

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(
            Detection, self.get_parameter('detections_topic').value,
            self.on_detection, 10)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.on_camera_info, sensor_qos)
        self.create_subscription(
            VehicleLocalPosition, self.get_parameter('local_position_topic').value,
            self.on_local_position, sensor_qos)
        self.create_subscription(
            VehicleAttitude, self.get_parameter('attitude_topic').value,
            self.on_attitude, sensor_qos)
        self.create_subscription(
            VehicleGlobalPosition, self.get_parameter('global_position_topic').value,
            self.on_global_position, sensor_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/sar/markers', 10)
        self._sightings_path = os.path.join(self.output_dir, 'sightings.csv')
        with open(self._sightings_path, 'w', newline='') as f:
            csv.writer(f).writerow(
                ['label', 'u', 'v', 'drone_n', 'drone_e', 'drone_z',
                 'qw', 'qx', 'qy', 'qz'])
        self.get_logger().info(f'geotag node up, output: {self.output_dir}')

    def log_sighting(self, det):
        q = self._last_q
        with open(self._sightings_path, 'a', newline='') as f:
            csv.writer(f).writerow([
                det.label, f'{det.u:.1f}', f'{det.v:.1f}',
                f'{self.ned[0]:.3f}', f'{self.ned[1]:.3f}', f'{self.ned[2]:.3f}',
                f'{q[0]:.6f}', f'{q[1]:.6f}', f'{q[2]:.6f}', f'{q[3]:.6f}'])

    def on_camera_info(self, msg):
        if self.K is None:
            self.K = msg.k
            self.get_logger().info(f'camera intrinsics: fx={self.K[0]:.1f} cx={self.K[2]:.1f}')

    def on_local_position(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.ned = np.array([msg.x, msg.y, msg.z])

    def on_attitude(self, msg):
        q = msg.q
        self._last_q = (q[0], q[1], q[2], q[3])
        self.R_body_ned = quat_to_matrix(q[0], q[1], q[2], q[3])

    def on_global_position(self, msg):
        if msg.lat_lon_valid:
            self.global_pos = (msg.lat, msg.lon, msg.alt)

    def on_detection(self, det):
        if self.K is None or self.ned is None or self.R_body_ned is None:
            self.get_logger().warn('dropping detection: missing telemetry/intrinsics',
                                   throttle_duration_sec=5.0)
            return
        self.log_sighting(det)
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        gate = self.get_parameter('center_gate').value
        if gate < 1.0:
            if (abs(det.u - cx) > gate * cx) or (abs(det.v - cy) > gate * cy):
                return  # frame-edge sighting: large geotag lever arm
        ray_cam = np.array([(det.u - cx) / fx, (det.v - cy) / fy, 1.0])
        ray_body = self.cam_to_body @ ray_cam
        ray_ned = self.R_body_ned @ ray_body
        if ray_ned[2] < self.get_parameter('min_ray_down').value:
            self.get_logger().warn('dropping detection: ray not downward',
                                   throttle_duration_sec=5.0)
            return  # ray not going downward
        t = -self.ned[2] / ray_ned[2]   # ground plane z = 0 (takeoff level)
        if t < 0.5 or t > 200.0:
            return
        target_ned = self.ned + t * ray_ned

        lat = lon = alt = float('nan')
        if self.global_pos is not None:
            lat0, lon0, alt0 = self.global_pos
            dn = target_ned[0] - self.ned[0]
            de = target_ned[1] - self.ned[1]
            lat = lat0 + math.degrees(dn / EARTH_RADIUS)
            lon = lon0 + math.degrees(de / (EARTH_RADIUS * math.cos(math.radians(lat0))))
            alt = alt0 + self.ned[2]  # approx ground level AMSL

        self.add_sighting(det, target_ned, (lat, lon, alt))

    def add_sighting(self, det, ned, lla):
        radius = self.get_parameter('dedupe_radius_m').value
        best, best_d = None, radius
        for c in self.clusters:
            if c['label'] != det.label or c['aruco_id'] != det.aruco_id:
                continue
            d = math.hypot(c['n'] - ned[0], c['e'] - ned[1])
            if d < best_d:
                best, best_d = c, d
        if best is not None:
            n = best['count'] + 1
            for key, val in (('n', ned[0]), ('e', ned[1]),
                             ('lat', lla[0]), ('lon', lla[1]), ('alt', lla[2])):
                best[key] = (best[key] * best['count'] + val) / n
            best['count'] = n
        else:
            self.clusters.append({
                'label': det.label, 'aruco_id': det.aruco_id,
                'type': int(det.type),
                'n': float(ned[0]), 'e': float(ned[1]),
                'lat': lla[0], 'lon': lla[1], 'alt': lla[2], 'count': 1})
            self.get_logger().info(
                f'NEW FINDING {det.label} at N {ned[0]:.1f} E {ned[1]:.1f} '
                f'lat {lla[0]:.7f} lon {lla[1]:.7f} (#{len(self.clusters)})')
        self.write_outputs()
        self.publish_markers()

    def write_outputs(self):
        csv_path = os.path.join(self.output_dir, 'findings.csv')
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['label', 'aruco_id', 'type', 'north_m', 'east_m',
                        'latitude', 'longitude', 'alt_amsl_m', 'sightings'])
            for c in self.clusters:
                w.writerow([c['label'], c['aruco_id'], c['type'],
                            f"{c['n']:.3f}", f"{c['e']:.3f}",
                            f"{c['lat']:.8f}", f"{c['lon']:.8f}", f"{c['alt']:.2f}",
                            c['count']])
        geo = {'type': 'FeatureCollection', 'features': [
            {'type': 'Feature',
             'geometry': {'type': 'Point', 'coordinates': [c['lon'], c['lat']]},
             'properties': {'label': c['label'], 'sightings': c['count']}}
            for c in self.clusters if not math.isnan(c['lat'])]}
        with open(os.path.join(self.output_dir, 'findings.geojson'), 'w') as f:
            json.dump(geo, f, indent=2)

    def publish_markers(self):
        arr = MarkerArray()
        for i, c in enumerate(self.clusters):
            m = Marker()
            m.header.frame_id = 'map'   # ENU
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'sar_findings'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = c['e']    # ENU: x = east
            m.pose.position.y = c['n']    # ENU: y = north
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color.a = 1.0
            if c['type'] == Detection.TYPE_ARUCO:
                m.color.r, m.color.g, m.color.b = 0.1, 0.9, 0.1
            else:
                m.color.r, m.color.g, m.color.b = 1.0, 0.3, 0.1
            t = Marker()
            t.header = m.header
            t.ns = 'sar_labels'
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = c['e']
            t.pose.position.y = c['n']
            t.pose.position.z = 1.5
            t.pose.orientation.w = 1.0
            t.scale.z = 0.8
            t.color.a = 1.0
            t.color.r = t.color.g = t.color.b = 1.0
            t.text = f"{c['label']} x{c['count']}"
            arr.markers.extend([m, t])
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = GeotagNode()
    try:
        rclpy.spin(node)
    finally:
        node.write_outputs()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
