#!/usr/bin/env python3
"""Record the split-screen demo video headlessly.

Left: annotated camera feed (/sar/debug_image, Gazebo camera + detection
overlays). Right: top-down map (/sar/map_image, flight path + findings +
telemetry HUD). Output: output/demo.mp4.

In a desktop environment you would instead record Gazebo and RViz with OBS;
this recorder produces the same split-screen content without a display.

Usage: python3 scripts/record_demo.py [out.mp4] [max_seconds]
"""
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

OUT = sys.argv[1] if len(sys.argv) > 1 else 'output/demo.mp4'
MAX_S = float(sys.argv[2]) if len(sys.argv) > 2 else 300.0
FPS = 10.0


class Recorder(Node):
    def __init__(self):
        super().__init__('demo_recorder')
        self.bridge = CvBridge()
        self.cam = None
        self.map_img = None
        self.lock = threading.Lock()
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, '/sar/debug_image', self.on_cam, qos)
        self.create_subscription(Image, '/sar/map_image', self.on_map, qos)

    def on_cam(self, msg):
        with self.lock:
            self.cam = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def on_map(self, msg):
        with self.lock:
            self.map_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')


def main():
    rclpy.init()
    node = Recorder()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    node.get_logger().info('waiting for first frames...')
    t0 = time.time()
    while time.time() - t0 < 240:
        with node.lock:
            if node.cam is not None and node.map_img is not None:
                break
        if int(time.time() - t0) % 20 == 0:
            node.get_logger().info('still waiting for frames...')
        time.sleep(0.5)
    with node.lock:
        if node.cam is None or node.map_img is None:
            print('no frames received; is the survey stack running?')
            return
        cam0, map0 = node.cam, node.map_img

    ch, cw = cam0.shape[:2]
    mh = int(cw * map0.shape[0] / map0.shape[1])
    writer = cv2.VideoWriter(OUT, cv2.VideoWriter_fourcc(*'mp4v'), FPS,
                             (cw * 2, ch))
    print(f'recording {OUT} ({cw * 2}x{ch}) - Ctrl-C to stop')
    t0 = time.time()
    frames = 0
    try:
        while time.time() - t0 < MAX_S:
            with node.lock:
                cam = None if node.cam is None else node.cam.copy()
                map_img = None if node.map_img is None else node.map_img.copy()
            if cam is None or map_img is None:
                time.sleep(0.05)
                continue
            cam = cv2.resize(cam, (cw, ch))
            map_img = cv2.resize(map_img, (cw, ch))
            cv2.putText(cam, 'Gazebo camera + detections', (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(map_img, 'Mission map (RViz equivalent)', (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            writer.write(cv2.hconcat([cam, map_img]))
            frames += 1
            time.sleep(1.0 / FPS)
    except KeyboardInterrupt:
        pass
    writer.release()
    node.destroy_node()
    rclpy.shutdown()
    print(f'wrote {frames} frames to {OUT}')


if __name__ == '__main__':
    main()
