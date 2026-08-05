#!/usr/bin/env python3
"""Save /sar/debug_image frames to PNG for offline inspection."""
import os
import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

out_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/opencode/frames'
max_frames = int(sys.argv[2]) if len(sys.argv) > 2 else 12
os.makedirs(out_dir, exist_ok=True)

rclpy.init()
node = rclpy.create_node('frame_saver')
bridge = CvBridge()
count = [0]

def cb(msg):
    if count[0] < max_frames:
        img = bridge.imgmsg_to_cv2(msg, 'bgr8')
        path = os.path.join(out_dir, f'debug_{count[0]:03d}.png')
        cv2.imwrite(path, img)
        count[0] += 1
        node.get_logger().info(f'saved {path}')

qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=1)
node.create_subscription(Image, '/sar/debug_image', cb, qos)
t0 = time.time()
while count[0] < max_frames and time.time() - t0 < 240:
    rclpy.spin_once(node, timeout_sec=0.5)
print(f'saved {count[0]} frames to {out_dir}')
node.destroy_node()
rclpy.shutdown()
