"""ArUco + color-anomaly detection node for the SAR grid explorer.

Subscribes to the bridged Gazebo camera, runs OpenCV detection, publishes
sar_msgs/Detection events plus an annotated debug image.
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from sar_msgs.msg import Detection

# HSV ranges for color anomalies (OpenCV H: 0-179)
COLOR_RANGES = {
    'orange': ([(5, 150, 150)], [(20, 255, 255)]),
    'red':    ([(0, 150, 120), (170, 150, 120)], [(8, 255, 255), (179, 255, 255)]),
    'yellow': ([(22, 150, 150)], [(34, 255, 255)]),
}


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/sar/detections')
        self.declare_parameter('process_every_n', 2)
        self.declare_parameter('min_blob_area_px', 250)
        self.declare_parameter('enable_aruco', True)
        self.declare_parameter('enable_color', True)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_rate_divider', 5)

        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, cv2.aruco.DetectorParameters())

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.on_image, qos)
        self.det_pub = self.create_publisher(
            Detection, self.get_parameter('detections_topic').value, 10)
        self.debug_pub = self.create_publisher(Image, '/sar/debug_image', 1)
        self.frame_count = 0
        self.get_logger().info('detection node up')

    def on_image(self, msg: Image):
        self.frame_count += 1
        n = self.get_parameter('process_every_n').value
        if self.frame_count % n != 0:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        detections = []

        if self.get_parameter('enable_aruco').value:
            detections += self.detect_aruco(frame)
        if self.get_parameter('enable_color').value:
            detections += self.detect_color(frame)

        for det in detections:
            det.header = msg.header
            self.det_pub.publish(det)
        if detections:
            self.get_logger().info(
                f'{len(detections)} detection(s): {[d.label for d in detections]}',
                throttle_duration_sec=2.0)

        if (self.get_parameter('publish_debug_image').value and
                self.frame_count % (n * self.get_parameter('debug_rate_divider').value) == 0):
            self.publish_debug(msg, frame, detections)

    def detect_aruco(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        out = []
        if ids is not None:
            for c, marker_id in zip(corners, ids.flatten()):
                pts = c[0]
                cx, cy = pts[:, 0].mean(), pts[:, 1].mean()
                x, y, w, h = cv2.boundingRect(pts.astype(np.int32))
                det = Detection()
                det.type = Detection.TYPE_ARUCO
                det.aruco_id = int(marker_id)
                det.label = f'aruco_{marker_id}'
                det.u, det.v = float(cx), float(cy)
                det.bbox = [float(x), float(y), float(w), float(h)]
                det.confidence = 1.0
                out.append(det)
        return out

    def detect_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        min_area = self.get_parameter('min_blob_area_px').value
        kernel = np.ones((5, 5), np.uint8)
        out = []
        for color, (lows, highs) in COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for lo, hi in zip(lows, highs):
                mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < min_area:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                m = cv2.moments(cnt)
                if m['m00'] <= 0:
                    continue
                det = Detection()
                det.type = Detection.TYPE_COLOR
                det.aruco_id = -1
                det.label = color
                det.u = float(m['m10'] / m['m00'])
                det.v = float(m['m01'] / m['m00'])
                det.bbox = [float(x), float(y), float(w), float(h)]
                det.confidence = float(area / (w * h))
                out.append(det)
        return out

    def publish_debug(self, msg, frame, detections):
        dbg = frame.copy()
        for det in detections:
            x, y, w, h = [int(v) for v in det.bbox]
            cv2.rectangle(dbg, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(dbg, det.label, (x, max(0, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        out = self.bridge.cv2_to_imgmsg(cv2.cvtColor(dbg, cv2.COLOR_RGB2BGR), 'bgr8')
        out.header = msg.header
        self.debug_pub.publish(out)


def main():
    rclpy.init()
    node = DetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
