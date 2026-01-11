#!/bin/bash
# Monitor script - shows YOLO detection results with bounding boxes
cd "$(dirname "${BASH_SOURCE[0]}")"

echo "=========================================="
echo "  Isaac Sim -> ROS2 -> YOLO Monitor"
echo "=========================================="

echo ""
echo "=== Container Status ==="
docker ps --format "table {{.Names}}\t{{.Status}}" | grep -E "isaac|yolo|NAME"

echo ""
echo "=== Capturing Detection ==="
docker exec yolo_ros2_node bash -c '
source /opt/ros/humble/setup.bash
source /yolo_ws/install/setup.bash
python3 << "PY"
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from yolo_msgs.msg import DetectionArray
import numpy as np
import cv2

class DetectionViewer(Node):
    def __init__(self):
        super().__init__("viewer")
        qos_rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.img = None
        self.detections = None
        self.sub_img = self.create_subscription(Image, "/rgb", self.img_cb, qos_rel)
        self.sub_det = self.create_subscription(DetectionArray, "/yolo/detections", self.det_cb, qos_be)

    def img_cb(self, msg):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        if "rgb" in msg.encoding:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        self.img = img

    def det_cb(self, msg):
        self.detections = msg.detections

    def save_result(self):
        if self.img is None:
            print("No image received")
            return False
        img = self.img.copy()
        if self.detections:
            for det in self.detections:
                x = int(det.bbox.center.position.x)
                y = int(det.bbox.center.position.y)
                w = int(det.bbox.size.x)
                h = int(det.bbox.size.y)
                x1, y1 = x - w//2, y - h//2
                x2, y2 = x + w//2, y + h//2
                # Draw box
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # Draw label
                label = f"{det.class_name}: {det.score:.2f}"
                cv2.putText(img, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            print(f"Drew {len(self.detections)} detections")
        else:
            print("No detections yet")
        cv2.imwrite("/logs/detection.png", img)
        print("Saved /logs/detection.png")
        return True

rclpy.init()
node = DetectionViewer()
# Spin longer to ensure we get both image and detections
for _ in range(100):
    rclpy.spin_once(node, timeout_sec=0.05)
    if node.img is not None and node.detections is not None:
        break
node.save_result()
rclpy.shutdown()
PY
'

# Copy to host
docker cp yolo_ros2_node:/logs/detection.png ./logs/detection.png 2>/dev/null

echo ""
echo "=== Latest Detections ==="
docker exec yolo_ros2_node bash -c 'source /opt/ros/humble/setup.bash && source /yolo_ws/install/setup.bash && timeout 2 ros2 topic echo /yolo/detections 2>/dev/null' | grep -E "class_name|score" | head -10

echo ""
echo "=========================================="
echo "Image saved: ./logs/detection.png"
echo "=========================================="

# Open image
xdg-open ./logs/detection.png 2>/dev/null &
