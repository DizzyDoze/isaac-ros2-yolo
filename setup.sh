#!/bin/bash
#===============================================================================
# Isaac Sim -> ROS2 -> YOLO Pipeline Setup
# Uses official Isaac Sim warehouse scene with forklifts for reliable detection
#===============================================================================

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "${SCRIPT_DIR}"/{docker,config,scripts,logs}

echo "=========================================="
echo "  Isaac Sim -> ROS2 -> YOLO Setup"
echo "=========================================="

#-------------------------------------------------------------------------------
# docker-compose.yml
#-------------------------------------------------------------------------------
echo "Creating docker-compose.yml..."
cat > "${SCRIPT_DIR}/docker-compose.yml" << 'EOF'
services:
  isaac-sim:
    image: nvcr.io/nvidia/isaac-sim:4.2.0
    container_name: isaac_sim_ros2
    runtime: nvidia
    environment:
      - DISPLAY=${DISPLAY}
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
      - ACCEPT_EULA=Y
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      - LD_LIBRARY_PATH=/isaac-sim/exts/omni.isaac.ros2_bridge/humble/lib
      - ROS_DOMAIN_ID=0
      - FASTRTPS_DEFAULT_PROFILES_FILE=/isaac-sim/config/fastdds.xml
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ${HOME}/.Xauthority:/root/.Xauthority:ro
      - ./config:/isaac-sim/config:ro
      - ./scripts:/isaac-sim/scripts:ro
      - /dev/shm:/dev/shm
    network_mode: host
    ipc: host
    privileged: true
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
    entrypoint: ["/bin/bash", "-c"]
    command:
      - |
        cd /isaac-sim
        echo "[Isaac] Starting warehouse scene..."
        ./python.sh /isaac-sim/scripts/simple_bottle_scene.py
    healthcheck:
      test: ["CMD-SHELL", "pgrep -f 'simple_bottle' || exit 1"]
      interval: 30s
      timeout: 10s
      retries: 10
      start_period: 180s

  yolo-ros2:
    build:
      context: ./docker
      dockerfile: Dockerfile.yolo
    container_name: yolo_ros2_node
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ${HOME}/.Xauthority:/root/.Xauthority:ro
      - ./logs:/logs:rw
      - /dev/shm:/dev/shm
    network_mode: host
    ipc: host
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
    depends_on:
      isaac-sim:
        condition: service_healthy
    command:
      - /bin/bash
      - -c
      - |
        source /opt/ros/humble/setup.bash
        source /yolo_ws/install/setup.bash

        echo "[YOLO] Waiting for /rgb topic..."
        for i in $(seq 1 180); do
          if ros2 topic list 2>/dev/null | grep -q "^/rgb$"; then
            echo "[YOLO] Found /rgb!"
            sleep 3
            break
          fi
          echo "[YOLO] Waiting... ($i/180)"
          sleep 2
        done

        echo "[YOLO] Starting detection..."
        ros2 launch yolo_bringup yolo.launch.py \
          model:=/root/models/yolov8n.pt \
          input_image_topic:=/rgb \
          device:=cuda:0 \
          threshold:=0.1 \
          image_reliability:=1
EOF

#-------------------------------------------------------------------------------
# Dockerfile.yolo
#-------------------------------------------------------------------------------
echo "Creating Dockerfile.yolo..."
cat > "${SCRIPT_DIR}/docker/Dockerfile.yolo" << 'EOF'
FROM nvidia/cuda:12.2.0-devel-ubuntu22.04
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y software-properties-common curl gnupg lsb-release && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y \
    ros-humble-ros-base ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-vision-msgs ros-humble-rmw-fastrtps-cpp \
    python3-pip python3-colcon-common-extensions git && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip && \
    pip3 install "numpy<2.0" ultralytics opencv-python-headless torch torchvision lap

WORKDIR /yolo_ws/src
RUN git clone --depth 1 https://github.com/mgonzs13/yolo_ros.git && \
    pip3 install -r yolo_ros/requirements.txt

WORKDIR /yolo_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN mkdir -p /root/models && \
    python3 -c "from ultralytics import YOLO; m=YOLO('yolov8n.pt'); import shutil; shutil.move('yolov8n.pt', '/root/models/yolov8n.pt')"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /yolo_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /yolo_ws
EOF

#-------------------------------------------------------------------------------
# FastDDS config
#-------------------------------------------------------------------------------
echo "Creating FastDDS config..."
cat > "${SCRIPT_DIR}/config/fastdds.xml" << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="default_profile" is_default_profile="true">
        <rtps>
            <useBuiltinTransports>true</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
EOF

#-------------------------------------------------------------------------------
# Isaac Sim scene script (warehouse with forklifts)
#-------------------------------------------------------------------------------
echo "Creating Isaac Sim scene script..."
cat > "${SCRIPT_DIR}/scripts/simple_bottle_scene.py" << 'EOF'
#!/usr/bin/env python3
"""
Use official Isaac Sim warehouse scene with forklifts for YOLO detection.
Based on: https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_camera_publishing.html
"""
import sys
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": True})

import omni
from omni.isaac.core import World
from omni.isaac.core.utils import extensions, nucleus, stage as stage_utils
import omni.graph.core as og
import time

print("[Isaac] Enabling ROS2 bridge...", flush=True)
extensions.enable_extension("omni.isaac.ros2_bridge")

print("[Isaac] Creating world...", flush=True)
world = World()

# Load the official warehouse scene with forklifts
print("[Isaac] Loading warehouse scene...", flush=True)
assets_root = nucleus.get_assets_root_path()
if assets_root:
    usd_path = assets_root + "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
    print(f"[Isaac] Loading: {usd_path}", flush=True)
    stage_utils.add_reference_to_stage(usd_path, "/World/Warehouse")
else:
    print("[Isaac] ERROR: Cannot find Nucleus assets!", flush=True)

print("[Isaac] Creating ROS2 camera graph...", flush=True)

# Simple OmniGraph - just use viewport 0 which shows the scene
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/ROS_Camera", "evaluator_name": "push"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
            ("getRenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
            ("cameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "createViewport.inputs:execIn"),
            ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
            ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
            ("getRenderProduct.outputs:execOut", "cameraHelper.inputs:execIn"),
            ("getRenderProduct.outputs:renderProductPath", "cameraHelper.inputs:renderProductPath"),
        ],
        keys.SET_VALUES: [
            ("createViewport.inputs:viewportId", 0),
            ("cameraHelper.inputs:frameId", "sim_camera"),
            ("cameraHelper.inputs:topicName", "rgb"),
            ("cameraHelper.inputs:type", "rgb"),
        ],
    },
)

print("[Isaac] Starting simulation...", flush=True)
world.reset()

print("=" * 60, flush=True)
print("[Isaac] WAREHOUSE SCENE RUNNING", flush=True)
print("[Isaac] Publishing /rgb - scene has forklifts", flush=True)
print("=" * 60, flush=True)

frame_count = 0
start_time = time.time()

while simulation_app.is_running():
    world.step(render=True)
    frame_count += 1
    if frame_count % 300 == 0:
        fps = frame_count / (time.time() - start_time)
        print(f"[Isaac] {frame_count} frames ({fps:.1f} fps)", flush=True)

simulation_app.close()
EOF

#-------------------------------------------------------------------------------
# monitor.sh - captures detection with bounding boxes
#-------------------------------------------------------------------------------
echo "Creating monitor script..."
cat > "${SCRIPT_DIR}/monitor.sh" << 'MONITOR_EOF'
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
            print("No detections in this frame")
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
MONITOR_EOF
chmod +x "${SCRIPT_DIR}/monitor.sh"

#-------------------------------------------------------------------------------
# start.sh
#-------------------------------------------------------------------------------
echo "Creating start script..."
cat > "${SCRIPT_DIR}/start.sh" << 'EOF'
#!/bin/bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
xhost +local:root 2>/dev/null || true

echo "=========================================="
echo "  Isaac Sim -> ROS2 -> YOLO Pipeline"
echo "=========================================="

echo "Building YOLO container..."
docker compose build yolo-ros2

echo "Stopping existing..."
docker compose down 2>/dev/null || true

echo ""
echo "Starting Isaac Sim with warehouse scene..."
docker compose up -d isaac-sim

echo "Waiting for Isaac Sim (~3 min)..."
for i in {1..90}; do
    if docker exec isaac_sim_ros2 pgrep -f simple_bottle > /dev/null 2>&1; then
        echo "  Isaac Sim scene running!"
        break
    fi
    echo "  [$i/90] Loading..."
    sleep 5
done

echo ""
echo "Starting YOLO..."
docker compose up -d yolo-ros2

echo ""
echo "=========================================="
echo "Started! Wait 30s then run: ./monitor.sh"
echo "=========================================="
EOF
chmod +x "${SCRIPT_DIR}/start.sh"

#-------------------------------------------------------------------------------
# stop.sh
#-------------------------------------------------------------------------------
echo "Creating stop script..."
cat > "${SCRIPT_DIR}/stop.sh" << 'EOF'
#!/bin/bash
cd "$(dirname "${BASH_SOURCE[0]}")" && docker compose down
EOF
chmod +x "${SCRIPT_DIR}/stop.sh"

echo ""
echo "=========================================="
echo "  Setup Complete!"
echo "=========================================="
echo ""
echo "Files created:"
echo "  - docker-compose.yml"
echo "  - docker/Dockerfile.yolo"
echo "  - scripts/simple_bottle_scene.py (warehouse scene)"
echo "  - config/fastdds.xml"
echo "  - start.sh, stop.sh, monitor.sh"
echo ""
echo "Next steps:"
echo "  1. ./start.sh     # Start pipeline (~3 min first time)"
echo "  2. ./monitor.sh   # View detections with bounding boxes"
echo "  3. ./stop.sh      # Stop everything"
echo "=========================================="
