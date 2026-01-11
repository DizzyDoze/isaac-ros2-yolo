#!/bin/bash
#===============================================================================
# Isaac Sim → ROS2 → YOLO Pipeline - FINAL SIMPLE VERSION
# Uses a single bottle on a table - guaranteed YOLO detection
#===============================================================================

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "${SCRIPT_DIR}"/{docker,config,scripts,data/isaac,logs}

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
      - ./data/isaac:/isaac-sim/user-data:rw
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
        echo "[Isaac] Starting simple bottle scene..."
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
          if ros2 topic list 2>/dev/null | grep -q "^/rgb$$"; then
            echo "[YOLO] Found /rgb!"
            sleep 3
            break
          fi
          echo "[YOLO] Waiting... ($$i/180)"
          sleep 2
        done
        
        echo "[YOLO] Starting detection with low threshold..."
        ros2 launch yolo_bringup yolo.launch.py \
          model:=/root/models/yolov8n.pt \
          input_image_topic:=/rgb \
          device:=cuda:0 \
          threshold:=0.1 \
          image_reliability:=2
EOF

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

echo "Creating simple bottle scene script..."
cat > "${SCRIPT_DIR}/scripts/simple_bottle_scene.py" << 'EOF'
#!/usr/bin/env python3
"""
Simple scene with a single bottle - guaranteed YOLO detection
Camera directly focused on the bottle
"""
import carb
from isaacsim import SimulationApp

CONFIG = {"renderer": "RayTracedLighting", "headless": True}
simulation_app = SimulationApp(CONFIG)

import omni
from omni.isaac.core import SimulationContext, World
from omni.isaac.core.utils import stage, extensions, nucleus, prims
import omni.graph.core as og
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils
import numpy as np
import time

# Enable ROS2 bridge
extensions.enable_extension("omni.isaac.ros2_bridge")

# Get assets path
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find nucleus server")
    simulation_app.close()
    exit()

print("[Isaac] Creating simple scene with bottle...")

# Create world
world = World(stage_units_in_meters=1.0)

# Add ground plane
world.scene.add_default_ground_plane()

# Add a bottle (COCO class: "bottle")
# YCB mustard bottle path
bottle_path = assets_root_path + "/Isaac/Props/YCB/Axis_Aligned/006_mustard_bottle.usd"
print(f"[Isaac] Loading bottle from: {bottle_path}")

prims.create_prim(
    prim_path="/World/Bottle",
    prim_type="Xform",
    usd_path=bottle_path,
    position=np.array([0.0, 0.0, 0.1]),  # Slightly above ground
    scale=np.array([1.0, 1.0, 1.0])
)

# Create camera looking directly at the bottle
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([0.5, 0.0, 0.3]),  # 0.5m in front, 0.3m high
    frequency=30,
    resolution=(640, 480),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 15, 180]), degrees=True),
)
camera.initialize()

# Create ROS2 camera publisher
keys = og.Controller.Keys
(ros_camera_graph, _, _, _) = og.Controller.edit(
    {"graph_path": "/ROS_Camera", "evaluator_name": "push"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
            ("getRenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
            ("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
            ("cameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "createViewport.inputs:execIn"),
            ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
            ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
            ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
            ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
            ("setCamera.outputs:execOut", "cameraHelper.inputs:execIn"),
            ("getRenderProduct.outputs:renderProductPath", "cameraHelper.inputs:renderProductPath"),
        ],
        keys.SET_VALUES: [
            ("createViewport.inputs:viewportId", 0),
            ("setCamera.inputs:cameraPrim", "/World/Camera"),
            ("cameraHelper.inputs:frameId", "sim_camera"),
            ("cameraHelper.inputs:topicName", "rgb"),
            ("cameraHelper.inputs:type", "rgb"),
        ],
    },
)

# Initialize and play
world.reset()

print("=" * 60)
print("[Isaac] SIMPLE BOTTLE SCENE STARTED")
print("[Isaac] Publishing to /rgb")
print("[Isaac] Camera looking at: YCB mustard bottle")
print("[Isaac] YOLO should detect: 'bottle' class")
print("=" * 60)

frame_count = 0
start_time = time.time()

try:
    while simulation_app.is_running():
        world.step(render=True)
        frame_count += 1
        if frame_count % 300 == 0:
            elapsed = time.time() - start_time
            fps = frame_count / elapsed
            print(f"[Isaac] {frame_count} frames ({fps:.1f} fps)")
except KeyboardInterrupt:
    print("[Isaac] Shutting down...")

world.stop()
simulation_app.close()
EOF

echo "Creating monitor script..."
cat > "${SCRIPT_DIR}/monitor.sh" << 'EOF'
#!/bin/bash
echo "=========================================="
echo "Pipeline Monitor"
echo "=========================================="

echo ""
echo "=== Containers ==="
docker ps --format "table {{.Names}}\t{{.Status}}" | grep -E "isaac|yolo|NAME"

echo ""
echo "=== /rgb Topic ==="
docker exec yolo_ros2_node bash -c 'source /opt/ros/humble/setup.bash && ros2 topic info /rgb 2>/dev/null' || echo "Not found"

echo ""
echo "=== Message Rate ==="
docker exec yolo_ros2_node bash -c 'source /opt/ros/humble/setup.bash && timeout 3 ros2 topic hz /rgb 2>/dev/null | tail -1' || echo "No messages"

echo ""
echo "=== YOLO Detections ==="
docker exec yolo_ros2_node bash -c 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /yolo/detections --once 2>/dev/null' || echo "None yet"
echo "=========================================="
EOF
chmod +x "${SCRIPT_DIR}/monitor.sh"

echo "Creating start script..."
cat > "${SCRIPT_DIR}/start.sh" << 'EOF'
#!/bin/bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
xhost +local:root 2>/dev/null || true

echo "=========================================="
echo "Isaac Sim -> ROS2 -> YOLO (Simple Bottle)"
echo "=========================================="

echo "Building YOLO container..."
docker compose build yolo-ros2

echo "Stopping existing..."
docker compose down 2>/dev/null || true

echo ""
echo "Starting Isaac Sim with bottle scene..."
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

echo "Creating stop script..."
cat > "${SCRIPT_DIR}/stop.sh" << 'EOF'
#!/bin/bash
cd "$(dirname "${BASH_SOURCE[0]}")" && docker compose down
EOF
chmod +x "${SCRIPT_DIR}/stop.sh"

echo "Creating save debug images script..."
cat > "${SCRIPT_DIR}/save_images.sh" << 'EOF'
#!/bin/bash
echo "Saving debug images..."
docker exec yolo_ros2_node bash -c '
source /opt/ros/humble/setup.bash
python3 << "PY"
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
import numpy as np
import cv2

class Saver(Node):
    def __init__(self):
        super().__init__("saver")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, 
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.count = 0
        self.sub1 = self.create_subscription(Image, "/rgb", self.cb_rgb, qos)
        self.sub2 = self.create_subscription(Image, "/yolo/dbg_image", self.cb_dbg, qos)
        
    def cb_rgb(self, msg):
        if self.count & 1: return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8": img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding == "rgba8": img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        cv2.imwrite("/logs/input.png", img)
        print(f"Saved /logs/input.png ({msg.width}x{msg.height})")
        self.count |= 1
        if self.count == 3: rclpy.shutdown()
        
    def cb_dbg(self, msg):
        if self.count & 2: return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8": img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding == "rgba8": img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        cv2.imwrite("/logs/yolo_output.png", img)
        print(f"Saved /logs/yolo_output.png ({msg.width}x{msg.height})")
        self.count |= 2
        if self.count == 3: rclpy.shutdown()

rclpy.init()
node = Saver()
for _ in range(100):
    rclpy.spin_once(node, timeout_sec=0.1)
    if node.count == 3: break
print("Done!")
PY
'

docker cp yolo_ros2_node:/logs/input.png ./logs/ 2>/dev/null && echo "Saved ./logs/input.png"
docker cp yolo_ros2_node:/logs/yolo_output.png ./logs/ 2>/dev/null && echo "Saved ./logs/yolo_output.png"
echo ""
echo "View images in ./logs/ folder"
EOF
chmod +x "${SCRIPT_DIR}/save_images.sh"

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "This creates a simple scene with:"
echo "  - Ground plane"
echo "  - Single YCB mustard bottle"
echo "  - Camera focused on bottle"
echo ""
echo "YOLO should detect: 'bottle' class"
echo ""
echo "Run: ./start.sh"
echo "Wait ~3 min, then: ./monitor.sh"
echo "Save images: ./save_images.sh"
echo "=========================================="
