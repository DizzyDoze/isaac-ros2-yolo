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
