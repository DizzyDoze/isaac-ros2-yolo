<img width="1280" height="720" alt="image" src="https://github.com/user-attachments/assets/a87211a7-bb31-4876-80e1-e910acdcedf5" />

# Isaac Sim -> ROS2 -> YOLO Object Detection Pipeline

A complete pipeline for object detection using NVIDIA Isaac Sim simulation, ROS2 Humble, and YOLOv8.

```
Isaac Sim (Warehouse Scene) --> ROS2 /rgb topic --> YOLOv8 --> Detections
```

## Overview

This project demonstrates real-time object detection on simulated camera feeds:
- **Isaac Sim 4.2**: Renders a warehouse scene with forklifts
- **ROS2 Humble**: Transports camera images between containers
- **YOLOv8**: Performs object detection on the image stream

## Prerequisites

### Hardware
- NVIDIA GPU (tested with RTX series)
- 16GB+ RAM recommended
- 50GB+ disk space

### Software
- Ubuntu 22.04 or 24.04
- Docker with NVIDIA Container Toolkit
- NVIDIA Isaac Sim 4.2 (Docker image)

## Project Structure

```
isaac-ros2-yolo/
├── README.md                 # This file
├── setup.sh                  # Installs all dependencies
├── start.sh                  # Starts the pipeline
├── stop.sh                   # Stops all containers
├── monitor.sh                # Views detection results
├── docker-compose.yml        # Container orchestration
├── docker/
│   └── Dockerfile.yolo       # YOLO ROS2 container
├── scripts/
│   └── simple_bottle_scene.py  # Isaac Sim scene script
├── config/
│   └── fastdds.xml           # ROS2 DDS configuration
└── logs/
    └── detection.png         # Latest detection output
```

## Quick Start

### 1. Run Setup

```bash
chmod +x setup.sh
./setup.sh
```

This **creates all required files**:
- `docker-compose.yml`
- `scripts/simple_bottle_scene.py` (warehouse scene)
- `docker/Dockerfile.yolo`
- `config/fastdds.xml`
- `start.sh`, `stop.sh`, `monitor.sh`

**Important**: Run `./setup.sh` on each new machine before starting.

### 2. Start the Pipeline

```bash
./start.sh
```

This will:
1. Pull the Isaac Sim Docker image (~15GB, first run only)
2. Build the YOLO ROS2 container
3. Start Isaac Sim with the warehouse scene
4. Start YOLO detection node

**First run takes ~5-10 minutes** for image downloads.

### 3. Monitor Detections

```bash
./monitor.sh
```

This saves a detection image with bounding boxes to `logs/detection.png` and opens it.

### 4. Stop the Pipeline

```bash
./stop.sh
```

## Detailed Usage

### Check Container Status

```bash
docker ps
```

Expected output:
```
NAMES            STATUS
yolo_ros2_node   Up X minutes
isaac_sim_ros2   Up X minutes (healthy)
```

### View Container Logs

```bash
# Isaac Sim logs
docker logs isaac_sim_ros2

# YOLO logs
docker logs yolo_ros2_node
```

### Check ROS2 Topics

```bash
docker exec yolo_ros2_node bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list'
```

Key topics:
- `/rgb` - Camera images from Isaac Sim
- `/yolo/detections` - Detection results
- `/yolo/dbg_image` - Debug visualization

### View Raw Detections

```bash
docker exec yolo_ros2_node bash -c 'source /opt/ros/humble/setup.bash && ros2 topic echo /yolo/detections'
```

## Configuration

### Detection Threshold

Edit `docker-compose.yml`, change the `threshold` parameter:

```yaml
threshold:=0.1   # Lower = more detections, higher = more confident only
```

### Camera Resolution

Edit `scripts/simple_bottle_scene.py`, modify viewport settings.

### Scene

The default scene is the official Isaac Sim warehouse with forklifts. To use a different scene, modify `scripts/simple_bottle_scene.py`:

```python
usd_path = assets_root + "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
```

Available scenes in Isaac Sim Assets:
- `/Isaac/Environments/Simple_Warehouse/warehouse.usd`
- `/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd`
- `/Isaac/Environments/Simple_Room/simple_room.usd`
- `/Isaac/Environments/Office/office.usd`

## Troubleshooting

### "nvidia runtime not found"

Install NVIDIA Container Toolkit:
```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### No detections showing

1. **Re-run setup**: `./setup.sh` then `./stop.sh && ./start.sh`
2. Wait 2-3 minutes for Isaac Sim to fully load
3. Check if `/rgb` topic is publishing:
   ```bash
   docker exec yolo_ros2_node bash -c 'source /opt/ros/humble/setup.bash && ros2 topic hz /rgb'
   ```

### Container keeps restarting

Check logs for errors:
```bash
docker logs isaac_sim_ros2 2>&1 | tail -50
```

### GPU memory issues

Isaac Sim requires significant GPU memory. Close other GPU applications or reduce resolution.

## Dependencies

| Component | Version |
|-----------|---------|
| Isaac Sim | 4.2.0 |
| ROS2 | Humble |
| YOLOv8 | Latest (ultralytics) |
| CUDA | 12.2 |
| Python | 3.10 |
| FastDDS | Default with ROS2 Humble |

## How It Works

1. **Isaac Sim** loads a warehouse USD scene with forklifts
2. A virtual camera captures the scene at 30fps
3. Images are published to ROS2 `/rgb` topic via the Isaac ROS2 Bridge
4. **YOLO ROS2 node** subscribes to `/rgb`, runs YOLOv8 inference
5. Detections are published to `/yolo/detections`
6. **monitor.sh** visualizes results with bounding boxes

## References

- [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/)
- [Isaac Sim ROS2 Camera Tutorial](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_camera_publishing.html)
- [YOLOv8 Documentation](https://docs.ultralytics.com/)
- [yolo_ros Package](https://github.com/mgonzs13/yolo_ros)

## License

This project is for educational and research purposes.
