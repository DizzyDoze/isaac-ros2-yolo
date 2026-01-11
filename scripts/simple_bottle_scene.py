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
print("[Isaac] Publishing /rgb - scene has forklifts (truck class)", flush=True)
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
