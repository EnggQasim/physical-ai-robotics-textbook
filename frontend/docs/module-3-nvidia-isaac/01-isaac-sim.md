---
sidebar_position: 2
title: "Isaac Sim Fundamentals"
description: "Creating photorealistic robot simulations with NVIDIA Isaac Sim"
---

# Isaac Sim Fundamentals

**Isaac Sim** is NVIDIA's robotics simulation platform built on Omniverse. It provides photorealistic rendering, accurate physics, and seamless ROS2 integration.

![NVIDIA Isaac Platform Architecture](/img/generated/isaac/isaac-platform.svg)
*Figure: NVIDIA Isaac Platform - Isaac Sim, Isaac ROS, Isaac Perceptor, and Isaac Manipulator components.*

## Learning Objectives

- Navigate the Isaac Sim interface
- Import and configure robot models
- Create simulation environments
- Connect Isaac Sim to ROS2

## Getting Started with Isaac Sim

### Launching Isaac Sim

```bash
# Via Omniverse Launcher
# 1. Open NVIDIA Omniverse Launcher
# 2. Navigate to Library > Isaac Sim
# 3. Click Launch

# Via Docker (recommended for reproducibility)
docker run --gpus all -e "ACCEPT_EULA=Y" \
  --rm --network=host \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/nvidia:/root/.nv:rw \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

### Interface Overview

```text
┌─────────────────────────────────────────────────────────────────┐
│  File  Edit  Window  Isaac  Help                     [_][□][X] │
├─────────────────────────────────────────────────────────────────┤
│ ┌───────────┐ ┌─────────────────────────────┐ ┌───────────────┐│
│ │  Stage    │ │                             │ │  Property     ││
│ │           │ │                             │ │               ││
│ │  /World   │ │      3D Viewport            │ │  Transform    ││
│ │   /Robot  │ │                             │ │  Physics      ││
│ │   /Ground │ │                             │ │  Materials    ││
│ │   /Lights │ │                             │ │               ││
│ │           │ │                             │ │               ││
│ └───────────┘ └─────────────────────────────┘ └───────────────┘│
├─────────────────────────────────────────────────────────────────┤
│ ┌─────────────────────────────────────────────────────────────┐│
│ │  Content Browser  │  Console  │  Timeline  │  Extensions   ││
│ └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

## Creating Your First Simulation

### Step 1: Create a New Stage

```python
# Python script for Isaac Sim
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, GroundPlane

# Create simulation world
world = World(stage_units_in_meters=1.0)

# Add ground plane
world.scene.add_default_ground_plane()

# Add a dynamic cube
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="my_cube",
        position=[0, 0, 1.0],
        size=0.5,
        color=[1.0, 0.0, 0.0]  # Red
    )
)

# Initialize and run
world.reset()
for i in range(1000):
    world.step(render=True)
```

### Step 2: Import a Robot

Isaac Sim supports multiple robot formats:

```python
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot

# Import from URDF
from omni.isaac.urdf import _urdf
urdf_interface = _urdf.acquire_urdf_interface()

# Configure import settings
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = False
import_config.import_inertia_tensor = True

# Import the robot
result = urdf_interface.parse_urdf(
    "/path/to/robot.urdf",
    import_config
)

# Or import from USD directly
robot = world.scene.add(
    Robot(
        prim_path="/World/Robot",
        usd_path="omniverse://localhost/NVIDIA/Assets/Robots/Carter/carter_v1.usd",
        name="carter"
    )
)
```

### Step 3: Add Sensors

```python
from omni.isaac.sensor import Camera, IMUSensor, LidarRtx

# Add RGB camera
camera = Camera(
    prim_path="/World/Robot/Camera",
    position=[0, 0, 0.5],
    frequency=30,
    resolution=(640, 480)
)
camera.initialize()

# Add LiDAR sensor
lidar = world.scene.add(
    LidarRtx(
        prim_path="/World/Robot/Lidar",
        name="lidar",
        rotation_frequency=10,
        horizontal_fov=360.0,
        vertical_fov=30.0,
        min_range=0.1,
        max_range=100.0
    )
)

# Add IMU sensor
imu = IMUSensor(
    prim_path="/World/Robot/IMU",
    name="imu",
    frequency=100,
    translation=[0, 0, 0]
)
```

## Physics Configuration

### Setting Up PhysX

```python
from omni.isaac.core.utils.physics import set_physics_properties

# Configure physics scene
set_physics_properties(
    stage=world.stage,
    physics_dt=1/120.0,  # Physics timestep
    rendering_dt=1/60.0,  # Rendering timestep
    gravity=[0.0, 0.0, -9.81],
    solver_type="TGS",  # Temporal Gauss-Seidel
    solver_position_iterations=8,
    solver_velocity_iterations=1
)
```

### Collision Configuration

```python
from pxr import UsdPhysics, PhysxSchema

# Get the prim
prim = world.stage.GetPrimAtPath("/World/Robot/base_link")

# Add collision API
collision_api = UsdPhysics.CollisionAPI.Apply(prim)

# Configure physics material
physics_material = UsdPhysics.MaterialAPI.Apply(prim)
physics_material.CreateStaticFrictionAttr(0.5)
physics_material.CreateDynamicFrictionAttr(0.5)
physics_material.CreateRestitutionAttr(0.1)
```

## ROS2 Bridge

Isaac Sim integrates directly with ROS2:

### Enabling the ROS2 Bridge

```python
from omni.isaac.ros2_bridge import ROS2Bridge

# Enable ROS2 extension
import omni.kit.commands
omni.kit.commands.execute("ROS2BridgeCreatePrim")

# Or via Python API
ros2_bridge = ROS2Bridge()
ros2_bridge.initialize()
```

### Publishing Sensor Data

```python
from omni.isaac.ros2_bridge import ROS2Camera, ROS2Lidar, ROS2IMU

# Create camera publisher
camera_pub = ROS2Camera(
    prim_path="/World/Robot/Camera",
    topic_name="/robot/camera/image_raw",
    frame_id="camera_link"
)

# Create LiDAR publisher
lidar_pub = ROS2Lidar(
    prim_path="/World/Robot/Lidar",
    topic_name="/robot/scan",
    frame_id="lidar_link"
)

# Create IMU publisher
imu_pub = ROS2IMU(
    prim_path="/World/Robot/IMU",
    topic_name="/robot/imu",
    frame_id="imu_link"
)
```

### Subscribing to Commands

```python
from omni.isaac.ros2_bridge import ROS2SubscribeTwist

# Subscribe to velocity commands
twist_sub = ROS2SubscribeTwist(
    topic_name="/cmd_vel",
    callback=lambda msg: robot.apply_wheel_velocities(msg)
)
```

## Environment Creation

### Adding Objects

```python
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere, FixedCuboid

# Dynamic objects (affected by physics)
box = DynamicCuboid(
    prim_path="/World/Box",
    name="box",
    position=[2.0, 0, 0.5],
    size=0.5,
    mass=1.0
)

# Static objects (fixed in place)
wall = FixedCuboid(
    prim_path="/World/Wall",
    name="wall",
    position=[5.0, 0, 1.0],
    size=[0.1, 4.0, 2.0]
)

world.scene.add(box)
world.scene.add(wall)
```

### Loading Pre-built Environments

```python
# Load warehouse environment
from omni.isaac.core.utils.nucleus import get_assets_root_path

assets_root = get_assets_root_path()
warehouse_path = assets_root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"

add_reference_to_stage(
    usd_path=warehouse_path,
    prim_path="/World/Warehouse"
)
```

### Domain Randomization

```python
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.rotations import euler_angles_to_quat
import random

def randomize_environment():
    """Randomize environment for training."""

    # Randomize lighting
    light = get_prim_at_path("/World/Light")
    light.GetAttribute("intensity").Set(random.uniform(500, 2000))

    # Randomize object positions
    for i in range(10):
        obj = get_prim_at_path(f"/World/Object_{i}")
        new_pos = [
            random.uniform(-5, 5),
            random.uniform(-5, 5),
            random.uniform(0.1, 2.0)
        ]
        obj.GetAttribute("xformOp:translate").Set(new_pos)

    # Randomize textures
    from omni.isaac.core.utils.semantics import add_update_semantics
    # ... texture randomization code
```

## Complete Example: Mobile Robot Simulation

```python
#!/usr/bin/env python3
"""Complete Isaac Sim mobile robot simulation."""

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import GroundPlane
from omni.isaac.sensor import Camera, LidarRtx
from omni.isaac.ros2_bridge import ROS2Bridge
import numpy as np

class MobileRobotSim:
    """Mobile robot simulation with sensors and ROS2."""

    def __init__(self):
        # Create world
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        # Load robot
        self.robot = self.world.scene.add(
            Robot(
                prim_path="/World/Robot",
                usd_path="path/to/robot.usd",
                name="mobile_robot"
            )
        )

        # Add sensors
        self.setup_sensors()

        # Setup ROS2 bridge
        self.setup_ros2()

    def setup_sensors(self):
        """Configure robot sensors."""
        self.camera = Camera(
            prim_path="/World/Robot/camera_link/Camera",
            frequency=30,
            resolution=(640, 480)
        )

        self.lidar = LidarRtx(
            prim_path="/World/Robot/lidar_link/Lidar",
            rotation_frequency=10,
            horizontal_fov=360.0,
            max_range=50.0
        )

    def setup_ros2(self):
        """Initialize ROS2 bridge."""
        self.ros2_bridge = ROS2Bridge()

        # Publishers and subscribers created automatically
        # when ROS2 bridge extension is enabled

    def run(self):
        """Main simulation loop."""
        self.world.reset()

        while True:
            # Get sensor data
            rgb = self.camera.get_rgba()
            depth = self.camera.get_depth()
            lidar_data = self.lidar.get_point_cloud()

            # Physics step
            self.world.step(render=True)

if __name__ == "__main__":
    sim = MobileRobotSim()
    sim.run()
```

## Synthetic Data Generation

### Automatic Annotation

```python
from omni.isaac.synthetic_data import SyntheticDataHelper

# Initialize synthetic data helper
sd_helper = SyntheticDataHelper()

# Get various annotations
rgb = sd_helper.get_rgb(camera_path="/World/Robot/Camera")
depth = sd_helper.get_depth(camera_path="/World/Robot/Camera")
semantic = sd_helper.get_semantic_segmentation(camera_path="/World/Robot/Camera")
instance = sd_helper.get_instance_segmentation(camera_path="/World/Robot/Camera")
bbox_2d = sd_helper.get_bounding_box_2d(camera_path="/World/Robot/Camera")
bbox_3d = sd_helper.get_bounding_box_3d(camera_path="/World/Robot/Camera")
```

### Generating Training Data

```python
import json
import os

def generate_training_data(num_samples=1000, output_dir="./dataset"):
    """Generate labeled training data."""

    os.makedirs(f"{output_dir}/images", exist_ok=True)
    os.makedirs(f"{output_dir}/labels", exist_ok=True)

    annotations = []

    for i in range(num_samples):
        # Randomize scene
        randomize_environment()

        # Step simulation to settle physics
        for _ in range(10):
            world.step(render=False)

        # Capture data
        world.step(render=True)

        rgb = sd_helper.get_rgb(camera_path="/World/Robot/Camera")
        bbox_2d = sd_helper.get_bounding_box_2d(camera_path="/World/Robot/Camera")

        # Save image
        from PIL import Image
        img = Image.fromarray(rgb)
        img.save(f"{output_dir}/images/{i:06d}.png")

        # Save annotation
        annotation = {
            "image_id": i,
            "bounding_boxes": bbox_2d.tolist()
        }
        annotations.append(annotation)

        if i % 100 == 0:
            print(f"Generated {i}/{num_samples} samples")

    # Save all annotations
    with open(f"{output_dir}/annotations.json", "w") as f:
        json.dump(annotations, f)
```

## Summary

- **Isaac Sim** provides photorealistic simulation with RTX rendering
- **PhysX 5** delivers accurate physics for robot dynamics
- **ROS2 Bridge** enables seamless integration with ROS2 ecosystem
- **Sensor simulation** includes cameras, LiDAR, and IMU with noise models
- **Synthetic data** generation enables scalable ML training

**Next**: [Isaac ROS Integration](./isaac-ros) - GPU-accelerated perception with Isaac ROS.
