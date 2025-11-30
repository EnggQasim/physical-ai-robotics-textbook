---
sidebar_position: 2
title: "Isaac Sim بنیادیات"
description: "NVIDIA Isaac Sim کے ساتھ فوٹو ریئلسٹک روبوٹ سمیولیشنز بنانا"
---

# Isaac Sim بنیادیات

**Isaac Sim** NVIDIA کا روبوٹکس سمیولیشن پلیٹ فارم ہے جو Omniverse پر بنایا گیا ہے۔ یہ فوٹو ریئلسٹک رینڈرنگ، درست فزکس، اور ہموار ROS2 انٹیگریشن فراہم کرتا ہے۔

## سیکھنے کے مقاصد (Learning Objectives)

- Isaac Sim انٹرفیس نیویگیٹ کریں
- روبوٹ ماڈلز امپورٹ اور کنفیگر کریں
- سمیولیشن ماحول بنائیں
- Isaac Sim کو ROS2 سے جوڑیں

## Isaac Sim شروع کرنا

### Isaac Sim چلانا

```bash
# Omniverse Launcher کے ذریعے
# 1. NVIDIA Omniverse Launcher کھولیں
# 2. Library > Isaac Sim پر جائیں
# 3. Launch پر کلک کریں

# Docker کے ذریعے (دوبارہ قابل پیداوار کے لیے تجویز کردہ)
docker run --gpus all -e "ACCEPT_EULA=Y" \
  --rm --network=host \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

### انٹرفیس کا جائزہ

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
│ └───────────┘ └─────────────────────────────┘ └───────────────┘│
├─────────────────────────────────────────────────────────────────┤
│ │  Content Browser  │  Console  │  Timeline  │  Extensions   │ │
└─────────────────────────────────────────────────────────────────┘
```

## اپنی پہلی سمیولیشن بنانا

### مرحلہ 1: نیا Stage بنائیں

```python
# Isaac Sim کے لیے Python سکرپٹ
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, GroundPlane

# سمیولیشن ورلڈ بنائیں
world = World(stage_units_in_meters=1.0)

# گراؤنڈ پلین شامل کریں
world.scene.add_default_ground_plane()

# ڈائنامک کیوب شامل کریں
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="my_cube",
        position=[0, 0, 1.0],
        size=0.5,
        color=[1.0, 0.0, 0.0]  # سرخ
    )
)

# شروع کریں اور چلائیں
world.reset()
for i in range(1000):
    world.step(render=True)
```

### مرحلہ 2: روبوٹ امپورٹ کریں

Isaac Sim متعدد روبوٹ فارمیٹس سپورٹ کرتا ہے:

```python
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot

# URDF سے امپورٹ کریں
from omni.isaac.urdf import _urdf
urdf_interface = _urdf.acquire_urdf_interface()

# امپورٹ سیٹنگز کنفیگر کریں
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = False
import_config.import_inertia_tensor = True

# روبوٹ امپورٹ کریں
result = urdf_interface.parse_urdf(
    "/path/to/robot.urdf",
    import_config
)
```

### مرحلہ 3: سینسرز شامل کریں

```python
from omni.isaac.sensor import Camera, IMUSensor, LidarRtx

# RGB کیمرہ شامل کریں
camera = Camera(
    prim_path="/World/Robot/Camera",
    position=[0, 0, 0.5],
    frequency=30,
    resolution=(640, 480)
)
camera.initialize()

# LiDAR سینسر شامل کریں
lidar = world.scene.add(
    LidarRtx(
        prim_path="/World/Robot/Lidar",
        name="lidar",
        rotation_frequency=10,
        horizontal_fov=360.0,
        max_range=100.0
    )
)

# IMU سینسر شامل کریں
imu = IMUSensor(
    prim_path="/World/Robot/IMU",
    name="imu",
    frequency=100,
    translation=[0, 0, 0]
)
```

## فزکس کنفیگریشن

### PhysX سیٹ اپ کرنا

```python
from omni.isaac.core.utils.physics import set_physics_properties

# فزکس سین کنفیگر کریں
set_physics_properties(
    stage=world.stage,
    physics_dt=1/120.0,  # فزکس ٹائم سٹیپ
    rendering_dt=1/60.0,  # رینڈرنگ ٹائم سٹیپ
    gravity=[0.0, 0.0, -9.81],
    solver_type="TGS",  # Temporal Gauss-Seidel
    solver_position_iterations=8,
    solver_velocity_iterations=1
)
```

## ROS2 برج

Isaac Sim براہ راست ROS2 کے ساتھ انٹیگریٹ ہوتا ہے:

### ROS2 برج فعال کرنا

```python
from omni.isaac.ros2_bridge import ROS2Bridge

# ROS2 ایکسٹینشن فعال کریں
import omni.kit.commands
omni.kit.commands.execute("ROS2BridgeCreatePrim")

# یا Python API کے ذریعے
ros2_bridge = ROS2Bridge()
ros2_bridge.initialize()
```

### سینسر ڈیٹا پبلش کرنا

```python
from omni.isaac.ros2_bridge import ROS2Camera, ROS2Lidar, ROS2IMU

# کیمرہ پبلشر بنائیں
camera_pub = ROS2Camera(
    prim_path="/World/Robot/Camera",
    topic_name="/robot/camera/image_raw",
    frame_id="camera_link"
)

# LiDAR پبلشر بنائیں
lidar_pub = ROS2Lidar(
    prim_path="/World/Robot/Lidar",
    topic_name="/robot/scan",
    frame_id="lidar_link"
)
```

## سنتھیٹک ڈیٹا جنریشن

### خودکار اینوٹیشن

```python
from omni.isaac.synthetic_data import SyntheticDataHelper

# سنتھیٹک ڈیٹا ہیلپر شروع کریں
sd_helper = SyntheticDataHelper()

# مختلف اینوٹیشنز حاصل کریں
rgb = sd_helper.get_rgb(camera_path="/World/Robot/Camera")
depth = sd_helper.get_depth(camera_path="/World/Robot/Camera")
semantic = sd_helper.get_semantic_segmentation(camera_path="/World/Robot/Camera")
bbox_2d = sd_helper.get_bounding_box_2d(camera_path="/World/Robot/Camera")
bbox_3d = sd_helper.get_bounding_box_3d(camera_path="/World/Robot/Camera")
```

## خلاصہ (Summary)

- **Isaac Sim** RTX رینڈرنگ کے ساتھ فوٹو ریئلسٹک سمیولیشن فراہم کرتا ہے
- **PhysX 5** روبوٹ ڈائنامکس کے لیے درست فزکس فراہم کرتا ہے
- **ROS2 برج** ROS2 ایکو سسٹم کے ساتھ ہموار انٹیگریشن فعال کرتا ہے
- **سینسر سمیولیشن** میں شور ماڈلز کے ساتھ کیمرے، LiDAR، اور IMU شامل ہیں
- **سنتھیٹک ڈیٹا** جنریشن سکیل ایبل ML ٹریننگ فعال کرتی ہے

**اگلا**: [Isaac ROS انٹیگریشن](./isaac-ros) - Isaac ROS کے ساتھ GPU ایکسلریٹڈ پرسیپشن۔
