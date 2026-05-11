# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

```bash
# Source ROS2 environment (required in every shell)
source /opt/ros/humble/setup.bash
source install/setup.bash   # after first build

# Build all packages
colcon build --symlink-install

# Build a single package
colcon build --symlink-install --packages-select ids830_hw

# Run tests (flake8, pep257, copyright checks)
colcon test --packages-select flydigi
colcon test-result --verbose

# Launch full robot (lidar on /dev/ttyUSB0, SLCAN on /dev/ttyACM0)
ros2 launch bringup bringup.launch.py

# Launch with non-default lidar port
ros2 launch bringup bringup.launch.py lidar_serial_port:=/dev/ttyUSB1

# Launch SLAM mapping (after bringup)
ros2 launch slam slam.launch.xml

# Launch Flydigi gamepad controller
ros2 launch flydigi flydigi.launch.py

# Visualize in RViz
ros2 launch bringup rviz.launch.py
```

## Architecture

IvastBot is a ROS2 differential-drive robot with four packages:

### `ids830_hw` (C++)
ros2_control `SystemInterface` plugin for IDS830 Low-Voltage DC Servo motors. Communication uses a **SLCAN adapter** (`/dev/ttyACM0` at 921600 baud) which translates serial ASCII to CAN bus at 500 kbps (S6).

- IDS830 uses a **proprietary CAN protocol** (not CANopen). Each 8-byte frame: `[Group][FuncCode][Reg1][Data1_H][Data1_L][Reg2][Data2_H][Data2_L]`
- Speed formula: `value = (RPM / 3000) * 8192`
- Key registers: `0x00` enable, `0x02` speed mode, `0x06` target speed, `0x36` PC/PLC mode, `0xE8/0xE9` encoder high/low
- **Motor CAN IDs**: left=4, right=2 (set in `bringup/urdf/ivastbot.urdf.xacro`)
- On `on_configure()`: must call `motor_unlock_pc_mode()` (clears bit 5 of reg 0x36) before motors will accept PC speed commands — this was discovered via reverse engineering
- A background `rx_loop()` thread collects CAN responses with 100ms freshness timeout
- Right motor direction is inverted in both read and write paths
- Idle management: stops sending CAN frames after `SETTLE_THRESHOLD` (50) consecutive zero-command cycles

Hardware parameters (from `ids830.ros2_control.xacro`):
- `gear_ratio`: 9, `encoder_ppr`: 2500, `wheel_radius_m`: 0.085, `wheel_track_m`: 0.22327

### `bringup` (Python)
Main launch orchestration and robot URDF.

- `bringup.launch.py` starts: robot_state_publisher → controller_manager → (2s delay) → joint_state_broadcaster + drivetrain_controller → sllidar_ros2
- `controllers.yaml`: DiffDriveController at 100 Hz, max linear 1.5 m/s, max angular 2.2 rad/s
- `/cmd_vel` is remapped from `/drivetrain_controller/cmd_vel_unstamped`

URDF TF tree: `map` → `odom` → `base_footprint` → `base_link` → `laser_frame` (x=0.22, z=0.055 from base_link — 22cm forward, 14cm from ground)

### `flydigi` (Python)
Gamepad controller node using pygame. Publishes `geometry_msgs/Twist` to `/cmd_vel` at 20 Hz with smooth velocity ramping (accel=0.5 m/s², decel=8.0 m/s²). Auto-detects Flydigi controllers by name. Controls: Left Stick Y = linear, Right Stick X = angular, RB/LB = speed ±0.05 m/s.

### `slam` (CMake — config only)
Wraps slam_toolbox configuration. Subscribes to `/scan`. Run in `mapping` mode by default; change to `localization` in `config/slam.yaml` when a map exists.

## Key Device Paths

| Device | Default path | Baud |
|--------|-------------|------|
| SLCAN adapter (CAN→USB) | `/dev/ttyACM0` | 921600 |
| RPLidar A2M8 | `/dev/ttyUSB1` | 115200 |

## Known Open TODOs

- `controllers.yaml` and `ids830.ros2_control.xacro`: `wheel_radius` and `wheel_separation` are estimates from URDF/SolidWorks — need physical measurement with calipers/ruler for accurate odometry
- `ivastbot.urdf.xacro`: LiDAR position (`x=0.03, z=0.14`) marked as TODO to verify

## Package Layout Note

The `flydigi` package has a duplicated nested structure (`src/flydigi/flydigi/`). The canonical source files are at `src/flydigi/flydigi/flydigi_node.py` and `src/flydigi/launch/`. The inner `src/flydigi/flydigi/` directory is an older duplicate and should be treated as stale.
