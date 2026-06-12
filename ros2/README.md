# Phase 4 — ROS2 SCARA Simulation + Visual Servoing

Full simulation of a 4-DOF SCARA robot with pick-and-place on a conveyor belt,
using ROS2 Humble, Gazebo Ignition Fortress, and ros2_control.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Project Architecture](#project-architecture)
3. [Robot Parameters](#robot-parameters)
4. [Installation and Build](#installation-and-build)
5. [Launch](#launch)
6. [Node Descriptions](#node-descriptions)
7. [Pick-and-Place Scenario](#pick-and-place-scenario)
8. [Topics and Interfaces](#topics-and-interfaces)
9. [Fixed Bugs](#fixed-bugs)
10. [macOS → Linux Sync](#macos--linux-sync)

---

## Prerequisites

### System
- Ubuntu 22.04
- ROS2 Humble
- Gazebo Ignition Fortress 6

### Required ROS2 packages

```bash
sudo apt install \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-robot-state-publisher \
  ros-humble-xacro
```

### Python (virtual env `env/`)

```bash
# Already installed in env/ (ultralytics, opencv, numpy, matplotlib, sympy...)
source env/bin/activate
```

---

## Project Architecture

```
ros2/
├── src/
│   ├── scara_description/          # Robot URDF/XACRO
│   │   └── urdf/scara_robot.urdf.xacro
│   ├── scara_moveit_config/        # ros2_control + MoveIt2 configuration
│   │   └── config/ros2_controllers.yaml
│   └── scara_visual_servoing/      # Main ROS2 nodes
│       ├── launch/
│       │   ├── gazebo_sim.launch.py   # Full launch (Gazebo + VS)
│       │   └── bringup.launch.py      # Launch without Gazebo (debug)
│       └── scara_visual_servoing/
│           ├── vs_node.py             # PBVS visual servoing node
│           ├── sim_target_node.py     # Conveyor belt simulation
│           ├── gazebo_bridge.py       # Home pose initialisation
│           └── vision_node.py         # [Phase 5] YOLO detection + PLC write (snap7)
├── rebuild.sh                         # Full rebuild script
└── README.md                          # This file
```

---

## Robot Parameters

### DH Parameters (4-DOF SCARA)

| Parameter | Value | Description |
|-----------|--------|-------------|
| a2        | 0.40 m | Link 1 length |
| a3        | 0.30 m | Link 2 length |
| d3        | 0.10 m | Link 3 vertical offset |
| d4        | 0.15 m | End-effector offset |

### Joints (DH convention order)

| Joint | Type | Limits | Init | Description |
|-------|------|---------|------|-------------|
| joint_1theta | Revolute (Z+) | ±135° | 0 rad | Base rotation |
| joint_1z     | Prismatic (Z+) | 0–0.4 m | 0.35 m | Vertical translation |
| joint3       | Revolute (Z-) | ±90° | **0.3 rad** | Elbow rotation |
| joint4       | Revolute (Z-) | ±180° | 0 rad | Wrist rotation |

> **Note**: joint3 starts at 0.3 rad (not 0) to avoid the configuration singularity
> that occurs when both links are perfectly aligned (θ3 = 0).

### Forward Kinematics (FK)

```
px = a2·cos(θ1) + a3·cos(θ1 + θ3)
py = a2·sin(θ1) + a3·sin(θ1 + θ3)
pz = d2 - d3 - d4   (d3=0.10, d4=0.15 → total offset 0.25 m)
```

Home pose [0, 0.40, 0.3, 0] → tool at z = 0.40 − 0.25 = **0.15 m**

### Workspace

- Radius: r ∈ [0.10, 0.70] m
- Tool height: pz ∈ [−0.25, +0.15] m (via d2 ∈ [0, 0.4])
- Conveyor pick zone: **(0.55, 0.0, 0.09)** m — 6 cm of descent available from home

---

## Installation and Build

### First installation

```bash
cd ~/Documents/project-10/ros2

# Source ROS2
source /opt/ros/humble/setup.bash

# Build the 3 packages
colcon build --packages-select \
  scara_description \
  scara_moveit_config \
  scara_visual_servoing \
  --symlink-install

source install/setup.bash
```

### Rebuild after modification

```bash
# Full rebuild with clean
bash rebuild.sh

# OR quick rebuild (Python only, no URDF/YAML change)
colcon build --packages-select scara_visual_servoing --symlink-install
source install/setup.bash
```

### Rebuild required when modifying:
- `scara_robot.urdf.xacro` → rebuild `scara_description`
- `ros2_controllers.yaml` → rebuild `scara_moveit_config`
- `*.py` → rebuild `scara_visual_servoing` (or just `touch` + relaunch with symlink-install)

---

## Launch

```bash
# Source (required in every terminal)
source /opt/ros/humble/setup.bash
source ~/Documents/project-10/ros2/install/setup.bash

# Full simulation (Gazebo GUI + VS + conveyor belt)
ros2 launch scara_visual_servoing gazebo_sim.launch.py

# Headless (CI, server)
ros2 launch scara_visual_servoing gazebo_sim.launch.py headless:=true

# With MoveIt2
ros2 launch scara_visual_servoing gazebo_sim.launch.py with_moveit:=true
```

### Startup Sequence (timings)

| t (s) | Event |
|-------|-----------|
| 0     | Gazebo Ignition starts |
| 2     | Remove old entities (if restarting) |
| 4     | Spawn SCARA robot |
| 5.5   | Spawn conveyor belt (green, static) |
| 6.0   | Spawn deposit zone (blue, static) |
| 6.5   | Spawn object sphere (red, **dynamic**) |
| 20    | Load `joint_state_broadcaster` |
| 26    | Load `joint_trajectory_controller` |
| 28    | `sim_target_node` starts — conveyor moving |

---

## Node Descriptions

### `vs_node.py` — PBVS Visual Servoing

Main control node. Implements a PBVS (Position-Based Visual Servoing) loop with:

- **Controller**: `VSController` (DLS — Damped Least Squares)
- **Frequency**: 30 Hz (dt = 0.033 s)
- **Gain**: 1.2 (fixed gain, `adaptive=True` via ROS2 parameter)
- **Safety**: velocity saturation (dq_max = [2.0, 0.2, 2.0, 2.0] rad/s or m/s)
- **TCP publication**: `/vs/tcp_pose` (PoseStamped, 30 Hz) — used by sim_target_node

**Pick-and-place state machine**:

```
TRACKING ──(|et|<2mm)──► WAIT_PICK ──(1s)──► CARRYING ──(|et|<2mm)──► WAIT_DEP ──(1.5s)──► TRACKING
```

- `TRACKING`: robot follows conveyor target (published by sim_target_node)
- `WAIT_PICK`: hold 1 s, publish `status=PICKED`
- `CARRYING`: robot moves to deposit `(0.0, −0.55, 0.09)`
- `WAIT_DEP`: pause 1.5 s, publish `status=DEPOSITED`

**Trajectory publication**:
- `header.stamp = Time(sec=0)` → JTC interprets as "start at reception"
- `time_from_start = max(dt×1.2, 40ms)` → tight execution window

### `sim_target_node.py` — Conveyor Belt Simulation

Simulates an object (red sphere) moving along a conveyor belt and synchronises
its position in Gazebo with the logical scenario state.

| Parameter | Value |
|-----------|--------|
| Conveyor X position | 0.55 m (facing robot, arm extended) |
| Object entry Y | −0.45 m |
| Object exit Y | +0.45 m |
| Pick zone Y | 0.0 m |
| Sphere centre Z | 0.09 m (belt top 0.05 + radius 0.04) |
| Belt speed | 0.04 m/s |
| Pick zone pause | up to 60 s |
| Deposit point | (0.0, −0.55, 0.09) m |

**Gazebo sphere movement** — two approaches in priority order:
1. **Python `ignition.transport` bindings** (in-process, same transport partition as Gazebo — reliable)
2. **Subprocess `ign service`** (fallback, 2000 ms timeout)

### `vision_node.py` — YOLO Detection + PLC Interface (Phase 5)

Vision node for the **real robot**. Subscribes to the ROS2 camera feed, detects
target objects with YOLOv8, computes their world-frame coordinates, and writes
the result directly to a **Siemens PLC** (S7 API) via the **snap7** protocol.

- **Subscription**: `/camera/image_raw` (Image)
- **Publication**: `/yolo/detections` (annotated Image, 30 Hz)
- **YOLO model**: `yolov8n.pt` (COCO classes used: 32=ball, 47=cup, 49=orange)
- **World position computation**:

  ```
  x_cam = (cx_px − 320) × z_dist / f        (f = 554.25 px, z_dist = 0.75 m)
  y_cam = (cy_px − 240) × z_dist / f
  world_x = cam_x − y_cam                   (cam_x = 0.55 m)
  world_y = cam_y − x_cam                   (cam_y = 0.0 m)
  world_z = 0.05 m                           (fixed belt height)
  ```

- **PLC write**: 3 big-endian floats (12 bytes) → **DB1, offset 0** (X, Y, Z) via `snap7.client.Client`
- **Auto-reconnect**: on write failure, immediate reconnection attempt

**Additional requirements (real robot)**:

```bash
pip install python-snap7       # snap7 bindings
# snap7 must be compiled on the system (libsnap7.so)
# Siemens PLC reachable at 192.168.0.10 (rack=0, slot=1)
```

> **Note**: this node is designed for the physical robot and does not work in
> pure simulation (no Gazebo, no `/vs/target_pose`). It replaces the logic of
> `sim_target_node.py` to provide the target position from real vision.

### `gazebo_bridge.py` — Initialisation

Sends the home pose [0, 0.35, 0.3, 0] to the JTC at startup.

---

## Pick-and-Place Scenario

```
1. Object (red sphere, r=4 cm) enters the belt from y=−0.45 m
2. Object moves in +Y at 0.04 m/s (along belt x=0.55 m)
3. When y ≈ 0.0 m (pick zone) → belt stops for 3 s
4. sim_target_node publishes target (0.55, 0.0, 0.09) on /vs/target_pose
5. vs_node converges to target: |et| < 2 mm (6 cm descent from home z=0.15)
6. Hold 1 s → status "PICKED" published → sphere follows arm TCP
7. vs_node moves to deposit (0.0, −0.55, 0.09) with status=CARRYING
8. On convergence: hold 1.5 s → status "DEPOSITED" published
9. New object on belt from y=−0.45 → cycle repeats
```

---

## Topics and Interfaces

### Published Topics

| Topic | Type | Node | Description |
|-------|------|------|-------------|
| `/joint_trajectory_controller/joint_trajectory` | `JointTrajectory` | vs_node | Joint commands |
| `/vs/status` | `String` | vs_node | TRACKING / PICKED / DEPOSITED / SINGULAR |
| `/vs/error` | `Vector3` | vs_node | Error norm (mm, deg, 0) |
| `/vs/target_pose` | `PoseStamped` | sim_target_node | Pick target position |
| `/vs/deposit_pose` | `PoseStamped` | sim_target_node | Deposit position |
| `/vs/target_marker` | `Marker` | sim_target_node | RViz visualisation (sphere) |

### Subscribed Topics

| Topic | Type | Node | QoS |
|-------|------|------|-----|
| `/joint_states` | `JointState` | vs_node | **BEST_EFFORT** (required) |
| `/vs/tcp_pose` | `PoseStamped` | sim_target_node | RELIABLE |
| `/vs/status` | `String` | sim_target_node | RELIABLE |

> **Important**: `joint_state_broadcaster` publishes with BEST_EFFORT.
> Subscribing with RELIABLE → no messages received.

---

## Fixed Bugs

### 1. Robot invisible in Gazebo
**Cause**: Spawn too early, Gazebo not yet ready.
**Fix**: `TimerAction(period=4.0)` before spawn.

### 2. No `/joint_states` received
**Cause**: QoS mismatch — vs_node subscribed RELIABLE, JSB publishes BEST_EFFORT.
**Fix**: `QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT)`.

### 3. Permanent singularity (robot does not move)
**Cause**: joint3 initial = 0 rad → aligned links → insufficient Jacobian rank.
**Fix**: `initial_value = 0.3 rad` in URDF + `self.q = [0, 0.40, 0.3, 0]` in vs_node.

### 4. Trajectories rejected "ends in the past"
**Cause**: `header.stamp = now()` + `time_from_start = 50ms` → trajectory expired on receipt.
**Fix**: `header.stamp = Time(sec=0, nanosec=0)` → JTC interprets "start at reception" + `time_from_start = 40ms`.

### 5. JTC "Failed to activate" (spawner timeout)
**Cause**: Spawner too short for `switch_controller` confirmation.
**Fix**: `--controller-manager-timeout 30` + `open_loop_control: true`.

### 6. Object stuck in pick zone (timeout)
**Cause**: After timeout, object immediately re-entered the zone.
**Fix**: 60 s timeout + skip to `PICK_ZONE_Y + 0.10` after timeout.

### 7. Numeric parameters treated as strings
**Cause**: `DeclareLaunchArgument` without `value_type=float`.
**Fix**: `value_type=float` for gain, target_x/y/z.

### 8. Wrong joint names
**Cause**: Inherited from Phase 3.
**Fix**: `JOINT_NAMES = ['joint_1theta', 'joint_1z', 'joint3', 'joint4']`.

### 9. Sphere spawns at (0, 0, 0) instead of desired position
**Cause**: In Ignition Fortress, `ros_gz_sim create` may ignore the `<pose>` tag inside the SDF.
**Fix**: Always pass `-x -y -z` as CLI arguments to `create` (take priority over SDF).

### 10. Static / kinematic sphere ignored by `set_pose`
**Cause**: In Ignition Fortress + DART, `set_pose` on `static=true` or `kinematic=true` bodies is silently ignored.
**Fix**: **Dynamic** body (neither static nor kinematic) + `<collision>` → sphere rests on belt through contact physics and responds to `set_pose`.

### 11. `<gravity>false</gravity>` ignored by DART
**Cause**: DART physics engine (Ignition Fortress default) does not support per-link gravity disable. Tag is ignored → sphere fell through the floor.
**Fix**: Dynamic body with collision (sphere r=0.04 m, static belt with collision) — contact physics keeps the sphere on the belt.

### 12. `set_pose` subprocess timeout
**Cause**: `ign service` launched as subprocess cannot reach the Ignition transport network of Gazebo (multicast discovery isolation in Docker container).
**Fix**: Use Python **`ignition.transport` bindings** directly inside the `sim_target_node` process — same transport partition as Gazebo → guaranteed communication. Subprocess kept as fallback with 2000 ms timeout.

### 13. Robot resetting in loop (changing target)
**Cause**: Every `/vs/target_pose` publication reset the `_converged` flag, even when the target only moved a few mm (publication noise).
**Fix**: `_cb_target_pose` only resets `_converged` if the target jumps by more than 50 mm.

---

## macOS → Linux Sync

Files are developed on macOS (`/Users/lorenzo/test/ros2/`) and manually synced to Linux (`~/Documents/project-10/ros2/`).

### Files to sync after each modification

```bash
# From macOS to Linux (example with scp)
scp src/scara_description/urdf/scara_robot.urdf.xacro        linux:~/Documents/project-10/ros2/src/scara_description/urdf/
scp src/scara_moveit_config/config/ros2_controllers.yaml      linux:~/Documents/project-10/ros2/src/scara_moveit_config/config/
scp src/scara_visual_servoing/scara_visual_servoing/vs_node.py linux:~/Documents/project-10/ros2/src/scara_visual_servoing/scara_visual_servoing/
scp src/scara_visual_servoing/scara_visual_servoing/sim_target_node.py linux:~/Documents/project-10/ros2/src/scara_visual_servoing/scara_visual_servoing/
scp src/scara_visual_servoing/launch/gazebo_sim.launch.py     linux:~/Documents/project-10/ros2/src/scara_visual_servoing/launch/
```

### After sync

```bash
cd ~/Documents/project-10/ros2

# Force detection of modified files
find src/scara_description src/scara_moveit_config src/scara_visual_servoing -exec touch {} +

# Rebuild
colcon build --packages-select scara_description scara_moveit_config scara_visual_servoing --symlink-install

source install/setup.bash
```

### Quick checks on Linux

```bash
# joint3 initial = 0.3 (anti-singularity)
grep -A10 'name="joint3"' src/scara_description/urdf/scara_robot.urdf.xacro | grep initial_value
# Expected: <param name="initial_value">0.3</param>

# Trajectories with stamp=0
grep "Time(sec=0" src/scara_visual_servoing/scara_visual_servoing/vs_node.py
# Expected: msg.header.stamp = Time(sec=0, nanosec=0)

# JTC open loop
grep "open_loop" src/scara_moveit_config/config/ros2_controllers.yaml
# Expected: open_loop_control: true
```

---

## Useful Commands During Execution

```bash
# Watch joints in real time
ros2 topic echo /joint_states

# Watch VS status
ros2 topic echo /vs/status

# Watch convergence error (mm and degrees)
ros2 topic echo /vs/error

# Watch sent trajectories
ros2 topic echo /joint_trajectory_controller/joint_trajectory

# Controller status
ros2 control list_controllers

# Manually send a test trajectory (home)
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [joint_1theta, joint_1z, joint3, joint4],
    points: [{positions: [0.3, 0.30, 0.3, 0.0], time_from_start: {sec: 2}}]}'
```
