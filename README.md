# Vision-Based Control for Autonomous Robotic Systems
**M1 Internship — DNIIT / DUT Da Nang**
Supervisor: Dr. Vo Nhu Thanh

---

## Context

This internship project focuses on developing a **visual servoing** system to guide a **4-DOF SCARA** robot in grasping moving objects (dynamic *pick-and-place* application on a conveyor belt).

The target robot is a **SCARA** (*Selective Compliance Assembly Robot Arm*) with 3 revolute joints + 1 prismatic joint, mounted above a conveyor belt. The camera is positioned above the scene to detect objects.

---

## Project Structure

```
.
├── README.md
├── plan_stage.md              ← Detailed plan for the 5 project phases
├── docs/                      ← Reference documentation (PDFs)
│   ├── offre-stage.pdf        ← Official internship subject
│   ├── Mobile_Manipulator.pdf ← Robot technical report (4-DOF, stereo vision)
│   ├── PBL6_2.pdf             ← Delta robot report (sorting, kinematics reference)
│   └── 1 - Thuyetminh_...pdf ← SCARA + YOLO report (vision reference)
├── matlab/
│   ├── phase1/                ← Phase 1: MATLAB Simulation
│   │   ├── main_phase1.m
│   │   ├── robot_parameters.m
│   │   ├── forward_kinematics.m
│   │   ├── inverse_kinematics.m
│   │   ├── workspace_analysis.m
│   │   ├── joint_space_trajectory.m
│   │   ├── cartesian_trajectory.m
│   │   ├── plot_trajectory.m
│   │   ├── pid_simulation.m
│   │   └── animate_robot.m
│   └── phase3/                ← Phase 3: MATLAB Closed-loop Simulation
│       └── main_phase3.m
├── python/
│   ├── phase2/                ← Phase 2: Object recognition and pose estimation
│   │   ├── main_phase2.py
│   │   ├── camera_calibration.py
│   │   ├── detect_objects.py
│   │   ├── pose_estimation.py
│   │   ├── robot_transform.py
│   │   ├── kalman_tracker.py
│   │   ├── realsense_capture.py
│   │   ├── generate_markers.py
│   │   └── requirements.txt
│   └── phase3/                ← Phase 3: Visual servoing controller
│       ├── main_phase3.py         ← Main pipeline (state machine, CLI)
│       ├── vs_controller.py       ← PBVS controller + SCARA kinematics
│       ├── visual_error.py        ← Visual error and interaction matrix
│       ├── gripper_controller.py  ← Pick-and-place sequencer (9 states)
│       └── simulation_gui.py      ← Interactive 2D simulation (matplotlib)
└── ros2/
    └── src/
        └── scara_visual_servoing/
            └── scara_visual_servoing/
                ├── vs_node.py             ← Phase 4: ROS2 visual servoing (PBVS)
                ├── sim_target_node.py     ← Phase 4: Gazebo conveyor belt simulation
                ├── gazebo_bridge.py       ← Phase 4: Home pose initialisation
                └── vision_node.py         ← Phase 5: YOLO detection + PLC write (snap7)
```

---

## Project Plan

The project is divided into **5 sequential phases**. See [plan_stage.md](plan_stage.md) for full details.

| Phase | Theme | Status |
|-------|-------|--------|
| **1** | MATLAB Simulation (model, kinematics, control) | ✅ Completed |
| **2** | Object recognition and pose estimation (Python, OpenCV) | ✅ Completed |
| **3** | PBVS visual servoing (Python + MATLAB) | ✅ Completed |
| **4** | ROS2 + Gazebo Ignition Fortress integration — pick-and-place simulation | ✅ Completed |
| **5** | Experimental tests on real robot (YOLO vision + Siemens PLC) | 🚧 In progress |

---

## Phase 1 — MATLAB Simulation

### Prerequisites

- MATLAB R2021b or later (no toolbox required — everything implemented from scratch)

### Run the simulation

```matlab
cd matlab/phase1
clear                % clear MATLAB function cache
main_phase1          % full run with figures + animation
main_phase1(false)   % console only, no figures
```

### What `main_phase1` does

The script runs 7 steps in sequence and prints a console report at each step.

---

### File Descriptions

#### `robot_parameters.m`

Defines the complete SCARA robot model as a MATLAB `robot` struct.

Topology: **θ1 (R) → d2 (P) → θ3 (R) → θ4 (R)**

DH table from the PBL6 report (Table 2):

| Link | θ | a (m) | α (°) | d (m) |
|------|---|-------|-------|-------|
| 1 | θ1 var | 0 | 0° | 0 |
| 2 | 0° fixed | **0.300** | 0° | d2 var |
| 3 | θ3 var | **0.160** | 180° | −0.150 |
| 4 | θ4 var | 0 | 0° | 0.059 |

Physical dimensions (Table 1 of the report):
- Upper arm a2 = 300 mm, forearm a3 = 160 mm
- Vertical stroke d3 = 150 mm, gripper offset d4 = 59 mm
- Max XY reach: 460 mm, dead zone: r > 340 mm

Also contains: joint limits `[θ1∈±135°, d2∈[0,200mm], θ3∈±90°, θ4∈±180°]`, PID gains, per-axis dynamic parameters.

---

#### `forward_kinematics.m`

**Problem:** given `q = [θ1, d2, θ3, θ4]`, where is the end-effector?

**Method:** composition of the 4 SCARA-specific DH matrices:

$$T_{0 \to 4} = A_1(\theta_1) \cdot A_2(d_2) \cdot A_3(\theta_3) \cdot A_4(\theta_4)$$

End-effector position (analytical formulas from the report FK):

$$P_x = a_2 \cos\theta_1 + a_3 \cos(\theta_1 + \theta_3)$$
$$P_y = a_2 \sin\theta_1 + a_3 \sin(\theta_1 + \theta_3)$$
$$P_z = d_2 - d_3 - d_4$$

```matlab
[T_end, T_all] = forward_kinematics(robot, q)
```

---

#### `inverse_kinematics.m`

**Problem:** given a target pose `(Px, Py, Pz)`, what commands to send?

**Method:** **Analytical** IK — closed-form from PBL6 report (eq. [1.8]–[1.11]):

1. $\cos\theta_3 = \dfrac{P_x^2 + P_y^2 - a_2^2 - a_3^2}{2 a_2 a_3}$ → 2 solutions (elbow up/down)

2. $\theta_1 = \text{atan2}\!\left(\dfrac{P_y k_1 - P_x k_2}{D},\ \dfrac{P_x k_1 + P_y k_2}{D}\right)$

3. $\theta_4 = -\theta_1 - \theta_3$ (orientation held at 0°)

4. $d_2 = P_z + d_3 + d_4$

Advantage over numerical methods: **exact solution in one computation**, no iteration, residual error = 0.0000 mm.

```matlab
[q_sol, success, err] = inverse_kinematics(robot, T_des, q0)
```

---

#### `workspace_analysis.m`

Visualises the SCARA **workspace** (envelope of all positions reachable by the end-effector).

**Method:** Monte Carlo — 30 000 random configurations in `[q_min, q_max]`, forward kinematics for each.

Expected results (consistent with Figures 11–12 of the PBL6 report):
- XY: toroidal ring between r = 340 mm and r = 460 mm (dead zone because θ3 ∈ ±90°)
- Z: horizontal working plane at variable height between −209 mm and −9 mm

Generates 4 figures: 3D view, XY projection, XZ projection, SCARA arm structure.

```matlab
workspace_analysis(robot)
workspace_analysis(robot, 50000)  % more samples
```

---

#### `joint_space_trajectory.m`

Generates a **point-to-point joint-space trajectory** from `q_start` to `q_end`.

3 available profiles:

| Profile | Continuity | Notes |
|--------|-----------|--------------|
| `cubic` | C¹ (velocity) | Standard, simple |
| `quintic` | C² (acceleration) | Smoothest motion |
| `trapezoidal` | C⁰ | Close to industrial drives |

Returns `traj.q`, `traj.dq`, `traj.ddq` (positions, velocities, accelerations).

```matlab
traj = joint_space_trajectory(q_start, q_end, T_total, dt, 'quintic')
```

---

#### `cartesian_trajectory.m`

Generates a **linear Cartesian trajectory** (end-effector follows a straight line).

- **Position:** linear interpolation with cubic smoothing
- **Orientation:** SLERP (*Spherical Linear Interpolation*) — correct rotation interpolation
- IK called at each point to obtain joint angles

```matlab
traj = cartesian_trajectory(robot, p_start, p_end, R_start, R_end, T_total, dt)
```

---

#### `plot_trajectory.m`

Visualises a generated trajectory (joint-space or Cartesian).

- **Joint-space** trajectory: 3 subplots (positions, velocities, accelerations) in °, °/s, °/s²
- **Cartesian** trajectory: XYZ coordinates vs time + 3D path

```matlab
plot_trajectory(traj)
```

---

#### `pid_simulation.m`

Simulates **closed-loop trajectory tracking** with an independent PID + feedforward controller on each axis.

**Dynamic model (parameters from `robot.J_eff`, `robot.B_vis`):**

$$J_{eff,i} \cdot \ddot{q}_i = \tau_i - B_{vis,i} \cdot \dot{q}_i$$

**Control law (feedforward + PID):**

$$\tau_i = \underbrace{J_{eff,i} \cdot \ddot{q}_{ref} + B_{vis,i} \cdot \dot{q}_{ref}}_{\text{feedforward}} + K_p e_i + K_i \int e_i \, dt + K_d (\dot{q}_{ref} - \dot{q}_i)$$

Safety: anti-windup, torque saturation, joint limits.
Integration via **Heun's method (RK2)** to avoid phase error accumulation.

Phase 1 results: RMSE < 0.5° on all axes (θ1, θ3, θ4) and < 0.5 mm on d2.

```matlab
result = pid_simulation(robot, traj, Kp, Ki, Kd)
```

---

#### `animate_robot.m`

Animates the SCARA in 3D along a trajectory. Reconstructs the physical robot segments (vertical column, horizontal links) for a realistic rendering.

```matlab
animate_robot(robot, traj)                 % normal speed
animate_robot(robot, traj, 'speed', 2)     % 2× faster
animate_robot(robot, traj, 'trail', false) % no end-effector trail
```

---

### Data Flow Between Modules

```
robot_parameters
       │
       ├──► forward_kinematics ──► workspace_analysis
       │           │
       │           └──► inverse_kinematics ◄── cartesian_trajectory
       │
       ├──► joint_space_trajectory ──► pid_simulation
       │                    │
       │                    └──► animate_robot
       │
       └──► main_phase1  (orchestrates everything)
```

### Phase 1 Success Criteria

- FK: $P_z = d_2 - d_3 - d_4$ always negative (end-effector below mounting plane) ✓
- Analytical IK: residual error = 0.0000 mm ✓
- Workspace: toroidal XY ring, r ∈ [340, 460] mm ✓
- PID+FF tracking: RMSE < 0.5° (revolute axes) and < 0.5 mm (prismatic axis) ✓

---

---

## Phase 2 — Object Recognition and Pose Estimation

### Prerequisites

```bash
cd python/phase2
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Optional — YOLO
pip install ultralytics

# Optional — Intel RealSense D4xx (RGB-D)
pip install pyrealsense2
```

### Run the pipeline

```bash
# Generate an ArUco marker sheet to display on phone
python generate_markers.py

# Test on a static image
python main_phase2.py --image test_images/test_aruco_scene.jpg --method aruco

# Live webcam (ArUco)
python main_phase2.py --live --method aruco --size 0.08  # 8 cm marker

# RealSense D435 stream (RGB-D, automatic firmware calibration)
python main_phase2.py --realsense --method aruco --size 0.08

# Webcam with YOLO (pre-trained COCO model)
python main_phase2.py --live --method yolo --model yolov8n.pt

# Camera calibration (physical chessboard required, not needed with RealSense)
python camera_calibration.py --live --rows 6 --cols 9 --size 25
```

### Selecting the right camera

```bash
# List all available cameras (webcam + RealSense)
python main_phase2.py --list-cams

# On macOS: index 0 = Continuity Camera (iPhone), index 1 = FaceTime
python main_phase2.py --live --cam 1 --method aruco
```

### RealSense D435 — RGB-D mode

When the RealSense is connected, the pipeline:
1. Loads **intrinsics directly from firmware** (no manual calibration needed).
2. Automatically aligns the depth stream onto the colour stream.
3. Activates `estimate_pose_yolo_rgbd()` (YOLO) — exact per-pixel 3D back-projection.
4. Provides a uint16 depth map in mm for `get_3d_point()`.

```bash
# Test the RealSense module standalone
python realsense_capture.py               # auto_camera() — RealSense if available, else webcam
python realsense_capture.py --list        # list RealSense cameras
python realsense_capture.py --cam 1 --z 900  # webcam index 1, work plane at 900 mm
```

If no RealSense is connected and `--realsense` is not passed, the pipeline falls back to `MockRealSense` (webcam + constant depth at working plane `--z`).

### Module Descriptions

#### `camera_calibration.py`
Intrinsic calibration from a chessboard. Generates `calibration_data/camera_params.npz` containing the K matrix (3×3) and distortion coefficients. RMS < 0.5 px = excellent calibration.

#### `detect_objects.py`
Two detection methods:
- **ArUco** — printed markers, exact pose (~0.5 mm), no learning. Ideal for testing and calibration.
- **YOLO** — natural object detection via YOLOv8 (ultralytics). Requires `pip install ultralytics`.

#### `pose_estimation.py`
6D pose estimation (position + orientation) in the camera frame:
- `estimate_pose_aruco()` — exact solvePnP on 4 marker corners
- `estimate_pose_yolo_rgbd()` — 3D position from depth map (RealSense D435)
- `estimate_pose_yolo_flat()` — back-projection onto known horizontal plane

#### `robot_transform.py`
Camera → robot frame transformation via homogeneous matrix T (4×4).
Hand-eye calibration (`hand_eye_calibrate()`) or direct geometric measurement.

#### `kalman_tracker.py`
Constant-velocity Kalman filter. Compensates for **pipeline latency (~150 ms)** by predicting the future object position on the conveyor. Multi-object tracker with nearest-neighbour association.

#### `realsense_capture.py`
Wrapper around `pyrealsense2` for the Intel RealSense D435 (or D415/D455):
- `RealSenseCapture` — aligned RGB + depth streams, firmware intrinsics
- `MockRealSense` — ordinary webcam emulation (constant depth)
- `auto_camera()` — automatic selection (RealSense if connected, otherwise webcam)

### Phase 2 Data Flow

```
Camera (webcam / RealSense D435)
       │
       ├─ colour frame (BGR)
       └─ depth frame (uint16 mm) ← RealSense only
       │
       ▼
[detect_objects]      → pixel position
       │
       ▼
[pose_estimation]     → 6D pose in camera frame (mm)
       │              (solvePnP ArUco | RealSense back-projection | flat plane)
       ▼
[robot_transform]     → pose in robot frame (mm)
       │
       ▼
[kalman_tracker]      → predicted position at t+150ms  ← actual pick target
       │
       ▼
  Phase 3: visual servoing controller
```

### Phase 2 Success Criteria

- ArUco detection: 100% on visible markers, latency < 5 ms ✓
- Pose estimation: solvePnP residual error < 1 mm (with real calibration) ✓
- Kalman: prediction error at +150 ms < 5 mm for v < 200 mm/s ✓

---

## Phase 3 — PBVS Visual Servoing

### Strategy: PBVS (Position-Based Visual Servoing)

The error is defined in 3D space (robot frame):

$$e = \begin{bmatrix} t_{current} - t_{desired} \\ \theta \cdot u \end{bmatrix} \in \mathbb{R}^6$$

Control law (integral controller):

$$v_c = -\lambda(\|e\|) \cdot L_s^+ \cdot e \qquad \dot{q} = J^+(q) \cdot v_c$$

- $L_s = \text{diag}(I_3,\; L_\omega)$ — block-diagonal PBVS interaction matrix
- $\lambda(\|e\|)$ — adaptive sigmoid gain ($\lambda \in [0.05,\, 1.5]$, inflection at 10 mm)
- $J^+(q)$ — DLS damped pseudo-inverse (robust to singularities)
- Secondary task: joint limit avoidance (Liegeois method, weight $w_0 = 0.1$)

### Main State Machine (`PipelineState`)

```
SEARCHING → PRE_APPROACH → TRACKING → APPROACH → CONVERGED
                ↑ (target jump >50 mm)               ↓
                └──────────────────────────────── EMERGENCY
```

| State | Trigger | Action |
|------|-------------|--------|
| `SEARCHING` | No object detected | Robot at rest |
| `PRE_APPROACH` | θ1/θ3 error > 8° | Joint-space interpolation to IK solution |
| `TRACKING` | Object detected, \|e\| > 50 mm | VS active, λ = 1.5 |
| `APPROACH` | \|e\| < 50 mm | Fine VS, λ = 1.0, v_max = 80 mm/s |
| `CONVERGED` | \|e_t\| < 2 mm and \|e_r\| < 1° | Gripper sequencer activated |
| `EMERGENCY` | σ_min(J) < threshold | Emergency stop |

### Pick-and-Place Sequencer (`PickPlaceState`)

```
IDLE → APPROACH → GRASPING → LIFTING → TRANSPORT → LOWERING → RELEASING → RETURNING → DONE
```

Automatically triggered on VS convergence. TRANSPORT and RETURNING transitions trigger a new `PRE_APPROACH` when the target changes by more than 50 mm.

### Non-singular Rest Configuration

The robot starts at $q_0 = [0°,\; 100\text{mm},\; -45°,\; 0°]$ (θ3 = −45°, r ≈ 430 mm, σ_min = 0.063). The point θ3 = 0° is singular (r = 460 mm, σ_min ≈ 0) and is avoided by the pre-approach.

### Validated Results (Python simulation)

- Convergence in **~30 frames** from a far position (after pre-approach)
- Final error: **< 2 mm** / **< 1°** ✓
- Effective workspace: ring $r \in [340,\; 460]$ mm, angles $\theta_1 \in [\pm 135°]$ (±163° at IK limits)
- Full pick-and-place cycle (APPROACH→DONE) validated in GUI simulation

### Run the interactive simulation (matplotlib GUI)

```bash
cd python/phase3
source ../phase2/.venv/bin/activate

# Interactive 2D simulation — left click: place object, right click: deposit, SPACE: start
python simulation_gui.py
```

Controls:
- **Left click** — place object (green area = workspace pixel-exact by IK computation)
- **Right click** — move the deposit zone
- **SPACE** — start / pause (arm goes to pre-approach then VS)
- **R** — reset

### Run the real controller

```bash
# Fixed target (camera bypass) — useful to validate without hardware
python main_phase3.py --live --cam 1 --force-target 350 50 -150

# Target "behind" the robot (requires pre-approach)
python main_phase3.py --live --cam 1 --force-target -300 200 -150

# Live webcam (ArUco)
python main_phase3.py --live --cam 1 --method aruco --size 0.08

# RealSense D435
python main_phase3.py --realsense --method aruco --size 0.05

# Python closed-loop simulation (no camera)
python main_phase3.py --sim

# Test on static image
python main_phase3.py --image ../phase2/test_images/test_aruco_scene.jpg
```

### Run the MATLAB simulation (closed-loop)

```matlab
cd matlab/phase3
main_phase3      % Runs simulation + displays 6 convergence graphs
```

Configurable (parameters at the top of the script): `use_conveyor`, `v_conveyor`, `lambda_nom`, `dt`, `T_sim`.

### Module Descriptions

#### `visual_error.py`
Visual error $e \in \mathbb{R}^6$ and interaction matrix $L_s$ computation:
- `compute_error(t_cur, R_cur, t_des, R_des)` → `VisualError` (with `.converged`)
- `axis_angle(R)` — axis-angle representation $\theta \cdot u$ (inverse Rodrigues formula)
- `interaction_matrix_pbvs(R_co, t_co, R_cd)` — exact block-diagonal $L_s$ (Chaumette 2006)

#### `vs_controller.py`
Full PBVS controller + shared kinematics:
- `ScaraParams` — geometric parameters and limits (mirror of `robot_parameters.m`)
- `ik_solutions(x_m, y_m, params)` — analytical SCARA IK, returns all (θ1, θ3) solutions within limits. Shared by `main_phase3.py` and `simulation_gui.py`
- `scara_jacobian(q, params)` — analytical Jacobian $J \in \mathbb{R}^{6 \times 4}$
- `damped_pinv(J)` — DLS pseudo-inverse, returns $\sigma_{\min}$ and singularity flag
- `VSController.update(error, q, dt)` → `VSCommand` (joint velocities + diagnostics)
- `simulate_pbvs(...)` — integrated Python simulation (FK + Euler integrator)

#### `gripper_controller.py`
Pick-and-place task sequencer:
- `GripperController` — gripper model (OPEN / CLOSING / CLOSED / OPENING), open/close timers
- `PickPlaceSequencer` — 9-state machine (IDLE → … → DONE), provides VS target and gripper command at each frame
- `draw_hud(frame)` — overlays current state as BGR overlay on camera frame
- VS target changes automatically (object → approach point → lift height → deposit → home return)

#### `simulation_gui.py`
Interactive 2D simulation of the full pick-and-place cycle:
- **XY view** — SCARA arm + pixel-exact workspace (mask computed by vectorised IK on 320×320 grid)
- **XZ view** — height profile (lift, transport, lower)
- **State panel** — real-time state machine, |e_t| error bar
- Integrated joint-space pre-approach: triggered at object placement *and* automatically on each sequencer target change (pick → deposit → home)
- Animated object follows the arm during GRASPING / LIFTING / TRANSPORT / LOWERING

#### `main_phase3.py`
Full Phase 2 + Phase 3 orchestration:
- `Phase3Pipeline` — 6 states: `SEARCHING → PRE_APPROACH → TRACKING → APPROACH → CONVERGED → EMERGENCY`
- Two input paths: `force_target` (camera bypass) and full camera pipeline (ArUco/YOLO)
- Pre-approach in both paths: detects sequencer target jumps ≥ 50 mm
- `effective_target` — target computed from `sequencer._target_pos` (not the raw object), avoids APPROACH freeze during LIFTING→TRANSPORT transitions
- Live HUD: state, |e_t|, |e_r|, current θ1/d2/θ3/θ4

#### `matlab/phase3/main_phase3.m`
MATLAB script for closed-loop simulation validation:
- Supports conveyor mode (`use_conveyor = true`, `v_conveyor`)
- Kalman latency compensation ($+150$ ms)
- 6 graphs: convergence, adaptive gain, XY trajectory, joint configurations, commands

### Phase 3 Data Flow

```
[Phase 2 pipeline]
  detection + 6D pose → Kalman → t_predicted
         │
         ▼
  ik_solutions(t_predicted)
  → PRE_APPROACH if θ1/θ3 far (> 8°) from IK solution
         │ (pre-approach complete)
         ▼
[visual_error.py]
  compute_error(t_cur, R_cur, t_effective, R_des)
       → e = [e_t ; e_r]  ∈ R^6
         │
         ▼
[vs_controller.py]
  VSController.update(e, q) → dq (joint velocities)
         │
         ├──► [gripper_controller.py]  (pick-and-place sequencer)
         │         → new target → jump detected → new PRE_APPROACH
         ▼
  SCARA Robot: q ← q + dq * dt
```

### Phase 3 Success Criteria

- Stable convergence from any initial position in the workspace ✓
- Final error < 2 mm / < 1° ✓ (validated in Python simulation)
- Joint-space pre-approach: reaches θ1 = ±163° without crossing the θ3 = 0° singularity ✓
- Full pick-and-place cycle (9 states) validated in GUI simulation ✓
- Automatic detection and pre-approach on each sequencer target change ✓
- Singularity detection + joint saturation ✓
- Conveyor belt support (moving object) via Kalman compensation ✓

---

## References

- **1 - Thuyetminh_PBL6...pdf** — SCARA YOLO report (source of robot model: Tables 1 & 2, analytical IK, S-curve profiles)
- **offre-stage.pdf** — Official subject: visual servoing for dynamic pick-and-place, YOLO + 6D pose estimation, ROS2 + MoveIt2
- **Mobile_Manipulator.pdf** — Stereo vision and arm mechanics reference
- Craig, J.J. — *Introduction to Robotics: Mechanics and Control*
- Chaumette, F. & Hutchinson, S. — *Visual Servo Control* (IEEE Robotics & Automation Magazine, 2006)

---

## Phase 3 Summary and Phase 4 Prerequisites

### What Phase 3 covers

| Component | File | Status |
|-----------|---------|--------|
| PBVS visual error + interaction matrix | `visual_error.py` | ✅ |
| VS controller (DLS, adaptive gain, joint limits) | `vs_controller.py` | ✅ |
| Shared analytical IK | `vs_controller.ik_solutions()` | ✅ |
| 9-state pick-and-place sequencer | `gripper_controller.py` | ✅ |
| Joint-space pre-approach (any initial position) | `main_phase3.py` | ✅ |
| Real-time camera pipeline (ArUco / YOLO / RealSense) | `main_phase3.py` | ✅ |
| Interactive 2D simulation (pixel-exact workspace) | `simulation_gui.py` | ✅ |
| MATLAB closed-loop + conveyor validation | `matlab/phase3/main_phase3.m` | ✅ |

### What is missing before Phase 4

| Element | Priority | Notes |
|---------|----------|-------|
| **SCARA URDF/XACRO model** | High | Required for Gazebo and MoveIt2. Can be generated from DH parameters in `robot_parameters.m` |
| **Motor and encoder drivers** | High | `q_current` interface — currently simulated in `main_phase3.py` by Euler integration |
| **ROS2 package (colcon)** | High | Wrapping `Phase3Pipeline.process_frame()` as a ROS2 node |
| **phase3 requirements.txt** | Low | Phase 3 uses the phase2 venv (`../phase2/.venv`) |

---

## Phase 4 — ROS2 + MoveIt2 + Gazebo Integration

### Overview

Phase 4 connects the visual servoing pipeline (Phase 3) to ROS2 Humble via:
- **`scara_description`** — URDF/XACRO model of the 4-DOF SCARA robot
- **`scara_moveit_config`** — MoveIt2 configuration (SRDF, IK, controllers, limits)
- **`scara_visual_servoing`** — ROS2 nodes wrapping the Phase 3 pipeline
- **Docker** — turnkey ROS2 + Gazebo + MoveIt2 environment for macOS

```
src/
├── scara_description/         ← URDF/XACRO + display launch
├── scara_moveit_config/       ← SRDF, kinematics.yaml, controllers, MoveIt2 launch
└── scara_visual_servoing/     ← ROS2 VS nodes + Gazebo simulation
Dockerfile
docker-compose.yml
```

ROS2 node architecture:

```
/vs/target_pose  ──────────────────────────────────────────────► [vs_node]
/joint_states ──────────────────────────────────────────────► [vs_node]
                                                                          │
                                                                  PBVS + DLS
                                                                  State machine
                                                                  TRACKING → CARRYING
                                                                          │
                                   /joint_trajectory_controller/joint_trajectory ◄─┘
                                             │
                                   [ros2_control / Gazebo DART]
                                             │
                              ┌──────────────┤
                         /joint_states   /vs/tcp_pose (30 Hz)
                              │               │
                         [vs_node]     [sim_target_node] → /vs/target_pose
                                             conveyor belt + Gazebo set_pose
```

### Prerequisites
**Native Linux/WSL2**
- Ubuntu 22.04
- ROS2 Humble Desktop Full
- `ros-humble-moveit`, `ros-humble-gazebo-ros2-control`

---

### Build Linux / WSL2

```bash
# Install ROS2 Humble + dependencies
sudo apt install ros-humble-moveit ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-joint-state-publisher-gui

# From workspace root
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y

colcon build --packages-select \
    scara_description scara_moveit_config scara_visual_servoing \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

# Verify packages are installed
ros2 pkg list | grep scara
```

---

### Kinematic Architecture in URDF

The URDF (`scara_robot.urdf.xacro`) exactly mirrors the DH parameters from Phase 1:

| URDF Joint | Type | Axis | Limits | FK |
|------------|------|-----|--------|----|
| `joint1` | revolute | +Z | [−135°, +135°] | θ1 |
| `joint2` | prismatic | +Z | [0, 200 mm] | d2 (Pz ↑ when d2 ↑) |
| `joint3` | revolute | +Z | [−90°, +90°] | θ3, origin at (a2=0.3, 0, 0) |
| `joint4` | revolute | +Z | [−180°, +180°] | θ4, origin at (a3=0.16, 0, −d3=−0.15) |
| `ee_joint` | fixed | — | — | origin at (0, 0, −d4=−0.059) |

FK verification:
- Px = a2·cos(θ1) + a3·cos(θ1+θ3) ✓
- Py = a2·sin(θ1) + a3·sin(θ1+θ3) ✓
- Pz = d2 − d3 − d4 = d2 − 0.209 m ✓ (EE always below mounting plane)

---

### Main ROS2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | → vs_node | Current joint state (BEST_EFFORT) |
| `/vs/target_pose` | `geometry_msgs/PoseStamped` | sim_target → vs_node | Target pose on belt |
| `/vs/deposit_pose` | `geometry_msgs/PoseStamped` | sim_target → vs_node | Deposit point |
| `/vs/status` | `std_msgs/String` | vs_node → | TRACKING / PICKED / CARRYING / DEPOSITED |
| `/vs/tcp_pose` | `geometry_msgs/PoseStamped` | vs_node → sim_target | Arm TCP position (for sphere in CARRYING) |
| `/vs/target_marker` | `visualization_msgs/Marker` | sim_target → | RViz2 sphere (target) |
| `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | vs_node → | Position command |

---

### Troubleshooting

| Problem | Solution |
|----------|----------|
| `Cannot open display :0` | `xhost + 127.0.0.1` then relaunch |
| `joint_trajectory_controller not active` | Wait ~10 s after Gazebo launch, or relaunch `gazebo_bridge` |
| `ModuleNotFoundError: vs_controller` | Check that `python/phase3/` is accessible from `/ros2_ws` |
| `No module named cv_bridge` | `apt install ros-humble-cv-bridge` inside the container |
| Gazebo does not start (macOS) | Try `headless:=true` + `ros2 launch ... use_sim_time:=true` |
| `rosdep: package not found` | `rosdep update && rosdep install ...` inside the container |
| colcon build error | `colcon build --packages-select scara_visual_servoing 2>&1 | tail -30` |

---

### Phase 4 Success Criteria

- URDF loads without error in RViz2 + Gazebo ✓
- All joints controllable via `joint_state_publisher_gui` ✓
- VS node converges in simulation (|et| < 2 mm) ✅
- Gazebo + ros2_control: position commands executed correctly ✅
- URDF FK = analytical FK from `vs_controller.py` ✅
- Red sphere visible and moving on the belt in Gazebo ✅
- Sphere follows arm TCP during CARRYING phase ✅
- Full pick-and-place cycle (TRACKING → WAIT_PICK → CARRYING → WAIT_DEP) looping ✅
