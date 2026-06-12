# Python — Vision & PBVS Visual Servoing for SCARA

This folder contains all Python code for the project, split into two phases.

```
python/
├── phase2/    ← Object detection and pose estimation
└── phase3/    ← PBVS visual servoing controller + simulation
```

---

## Phase 2 — Object Detection and Pose Estimation

### Objective

Detect objects on the conveyor belt from a fixed camera, estimate their 6D pose (position + orientation) in the robot frame, and predict their future position to compensate for pipeline latency.

### Run

```bash
cd python/phase2
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Generate an ArUco marker sheet (print or display on screen)
python generate_markers.py

# Test on a static image
python main_phase2.py --image test_images/test_aruco_scene.jpg --method aruco

# Live webcam (ArUco)
python main_phase2.py --live --cam 1 --method aruco --size 0.08

# YOLO on webcam
python main_phase2.py --live --cam 1 --method yolo --model yolov8n.pt

# RealSense D435 (RGB-D, auto calibration from firmware)
python main_phase2.py --realsense --method aruco --size 0.05

# Camera calibration (physical chessboard required, not needed with RealSense)
python camera_calibration.py --live --rows 6 --cols 9 --size 25

# List available cameras
python main_phase2.py --list-cams
```

### Files

#### `requirements.txt`
Python dependencies: `numpy`, `opencv-python`, `scipy`.
Optional: `ultralytics` (YOLO), `pyrealsense2` (RealSense D435).

#### `camera_calibration.py`
Intrinsic calibration from a chessboard pattern.
Produces `calibration_data/camera_params.npz` containing the K matrix (3×3) and distortion coefficients.
RMS < 0.5 px = excellent calibration.
Not needed with a RealSense D435 (parameters read from firmware).

```python
load_calibration("calibration_data/camera_params.npz")  # → dict {K, dist}
```

#### `detect_objects.py`
Two detection methods, both returning normalised `Detection` objects:

- **ArUco** — printed markers, processing time < 5 ms, no learning required. Provides 4 exact corners for solvePnP.
- **YOLO (YOLOv8)** — natural object detection, confidence score, pixel bounding box.

```python
Detection.method        # 'aruco' | 'yolo'
Detection.bbox          # (x, y, w, h) pixels
Detection.corners       # 4×2 (ArUco only)
Detection.center_px     # (cx, cy)
```

#### `pose_estimation.py`
6D pose estimation (position + orientation) in the camera frame.

- `estimate_pose_aruco(detections, K, D, marker_size_m)` — exact solvePnP on 4 marker corners
- `estimate_pose_yolo_rgbd(detections, K, D, depth_frame)` — 3D position from RealSense depth map
- `estimate_pose_yolo_flat(detections, K, D, z_plane_m)` — back-projection onto known horizontal plane

```python
Pose6D.t_cam       # position (x, y, z) in metres, camera frame
Pose6D.R_cam       # 3×3 rotation matrix
Pose6D.T_cam       # 4×4 homogeneous matrix
Pose6D.euler_deg   # ZYX Euler angles in degrees
```

#### `robot_transform.py`
Transforms a `Pose6D` from camera frame to robot frame via the extrinsic matrix T (4×4).

- `RobotTransform.transform(pose_cam)` → `Pose6D` in robot frame
- `hand_eye_calibrate(R_list, t_list, R_gripper_list, t_gripper_list)` — hand-eye calibration
- `save_transform()` / `load_transform()` — persist to `.npz`

The matrix T is determined once at camera installation (geometric measurement or hand-eye calibration).

#### `kalman_tracker.py`
Constant-velocity Kalman filter (6D state: position + velocity).

**Why?** The vision pipeline takes ~150 ms. Without prediction, the robot targets the object's old position. The Kalman filter predicts where the object will be when the command is executed.

- `KalmanTracker(dt, sigma_accel, sigma_pos)` — single-object tracker
- `tracker.update(pos_3d)` — update with new measurement
- `tracker.predict_at(latency_s)` → predicted position at t + latency
- `MultiObjectTracker` — automatic association by nearest neighbour

#### `realsense_capture.py`
Wrapper around `pyrealsense2`:

- `RealSenseCapture` — aligned RGB + depth streams, firmware intrinsics
- `MockRealSense` — webcam emulation (constant depth, for tests without hardware)
- `auto_camera()` — automatically selects RealSense if connected, otherwise webcam

#### `generate_markers.py`
Generates an ArUco marker sheet (DICT_4X4_50 dictionary) as a PDF/image for printing.

#### `main_phase2.py`
Orchestrates the full real-time pipeline with OpenCV display:
1. Capture (webcam / RealSense)
2. Detection (ArUco or YOLO)
3. Pose estimation
4. Transform to robot frame
5. Kalman update
6. HUD display (pose, 3D axes, bounding box)

### Data Flow

```
Camera
  ├── color frame (BGR)   → detect_objects → pose_estimation
  └── depth frame (mm)    ↗ (RealSense only)
                              ↓
                        robot_transform → kalman_tracker → t_predicted [m]
                                                               ↓
                                                         Phase 3: VS
```

### Expected Results

| Metric | Target value |
|---|---|
| ArUco detection | 100% on visible markers, < 5 ms |
| solvePnP error | < 1 mm (with real calibration) |
| Kalman error at +150 ms | < 5 mm for v < 200 mm/s |

---

## Phase 3 — PBVS Visual Servoing

### Objective

Guide the SCARA robot from its current position to the detected object, then execute the full pick-and-place cycle (approach → grasp → lift → transport → deposit → return). The control law is a PBVS (Position-Based Visual Servoing) controller.

### Run

```bash
cd python/phase3
source ../phase2/.venv/bin/activate   # Phase 3 reuses the Phase 2 venv

# Interactive 2D simulation (matplotlib) — no hardware required
python simulation_gui.py

# Full pipeline with fixed target (camera bypass)
python main_phase3.py --live --cam 1 --force-target 350 50 -150   # x y z in mm

# Target requiring pre-approach (behind the robot)
python main_phase3.py --live --cam 1 --force-target -300 200 -150

# Real camera pipeline (ArUco)
python main_phase3.py --live --cam 1 --method aruco --size 0.08

# RealSense D435
python main_phase3.py --realsense --method aruco --size 0.05

# Closed-loop simulation without camera
python main_phase3.py --sim
```

### GUI Controls (`simulation_gui.py`)

| Key / Action | Effect |
|---|---|
| Left click | Place object (green area = workspace reachable by IK) |
| Right click | Move the deposit zone |
| SPACE | Start / pause |
| R | Reset |

### PBVS Control Law

The visual error is defined in 3D space (robot frame):

```
e = [t_current − t_desired ;  θ·u]  ∈ R⁶
                 ↑ position     ↑ axis-angle

v_c = −λ(‖e‖) · Ls⁺ · e       (Cartesian velocity)
q̇   = J⁺(q)  · v_c             (joint velocities)
```

- **Adaptive λ**: sigmoid ∈ [0.05, 1.5], increases with large error, slows for fine positioning
- **J⁺**: DLS pseudo-inverse (damped), robust to singularities
- **Secondary task**: joint limit avoidance (Liegeois method, weight 0.1)

### Main State Machine (`PipelineState`)

```
SEARCHING → PRE_APPROACH → TRACKING → APPROACH → CONVERGED
                ↑ (target jump > 50 mm)               ↓
                └──────────────────────────────── EMERGENCY
```

| State | Trigger | Action |
|---|---|---|
| `SEARCHING` | No object detected | Robot idle |
| `PRE_APPROACH` | θ1 or θ3 error > 8° | Joint-space interpolation to IK solution |
| `TRACKING` | Object detected, ‖e‖ > 50 mm | VS active, λ = 1.5 |
| `APPROACH` | ‖e‖ < 50 mm | Fine VS, λ = 1.0 |
| `CONVERGED` | ‖e_t‖ < 2 mm and ‖e_r‖ < 1° | Gripper sequencer activated |
| `EMERGENCY` | σ_min(J) < threshold | Emergency stop |

### Pick-and-Place Sequencer (`PickPlaceState`)

```
IDLE → APPROACH → GRASPING → LIFTING → TRANSPORT → LOWERING → RELEASING → RETURNING → DONE
```

Each TRANSPORT and RETURNING transition triggers a new `PRE_APPROACH` if the target jumps by more than 50 mm (e.g. pick→deposit, deposit→home).

### Non-singular Rest Configuration

The robot starts at `q₀ = [0°, 100 mm, −45°, 0°]`.
θ3 = 0° is singular (σ_min ≈ 0, r = 460 mm) and is systematically avoided by the pre-approach.

### Files

#### `visual_error.py`
Visual error and interaction matrix computation.

- `compute_error(t_cur, R_cur, t_des, R_des, params)` → `VisualError`
- `axis_angle(R)` — θ·u representation from a rotation matrix (inverse Rodrigues)
- `interaction_matrix_pbvs(R_co, t_co, R_cd)` — exact block-diagonal Ls (Chaumette 2006)

```python
VisualError.e           # error vector (6,)
VisualError.norm_t_mm   # position error norm in mm
VisualError.converged   # bool: ‖e_t‖ < 2 mm and ‖e_r‖ < 1°
```

#### `vs_controller.py`
PBVS controller + kinematics shared across all modules.

- `ScaraParams` — geometric parameters and joint limits (mirrors `robot_parameters.m`)
  - a2=300 mm, a3=160 mm, d3=150 mm, d4=59 mm
  - q_min=[-2.356, 0, -1.571, -3.142], q_max=[2.356, 0.200, 1.571, 3.142]
  - dq_max=[2.0 rad/s, 0.1 m/s, 2.0 rad/s, 2.0 rad/s]
- `ik_solutions(x_m, y_m, params)` — analytical IK, returns all valid (θ1, θ3) pairs
- `scara_jacobian(q, params)` — analytical Jacobian J ∈ R^(6×4)
- `damped_pinv(J)` — DLS pseudo-inverse, returns σ_min and singularity flag
- `VSController.update(error, q, dt)` → `VSCommand` (dq, v_c, singular, saturated)

#### `gripper_controller.py`
Pick-and-place task sequencer.

- `GripperController` — gripper model (OPEN / CLOSING / CLOSED / OPENING), open/close timers
- `PickPlaceSequencer(drop_pos_m)` — 9-state machine
  - `.update(vs_converged, object_pos_m, t_ee_m, q_current)` → (state, vs_target, gripper_close)
  - `._target_pos` — current VS target (changes at each transition)
- `draw_hud(frame)` — state overlay in BGR on camera frame

#### `simulation_gui.py`
Full 2D pick-and-place cycle simulation (matplotlib, no hardware).

- **XY view** — horizontal projection: arm + pixel-exact workspace (IK mask vectorised over 320×320 grid)
- **XZ view** — height profile (lift, transport, lower)
- **State panel** — real-time state machine + |e_t| error bar
- Integrated pre-approach: triggered on click AND on each target change from sequencer
- Animated object follows the arm during GRASPING / LIFTING / TRANSPORT / LOWERING

#### `main_phase3.py`
Full Phase 2 + Phase 3 pipeline, real-time or simulation mode.

- `Phase3Pipeline(cam_params, robot_tf, force_target)` — global state machine
  - `.process_frame(frame)` → (annotated_frame, VSCommand, PipelineState)
  - Two paths: `force_target` (camera bypass) and full camera pipeline
  - `effective_target` computed from `sequencer._target_pos` (not the raw object)
  - VS runs every frame outside pre-approach (never frozen)
- CLI: `--force-target x y z`, `--method aruco|yolo`, `--realsense`, `--live`, `--sim`

### Phase 3 Data Flow

```
[Phase 2] t_predicted (m)
       ↓
ik_solutions → PRE_APPROACH if θ1/θ3 off by > 8°
       ↓ (pre-approach complete)
compute_error(t_cur, R_cur, t_effective, R_des) → e ∈ R⁶
       ↓
VSController.update(e, q) → dq (rad/s, m/s)
       ↓
PickPlaceSequencer.update → new target → jump > 50 mm → PRE_APPROACH
       ↓
q ← q + dq × dt
```

### Validated Results (Python simulation)

| Metric | Value |
|---|---|
| Convergence from far position | ~30 frames after pre-approach |
| Final position error | < 2 mm |
| Final orientation error | < 1° |
| Effective workspace | r ∈ [340, 460] mm, θ1 ∈ ±135° |
| Full pick-and-place cycle | Validated in GUI simulation |
