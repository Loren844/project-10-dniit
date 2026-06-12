# MATLAB — 4-DOF SCARA Robot Simulation

This folder contains all MATLAB simulation code for the project, split into two phases.

```
matlab/
├── phase1/    ← Modelling, kinematics, trajectories, PID simulation
└── phase3/    ← Closed-loop validation of the PBVS controller
```

**Requirements**: MATLAB R2021b or later. No toolbox required — everything is implemented from scratch (no Robotics Toolbox).

---

## Phase 1 — Robot Modelling and Simulation

### Objective

Build the complete 4-DOF SCARA model from scratch: forward/inverse kinematics, workspace analysis, trajectory generation, closed-loop PID simulation, 3D animation.

### Run

```matlab
cd matlab/phase1
clear                % clear MATLAB function cache
main_phase1          % full run: 7 steps + console report + figures
main_phase1(false)   % console only, no figures
```

Typical runtime: ~5 seconds. Generates 8 figures + structured console report.

### Robot Model (DH)

Topology: **θ1 (R) → d2 (P) → θ3 (R) → θ4 (R)**

| Link | θ | a [m] | α [°] | d [m] |
|------|---|-------|-------|-------|
| 1 | θ1 var | 0 | 0° | 0 |
| 2 | 0° fixed | **0.300** | 0° | d2 var |
| 3 | θ3 var | **0.160** | 180° | −0.150 |
| 4 | θ4 var | 0 | 0° | 0.059 |

Joint limits:

| Joint | Min | Max |
|---|---|---|
| θ1 | −135° | +135° |
| d2 | 0 mm | 200 mm |
| θ3 | −90° | +90° |
| θ4 | −180° | +180° |

### Files

#### `robot_parameters.m`
Defines the complete `robot` struct used by all other scripts.

Contains: DH parameters (a, alpha, d, theta_offset), joint limits, per-axis PID gains, dynamic parameters (effective inertia `J_eff`, viscous friction `B_vis`).

```matlab
robot = robot_parameters();
% robot.a2, robot.a3, robot.d3, robot.d4
% robot.q_min, robot.q_max
% robot.Kp, robot.Ki, robot.Kd  (4×1 vectors)
```

#### `forward_kinematics.m`
Forward kinematics by DH matrix composition.

Computes end-effector position and orientation for configuration `q = [θ1, d2, θ3, θ4]`.

Analytical formulas:
```
Px = a2·cos(θ1) + a3·cos(θ1+θ3)
Py = a2·sin(θ1) + a3·sin(θ1+θ3)
Pz = d2 − d3 − d4
```

```matlab
[T_end, T_all] = forward_kinematics(robot, q)
% T_end : 4×4 homogeneous matrix, end-effector→base
% T_all : {T01, T02, T03, T04} (all intermediate transforms)
```

#### `inverse_kinematics.m`
Analytical inverse kinematics — closed-form solution (no iteration).

1. cos(θ3) = (Px² + Py² − a2² − a3²) / (2·a2·a3) → 2 solutions (elbow up/down)
2. θ1 = atan2(...) from both θ3 solutions
3. θ4 = −θ1 − θ3 (end-effector orientation held at 0°)
4. d2 = Pz + d3 + d4

Residual error = 0.0000 mm (exact solution).

```matlab
[q_sol, success, err_mm] = inverse_kinematics(robot, T_des, q0)
% q_sol : [θ1, d2, θ3, θ4] in rad/m
% success : bool
% err_mm : residual position error in mm
```

#### `workspace_analysis.m`
Visualises the SCARA workspace via Monte Carlo (30 000 random configurations).

Expected results:
- **XY**: toroidal ring r ∈ [340, 460] mm (dead zone when |θ3| > 90°)
- **Z**: horizontal plane z ∈ [−209, −9] mm

Generates 4 figures: 3D view, XY projection, XZ projection, SCARA arm structure.

```matlab
workspace_analysis(robot)
workspace_analysis(robot, 50000)  % more samples
```

#### `joint_space_trajectory.m`
Point-to-point trajectory in joint space.

Three available profiles:

| Profile | Continuity | Use |
|---|---|---|
| `cubic` | C¹ (velocity) | Standard, simple |
| `quintic` | C² (acceleration) | Smoothest motion |
| `trapezoidal` | C⁰ | Close to industrial drives |

```matlab
traj = joint_space_trajectory(q_start, q_end, T_total, dt, 'quintic')
% traj.q    : (N×4) positions
% traj.dq   : (N×4) velocities
% traj.ddq  : (N×4) accelerations
```

#### `cartesian_trajectory.m`
Linear trajectory in Cartesian space (end-effector follows a straight line).

- Position: linear interpolation with cubic smoothing
- Orientation: SLERP (Spherical Linear Interpolation)
- IK called at each point to obtain joint angles

```matlab
traj = cartesian_trajectory(robot, p_start, p_end, R_start, R_end, T_total, dt)
```

#### `plot_trajectory.m`
Visualises a generated trajectory.

- **Joint-space** trajectory: 3 subplots (positions °, velocities °/s, accelerations °/s²)
- **Cartesian** trajectory: XYZ coordinates vs time + 3D path

```matlab
plot_trajectory(traj)
```

#### `pid_simulation.m`
Closed-loop simulation with PID + feedforward controller on each axis.

Simplified dynamic model (1st order per axis):
```
J_eff · q̈ = τ − B_vis · q̇
```

Control law:
```
τ = J_eff·q̈_ref + B_vis·q̇_ref   (feedforward)
  + Kp·e + Ki·∫e·dt + Kd·(q̇_ref − q̇)   (PID)
```

Safety features: anti-windup, torque saturation, joint limits.
Integration via Heun's method (RK2).

```matlab
result = pid_simulation(robot, traj, Kp, Ki, Kd)
% result.q_actual   : simulated positions
% result.e          : tracking errors
% result.rmse       : RMSE per axis
```

Phase 1 results: RMSE < 0.5° (revolute axes), < 0.5 mm (prismatic axis).

#### `animate_robot.m`
3D animation of the SCARA along a trajectory.

Reconstructs physical arm segments (vertical column, horizontal links) for a realistic rendering. Displays end-effector trail.

```matlab
animate_robot(robot, traj)
animate_robot(robot, traj, 'speed', 2)      % 2× faster
animate_robot(robot, traj, 'trail', false)  % no trail
```

#### `main_phase1.m`
Orchestrates all 7 steps in order with a structured console report:

1. Load robot parameters
2. Test forward kinematics (validate analytical formulas)
3. Test inverse kinematics (residual error = 0 mm)
4. Workspace analysis (Monte Carlo)
5. Generate quintic joint-space trajectory
6. Closed-loop PID simulation
7. 3D animation

### Data Flow

```
robot_parameters
   ├── forward_kinematics → workspace_analysis
   │         └── inverse_kinematics ← cartesian_trajectory
   ├── joint_space_trajectory → pid_simulation → animate_robot
   └── main_phase1  (orchestrates everything)
```

---

## Phase 3 — MATLAB Closed-Loop Validation of the PBVS Controller

### Objective

Validate PBVS visual servoing loop convergence in MATLAB simulation before deployment. Test edge cases (conveyor belt, Kalman latency compensation, adaptive gain).

This script is **complementary** to the Python simulation (`python/phase3/simulation_gui.py`): it validates the same algorithms in the MATLAB environment with detailed convergence plots.

### Run

```matlab
cd matlab/phase3
% Add phase1 to path (shared IK, FK)
addpath('../phase1')
main_phase3
```

Typical runtime: ~2 seconds. Generates 6 convergence figures.

### Configure the simulation (top of the script)

| Parameter | Default | Description |
|---|---|---|
| `dt` | 0.033 s | Time step (~30 Hz) |
| `T_sim` | 15 s | Maximum simulation duration |
| `lambda_nom` | 0.5 | Nominal VS gain |
| `lambda_min/max` | 0.05 / 2.0 | Adaptive gain bounds |
| `adaptive` | true | Sigmoid gain as a function of ‖e‖ |
| `thr_t_mm` | 2.0 mm | Position convergence threshold |
| `thr_r_deg` | 1.0° | Orientation convergence threshold |
| `q0` | [0.5, 0.10, −0.30, 0.20] | Initial configuration (rad/m) |
| `t_des` | [0.350, 0.050, −0.150] m | Target position |
| `use_conveyor` | false | Enable conveyor belt |
| `v_conveyor` | [0.05, 0, 0] m/s | Conveyor speed |

### File

#### `main_phase3.m`
Self-contained script (all functions at the bottom or imported from `phase1/`).

Simulation steps:
1. Load robot parameters (from `phase1/robot_parameters.m`)
2. Initialise PBVS controller (adaptive gain, interaction matrix)
3. Simulation loop (FK → error → VS command → saturation → integration)
4. If `use_conveyor = true`: target moves + Kalman compensation (+150 ms)
5. Display 6 figures: e_t/e_r convergence, adaptive gain, XY trajectory, joint configurations, dq commands

**Note on DH convention**: with the SCARA DH table (link 3: α = 180°), FK always produces R with trace(R) = −1 regardless of configuration. Rotation error is therefore computed in a frame adapted to this convention (see comments in the script).

### Expected Results

| Metric | Value |
|---|---|
| Position convergence ‖e_t‖ < 2 mm | ~30 iterations (~1 s at 30 Hz) |
| Orientation convergence ‖e_r‖ < 1° | Simultaneous |
| Conveyor compensation (Kalman) | Residual error < 5 mm at v = 50 mm/s |
| Joint limits never exceeded | Guaranteed by saturation |
