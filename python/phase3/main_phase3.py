from __future__ import annotations
import math
import os
import sys
import time
import argparse
import warnings
from enum import Enum, auto
import snap7
from snap7.util import set_bool, set_dint
import struct
import numpy as np
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'phase2'))

from visual_error import compute_error, VisualError
from vs_controller import VSController, VSCommand, ScaraParams, simulate_pbvs, ik_solutions
from gripper_controller import PickPlaceSequencer, PickPlaceState, PP_STATE_COLORS, PP_STATE_LABELS

class PipelineState(Enum):
    SEARCHING    = auto()
    PRE_APPROACH = auto()
    TRACKING     = auto()
    APPROACH     = auto()
    CONVERGED    = auto()
    EMERGENCY    = auto()

APPROACH_THR_MM   = 50.0
CONVERGE_THR_MM   = 2.0
CONVERGE_THR_DEG  = 1.0
GAIN_TRACKING     = 1.5
GAIN_APPROACH     = 1.0
APPROACH_V_MAX    = 0.08

STATE_COLORS = {
    PipelineState.SEARCHING    : (100, 100, 100),
    PipelineState.PRE_APPROACH : (255, 180,  50),
    PipelineState.TRACKING     : (  0, 165, 255),
    PipelineState.APPROACH     : (  0, 255, 255),
    PipelineState.CONVERGED    : (  0, 255,   0),
    PipelineState.EMERGENCY    : (  0,   0, 255),
}

def _load_phase2_assets(calib_path: str, tf_path: str):
    from camera_calibration import load_calibration
    from robot_transform import load_transform, make_transform_from_geometry

    def make_default_transform():
        return make_transform_from_geometry(
            translation_m=[0.0, 0.0, 1.0],
            rotation_deg=[180.0, 0.0, 0.0]
        )

    def make_dummy_calibration():
        K = np.array([[800, 0, 640], [0, 800, 360], [0, 0, 1]], dtype=np.float64)
        return {"K": K, "dist": np.zeros((1, 5)), "rms": -1.0,
                "img_width": 1280, "img_height": 720}

    cam = load_calibration(calib_path) if os.path.exists(calib_path) else make_dummy_calibration()
    tf  = load_transform(tf_path) if os.path.exists(tf_path) else make_default_transform()
    return cam, tf

class Phase3Pipeline:
    def __init__(self, cam_params: dict, robot_tf, method: str = "aruco", marker_size: float = 0.05, work_plane_z: float = 0.80, yolo_model: str = None, force_target: np.ndarray = None, dt: float = 0.033):
        if method == "yolo":
            try:
                import ultralytics
            except ImportError:
                raise ImportError("ultralytics")
            if yolo_model is None:
                raise ValueError("yolo_model")

        self.cam_params = cam_params
        self.robot_tf = robot_tf
        self.method = method
        self.marker_size = marker_size
        self.work_plane_z = work_plane_z
        self.yolo_model = yolo_model
        self.force_target = (force_target / 1000.0) if force_target is not None else None
        self.dt = dt

        self.params = ScaraParams()
        self.ctrl = VSController(
            self.params,
            gain=GAIN_TRACKING,
            gain_min=0.05,
            gain_max=GAIN_TRACKING,
            adaptive=True,
            v_max_m_s=0.20,
        )

        self.q_current = np.array([0.0, 0.10, -np.pi/4, 0.0])
        self._preapproach_goal = None
        self._last_target_xy = None

        self.state = PipelineState.SEARCHING
        self.R_desired = np.eye(3)

        from kalman_tracker import MultiObjectTracker
        self.mot = MultiObjectTracker(dt=dt, sigma_accel=1.0, sigma_pos=0.005, max_missed=5, latency_s=0.150)

        self.errors = []
        self.commands = []
        self.timestamps = []
        self._t0 = time.time()

        self._dbg_t_cur = np.zeros(3)
        self._dbg_t_target = None
        self._dbg_n_confirmed = 0
        self._dbg_frame = 0

        self.sequencer = PickPlaceSequencer(
            drop_pos_m=np.array([0.20, -0.300, -0.100]),
            home_q=np.array([0.0, 0.10, 0.0, 0.0]),
            lift_height_m=0.080,
            approach_height_m=0.040,
            place_height_m=0.020,
            approach_thr_mm=5.0,
            dt=dt,
        )

    def process_frame(self, frame: np.ndarray, depth_frame: np.ndarray = None) -> tuple[np.ndarray, VSCommand | None, PipelineState]:
        from detect_objects import detect_aruco, detect_yolo, draw_detections
        from pose_estimation import estimate_pose_aruco, estimate_pose_yolo_flat, estimate_pose_yolo_rgbd, draw_pose_axes

        self._dbg_frame += 1

        if self.force_target is not None:
            detections = []
            poses_cam = []
            poses_robot = []
            tracked = []
            t_target = self.force_target.copy()
            R_target = self.R_desired

            t_cur = self._current_ee_position()
            R_cur = self._current_ee_rotation()
            self._dbg_t_cur = t_cur
            self._dbg_t_target = t_target
            self._dbg_n_confirmed = 1

            _seq_st = self.sequencer.state
            if (_seq_st not in (PickPlaceState.IDLE, PickPlaceState.DONE) and self.sequencer._target_pos is not None):
                effective_target = self.sequencer._target_pos.copy()
            else:
                effective_target = t_target

            eff_xy = effective_target[:2]
            if (self._last_target_xy is None or np.linalg.norm(eff_xy - self._last_target_xy) > 0.05):
                self._last_target_xy = eff_xy.copy()
                if self._preapproach_goal is None:
                    sols = ik_solutions(float(effective_target[0]), float(effective_target[1]), self.params)
                    if sols:
                        t1_g, t3_g = min(sols, key=lambda s: abs(s[1]))
                        d2_g = float(np.clip(effective_target[2] + self.params.d3 + self.params.d4, self.params.q_min[1], self.params.q_max[1]))
                        q_goal = np.array([t1_g, d2_g, t3_g, 0.0])
                        diff = np.abs(q_goal[[0, 2]] - self.q_current[[0, 2]])
                        if np.max(diff) > np.radians(8.0):
                            self._preapproach_goal = q_goal

            if self._preapproach_goal is not None:
                q_diff = self._preapproach_goal - self.q_current
                if np.max(np.abs(q_diff[[0, 2]])) > np.radians(8.0):
                    dstep = (np.sign(q_diff) * np.minimum(np.abs(q_diff), self.params.dq_max * self.dt))
                    self.q_current = np.clip(self.q_current + dstep, self.params.q_min, self.params.q_max)
                    self.state = PipelineState.PRE_APPROACH
                    annotated = frame.copy()
                    annotated = self._draw_hud(annotated, [], [], None)
                    annotated = self.sequencer.draw_hud(annotated)
                    return annotated, None, self.state
                else:
                    self._preapproach_goal = None

            err = compute_error(t_cur, R_cur, effective_target, R_target, thr_t_mm=CONVERGE_THR_MM, thr_r_deg=CONVERGE_THR_DEG)
            self.errors.append(err)
            self.timestamps.append(time.time() - self._t0)

            pp_state, pp_target, gripper_close = self.sequencer.update(vs_converged=err.converged, object_pos_m=t_target, t_ee_m=t_cur, q_current=self.q_current)

            if pp_target is not None and not np.allclose(pp_target, effective_target):
                err = compute_error(t_cur, R_cur, pp_target, R_target, thr_t_mm=CONVERGE_THR_MM, thr_r_deg=CONVERGE_THR_DEG)

            if err.converged:
                self.state = PipelineState.CONVERGED
            elif err.norm_t_mm < APPROACH_THR_MM:
                self.state = PipelineState.APPROACH
                self.ctrl.tune(gain=GAIN_APPROACH, gain_max=GAIN_APPROACH)
                self.ctrl.params.dq_max = self.params.dq_max.copy()
            else:
                self.state = PipelineState.TRACKING
                self.ctrl.tune(gain=GAIN_TRACKING, gain_max=GAIN_TRACKING)
                self.ctrl.params.dq_max = self.params.dq_max.copy()

            cmd = self.ctrl.update(err, self.q_current, dt=self.dt)
            self.commands.append(cmd)
            if cmd.singular:
                self.state = PipelineState.EMERGENCY
            self.q_current = np.clip(self.q_current + cmd.dq * self.dt, self.params.q_min, self.params.q_max)

            annotated = frame.copy()
            annotated = self._draw_hud(annotated, poses_robot, tracked, cmd)
            annotated = self.sequencer.draw_hud(annotated)
            return annotated, cmd, self.state

        if self.method == "aruco":
            detections = detect_aruco(frame)
        else:
            detections = detect_yolo(frame, model_path=self.yolo_model, conf_threshold=0.5)

        if self.method == "aruco":
            poses_cam = estimate_pose_aruco(detections, self.cam_params, marker_size_m=self.marker_size)
        elif depth_frame is not None:
            poses_cam = estimate_pose_yolo_rgbd(detections, self.cam_params, depth_frame)
        else:
            poses_cam = estimate_pose_yolo_flat(detections, self.cam_params, self.work_plane_z)

        poses_robot = [self.robot_tf.transform(p) for p in poses_cam]
        measurements = [(p.label, p.position_m) for p in poses_robot]
        tracked = self.mot.update(measurements)
        target_pose = None

        if poses_robot:
            confirmed = [t for t in tracked if t.confirmed]
            if confirmed:
                best = min(confirmed, key=lambda t: np.linalg.norm(t.predicted_m))
                t_target = best.predicted_m
                R_target = self.R_desired
            else:
                target_pose = poses_robot[0]
                t_target = target_pose.position_m
                R_target = target_pose.R_cam

            _seq_pre = self.sequencer.state
            if (_seq_pre not in (PickPlaceState.IDLE, PickPlaceState.DONE) and self.sequencer._target_pos is not None):
                _eff_xy = self.sequencer._target_pos[:2]
            else:
                _eff_xy = t_target[:2]

            if (self._last_target_xy is None or np.linalg.norm(_eff_xy - self._last_target_xy) > 0.05):
                self._last_target_xy = _eff_xy.copy()
                if self._preapproach_goal is None:
                    sols = ik_solutions(float(_eff_xy[0]), float(_eff_xy[1]), self.params)
                    if sols:
                        t1_g, t3_g = min(sols, key=lambda s: abs(s[1]))
                        d2_g = float(np.clip(t_target[2] + self.params.d3 + self.params.d4, self.params.q_min[1], self.params.q_max[1]))
                        q_goal = np.array([t1_g, d2_g, t3_g, 0.0])
                        diff = np.abs(q_goal[[0, 2]] - self.q_current[[0, 2]])
                        if np.max(diff) > np.radians(8.0):
                            self._preapproach_goal = q_goal

            if self._preapproach_goal is not None:
                q_diff = self._preapproach_goal - self.q_current
                if np.max(np.abs(q_diff[[0, 2]])) > np.radians(8.0):
                    dstep = (np.sign(q_diff) * np.minimum(np.abs(q_diff), self.params.dq_max * self.dt))
                    self.q_current = np.clip(self.q_current + dstep, self.params.q_min, self.params.q_max)
                    self.state = PipelineState.PRE_APPROACH
                    annotated = draw_detections(frame, detections)
                    annotated = self._draw_hud(annotated, poses_robot, tracked, None)
                    annotated = self.sequencer.draw_hud(annotated)
                    return annotated, None, self.state
                else:
                    self._preapproach_goal = None

            t_cur = self._current_ee_position()
            R_cur = self._current_ee_rotation()
            self._dbg_t_cur = t_cur
            self._dbg_t_target = t_target
            self._dbg_n_confirmed = len([t for t in tracked if t.confirmed])

            seq_st = self.sequencer.state
            if (seq_st not in (PickPlaceState.IDLE, PickPlaceState.DONE) and self.sequencer._target_pos is not None):
                effective_target = self.sequencer._target_pos
            else:
                effective_target = t_target

            err = compute_error(t_cur, R_cur, effective_target, R_target, thr_t_mm=CONVERGE_THR_MM, thr_r_deg=CONVERGE_THR_DEG)
            self.errors.append(err)
            self.timestamps.append(time.time() - self._t0)

            pp_state, pp_target, gripper_close = self.sequencer.update(vs_converged=err.converged, object_pos_m=t_target, t_ee_m=t_cur, q_current=self.q_current)

            if pp_target is not None and not np.allclose(pp_target, effective_target):
                err = compute_error(t_cur, R_cur, pp_target, R_target, thr_t_mm=CONVERGE_THR_MM, thr_r_deg=CONVERGE_THR_DEG)

            if err.converged:
                self.state = PipelineState.CONVERGED
            elif err.norm_t_mm < APPROACH_THR_MM:
                self.state = PipelineState.APPROACH
                self.ctrl.tune(gain=GAIN_APPROACH, gain_max=GAIN_APPROACH)
                self.ctrl.params.dq_max = self.params.dq_max.copy()
            else:
                self.state = PipelineState.TRACKING
                self.ctrl.tune(gain=GAIN_TRACKING, gain_max=GAIN_TRACKING)
                self.ctrl.params.dq_max = self.params.dq_max.copy()

            cmd = self.ctrl.update(err, self.q_current, dt=self.dt)
            self.commands.append(cmd)

            if cmd.singular:
                self.state = PipelineState.EMERGENCY

            self.q_current = np.clip(self.q_current + cmd.dq * self.dt, self.params.q_min, self.params.q_max)

        else:
            self.state = PipelineState.SEARCHING
            self._dbg_t_target = None
            cmd = None

        annotated = draw_detections(frame, detections)
        annotated = draw_pose_axes(annotated, poses_cam, self.cam_params, axis_length_m=self.marker_size * 0.8)
        annotated = self._draw_hud(annotated, poses_robot, tracked, cmd)
        annotated = self.sequencer.draw_hud(annotated)

        return annotated, cmd, self.state

    def _current_ee_position(self) -> np.ndarray:
        t1, d2, t3, t4 = self.q_current
        p = self.params
        px = p.a2 * np.cos(t1) + p.a3 * np.cos(t1 + t3)
        py = p.a2 * np.sin(t1) + p.a3 * np.sin(t1 + t3)
        pz = d2 - p.d3 - p.d4
        return np.array([px, py, pz])

    def _current_ee_rotation(self) -> np.ndarray:
        phi = self.q_current[0] + self.q_current[2] + self.q_current[3]
        c, s = np.cos(phi), np.sin(phi)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def _draw_hud(self, frame, poses_robot, tracked, cmd) -> np.ndarray:
        h, w = frame.shape[:2]
        state_color = STATE_COLORS[self.state]

        cv2.rectangle(frame, (0, 0), (w, 48), (30, 30, 30), -1)
        cv2.putText(frame, f"STATE: {self.state.name}", (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 1.0, state_color, 2)

        if self.errors:
            err = self.errors[-1]
            et_mm = err.norm_t_mm
            er_deg = err.norm_r_deg

            err_txt = f"|et|={et_mm:.1f}mm  |er|={er_deg:.1f}deg"
            cv2.putText(frame, err_txt, (w // 2 - 150, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)

            bar_x, bar_y, bar_h = w - 220, 10, 14
            bar_max_mm = 400.0
            fill = int(min(1.0, et_mm / bar_max_mm) * 200)
            bar_color = (
                (0, 255, 0)   if et_mm < CONVERGE_THR_MM else
                (0, 255, 255) if et_mm < APPROACH_THR_MM else
                (0, 100, 255)
            )
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + 200, bar_y + bar_h), (60, 60, 60), -1)
            if fill > 0:
                cv2.rectangle(frame, (bar_x, bar_y), (bar_x + fill, bar_y + bar_h), bar_color, -1)
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + 200, bar_y + bar_h), (120, 120, 120), 1)
            thr_approach_x = bar_x + int(APPROACH_THR_MM / bar_max_mm * 200)
            thr_conv_x     = bar_x + int(CONVERGE_THR_MM  / bar_max_mm * 200)
            cv2.line(frame, (thr_approach_x, bar_y), (thr_approach_x, bar_y + bar_h), (0, 200, 255), 1)
            cv2.line(frame, (thr_conv_x,     bar_y), (thr_conv_x,     bar_y + bar_h), (0, 255, 0),   1)

        ee_mm = self._dbg_t_cur * 1000
        r_ee  = float(np.linalg.norm(ee_mm[:2]))
        ee_txt = (f"EE sim: X={ee_mm[0]:+.0f} Y={ee_mm[1]:+.0f} Z={ee_mm[2]:+.0f} mm r={r_ee:.0f} mm")
        cv2.rectangle(frame, (0, 50), (w, 74), (20, 20, 40), -1)
        cv2.putText(frame, ee_txt, (10, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (160, 200, 255), 1)

        if self._dbg_t_target is not None:
            tg_mm = self._dbg_t_target * 1000
            r_tg  = float(np.linalg.norm(tg_mm[:2]))
            ws_ok = 140 <= r_tg <= 460
            ws_col = (0, 255, 0) if ws_ok else (0, 0, 255)
            tg_txt = (f"TGT cam: X={tg_mm[0]:+.0f} Y={tg_mm[1]:+.0f} Z={tg_mm[2]:+.0f} mm r={r_tg:.0f} mm")
            cv2.rectangle(frame, (0, 76), (w, 100), (20, 40, 20), -1)
            cv2.putText(frame, tg_txt, (10, 94), cv2.FONT_HERSHEY_SIMPLEX, 0.50, ws_col, 1)
            if not ws_ok:
                cv2.putText(frame, "! HORS WORKSPACE [140-460mm]", (w - 320, 94), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 255), 2)

        n_conf = self._dbg_n_confirmed
        kal_col = (0, 255, 0) if n_conf > 0 else (100, 100, 100)
        cv2.putText(frame, f"Kalman: {n_conf}", (w - 200, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.45, kal_col, 1)

        if cmd is not None:
            dq_str = "  ".join([f"dq{i+1}={v:.3f}" for i, v in enumerate(cmd.dq)])
            cv2.rectangle(frame, (0, h - 92), (w, h - 78), (15, 15, 15), -1)
            cv2.putText(frame, dq_str, (10, h - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 255), 1)

        for i, p in enumerate(poses_robot):
            x_mm, y_mm, z_mm = p.position_m * 1000
            txt = f"[{p.label}] X={x_mm:+.0f} Y={y_mm:+.0f} Z={z_mm:+.0f} mm"
            cv2.putText(frame, txt, (10, 108 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 200), 1)

        for obj in tracked:
            if obj.confirmed:
                px, py = self._project_3d(obj.predicted_m, frame.shape)
                if px is not None:
                    cv2.drawMarker(frame, (px, py), (0, 165, 255), cv2.MARKER_DIAMOND, 16, 2)

        return frame

    def _project_3d(self, point_m, shape):
        K = self.cam_params["K"]
        try:
            p_cam = self.robot_tf.inverse().transform_point(point_m)
            if p_cam[2] <= 0:
                return None, None
            u = int(K[0, 0] * p_cam[0] / p_cam[2] + K[0, 2])
            v = int(K[1, 1] * p_cam[1] / p_cam[2] + K[1, 2])
            h, w = shape[:2]
            if 0 <= u < w and 0 <= v < h:
                return u, v
        except Exception:
            pass
        return None, None

    def plot_convergence(self, save_path: str = None):
        if not self.errors:
            return

        import matplotlib.pyplot as plt

        ts  = self.timestamps
        et  = [e.norm_t_mm  for e in self.errors]
        er  = [e.norm_r_deg for e in self.errors]
        lam = [c.gain for c in self.commands] if self.commands else [0]*len(ts)

        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

        axes[0].plot(ts, et, "b-")
        axes[0].axhline(CONVERGE_THR_MM,  ls="--", color="r", alpha=0.6)
        axes[0].axhline(APPROACH_THR_MM, ls=":",  color="orange", alpha=0.6)

        ax2 = axes[0].twinx()
        ax2.plot(ts, er, "g--", alpha=0.7)
        ax2.axhline(CONVERGE_THR_DEG, ls="--", color="darkgreen", alpha=0.4)

        axes[1].plot(ts, lam, "k-")

        plt.tight_layout()
        if save_path:
            os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
            fig.savefig(save_path, dpi=100)
        else:
            plt.show()

        return fig

def run_simulation(args):
    q0 = np.array([0.5, 0.08, -0.3, 0.2])
    t_des = np.array([0.350, 0.050, -0.150])
    R_des = np.eye(3)

    history, converged, ctrl = simulate_pbvs(q0=q0, t_desired=t_des, R_desired=R_des, dt=0.033, max_iter=500, gain=0.5, adaptive=True, verbose=True)

    try:
        import matplotlib
        matplotlib.use("TkAgg" if sys.platform == "darwin" else "Agg")
        fig = ctrl.plot_history(show=False)
        os.makedirs("test_images", exist_ok=True)
        path = "test_images/phase3_sim_convergence.png"
        fig.savefig(path, dpi=120)
        try:
            import matplotlib.pyplot as plt
            plt.show()
        except Exception:
            pass
    except ImportError:
        pass

    return converged

def run_live(args):
    plc = snap7.client.Client()
    automate_connecte = False
    try:
        plc.connect('192.168.0.10', 0, 1)
        automate_connecte = True
    except Exception as e:
        pass

    calib_path = os.path.join(os.path.dirname(__file__), '..', 'phase2', 'calibration_data', 'camera_params.npz')
    tf_path = os.path.join(os.path.dirname(__file__), '..', 'phase2', 'calibration_data', 'cam_to_robot.npz')
    cam_params, robot_tf = _load_phase2_assets(calib_path, tf_path)

    pipeline = Phase3Pipeline(cam_params=cam_params, robot_tf=robot_tf, method=args.method, marker_size=args.size, work_plane_z=args.z, yolo_model=args.yolo_model, force_target=np.array(args.force_target) if args.force_target else None, dt=1.0 / 30)

    use_rs = False
    if args.realsense:
        try:
            from realsense_capture import auto_camera
            sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'phase2'))
            camera_ctx = auto_camera(cam_index=args.cam, work_plane_z_mm=args.z * 1000)
            use_rs = True
        except ImportError:
            pass

    if not use_rs:
        cap = cv2.VideoCapture(args.cam if args.live else args.cam)
        if not cap.isOpened():
            sys.exit(1)

    frame_idx = 0

    def read_frame():
        if use_rs:
            return camera_ctx.read()
        ret, f = cap.read()
        return (f, None) if ret else (None, None)

    ctx = camera_ctx if use_rs else None

    ratio_deg = 100.0
    ratio_mm = 50.0

    def run_loop(read_fn):
        nonlocal frame_idx
        while True:
            color, depth = read_fn()
            if color is None:
                break

            annotated, cmd, state = pipeline.process_frame(color, depth)

            if automate_connecte:
                try:
                    theta1_deg = float(math.degrees(pipeline.q_current[0]))
                    z_mm = float(pipeline.q_current[1] * 1000.0)
                    theta3_deg = float(math.degrees(pipeline.q_current[2]))
                    theta4_deg = 45.0 

                    data_coords = bytearray(32)
                    
                    set_real(data_coords, 0, theta1_deg)
                    set_real(data_coords, 4, z_mm)
                    set_real(data_coords, 8, theta3_deg)
                    set_real(data_coords, 12, theta4_deg)
                    
                    set_real(data_coords, 16, 22000.0)
                    set_real(data_coords, 20, 25000.0)
                    set_real(data_coords, 24, 22000.0)
                    set_real(data_coords, 28, 1800.0)
                    
                    plc.write_area(0x83, 0, 100, data_coords)

                    m34_byte = plc.read_area(0x83, 0, 34, 1)
                    gripper_close = pipeline.sequencer.gripper.state.name in ["CLOSED", "CLOSING"]
                    set_bool(m34_byte, 0, 5, gripper_close)
                    plc.write_area(0x83, 0, 34, m34_byte)

                    motor_on = (state != PipelineState.SEARCHING and state != PipelineState.EMERGENCY)
                    if motor_on:
                        m50 = bytearray(1)
                        set_bool(m50, 0, 0, True)
                        plc.write_area(0x83, 0, 50, m50)
                        
                        time.sleep(0.01) 
                        
                        set_bool(m50, 0, 0, False)
                        plc.write_area(0x83, 0, 50, m50)

                except Exception as e:
                    pass

            cv2.imshow("Phase 3", annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                fname = f"test_images/phase3_frame_{frame_idx:04d}.jpg"
                os.makedirs("test_images", exist_ok=True)
                cv2.imwrite(fname, annotated)
            elif key == ord('r'):
                pipeline.ctrl.reset()
                pipeline.state = PipelineState.SEARCHING

            frame_idx += 1

    try:
        if use_rs:
            with camera_ctx as _:
                run_loop(read_frame)
        else:
            run_loop(read_frame)
            cap.release()

    except KeyboardInterrupt:
        # Si tu fais Ctrl+C dans le terminal
        print("\nArrêt manuel détecté !")
    except Exception as e:
        # Si une erreur fait planter le script
        print(f"\nErreur dans le programme : {e}")

    finally:
        # CE BLOC S'EXÉCUTE TOUJOURS À LA FIN, MÊME EN CAS DE CRASH
        if automate_connecte:
            try:
                print("Fermeture propre de la connexion automate...")
                # 1. On coupe les moteurs par sécurité
                m2_byte = plc.read_area(0x83, 0, 2, 1)
                set_bool(m2_byte, 0, 4, False)
                set_bool(m2_byte, 0, 6, False)
                plc.write_area(0x83, 0, 2, m2_byte)
                
                # 2. On ferme la porte !
                plc.disconnect()
                print("-> Déconnexion RÉUSSIE. L'automate est prêt pour le prochain test.")
            except Exception as e:
                print(f"-> Échec de la déconnexion : {e}")
                
        cv2.destroyAllWindows()
        pipeline.plot_convergence(save_path="test_images/phase3_session.png")

    cv2.destroyAllWindows()
    pipeline.plot_convergence(save_path="test_images/phase3_session.png")

def run_image(args):
    frame = cv2.imread(args.image)
    if frame is None:
        sys.exit(1)

    calib_path = os.path.join(os.path.dirname(__file__), '..', 'phase2', 'calibration_data', 'camera_params.npz')
    tf_path = os.path.join(os.path.dirname(__file__), '..', 'phase2', 'calibration_data', 'cam_to_robot.npz')
    cam_params, robot_tf = _load_phase2_assets(calib_path, tf_path)

    pipeline = Phase3Pipeline(cam_params=cam_params, robot_tf=robot_tf, method=args.method, marker_size=args.size, yolo_model=args.yolo_model)

    annotated, cmd, state = pipeline.process_frame(frame)

    cv2.imshow("Phase 3", annotated)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def parse_args():
    parser = argparse.ArgumentParser()
    src = parser.add_mutually_exclusive_group()
    src.add_argument("--sim", action="store_true")
    src.add_argument("--image", metavar="FILE")
    src.add_argument("--live", action="store_true")
    src.add_argument("--realsense", action="store_true")

    parser.add_argument("--force-target", type=float, nargs=3, metavar=("X", "Y", "Z"), dest="force_target")
    parser.add_argument("--method", choices=["aruco","yolo"], default="aruco")
    parser.add_argument("--yolo-model", type=str, default="yolov8n.pt", dest="yolo_model")
    parser.add_argument("--size", type=float, default=0.05)
    parser.add_argument("--z", type=float, default=0.80)
    parser.add_argument("--cam", type=int, default=0)
    parser.add_argument("--gain", type=float, default=0.5)
    parser.add_argument("--no-adapt", action="store_true")
    return parser.parse_args()

def main():
    args = parse_args()

    if args.sim:
        converged = run_simulation(args)
        sys.exit(0 if converged else 1)
    elif args.image:
        run_image(args)
    elif args.live or args.realsense:
        run_live(args)
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()
