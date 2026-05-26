"""
main_phase3.py
Pipeline complet Phase 3 — Asservissement visuel PBVS en temps réel.

Intègre :
  Phase 2 : perception (detect_objects + pose_estimation + robot_transform + kalman_tracker)
  Phase 3 : contrôle  (visual_error + vs_controller)

Modes de fonctionnement
-----------------------
  --sim          : simulation Python boucle fermée (sans caméra ni robot)
  --image FILE   : test statique sur une image
  --live         : flux webcam temps réel (ArUco)
  --realsense    : flux RealSense D435 RGB-D

États du pipeline
-----------------
  SEARCHING  → aucun objet détecté, robot en position de repos
  TRACKING   → objet suivi par Kalman, pas encore en zone de saisie
  APPROACH   → erreur < 50 mm, asservissement actif (λ réduit)
  CONVERGED  → erreur < 2 mm, robot prêt à saisir

Usage :
  python main_phase3.py --sim
  python main_phase3.py --live --cam 1 --method aruco --size 0.08
  python main_phase3.py --realsense --method aruco --size 0.05
"""

from __future__ import annotations

import os
import sys
import time
import argparse
import warnings
from enum import Enum, auto

import numpy as np
import cv2

# Ajouter Phase 2 au chemin Python
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'phase2'))

from visual_error       import compute_error, VisualError
from vs_controller      import VSController, VSCommand, ScaraParams, simulate_pbvs, ik_solutions
from gripper_controller import (PickPlaceSequencer, PickPlaceState,
                                 PP_STATE_COLORS, PP_STATE_LABELS)


# ─────────────────────────────────────────────────────────────────────────────
# État de la machine à états du pipeline
# ─────────────────────────────────────────────────────────────────────────────

class PipelineState(Enum):
    SEARCHING    = auto()   # Aucun objet détecté
    PRE_APPROACH = auto()   # Pré-approche articulaire (espace joint avant VS)
    TRACKING     = auto()   # Objet suivi, en dehors de la zone d'approche
    APPROACH     = auto()   # Zone d'approche (<50 mm), asservissement fin
    CONVERGED    = auto()   # Convergé, prêt à saisir
    EMERGENCY    = auto()   # Arrêt d'urgence (singularité / butée)


# ─────────────────────────────────────────────────────────────────────────────
# Paramètres de la session
# ─────────────────────────────────────────────────────────────────────────────

APPROACH_THR_MM   = 50.0   # mm — transition TRACKING → APPROACH
CONVERGE_THR_MM   = 2.0    # mm
CONVERGE_THR_DEG  = 1.0    # °
GAIN_TRACKING     = 1.5    # λ en phase TRACKING (convergence rapide)
GAIN_APPROACH     = 1.0    # λ en phase APPROACH  (réduit vs TRACKING, précis sous 10mm)
APPROACH_V_MAX    = 0.08   # m/s — vitesse max en approche

STATE_COLORS = {
    PipelineState.SEARCHING    : (100, 100, 100),
    PipelineState.PRE_APPROACH : (255, 180,  50),   # bleu clair
    PipelineState.TRACKING     : (  0, 165, 255),  # orange
    PipelineState.APPROACH     : (  0, 255, 255),  # jaune
    PipelineState.CONVERGED    : (  0, 255,   0),  # vert
    PipelineState.EMERGENCY    : (  0,   0, 255),  # rouge
}


# ─────────────────────────────────────────────────────────────────────────────
# Helpers — chargement Phase 2 (calibration + transform)
# ─────────────────────────────────────────────────────────────────────────────

def _load_phase2_assets(calib_path: str, tf_path: str):
    """Charge calibration caméra + transformation cam→robot (Phase 2)."""
    from camera_calibration import load_calibration
    from robot_transform    import load_transform, make_transform_from_geometry

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
    tf  = load_transform(tf_path)      if os.path.exists(tf_path)     else make_default_transform()
    return cam, tf


# ─────────────────────────────────────────────────────────────────────────────
# Contrôleur VS avec machine à états
# ─────────────────────────────────────────────────────────────────────────────

class Phase3Pipeline:
    """
    Pipeline complet Phase 3 : perception → erreur → commande.

    Gère la machine à états et encapsule le VSController.
    """

    def __init__(
        self,
        cam_params:   dict,
        robot_tf,
        method:       str   = "aruco",
        marker_size:  float = 0.05,
        work_plane_z: float = 0.80,
        yolo_model:   str   = None,
        force_target: np.ndarray = None,   # cible robot fixée (mm) — bypass caméra
        dt:           float = 0.033,
    ):
        if method == "yolo":
            try:
                import ultralytics  # noqa: F401
            except ImportError:
                raise ImportError(
                    "Le module 'ultralytics' est requis pour --method yolo.\n"
                    "Installer avec : pip install ultralytics"
                )
            if yolo_model is None:
                raise ValueError("--yolo-model requis avec --method yolo (ex: yolov8n.pt)")

        self.cam_params   = cam_params
        self.robot_tf     = robot_tf
        self.method       = method
        self.marker_size  = marker_size
        self.work_plane_z = work_plane_z
        self.yolo_model   = yolo_model
        self.force_target = (force_target / 1000.0) if force_target is not None else None
        self.dt           = dt

        self.params = ScaraParams()
        self.ctrl   = VSController(
            self.params,
            gain=GAIN_TRACKING,
            gain_min=0.05,
            gain_max=GAIN_TRACKING,
            adaptive=True,
            v_max_m_s=0.20,
        )

        # Configuration courante (fictive — en attendant l'encodeur réel)
        # θ3=-45° : non-singulier (r≈430mm), accessible depuis toutes les
        # directions sans traverser la singularité d'extension (θ3=0, r=460mm)
        self.q_current = np.array([0.0, 0.10, -np.pi/4, 0.0])

        # Pré-approche articulaire : config cible IK, active tant que différence > seuil
        self._preapproach_goal:   np.ndarray | None = None
        self._last_target_xy:     np.ndarray | None = None   # détecte les changements de cible

        # Machine à états
        self.state    = PipelineState.SEARCHING
        self.R_desired = np.eye(3)   # orientation désirée : top-down

        # Kalman (Phase 2)
        from kalman_tracker import MultiObjectTracker
        self.mot = MultiObjectTracker(
            dt=dt, sigma_accel=1.0, sigma_pos=0.005,
            max_missed=5, latency_s=0.150
        )

        # Historique
        self.errors:   list[VisualError]  = []
        self.commands: list[VSCommand]    = []
        self.timestamps: list[float]      = []
        self._t0 = time.time()

        # Debug visuel (mis à jour à chaque frame)
        self._dbg_t_cur:    np.ndarray = np.zeros(3)
        self._dbg_t_target: np.ndarray | None = None
        self._dbg_n_confirmed: int = 0
        self._dbg_frame: int = 0

        # Séquenceur pick-and-place
        self.sequencer = PickPlaceSequencer(
            drop_pos_m       = np.array([0.20, -0.300, -0.100]),
            home_q           = np.array([0.0, 0.10, 0.0, 0.0]),
            lift_height_m    = 0.080,
            approach_height_m= 0.040,
            place_height_m   = 0.020,
            approach_thr_mm  = 5.0,
            dt               = dt,
        )

    # -------------------------------------------------------------------------
    def process_frame(
        self,
        frame:       np.ndarray,
        depth_frame: np.ndarray = None,
    ) -> tuple[np.ndarray, VSCommand | None, PipelineState]:
        """
        Traite un frame : détection → pose → erreur → commande.

        Retourne
        --------
        annotated    : frame BGR annoté
        cmd          : VSCommand ou None si pas d'objet
        state        : état courant de la machine
        """
        from detect_objects  import detect_aruco, detect_yolo, draw_detections
        from pose_estimation import (estimate_pose_aruco, estimate_pose_yolo_flat,
                                     estimate_pose_yolo_rgbd, draw_pose_axes)

        self._dbg_frame += 1

        # ── Mode force_target : bypass caméra ───────────────────────────
        if self.force_target is not None:
            detections  = []
            poses_cam   = []
            poses_robot = []
            tracked     = []
            t_target    = self.force_target.copy()
            R_target    = self.R_desired

            t_cur = self._current_ee_position()
            R_cur = self._current_ee_rotation()
            self._dbg_t_cur    = t_cur
            self._dbg_t_target = t_target
            self._dbg_n_confirmed = 1  # forcé = toujours "confirmé"

            # ── Cible effective (séquenceur ou objet) ────────────────────────
            # Calculée AVANT le check pré-approche pour détecter les changements
            # de cible du séquenceur (ex: fin LIFTING → TRANSPORT vers dépose).
            _seq_st = self.sequencer.state
            if (_seq_st not in (PickPlaceState.IDLE, PickPlaceState.DONE)
                    and self.sequencer._target_pos is not None):
                effective_target = self.sequencer._target_pos.copy()
            else:
                effective_target = t_target

            # ── Pré-approche articulaire ─────────────────────────────────────
            eff_xy = effective_target[:2]
            if (self._last_target_xy is None
                    or np.linalg.norm(eff_xy - self._last_target_xy) > 0.05):
                self._last_target_xy = eff_xy.copy()
                if self._preapproach_goal is None:   # ne pas écraser une PA active
                    sols = ik_solutions(float(effective_target[0]),
                                        float(effective_target[1]), self.params)
                    if sols:
                        t1_g, t3_g = min(sols, key=lambda s: abs(s[1]))
                        d2_g = float(np.clip(effective_target[2] + self.params.d3 + self.params.d4,
                                             self.params.q_min[1], self.params.q_max[1]))
                        q_goal = np.array([t1_g, d2_g, t3_g, 0.0])
                        diff   = np.abs(q_goal[[0, 2]] - self.q_current[[0, 2]])
                        if np.max(diff) > np.radians(8.0):
                            self._preapproach_goal = q_goal

            # Phase pré-approche active ? Interpolation joint-space, pas de VS
            if self._preapproach_goal is not None:
                q_diff = self._preapproach_goal - self.q_current
                if np.max(np.abs(q_diff[[0, 2]])) > np.radians(8.0):
                    dstep = (np.sign(q_diff)
                             * np.minimum(np.abs(q_diff), self.params.dq_max * self.dt))
                    self.q_current = np.clip(
                        self.q_current + dstep, self.params.q_min, self.params.q_max)
                    self.state = PipelineState.PRE_APPROACH
                    annotated = frame.copy()
                    annotated = self._draw_hud(annotated, [], [], None)
                    annotated = self.sequencer.draw_hud(annotated)
                    if self._dbg_frame % 30 == 0:
                        print(f"[{self._dbg_frame:5d}] PRÉ-APPROCHE"
                              f"  θ1={np.degrees(self.q_current[0]):+.1f}°"
                              f"  θ3={np.degrees(self.q_current[2]):+.1f}°"
                              f"  cible θ1={np.degrees(self._preapproach_goal[0]):+.1f}°")
                    return annotated, None, self.state
                else:
                    self._preapproach_goal = None   # terminé, passer au VS

            # ── PBVS : asservissement visuel (exécuté à chaque frame hors PA) ─
            err = compute_error(t_cur, R_cur, effective_target, R_target,
                                thr_t_mm=CONVERGE_THR_MM,
                                thr_r_deg=CONVERGE_THR_DEG)
            self.errors.append(err)
            self.timestamps.append(time.time() - self._t0)

            # Séquenceur — reçoit vs_converged calculé sur la bonne cible
            pp_state, pp_target, gripper_close = self.sequencer.update(
                vs_converged=err.converged,
                object_pos_m=t_target,
                t_ee_m=t_cur,
                q_current=self.q_current,
            )
            # Resynchroniser si le séquenceur vient de changer de cible
            if pp_target is not None and not np.allclose(pp_target, effective_target):
                err = compute_error(t_cur, R_cur, pp_target, R_target,
                                    thr_t_mm=CONVERGE_THR_MM,
                                    thr_r_deg=CONVERGE_THR_DEG)

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
            self.q_current = np.clip(
                self.q_current + cmd.dq * self.dt,
                self.params.q_min, self.params.q_max
            )

            # Diagnostic terminal toutes les 30 frames
            if self._dbg_frame % 30 == 0:
                r_tg = float(np.linalg.norm(t_target[:2])) * 1000
                r_ee = float(np.linalg.norm(t_cur[:2]))    * 1000
                ws = "OK" if 140 <= r_tg <= 460 else "HORS WS"
                print(f"[{self._dbg_frame:5d}] |et|={err.norm_t_mm:7.1f}mm"
                      f"  EE r={r_ee:5.0f}mm"
                      f"  TGT r={r_tg:5.0f}mm [{ws}]"
                      f"  seq={self.sequencer.state.name}"
                      f"  pince={self.sequencer.gripper}"
                      f"  état={self.state.name}")

            annotated = frame.copy()
            annotated = self._draw_hud(annotated, poses_robot, tracked, cmd)
            annotated = self.sequencer.draw_hud(annotated)
            return annotated, cmd, self.state
        if self.method == "aruco":
            detections = detect_aruco(frame)
        else:
            detections = detect_yolo(frame, model_path=self.yolo_model,
                                     conf_threshold=0.5)

        # ── Estimation de pose (repère caméra) ─────────────────────────────
        if self.method == "aruco":
            poses_cam = estimate_pose_aruco(detections, self.cam_params,
                                            marker_size_m=self.marker_size)
        elif depth_frame is not None:
            poses_cam = estimate_pose_yolo_rgbd(detections, self.cam_params,
                                                depth_frame)
        else:
            poses_cam = estimate_pose_yolo_flat(detections, self.cam_params,
                                                self.work_plane_z)

        # ── Transformation → repère robot ──────────────────────────────────
        poses_robot = [self.robot_tf.transform(p) for p in poses_cam]

        # ── Kalman (suivi multi-objets) ─────────────────────────────────────
        measurements = [(p.label, p.position_m) for p in poses_robot]
        tracked      = self.mot.update(measurements)

        # ── Sélection de la cible (objet le plus proche de la zone de saisie)
        target_pose = None
        if poses_robot:
            # Priorité : objet confirmé par Kalman, sinon première détection
            confirmed = [t for t in tracked if t.confirmed]
            if confirmed:
                # Utiliser la position prédite (compensation latence)
                best     = min(confirmed,
                               key=lambda t: np.linalg.norm(t.predicted_m))
                t_target = best.predicted_m
                R_target = self.R_desired
            else:
                target_pose = poses_robot[0]
                t_target    = target_pose.position_m
                R_target    = target_pose.R_cam
            # ── Pré-approche articulaire ───────────────────────────────────────
            # Cible effective calculée AVANT le check PA pour détecter les
            # changements de cible du séquenceur (pick→transport→home).
            _seq_pre = self.sequencer.state
            if (_seq_pre not in (PickPlaceState.IDLE, PickPlaceState.DONE)
                    and self.sequencer._target_pos is not None):
                _eff_xy = self.sequencer._target_pos[:2]
            else:
                _eff_xy = t_target[:2]
            if (self._last_target_xy is None
                    or np.linalg.norm(_eff_xy - self._last_target_xy) > 0.05):
                self._last_target_xy = _eff_xy.copy()
                if self._preapproach_goal is None:   # ne pas écraser une PA active
                    sols = ik_solutions(float(_eff_xy[0]), float(_eff_xy[1]), self.params)
                    if sols:
                        t1_g, t3_g = min(sols, key=lambda s: abs(s[1]))
                        d2_g = float(np.clip(t_target[2] + self.params.d3 + self.params.d4,
                                             self.params.q_min[1], self.params.q_max[1]))
                        q_goal = np.array([t1_g, d2_g, t3_g, 0.0])
                        diff   = np.abs(q_goal[[0, 2]] - self.q_current[[0, 2]])
                        if np.max(diff) > np.radians(8.0):
                            self._preapproach_goal = q_goal

            if self._preapproach_goal is not None:
                q_diff = self._preapproach_goal - self.q_current
                if np.max(np.abs(q_diff[[0, 2]])) > np.radians(8.0):
                    dstep = (np.sign(q_diff)
                             * np.minimum(np.abs(q_diff), self.params.dq_max * self.dt))
                    self.q_current = np.clip(
                        self.q_current + dstep, self.params.q_min, self.params.q_max)
                    self.state = PipelineState.PRE_APPROACH
                    annotated = draw_detections(frame, detections)
                    annotated = self._draw_hud(annotated, poses_robot, tracked, None)
                    annotated = self.sequencer.draw_hud(annotated)
                    if self._dbg_frame % 30 == 0:
                        print(f"[{self._dbg_frame:5d}] PRÉ-APPROCHE"
                              f"  θ1={np.degrees(self.q_current[0]):+.1f}°"
                              f"  θ3={np.degrees(self.q_current[2]):+.1f}°"
                              f"  cible θ1={np.degrees(self._preapproach_goal[0]):+.1f}°")
                    return annotated, None, self.state
                else:
                    self._preapproach_goal = None
            # ── Calcul de l'erreur PBVS ──────────────────────────────────
            t_cur = self._current_ee_position()
            R_cur = self._current_ee_rotation()
            self._dbg_t_cur    = t_cur
            self._dbg_t_target = t_target
            self._dbg_n_confirmed = len([t for t in tracked if t.confirmed])

            # Cible effective : le séquenceur peut imposer une cible différente
            # (point d'approche, de levée, de transport…). On la consulte AVANT
            # de le faire avancer pour calculer l'erreur sur la bonne cible.
            seq_st = self.sequencer.state
            if (seq_st not in (PickPlaceState.IDLE, PickPlaceState.DONE)
                    and self.sequencer._target_pos is not None):
                effective_target = self.sequencer._target_pos
            else:
                effective_target = t_target

            err = compute_error(t_cur, R_cur, effective_target, R_target,
                                thr_t_mm=CONVERGE_THR_MM,
                                thr_r_deg=CONVERGE_THR_DEG)
            self.errors.append(err)
            self.timestamps.append(time.time() - self._t0)

            # ── Séquenceur pick-and-place ─────────────────────────────────
            pp_state, pp_target, gripper_close = self.sequencer.update(
                vs_converged=err.converged,
                object_pos_m=t_target,
                t_ee_m=t_cur,
                q_current=self.q_current,
            )
            # Si le séquenceur vient de changer de cible (transition d'état),
            # resynchroniser l'erreur sur la nouvelle cible
            if pp_target is not None and not np.allclose(pp_target, effective_target):
                err = compute_error(t_cur, R_cur, pp_target, R_target,
                                    thr_t_mm=CONVERGE_THR_MM,
                                    thr_r_deg=CONVERGE_THR_DEG)

            # ── Machine à états VS ────────────────────────────────────────
            if err.converged:
                self.state = PipelineState.CONVERGED
            elif err.norm_t_mm < APPROACH_THR_MM:
                self.state = PipelineState.APPROACH
                # Ralentissement via le gain uniquement.
                # Bug corrigé : dq_max *= 0.5 se composait à chaque frame
                # → robot gelé après ~10 frames → oscillation APPROACH/TRACKING
                self.ctrl.tune(gain=GAIN_APPROACH, gain_max=GAIN_APPROACH)
                self.ctrl.params.dq_max = self.params.dq_max.copy()
            else:
                self.state = PipelineState.TRACKING
                self.ctrl.tune(gain=GAIN_TRACKING, gain_max=GAIN_TRACKING)
                self.ctrl.params.dq_max = self.params.dq_max.copy()

            # ── Commande articulaire ─────────────────────────────────────
            cmd = self.ctrl.update(err, self.q_current, dt=self.dt)
            self.commands.append(cmd)

            if cmd.singular:
                self.state = PipelineState.EMERGENCY

            # Intégration fictive de la configuration
            self.q_current = np.clip(
                self.q_current + cmd.dq * self.dt,
                self.params.q_min, self.params.q_max
            )

        else:
            # Aucun objet détecté
            self.state = PipelineState.SEARCHING
            self._dbg_t_target = None
            cmd = None

        # ── Diagnostic terminal toutes les 30 frames ──────────────────────
        if self._dbg_frame % 30 == 0:
            r_tg = float(np.linalg.norm(self._dbg_t_target[:2])) * 1000 \
                if self._dbg_t_target is not None else -1.0
            r_ee = float(np.linalg.norm(self._dbg_t_cur[:2])) * 1000
            ws   = "OK" if 140 <= r_tg <= 460 else "HORS WS"
            if self.errors:
                et = self.errors[-1].norm_t_mm
                print(f"[{self._dbg_frame:5d}] |et|={et:7.1f}mm"
                      f"  EE r={r_ee:5.0f}mm"
                      f"  TGT r={r_tg:5.0f}mm [{ws}]"
                      f"  Kalman={self._dbg_n_confirmed}"
                      f"  état={self.state.name}")

        # ── Annotation visuelle ───────────────────────────────────────────
        annotated = draw_detections(frame, detections)
        annotated = draw_pose_axes(annotated, poses_cam, self.cam_params,
                                   axis_length_m=self.marker_size * 0.8)
        annotated = self._draw_hud(annotated, poses_robot, tracked, cmd)
        annotated = self.sequencer.draw_hud(annotated)

        return annotated, cmd, self.state

    # -------------------------------------------------------------------------
    def _current_ee_position(self) -> np.ndarray:
        """FK simplifiée depuis la configuration courante → position effecteur."""
        t1, d2, t3, t4 = self.q_current
        p = self.params
        px = p.a2 * np.cos(t1) + p.a3 * np.cos(t1 + t3)
        py = p.a2 * np.sin(t1) + p.a3 * np.sin(t1 + t3)
        pz = d2 - p.d3 - p.d4
        return np.array([px, py, pz])

    def _current_ee_rotation(self) -> np.ndarray:
        """FK rotation : SCARA → rotation pure autour Z."""
        phi = self.q_current[0] + self.q_current[2] + self.q_current[3]
        c, s = np.cos(phi), np.sin(phi)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    # -------------------------------------------------------------------------
    def _draw_hud(self, frame, poses_robot, tracked, cmd) -> np.ndarray:
        """Superpose le HUD (état, erreur, commande) sur le frame."""
        h, w = frame.shape[:2]
        state_color = STATE_COLORS[self.state]

        # ── Bandeau 1 : état + erreur ──────────────────────────────────
        cv2.rectangle(frame, (0, 0), (w, 48), (30, 30, 30), -1)
        cv2.putText(frame, f"STATE: {self.state.name}",
                    (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 1.0, state_color, 2)

        if self.errors:
            err = self.errors[-1]
            et_mm = err.norm_t_mm
            er_deg = err.norm_r_deg

            # Valeurs numériques
            err_txt = f"|et|={et_mm:.1f}mm  |er|={er_deg:.1f}deg"
            cv2.putText(frame, err_txt,
                        (w // 2 - 150, 32), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (255, 255, 255), 1)

            # Barre de progression |et| (vert = converge, orange = tracking, rouge = loin)
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
            # Marqueurs seuils
            thr_approach_x = bar_x + int(APPROACH_THR_MM / bar_max_mm * 200)
            thr_conv_x     = bar_x + int(CONVERGE_THR_MM  / bar_max_mm * 200)
            cv2.line(frame, (thr_approach_x, bar_y), (thr_approach_x, bar_y + bar_h), (0, 200, 255), 1)
            cv2.line(frame, (thr_conv_x,     bar_y), (thr_conv_x,     bar_y + bar_h), (0, 255, 0),   1)

        # ── Bandeau 2 : positions simulées (debug VS) ──────────────────
        ee_mm = self._dbg_t_cur * 1000
        r_ee  = float(np.linalg.norm(ee_mm[:2]))
        ee_txt = (f"EE sim: X={ee_mm[0]:+.0f} Y={ee_mm[1]:+.0f} Z={ee_mm[2]:+.0f} mm"
                  f"  r={r_ee:.0f} mm")
        cv2.rectangle(frame, (0, 50), (w, 74), (20, 20, 40), -1)
        cv2.putText(frame, ee_txt, (10, 68),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (160, 200, 255), 1)

        if self._dbg_t_target is not None:
            tg_mm = self._dbg_t_target * 1000
            r_tg  = float(np.linalg.norm(tg_mm[:2]))
            # Vérification workspace SCARA [140–460 mm]
            ws_ok = 140 <= r_tg <= 460
            ws_col = (0, 255, 0) if ws_ok else (0, 0, 255)
            tg_txt = (f"TGT cam: X={tg_mm[0]:+.0f} Y={tg_mm[1]:+.0f} Z={tg_mm[2]:+.0f} mm"
                      f"  r={r_tg:.0f} mm")
            cv2.rectangle(frame, (0, 76), (w, 100), (20, 40, 20), -1)
            cv2.putText(frame, tg_txt, (10, 94),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, ws_col, 1)
            if not ws_ok:
                cv2.putText(frame, "! HORS WORKSPACE [140-460mm]",
                            (w - 320, 94), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 255), 2)

        # Kalman
        n_conf = self._dbg_n_confirmed
        kal_col = (0, 255, 0) if n_conf > 0 else (100, 100, 100)
        cv2.putText(frame, f"Kalman: {n_conf} confirmé(s)",
                    (w - 200, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.45, kal_col, 1)

        # ── Commande articulaire ──────────────────────────────────────
        if cmd is not None:
            dq_str = "  ".join([f"dq{i+1}={v:.3f}" for i, v in enumerate(cmd.dq)])
            cv2.rectangle(frame, (0, h - 92), (w, h - 78), (15, 15, 15), -1)
            cv2.putText(frame, dq_str,
                        (10, h - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 255), 1)

        # ── Positions robot des objets détectés ───────────────────────
        for i, p in enumerate(poses_robot):
            x_mm, y_mm, z_mm = p.position_m * 1000
            txt = f"[{p.label}] X={x_mm:+.0f} Y={y_mm:+.0f} Z={z_mm:+.0f} mm"
            cv2.putText(frame, txt,
                        (10, 108 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 255, 200), 1)

        # ── Positions prédites Kalman (orange) ────────────────────────
        for obj in tracked:
            if obj.confirmed:
                px, py = self._project_3d(obj.predicted_m, frame.shape)
                if px is not None:
                    cv2.drawMarker(frame, (px, py), (0, 165, 255),
                                   cv2.MARKER_DIAMOND, 16, 2)

        return frame

    def _project_3d(self, point_m, shape):
        """Projection approximative d'un point 3D robot sur l'image."""
        K = self.cam_params["K"]
        # Transformation inverse robot→caméra (approximation)
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

    # -------------------------------------------------------------------------
    def plot_convergence(self, save_path: str = None):
        """Trace les courbes de convergence de la session."""
        if not self.errors:
            print("Pas d'historique à tracer.")
            return

        import matplotlib.pyplot as plt

        ts  = self.timestamps
        et  = [e.norm_t_mm  for e in self.errors]
        er  = [e.norm_r_deg for e in self.errors]
        lam = [c.gain       for c in self.commands] if self.commands else [0]*len(ts)

        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        fig.suptitle("Phase 3 — Convergence PBVS (session live)", fontweight="bold")

        axes[0].plot(ts, et, "b-", label="|eₜ| (mm)")
        axes[0].axhline(CONVERGE_THR_MM,  ls="--", color="r", alpha=0.6,
                        label=f"Seuil {CONVERGE_THR_MM} mm")
        axes[0].axhline(APPROACH_THR_MM, ls=":",  color="orange", alpha=0.6,
                        label=f"Zone approche {APPROACH_THR_MM} mm")
        axes[0].set_ylabel("mm"); axes[0].legend(fontsize=8); axes[0].grid(True)

        ax2 = axes[0].twinx()
        ax2.plot(ts, er, "g--", alpha=0.7, label="|eᵣ| (°)")
        ax2.axhline(CONVERGE_THR_DEG, ls="--", color="darkgreen", alpha=0.4)
        ax2.set_ylabel("°"); ax2.legend(fontsize=8, loc="upper right")

        axes[1].plot(ts, lam, "k-", label="Gain λ")
        axes[1].set_ylabel("λ"); axes[1].set_xlabel("Temps (s)")
        axes[1].grid(True); axes[1].legend(fontsize=8)

        plt.tight_layout()
        if save_path:
            os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
            fig.savefig(save_path, dpi=100)
            print(f"Graphique sauvegardé : {save_path}")
        else:
            plt.show()

        return fig


# ─────────────────────────────────────────────────────────────────────────────
# Mode simulation (sans caméra)
# ─────────────────────────────────────────────────────────────────────────────

def run_simulation(args):
    """Lance la simulation Python boucle fermée (vs_controller.simulate_pbvs)."""
    print("\n╔══════════════════════════════════════════╗")
    print("║  Phase 3 — Simulation PBVS (Python)     ║")
    print("╚══════════════════════════════════════════╝\n")

    q0    = np.array([0.5, 0.08, -0.3, 0.2])
    t_des = np.array([0.350, 0.050, -0.150])   # r=354mm ∈ [340,460]mm ✓
    R_des = np.eye(3)

    print(f"q0    = {np.degrees(np.array([q0[0],0,q0[2],q0[3]]))} ° | d2={q0[1]*1e3:.0f} mm")
    print(f"t_des = {t_des*1e3} mm\n")

    history, converged, ctrl = simulate_pbvs(
        q0=q0, t_desired=t_des, R_desired=R_des,
        dt=0.033, max_iter=500, gain=0.5,
        adaptive=True, verbose=True,
    )

    print(f"\nRésultat : {'✓ CONVERGÉ' if converged else '✗ NON CONVERGÉ'}")
    print(f"Itérations : {len(history)}")

    if history:
        f = history[-1]
        print(f"Erreur finale : {f['norm_t_mm']:.3f} mm / {f['norm_r_deg']:.3f}°")

    # Affichage
    try:
        import matplotlib
        matplotlib.use("TkAgg" if sys.platform == "darwin" else "Agg")
        fig = ctrl.plot_history(show=False)
        os.makedirs("test_images", exist_ok=True)
        path = "test_images/phase3_sim_convergence.png"
        fig.savefig(path, dpi=120)
        print(f"\nGraphique sauvegardé : {path}")

        # Essayer d'afficher (si écran disponible)
        try:
            import matplotlib.pyplot as plt
            plt.show()
        except Exception:
            pass
    except ImportError:
        print("matplotlib non disponible — pas de graphique.")

    return converged


# ─────────────────────────────────────────────────────────────────────────────
# Mode live (webcam ou RealSense)
# ─────────────────────────────────────────────────────────────────────────────

def run_live(args):
    """Lance le pipeline Phase 3 en flux temps réel."""
    print("\n╔══════════════════════════════════════════════╗")
    print("║  Phase 3 — Pipeline Temps Réel (Vision)     ║")
    print("╚══════════════════════════════════════════════╝\n")

    # Charger les assets Phase 2
    calib_path = os.path.join(os.path.dirname(__file__),
                              '..', 'phase2', 'calibration_data', 'camera_params.npz')
    tf_path    = os.path.join(os.path.dirname(__file__),
                              '..', 'phase2', 'calibration_data', 'cam_to_robot.npz')
    cam_params, robot_tf = _load_phase2_assets(calib_path, tf_path)

    pipeline = Phase3Pipeline(
        cam_params=cam_params,
        robot_tf=robot_tf,
        method=args.method,
        marker_size=args.size,
        work_plane_z=args.z,
        yolo_model=args.yolo_model,
        force_target=np.array(args.force_target) if args.force_target else None,
        dt=1.0 / 30,
    )

    # ── Source vidéo ────────────────────────────────────────────────────────
    use_rs = False
    if args.realsense:
        try:
            from realsense_capture import auto_camera
            sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'phase2'))
            camera_ctx = auto_camera(cam_index=args.cam, work_plane_z_mm=args.z * 1000)
            use_rs = True
        except ImportError:
            print("[WARN] pyrealsense2 non disponible — fallback webcam.")

    if not use_rs:
        cap = cv2.VideoCapture(args.cam if args.live else args.cam)
        if not cap.isOpened():
            print(f"Impossible d'ouvrir la caméra (index {args.cam})", file=sys.stderr)
            sys.exit(1)

    print("Appuyer sur 'q' pour quitter, 's' pour sauvegarder, 'r' pour réinitialiser.")

    frame_idx = 0

    def read_frame():
        if use_rs:
            return camera_ctx.read()
        ret, f = cap.read()
        return (f, None) if ret else (None, None)

    ctx = camera_ctx if use_rs else None

    def run_loop(read_fn):
        nonlocal frame_idx
        while True:
            color, depth = read_fn()
            if color is None:
                break

            annotated, cmd, state = pipeline.process_frame(color, depth)

            cv2.imshow("Phase 3 — Visual Servoing", annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                fname = f"test_images/phase3_frame_{frame_idx:04d}.jpg"
                os.makedirs("test_images", exist_ok=True)
                cv2.imwrite(fname, annotated)
                print(f"Frame sauvegardé : {fname}")
            elif key == ord('r'):
                pipeline.ctrl.reset()
                pipeline.state = PipelineState.SEARCHING
                print("Contrôleur réinitialisé.")

            frame_idx += 1

    if use_rs:
        with camera_ctx as _:
            run_loop(read_frame)
    else:
        run_loop(read_frame)
        cap.release()

    cv2.destroyAllWindows()

    # Tracer la convergence de la session
    pipeline.plot_convergence(save_path="test_images/phase3_session.png")


# ─────────────────────────────────────────────────────────────────────────────
# Mode image statique
# ─────────────────────────────────────────────────────────────────────────────

def run_image(args):
    """Test statique sur une image."""
    frame = cv2.imread(args.image)
    if frame is None:
        print(f"Impossible de lire : {args.image}", file=sys.stderr)
        sys.exit(1)

    calib_path = os.path.join(os.path.dirname(__file__),
                              '..', 'phase2', 'calibration_data', 'camera_params.npz')
    tf_path    = os.path.join(os.path.dirname(__file__),
                              '..', 'phase2', 'calibration_data', 'cam_to_robot.npz')
    cam_params, robot_tf = _load_phase2_assets(calib_path, tf_path)

    pipeline = Phase3Pipeline(cam_params=cam_params, robot_tf=robot_tf,
                               method=args.method, marker_size=args.size,
                               yolo_model=args.yolo_model)

    annotated, cmd, state = pipeline.process_frame(frame)

    print(f"État     : {state.name}")
    if cmd is not None:
        print(f"Commande : {cmd}")
    if pipeline.errors:
        print(f"Erreur   : {pipeline.errors[-1]}")

    cv2.imshow("Phase 3 — Test image", annotated)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Phase 3 — Asservissement visuel PBVS (SCARA 4-DOF)"
    )
    src = parser.add_mutually_exclusive_group()
    src.add_argument("--sim",       action="store_true",
                     help="Simulation Python boucle fermée (sans caméra)")
    src.add_argument("--image",     metavar="FILE",
                     help="Test sur image statique")
    src.add_argument("--live",      action="store_true",
                     help="Flux webcam temps réel")
    src.add_argument("--realsense", action="store_true",
                     help="Flux RealSense D435 RGB-D")

    parser.add_argument("--force-target", type=float, nargs=3,
                        metavar=("X", "Y", "Z"), dest="force_target",
                        help="Cible VS forcée en repère robot (mm) — bypass caméra. "
                             "Ex: --force-target 350 50 -150")
    parser.add_argument("--method",  choices=["aruco","yolo"], default="aruco")
    parser.add_argument("--yolo-model", type=str, default="yolov8n.pt", dest="yolo_model",
                        help="Chemin vers le modèle YOLO (.pt), ex: yolov8n.pt")
    parser.add_argument("--size",    type=float, default=0.05,
                        help="Taille marqueur ArUco (m)")
    parser.add_argument("--z",       type=float, default=0.80,
                        help="Hauteur plan de travail (m)")
    parser.add_argument("--cam",     type=int,   default=0)
    parser.add_argument("--gain",    type=float, default=0.5,
                        help="Gain λ nominal du contrôleur")
    parser.add_argument("--no-adapt",action="store_true",
                        help="Désactiver le gain adaptatif")
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
        print("Aucun mode sélectionné. Exemple :")
        print("  python main_phase3.py --sim")
        print("  python main_phase3.py --live --cam 1 --method aruco --size 0.08")
        print("  python main_phase3.py --realsense --method aruco --size 0.05")
        sys.exit(1)


if __name__ == "__main__":
    main()
