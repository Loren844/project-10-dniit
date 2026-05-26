"""
gripper_controller.py
Gestion de la pince et séquence pick-and-place pour robot SCARA 4-DOF.

Séquence complète :
  IDLE → (objet détecté) →
  APPROACH   : asservissement visuel vers la position de saisie
  GRASPING   : fermeture de la pince (timer ~300 ms)
  LIFTING    : levée à hauteur de sécurité (+80 mm)
  TRANSPORT  : déplacement XY vers la zone de dépose
  LOWERING   : descente sur la zone de dépose (+20 mm au-dessus)
  RELEASING  : ouverture de la pince (timer ~250 ms)
  RETURNING  : retour à la position de veille (home)
  DONE       → IDLE (nouveau cycle)

Usage :
    from gripper_controller import GripperController, PickPlaceSequencer

    gripper   = GripperController()
    sequencer = PickPlaceSequencer(drop_pos_m=np.array([0.2, -0.30, -0.10]))

    # Dans la boucle temps réel :
    pp_state, vs_target, close_cmd = sequencer.update(
        vs_converged=error.converged,
        object_pos_m=t_object,
        t_ee_m=t_current,
        q_current=q,
    )
    gripper.update(dt)
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# États
# ─────────────────────────────────────────────────────────────────────────────

class GripperState(Enum):
    OPEN    = auto()
    CLOSING = auto()
    CLOSED  = auto()
    OPENING = auto()


class PickPlaceState(Enum):
    IDLE      = auto()   # en attente d'un objet
    APPROACH  = auto()   # asservissement visuel actif
    GRASPING  = auto()   # fermeture de la pince
    LIFTING   = auto()   # levée à hauteur de sécurité
    TRANSPORT = auto()   # déplacement vers la zone de dépose
    LOWERING  = auto()   # descente sur la zone de dépose
    RELEASING = auto()   # ouverture de la pince
    RETURNING = auto()   # retour à la position de veille
    DONE      = auto()   # cycle terminé (→ IDLE automatiquement)


# Couleurs HUD pour chaque état (BGR)
PP_STATE_COLORS = {
    PickPlaceState.IDLE:      (100, 100, 100),
    PickPlaceState.APPROACH:  (  0, 165, 255),  # orange
    PickPlaceState.GRASPING:  (  0, 255, 255),  # jaune
    PickPlaceState.LIFTING:   (255, 165,   0),  # bleu clair
    PickPlaceState.TRANSPORT: (255, 200,   0),
    PickPlaceState.LOWERING:  (  0, 200, 255),
    PickPlaceState.RELEASING: (128,   0, 255),
    PickPlaceState.RETURNING: (180, 180, 180),
    PickPlaceState.DONE:      (  0, 255,   0),  # vert
}

PP_STATE_LABELS = {
    PickPlaceState.IDLE:      "VEILLE",
    PickPlaceState.APPROACH:  "APPROCHE (VS)",
    PickPlaceState.GRASPING:  "SAISIE — fermeture pince",
    PickPlaceState.LIFTING:   "LEVÉE",
    PickPlaceState.TRANSPORT: "TRANSPORT",
    PickPlaceState.LOWERING:  "ABAISSEMENT",
    PickPlaceState.RELEASING: "DÉPOSE — ouverture pince",
    PickPlaceState.RETURNING: "RETOUR HOME",
    PickPlaceState.DONE:      "CYCLE TERMINÉ",
}


# ─────────────────────────────────────────────────────────────────────────────
# Contrôleur de pince (simulation timing)
# ─────────────────────────────────────────────────────────────────────────────

class GripperController:
    """
    Modèle temporel simple de la pince.

    Sur le robot réel, remplacer update() par l'envoi d'une commande
    au contrôleur de la pince (GPIO, CAN, RS485, etc.).

    Paramètres
    ----------
    close_time_s : temps pour fermer (s) — typiquement 0.25–0.40 s
    open_time_s  : temps pour ouvrir  (s) — typiquement 0.20–0.30 s
    """

    def __init__(self, close_time_s: float = 0.30, open_time_s: float = 0.25):
        self.close_time = close_time_s
        self.open_time  = open_time_s
        self.state      = GripperState.OPEN
        self._timer     = 0.0

    # -------------------------------------------------------------------------
    def command_close(self):
        """Déclenche la fermeture (ignoré si déjà fermée ou en fermeture)."""
        if self.state in (GripperState.OPEN, GripperState.OPENING):
            self.state  = GripperState.CLOSING
            self._timer = 0.0

    def command_open(self):
        """Déclenche l'ouverture (ignoré si déjà ouverte ou en ouverture)."""
        if self.state in (GripperState.CLOSED, GripperState.CLOSING):
            self.state  = GripperState.OPENING
            self._timer = 0.0

    # -------------------------------------------------------------------------
    def update(self, dt: float) -> GripperState:
        """Avance le timer interne. Appeler à chaque pas de la boucle."""
        if self.state == GripperState.CLOSING:
            self._timer += dt
            if self._timer >= self.close_time:
                self.state = GripperState.CLOSED
        elif self.state == GripperState.OPENING:
            self._timer += dt
            if self._timer >= self.open_time:
                self.state = GripperState.OPEN
        return self.state

    # -------------------------------------------------------------------------
    @property
    def is_idle(self) -> bool:
        """True si la pince n'est pas en transition."""
        return self.state in (GripperState.OPEN, GripperState.CLOSED)

    @property
    def progress(self) -> float:
        """Avancement de la transition courante : 0.0 → 1.0."""
        if self.state == GripperState.CLOSING:
            return min(1.0, self._timer / self.close_time)
        elif self.state == GripperState.OPENING:
            return min(1.0, self._timer / self.open_time)
        return 1.0 if self.state == GripperState.CLOSED else 0.0

    def __repr__(self) -> str:
        icons = {
            GripperState.OPEN:    "○ OUVERTE",
            GripperState.CLOSING: f"◑ FERMETURE {self.progress*100:.0f}%",
            GripperState.CLOSED:  "● FERMÉE",
            GripperState.OPENING: f"◐ OUVERTURE {self.progress*100:.0f}%",
        }
        return icons[self.state]


# ─────────────────────────────────────────────────────────────────────────────
# Séquenceur pick-and-place
# ─────────────────────────────────────────────────────────────────────────────

class PickPlaceSequencer:
    """
    Machine à états de la séquence complète de pick-and-place.

    Coordonne :
    - le visual servoing (Phase 3) pour les mouvements précis
    - la commande de la pince (GripperController)
    - les positions cibles de chaque phase

    Paramètres
    ----------
    drop_pos_m       : position de dépose [x, y, z] dans le repère robot (m)
    home_q           : configuration de veille [θ1, d2, θ3, θ4]
    lift_height_m    : hauteur de levée après saisie (m)
    approach_height_m: hauteur d'approche avant saisie (m) — descend en GRASPING
    place_height_m   : décalage Z au-dessus de la surface de dépose (m)
    approach_thr_mm  : seuil de convergence VS pour passer à l'étape suivante (mm)
    dt               : pas de temps de la boucle (s)
    """

    def __init__(
        self,
        drop_pos_m:        np.ndarray = None,
        home_q:            np.ndarray = None,
        lift_height_m:     float = 0.080,
        approach_height_m: float = 0.040,
        place_height_m:    float = 0.020,
        approach_thr_mm:   float = 5.0,
        dt:                float = 0.033,
    ):
        self.drop_pos         = drop_pos_m if drop_pos_m is not None \
                                else np.array([0.20, -0.300, -0.100])
        self.home_q           = home_q if home_q is not None \
                                else np.array([0.0, 0.10, 0.0, 0.0])
        self.lift_height      = lift_height_m
        self.approach_height  = approach_height_m
        self.place_height     = place_height_m
        self.approach_thr_mm  = approach_thr_mm
        self.dt               = dt

        self.gripper          = GripperController()
        self.state            = PickPlaceState.IDLE

        self._target_pos:    Optional[np.ndarray] = None
        self._lift_z:        float = 0.0
        self._last_obj_pos:  Optional[np.ndarray] = None
        self._timer:         float = 0.0

        self.cycle_count:    int = 0
        self.state_log:      list[tuple[float, PickPlaceState]] = []
        self._t:             float = 0.0

    # -------------------------------------------------------------------------
    def update(
        self,
        vs_converged:  bool,
        object_pos_m:  Optional[np.ndarray],
        t_ee_m:        np.ndarray,
        q_current:     np.ndarray,
    ) -> tuple[PickPlaceState, Optional[np.ndarray], bool]:
        """
        Met à jour la machine à états.

        Paramètres
        ----------
        vs_converged : True si |e_t| < seuil ET |e_r| < seuil
        object_pos_m : position de l'objet (repère robot, m) — None si non vu
        t_ee_m       : position courante de l'effecteur (FK, m)
        q_current    : configuration articulaire courante

        Retourne
        --------
        state        : état courant
        vs_target_m  : cible de position pour le contrôleur VS (None = pas de VS)
        gripper_close: True = commande fermeture, False = commande ouverture
        """
        self._t += self.dt
        self.gripper.update(self.dt)
        vs_target    = None
        gripper_close = (self.gripper.state in
                         (GripperState.CLOSED, GripperState.CLOSING))

        prev_state = self.state

        # ── IDLE : en attente d'un objet ─────────────────────────────────
        if self.state == PickPlaceState.IDLE:
            if object_pos_m is not None:
                self._last_obj_pos = object_pos_m.copy()
                self._target_pos   = self._approach_target(object_pos_m)
                self._transition(PickPlaceState.APPROACH)

        # ── APPROACH : VS actif vers la position d'approche ──────────────
        elif self.state == PickPlaceState.APPROACH:
            if object_pos_m is not None:
                self._last_obj_pos = object_pos_m.copy()
                self._target_pos   = self._approach_target(object_pos_m)
            vs_target = self._target_pos
            gripper_close = False

            if vs_converged:
                # Cible finale = position réelle de l'objet (pas d'offset Z)
                self._target_pos = self._last_obj_pos.copy()
                self.gripper.command_close()
                self._transition(PickPlaceState.GRASPING)

        # ── GRASPING : descente vers l'objet + fermeture pince ───────────
        elif self.state == PickPlaceState.GRASPING:
            vs_target     = self._last_obj_pos
            gripper_close = True

            if self.gripper.state == GripperState.CLOSED:
                self._lift_z     = t_ee_m[2] + self.lift_height
                self._target_pos = t_ee_m.copy()
                self._target_pos[2] = self._lift_z
                self._transition(PickPlaceState.LIFTING)

        # ── LIFTING : levée à hauteur de sécurité ────────────────────────
        elif self.state == PickPlaceState.LIFTING:
            vs_target     = self._target_pos
            gripper_close = True

            z_err_mm = abs(t_ee_m[2] - self._lift_z) * 1000
            if z_err_mm < self.approach_thr_mm:
                # Cible transport = dépose à hauteur sécurité
                transport         = self.drop_pos.copy()
                transport[2]      = self._lift_z
                self._target_pos  = transport
                self._transition(PickPlaceState.TRANSPORT)

        # ── TRANSPORT : déplacement XY vers la zone de dépose ────────────
        elif self.state == PickPlaceState.TRANSPORT:
            vs_target     = self._target_pos
            gripper_close = True

            xy_err_mm = np.linalg.norm(t_ee_m[:2] - self.drop_pos[:2]) * 1000
            if xy_err_mm < self.approach_thr_mm:
                place             = self.drop_pos.copy()
                place[2]         += self.place_height
                self._target_pos  = place
                self._transition(PickPlaceState.LOWERING)

        # ── LOWERING : descente sur la zone de dépose ────────────────────
        elif self.state == PickPlaceState.LOWERING:
            vs_target     = self._target_pos
            gripper_close = True

            z_des_m  = self.drop_pos[2] + self.place_height
            z_err_mm = abs(t_ee_m[2] - z_des_m) * 1000
            if z_err_mm < self.approach_thr_mm:
                self.gripper.command_open()
                self._transition(PickPlaceState.RELEASING)

        # ── RELEASING : ouverture de la pince ────────────────────────────
        elif self.state == PickPlaceState.RELEASING:
            vs_target     = None   # rester immobile pendant l'ouverture
            gripper_close = False

            if self.gripper.state == GripperState.OPEN:
                # Remonter d'abord avant de rentrer
                home_t    = self._fk(self.home_q)
                home_t[2] = self._lift_z
                self._target_pos = home_t
                self._transition(PickPlaceState.RETURNING)

        # ── RETURNING : retour à la position de veille ───────────────────
        elif self.state == PickPlaceState.RETURNING:
            vs_target     = self._target_pos
            gripper_close = False

            home_err_mm = np.linalg.norm(t_ee_m[:2] - self._target_pos[:2]) * 1000
            if home_err_mm < self.approach_thr_mm * 3:
                self.cycle_count += 1
                self._transition(PickPlaceState.DONE)

        # ── DONE : cycle terminé → retour IDLE ───────────────────────────
        elif self.state == PickPlaceState.DONE:
            self._transition(PickPlaceState.IDLE)

        return self.state, vs_target, gripper_close

    # -------------------------------------------------------------------------
    def _approach_target(self, obj_pos: np.ndarray) -> np.ndarray:
        """Position d'approche = objet + offset Z."""
        t = obj_pos.copy()
        t[2] += self.approach_height
        return t

    def _fk(self, q: np.ndarray) -> np.ndarray:
        """FK simplifiée SCARA → position effecteur (m)."""
        from vs_controller import ScaraParams
        p = ScaraParams()
        t1, d2, t3, _ = q
        return np.array([
            p.a2 * np.cos(t1) + p.a3 * np.cos(t1 + t3),
            p.a2 * np.sin(t1) + p.a3 * np.sin(t1 + t3),
            d2 - p.d3 - p.d4,
        ])

    def _transition(self, new_state: PickPlaceState):
        self.state = new_state
        self.state_log.append((self._t, new_state))
        print(f"  [PickPlace t={self._t:6.2f}s] {PP_STATE_LABELS[new_state]}"
              f"  | Pince: {self.gripper}"
              f"  | Cycles: {self.cycle_count}")

    def reset(self):
        """Réinitialise complètement la séquence."""
        self.state      = PickPlaceState.IDLE
        self._target_pos = None
        self._timer      = 0.0
        self.gripper.command_open()
        self._transition(PickPlaceState.IDLE)

    # -------------------------------------------------------------------------
    def draw_hud(self, frame: "np.ndarray") -> "np.ndarray":
        """Superpose le panneau pince + état séquence sur un frame OpenCV."""
        import cv2
        h, w = frame.shape[:2]

        color  = PP_STATE_COLORS[self.state]
        label  = PP_STATE_LABELS[self.state]
        gripper_str = str(self.gripper)

        # Fond du bandeau
        cv2.rectangle(frame, (0, h - 80), (w, h), (20, 20, 20), -1)

        cv2.putText(frame, f"SEQUENCE: {label}",
                    (10, h - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        cv2.putText(frame, f"PINCE: {gripper_str}  |  Cycles: {self.cycle_count}",
                    (10, h - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        # Barre de progression de la pince
        bar_w  = int(self.gripper.progress * 200)
        bar_color = (0, 255, 0) if self.gripper.state == GripperState.CLOSED \
                    else (0, 140, 255)
        cv2.rectangle(frame, (w - 220, h - 20), (w - 20, h - 8), (60, 60, 60), -1)
        if bar_w > 0:
            cv2.rectangle(frame, (w - 220, h - 20), (w - 220 + bar_w, h - 8),
                          bar_color, -1)

        return frame


# ─────────────────────────────────────────────────────────────────────────────
# Test autonome
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys
    from vs_controller import simulate_pbvs, ScaraParams

    print("=== Test gripper_controller.py — Séquence pick-and-place ===\n")

    params    = ScaraParams()
    sequencer = PickPlaceSequencer(
        drop_pos_m       = np.array([0.200, -0.280, -0.130]),
        home_q           = np.array([0.0, 0.10, 0.0, 0.0]),
        lift_height_m    = 0.080,
        approach_height_m= 0.040,
        place_height_m   = 0.020,
        approach_thr_mm  = 5.0,
        dt               = 0.033,
    )

    dt             = 0.033
    q              = np.array([0.5, 0.10, -0.3, 0.2])
    obj_pos        = np.array([0.350, 0.050, -0.150])   # objet sur le tapis
    R_des          = np.eye(3)
    max_steps      = 2000
    step           = 0
    prev_pp_state  = None

    # FK simplifiée
    def fk(q_):
        t1, d2, t3, _ = q_
        return np.array([
            params.a2*np.cos(t1) + params.a3*np.cos(t1+t3),
            params.a2*np.sin(t1) + params.a3*np.sin(t1+t3),
            d2 - params.d3 - params.d4,
        ])

    from visual_error  import compute_error
    from vs_controller import VSController

    ctrl = VSController(params, gain=1.0, adaptive=True)
    R_cur_fn = lambda q_: np.array([
        [np.cos(q_[0]+q_[2]+q_[3]), -np.sin(q_[0]+q_[2]+q_[3]), 0],
        [np.sin(q_[0]+q_[2]+q_[3]),  np.cos(q_[0]+q_[2]+q_[3]), 0],
        [0, 0, 1]
    ])

    print("Début simulation pick-and-place...\n")

    while step < max_steps:
        step += 1
        t_ee = fk(q)
        R_cur = R_cur_fn(q)

        # Mise à jour du séquenceur
        pp_state, vs_target, gripper_close = sequencer.update(
            vs_converged=(False),   # sera calculé ci-dessous
            object_pos_m=obj_pos if step > 5 else None,
            t_ee_m=t_ee,
            q_current=q,
        )

        if vs_target is not None:
            err = compute_error(t_ee, R_cur, vs_target, R_des)
            # Recalculer avec vs_converged correct
            pp_state, vs_target, gripper_close = sequencer.update(
                vs_converged=err.converged,
                object_pos_m=obj_pos if pp_state == PickPlaceState.APPROACH else None,
                t_ee_m=t_ee,
                q_current=q,
            )
            if vs_target is not None:
                cmd = ctrl.update(err, q, dt=dt)
                q = np.clip(q + cmd.dq * dt, params.q_min, params.q_max)
        else:
            err = None

        if pp_state == PickPlaceState.DONE or \
           (pp_state == PickPlaceState.IDLE and sequencer.cycle_count > 0):
            print(f"\n✓ Cycle terminé en {step} itérations ({step*dt:.1f} s)")
            break

    print(f"\nCycles accomplis : {sequencer.cycle_count}")
    print("Historique des transitions :")
    for t, s in sequencer.state_log:
        print(f"  t={t:6.2f}s → {PP_STATE_LABELS[s]}")
