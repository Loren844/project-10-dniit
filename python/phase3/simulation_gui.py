"""
simulation_gui.py
Simulation interactive 2D du robot SCARA PBVS pick-and-place.

Contrôles
---------
  Clic gauche  : placer l'objet à saisir (dans le workspace)
  Clic droit   : déplacer la zone de dépose
  ESPACE       : démarrer / pause
  R            : réinitialiser
  Q / Echap    : quitter

Usage
-----
  python simulation_gui.py
"""

from __future__ import annotations

import os
import sys

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── Backend compatible macOS + Linux + Windows ────────────────────────────────
for _backend in ("TkAgg", "Qt5Agg", "MacOSX", "Agg"):
    try:
        matplotlib.use(_backend)
        break
    except Exception:
        continue

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vs_controller      import ScaraParams, VSController, ik_solutions
from visual_error       import compute_error
from gripper_controller import PickPlaceSequencer, PickPlaceState, GripperState

# ─────────────────────────────────────────────────────────────────────────────
# Constantes
# ─────────────────────────────────────────────────────────────────────────────
DT          = 0.033       # pas de simulation (s)
GAIN_MAX    = 1.5
GAIN_MIN    = 0.05
THR_T_MM    = 2.0
THR_R_DEG   = 1.0
Z_WORK      = -0.150      # hauteur plan de travail (m)

# Couleurs des états de la séquence (RGB 0–1)
SEQ_COLORS = {
    PickPlaceState.IDLE:      "#888888",
    PickPlaceState.APPROACH:  "#FF8C00",
    PickPlaceState.GRASPING:  "#FFD700",
    PickPlaceState.LIFTING:   "#00BFFF",
    PickPlaceState.TRANSPORT: "#1E90FF",
    PickPlaceState.LOWERING:  "#9370DB",
    PickPlaceState.RELEASING: "#FF69B4",
    PickPlaceState.RETURNING: "#AAAAAA",
    PickPlaceState.DONE:      "#00FF7F",
}


# ─────────────────────────────────────────────────────────────────────────────
# Utilitaires FK
# ─────────────────────────────────────────────────────────────────────────────

def _ee_pos(q: np.ndarray, p: ScaraParams) -> np.ndarray:
    t1, d2, t3, _ = q
    return np.array([
        p.a2 * np.cos(t1) + p.a3 * np.cos(t1 + t3),
        p.a2 * np.sin(t1) + p.a3 * np.sin(t1 + t3),
        d2 - p.d3 - p.d4,
    ])

def _ee_rot(q: np.ndarray) -> np.ndarray:
    phi = q[0] + q[2] + q[3]
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def _desired_rot(t_des: np.ndarray) -> np.ndarray:
    """
    Orientation souhaitée : la pince pointe radialement vers la cible dans le plan XY.
    Pour un SCARA pick-and-place c'est la convention naturelle : φ_des = atan2(y, x).
    """
    phi = np.arctan2(t_des[1], t_des[0])
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def _arm_xy(q: np.ndarray, p: ScaraParams):
    """Retourne (épaule, coude, EE) en mm dans le plan XY."""
    t1, _, t3, _ = q
    shoulder = np.zeros(2)
    elbow    = np.array([p.a2 * np.cos(t1),
                          p.a2 * np.sin(t1)]) * 1000
    ee       = np.array([p.a2 * np.cos(t1) + p.a3 * np.cos(t1 + t3),
                          p.a2 * np.sin(t1) + p.a3 * np.sin(t1 + t3)]) * 1000
    return shoulder, elbow, ee


# ─────────────────────────────────────────────────────────────────────────────
# Application
# ─────────────────────────────────────────────────────────────────────────────

class ScaraSimGUI:

    _Q0    = np.array([0.0, 0.10, -np.pi/4, 0.0])   # config de repos (θ3=-45°, non-singulier)
    _DROP0 = np.array([0.20, -0.28, Z_WORK])    # dépose par défaut (m)

    def __init__(self):
        self.params  = ScaraParams()
        p = self.params
        # Rayon min effectif : contraint par θ3 ∈ [-90°, +90°]
        # r_min = sqrt(a2² + a3² + 2·a2·a3·cos(θ3_max))   (θ3_max=π/2 → sqrt(a2²+a3²))
        t3_lim = min(abs(p.q_max[2]), abs(p.q_min[2]))
        self.r_min   = np.sqrt(p.a2**2 + p.a3**2 + 2*p.a2*p.a3*np.cos(t3_lim)) * 1000
        self.r_max   = (p.a2 + p.a3) * 1000       # mm

        self.q       = self._Q0.copy()
        self.t_pick  = None                   # objet (m) — None = pas encore placé
        self.t_drop  = self._DROP0.copy()
        self._q_init = self._Q0.copy()   # config départ adaptée à la branche IK de la cible
        self._q_goal          = None    # solution IK de la cible (pré-approche)
        self._in_preapproach  = False   # vrai pendant la phase joint-space
        self._last_eff        = None    # dernière cible VS (détection changement de cible)

        self.ctrl      = self._make_ctrl()
        self.sequencer = self._make_sequencer()

        self.running   = False
        self.step_n    = 0

        self.traj_ee   = []   # positions EE historiques (mm)

        self._setup_figure()
        self._connect_events()

    # ──────────────────────────────────────────────────────────────────────────
    # IK analytique (vérification d'accessibilité)
    # ──────────────────────────────────────────────────────────────────────────

    def _ik_solutions(self, x_m: float, y_m: float) -> list:
        """D\u00e9l\u00e8gue \u00e0 vs_controller.ik_solutions (source unique de v\u00e9rit\u00e9)."""
        return ik_solutions(x_m, y_m, self.params)

    # ──────────────────────────────────────────────────────────────────────────
    # Factories
    # ──────────────────────────────────────────────────────────────────────────

    def _make_ctrl(self) -> VSController:
        return VSController(
            self.params,
            gain=GAIN_MAX, gain_min=GAIN_MIN, gain_max=GAIN_MAX,
            adaptive=True,
        )

    def _make_sequencer(self) -> PickPlaceSequencer:
        return PickPlaceSequencer(
            drop_pos_m        = self.t_drop.copy(),
            home_q            = self._Q0.copy(),
            lift_height_m     = 0.080,
            approach_height_m = 0.040,
            place_height_m    = 0.020,
            approach_thr_mm   = 5.0,
            dt                = DT,
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Création de la figure
    # ──────────────────────────────────────────────────────────────────────────

    def _setup_figure(self):
        BG = "#12121e"
        self.fig = plt.figure(figsize=(15, 8), facecolor=BG)
        try:
            self.fig.canvas.manager.set_window_title(
                "SCARA 4-DOF — Simulation PBVS Interactive")
        except Exception:
            pass

        # Layout : 2 colonnes, colonne gauche = vue XY, colonne droite = XZ + état
        gs = self.fig.add_gridspec(
            2, 2, left=0.04, right=0.97, top=0.91, bottom=0.07,
            width_ratios=[1.3, 1], wspace=0.32, hspace=0.40,
        )
        self.ax_xy    = self.fig.add_subplot(gs[:, 0])
        self.ax_xz    = self.fig.add_subplot(gs[0, 1])
        self.ax_state = self.fig.add_subplot(gs[1, 1])

        AXBG = "#0a0a18"
        for ax in (self.ax_xy, self.ax_xz, self.ax_state):
            ax.set_facecolor(AXBG)
            ax.tick_params(colors="#606080", labelsize=8)
            for sp in ax.spines.values():
                sp.set_color("#2a2a44")

        self.fig.text(
            0.5, 0.96,
            "SCARA 4-DOF  —  Simulation PBVS Pick-and-Place",
            ha="center", color="white", fontsize=13, fontweight="bold",
        )
        self.fig.text(
            0.5, 0.01,
            "Clic gauche : objet  |  Clic droit : dépose  |"
            "  ESPACE : démarrer/pause  |  R : reset  |  Q : quitter",
            ha="center", color="#606080", fontsize=8,
        )

        self._build_xy_view()
        self._build_xz_view()
        self._build_state_panel()

    # ── Vue XY ────────────────────────────────────────────────────────────────

    def _build_xy_view(self):
        ax = self.ax_xy
        M  = 530
        ax.set_xlim(-M, M); ax.set_ylim(-M, M)
        ax.set_aspect("equal")
        ax.set_title("Vue de dessus (XY)  —  cliquer pour placer",
                      color="white", fontsize=10, pad=6)
        ax.set_xlabel("X (mm)", color="#606080", fontsize=8)
        ax.set_ylabel("Y (mm)", color="#606080", fontsize=8)
        ax.grid(True, color="#1a1a32", linewidth=0.5, zorder=0)

        # ── Workspace pixel-exact (masque vectoris\u00e9) ─────────────────────────
        # On calcule sur une grille N\u00d7N l'IK analytique : m\u00eame logique que
        # _ik_solutions() \u2192 la visualisation correspond exactement \u00e0 la validation.
        _N  = 320
        _xg = np.linspace(-M, M, _N) / 1000   # m
        _yg = np.linspace(-M, M, _N) / 1000   # m
        _XX, _YY = np.meshgrid(_xg, _yg)
        _p  = self.params
        _a2, _a3 = _p.a2, _p.a3

        _c3_raw = (_XX**2 + _YY**2 - _a2**2 - _a3**2) / (2.0 * _a2 * _a3)
        _valid  = np.zeros((_N, _N), dtype=bool)
        for _sgn in (+1.0, -1.0):
            _c3 = np.clip(_c3_raw, -1.0, 1.0)
            _s3 = _sgn * np.sqrt(np.maximum(0.0, 1.0 - _c3**2))
            _t3 = np.arctan2(_s3, _c3)
            _t1 = np.arctan2(_YY, _XX) - np.arctan2(_a3 * _s3, _a2 + _a3 * _c3)
            _t1 = np.arctan2(np.sin(_t1), np.cos(_t1))
            _valid |= (
                (np.abs(_c3_raw) <= 1.0)
                & (_t1 >= _p.q_min[0]) & (_t1 <= _p.q_max[0])
                & (_t3 >= _p.q_min[2]) & (_t3 <= _p.q_max[2])
            )

        # RGBA : vert fonc\u00e9 = valid, rouge fonc\u00e9 = hors portée
        _rgba = np.zeros((_N, _N, 4), dtype=float)
        _rgba[ _valid] = [0.02, 0.13, 0.02, 0.95]
        _rgba[~_valid] = [0.09, 0.03, 0.03, 0.95]
        ax.imshow(_rgba, extent=[-M, M, -M, M], origin='lower',
                  zorder=1, interpolation='nearest', aspect='auto')

        # Contour de la fronti\u00e8re workspace (ligne verte)
        _xmm = np.linspace(-M, M, _N)
        _ymm = np.linspace(-M, M, _N)
        ax.contour(_xmm, _ymm, _valid.astype(float),
                   levels=[0.5], colors=['#2aaa2a'], linewidths=1.3, zorder=2)

        # Label
        _mid_r = (self.r_min + self.r_max) / 2
        _t1_lim_deg = float(np.degrees(self.params.q_max[0]))
        ax.text(_mid_r * 0.60, _mid_r * 0.60,
                f"WORKSPACE\n[{self.r_min:.0f}\u2013{self.r_max:.0f}] mm",
                color="#3a8a3a", fontsize=7, ha="center", va="center", zorder=3)
        ax.text(-M * 0.72, 0,
                f"\u03b81 > \u00b1{_t1_lim_deg:.0f}\u00b0\n(hors limites)",
                color="#993333", fontsize=7, ha="center", va="center", zorder=3)

        # Axes
        ax.axhline(0, color="#1e1e36", linewidth=0.7)
        ax.axvline(0, color="#1e1e36", linewidth=0.7)

        # Zone de dépose (patch + label, mis à jour dynamiquement)
        dx, dy = self.t_drop[:2] * 1000
        self._drop_circ = mpatches.Circle((dx, dy), 22,
                                           color="#FF4500", alpha=0.75, zorder=5)
        ax.add_patch(self._drop_circ)
        self._drop_lbl = ax.text(dx, dy - 34, "DÉPOSE",
                                  color="#FF6030", fontsize=7,
                                  ha="center", zorder=6)

        # Trajectoire
        self._traj_xy, = ax.plot([], [], "-", color="#00CED1",
                                   linewidth=0.9, alpha=0.55, zorder=3)

        # Cible courante du séquenceur (croix orange)
        self._seq_tgt, = ax.plot([], [], "+", color="#FF8C00",
                                   markersize=15, markeredgewidth=2.0, zorder=10)

        # Bras — 2 liens + 3 joints
        s, e, ee = _arm_xy(self.q, self.params)
        self._lnk1, = ax.plot([s[0], e[0]], [s[1], e[1]],
                               "-", color="#3377FF", linewidth=6,
                               solid_capstyle="round", zorder=7)
        self._lnk2, = ax.plot([e[0], ee[0]], [e[1], ee[1]],
                               "-", color="#77AAFF", linewidth=4,
                               solid_capstyle="round", zorder=7)
        self._j_sh,  = ax.plot(*s,  "o", color="white",   markersize=9,  zorder=8)
        self._j_el,  = ax.plot(*e,  "o", color="#99BBFF",  markersize=7,  zorder=8)
        self._j_ee,  = ax.plot(*ee, "s", color="#FFD700",  markersize=8,  zorder=9)

        # Pince (deux traits)
        self._grip_l, = ax.plot([], [], "-", color="#FFD700", linewidth=3, zorder=9)
        self._grip_r, = ax.plot([], [], "-", color="#FFD700", linewidth=3, zorder=9)

        # Objet (géré dynamiquement)
        self._pick_circ = None
        self._pick_lbl  = None

        # Message flash en bas-gauche
        self._msg = ax.text(-M + 10, -M + 12,
                             "Cliquer pour placer l'objet",
                             color="#00FFFF", fontsize=8,
                             va="bottom", fontfamily="monospace", zorder=12)

    # ── Vue XZ ────────────────────────────────────────────────────────────────

    def _build_xz_view(self):
        ax = self.ax_xz
        p  = self.params
        ax.set_xlim(-520, 520)
        z_lo = (-(p.d3 + p.d4)) * 1000 - 30
        z_hi = p.d2_base * 1000 + 30
        ax.set_ylim(z_lo, z_hi)
        ax.set_title("Vue de côté (XZ)", color="white", fontsize=10, pad=6)
        ax.set_xlabel("X (mm)", color="#606080", fontsize=8)
        ax.set_ylabel("Z (mm)", color="#606080", fontsize=8)
        ax.grid(True, color="#1a1a32", linewidth=0.5)

        # Plan de travail
        ax.axhline(Z_WORK * 1000, color="#1e4a1e", linewidth=1,
                    linestyle="--", alpha=0.8)
        ax.text(510, Z_WORK * 1000 + 4, f"Z={Z_WORK*1000:.0f}mm",
                color="#2a6a2a", fontsize=7, ha="right")

        # Trajectoire côté
        self._traj_xz, = ax.plot([], [], "-", color="#00CED1",
                                   linewidth=0.9, alpha=0.55, zorder=3)

        # Colonne verticale + EE
        ee_xz = _ee_pos(self.q, p)
        self._col_xz, = ax.plot([0, 0],
                                  [p.d2_base * 1000, ee_xz[2] * 1000],
                                  "-", color="#444466", linewidth=5, zorder=5)
        self._arm_xz, = ax.plot([0, ee_xz[0] * 1000],
                                  [ee_xz[2] * 1000, ee_xz[2] * 1000],
                                  "--", color="#3377FF", linewidth=1.5,
                                  alpha=0.7, zorder=4)
        self._ee_xz,  = ax.plot(ee_xz[0] * 1000, ee_xz[2] * 1000,
                                  "s", color="#FFD700", markersize=9, zorder=8)

        # Objet et dépose
        self._pick_xz, = ax.plot([], [], "o", color="#00FF7F",
                                   markersize=10, zorder=7)
        self._drop_xz, = ax.plot(self.t_drop[0] * 1000, self.t_drop[2] * 1000,
                                   "o", color="#FF4500", markersize=8, zorder=7)

    # ── Panneau état ──────────────────────────────────────────────────────────

    def _build_state_panel(self):
        ax = self.ax_state
        ax.set_title("État  /  Erreur  /  Séquence",
                      color="white", fontsize=10, pad=6)
        ax.axis("off")

        # Bloc texte
        self._stxt = ax.text(
            0.03, 0.97, "En attente d'un objet…",
            transform=ax.transAxes, color="white",
            fontsize=8.5, va="top", fontfamily="monospace",
            linespacing=1.6,
        )

        # Barre |et|
        bg = mpatches.FancyBboxPatch(
            (0.03, 0.04), 0.94, 0.15,
            transform=ax.transAxes,
            boxstyle="round,pad=0.01",
            facecolor="#1c1c2e", edgecolor="#2a2a44",
        )
        ax.add_patch(bg)
        self._ebar = mpatches.FancyBboxPatch(
            (0.03, 0.04), 0.01, 0.15,
            transform=ax.transAxes,
            boxstyle="round,pad=0.01",
            facecolor="#FF3333", edgecolor="none",
        )
        ax.add_patch(self._ebar)
        self._elbl = ax.text(0.50, 0.115, "",
                              transform=ax.transAxes,
                              color="white", fontsize=8,
                              ha="center", va="center")
        ax.text(0.03, 0.22, "0",
                transform=ax.transAxes, color="#555577", fontsize=7)
        ax.text(0.97, 0.22, "400 mm",
                transform=ax.transAxes, color="#555577", fontsize=7, ha="right")

    # ──────────────────────────────────────────────────────────────────────────
    # Gestion des événements
    # ──────────────────────────────────────────────────────────────────────────

    def _connect_events(self):
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("key_press_event",    self._on_key)

    def _on_click(self, event):
        if event.inaxes != self.ax_xy or event.xdata is None:
            return
        x_mm, y_mm = event.xdata, event.ydata
        r_mm = float(np.hypot(x_mm, y_mm))

        if event.button == 1:                   # ── gauche : objet ──────────
            if not (self.r_min <= r_mm <= self.r_max):
                self._flash(
                    f"⚠  Hors workspace  (r={r_mm:.0f}mm, "
                    f"plage [{self.r_min:.0f}–{self.r_max:.0f}]mm)",
                    "#FF4444",
                )
                return
            sols = self._ik_solutions(x_mm / 1000, y_mm / 1000)
            if not sols:
                lim = int(np.degrees(self.params.q_max[0]))
                self._flash(
                    f"⚠  Hors portée articulaire  "
                    f"(θ1 dépasserait ±{lim}°)  —  essayez une autre position",
                    "#FF8800",
                )
                return
            self.t_pick = np.array([x_mm / 1000, y_mm / 1000, Z_WORK])
            # Branche IK : θ3 le plus petit en valeur absolue
            t1_best, t3_best = min(sols, key=lambda s: abs(s[1]))
            t3_init = float(np.sign(t3_best) if t3_best != 0.0 else 1.0) * np.pi / 4
            self._q_init = np.array([0.0, self._Q0[1], t3_init, 0.0])
            # Objectif pré-approche = solution IK complète de la cible
            self._q_goal = np.array([t1_best, self._Q0[1], t3_best, 0.0])
            self._place_pick_marker(x_mm, y_mm)
            self._flash(
                f"Objet placé  ({x_mm:+.0f}, {y_mm:+.0f}, {Z_WORK*1000:.0f}) mm"
                "  —  ESPACE pour lancer",
                "#00FFFF",
            )
            if self.running:
                self._reset_sim()

        elif event.button == 3:                 # ── droit : dépose ──────────
            if not (self.r_min <= r_mm <= self.r_max):
                self._flash("⚠  Zone de dépose hors workspace !", "#FF8800")
                return
            if not self._ik_solutions(x_mm / 1000, y_mm / 1000):
                lim = int(np.degrees(self.params.q_max[0]))
                self._flash(
                    f"⚠  Zone de dépose hors portée articulaire  (θ1 > ±{lim}°)",
                    "#FF8800",
                )
                return
            self.t_drop = np.array([x_mm / 1000, y_mm / 1000, Z_WORK])
            self._drop_circ.center = (x_mm, y_mm)
            self._drop_lbl.set_position((x_mm, y_mm - 34))
            self._drop_xz.set_data([x_mm], [Z_WORK * 1000])
            self.sequencer = self._make_sequencer()
            self._flash(
                f"Dépose déplacée  ({x_mm:+.0f}, {y_mm:+.0f}) mm",
                "#FF8800",
            )

    def _on_key(self, event):
        k = event.key
        if k == " ":
            if self.t_pick is None:
                self._flash("Placer un objet d'abord (clic gauche) !", "#FF4444")
                return
            if not self.running:
                # Démarrage (ou redémarrage après cycle terminé) :
                # réinitialise toujours le bras dans la bonne branche IK
                self._reset_sim()
                self.running = True
                self._flash("▶  SIMULATION EN COURS", "#00FF44")
            else:
                # Pause en cours de simulation
                self.running = False
                self._flash("⏸  PAUSE  — ESPACE pour reprendre", "#FFD700")
        elif k in ("r", "R"):
            self._full_reset()
        elif k in ("q", "Q", "escape"):
            plt.close(self.fig)

    # ──────────────────────────────────────────────────────────────────────────
    # Reset
    # ──────────────────────────────────────────────────────────────────────────

    def _full_reset(self):
        self.running = False
        self.t_pick  = None
        self._q_init = self._Q0.copy()
        self._q_goal = None
        self._in_preapproach = False
        if self._pick_circ is not None:
            self._pick_circ.remove(); self._pick_circ = None
        if self._pick_lbl is not None:
            self._pick_lbl.remove();  self._pick_lbl  = None
        self._pick_xz.set_data([], [])
        self._seq_tgt.set_data([], [])
        self._reset_sim()
        self._flash("Réinitialisé  —  cliquer pour placer un objet", "#00FFFF")

    def _reset_sim(self):
        self.q            = self._q_init.copy()   # branche IK adaptée à la cible
        self._in_preapproach = True               # redémarrer la pré-approche
        self._last_eff    = None                  # réinitialise la détection de changement
        self.ctrl         = self._make_ctrl()
        self.sequencer = self._make_sequencer()
        self.traj_ee   = []
        self.step_n    = 0
        self._traj_xy.set_data([], [])
        self._traj_xz.set_data([], [])
        # Remettre le marqueur objet à sa position d'origine
        if self.t_pick is not None and self._pick_circ is not None:
            ox, oy = self.t_pick[:2] * 1000
            self._pick_circ.center = (ox, oy)
            self._pick_circ.set_visible(True)
            if self._pick_lbl is not None:
                self._pick_lbl.set_position((ox, oy + 28))
                self._pick_lbl.set_visible(True)
            self._pick_xz.set_data([ox], [Z_WORK * 1000])

    # ──────────────────────────────────────────────────────────────────────────
    # Marqueurs
    # ──────────────────────────────────────────────────────────────────────────

    def _place_pick_marker(self, x_mm, y_mm):
        if self._pick_circ is not None:
            self._pick_circ.remove()
        if self._pick_lbl is not None:
            self._pick_lbl.remove()
        self._pick_circ = mpatches.Circle(
            (x_mm, y_mm), 20, color="#00FF7F", alpha=0.85, zorder=6)
        self.ax_xy.add_patch(self._pick_circ)
        self._pick_lbl = self.ax_xy.text(
            x_mm, y_mm + 28, "OBJET",
            color="#00FF7F", fontsize=7, ha="center", zorder=7)
        self._pick_xz.set_data([x_mm], [Z_WORK * 1000])

    def _flash(self, msg: str, color: str = "white"):
        self._msg.set_text(msg)
        self._msg.set_color(color)

    # ──────────────────────────────────────────────────────────────────────────
    # Simulation — un pas
    # ──────────────────────────────────────────────────────────────────────────

    def _step(self):
        self.step_n += 1
        p = self.params

        # Cible courante du séquenceur (lue AVANT sequencer.update)
        seq_st = self.sequencer.state
        if (seq_st not in (PickPlaceState.IDLE, PickPlaceState.DONE)
                and self.sequencer._target_pos is not None):
            eff = self.sequencer._target_pos.copy()
        else:
            eff = self.t_pick.copy()

        # ── Détection de changement de cible → pré-approche automatique ───────
        # Compare la cible actuelle avec celle du pas précédent.
        # Déclenche une pré-approche joint-space si le saut XY > 50 mm
        # ET que θ1/θ3 actuels sont loin de la solution IK (> 8°).
        if (self._last_eff is not None
                and np.linalg.norm(eff[:2] - self._last_eff[:2]) > 0.05
                and self._q_goal is None):          # ne pas écraser une pré-approche active
            sols = self._ik_solutions(float(eff[0]), float(eff[1]))
            if sols:
                t1_g, t3_g = min(sols, key=lambda s: abs(s[1]))
                if np.max(np.abs([t1_g - self.q[0], t3_g - self.q[2]])) > np.radians(8):
                    self._q_goal = np.array([t1_g, self.q[1], t3_g, 0.0])
        self._last_eff = eff.copy()   # toujours mis à jour (avant le return anticipé)

        # ── Phase 0 : pré-approche en espace articulaire ──────────────────────
        # Le PBVS (Jacobien) converge localement : pour les grandes erreurs θ1/θ3
        # on interpole d'abord en joint-space, puis VS prend le relais.
        if self._q_goal is not None:
            q_diff = self._q_goal - self.q
            # Seuil : θ1 et θ3 à moins de 8° de la solution IK cible
            if np.max(np.abs([q_diff[0], q_diff[2]])) > np.radians(8):
                # Seuls θ1 (joint 0) et θ3 (joint 2) bougent ;
                # d2 et θ4 restent gérés par VS/séquenceur.
                dstep = np.zeros(4)
                for i in (0, 2):
                    dstep[i] = (np.sign(q_diff[i])
                                * min(abs(q_diff[i]), p.dq_max[i] * DT))
                self.q = np.clip(self.q + dstep, p.q_min, p.q_max)
                t_c = _ee_pos(self.q, p)
                R_c = _ee_rot(self.q)
                err = compute_error(t_c, R_c, eff, np.eye(3),
                                    thr_t_mm=THR_T_MM, thr_r_deg=THR_R_DEG)
                self.traj_ee.append(t_c * 1000)
                self._in_preapproach = True
                return err, seq_st          # retourne l'état séquenceur courant
            self._q_goal = None             # pré-approche terminée → VS prend le relais
        self._in_preapproach = False

        # ── Phase 1+ : asservissement visuel ──────────────────────────────────
        t_c = _ee_pos(self.q, p)
        R_c = _ee_rot(self.q)

        # R_d = I pour le SCARA : θ4 compense l'orientation seul,
        # sans interférer avec la tâche de position (θ1, θ3).
        R_d = np.eye(3)
        err = compute_error(t_c, R_c, eff, R_d,
                             thr_t_mm=THR_T_MM, thr_r_deg=THR_R_DEG)

        pp_state, pp_target, _ = self.sequencer.update(
            vs_converged=err.converged,
            object_pos_m=self.t_pick,
            t_ee_m=t_c,
            q_current=self.q,
        )

        if pp_target is not None and not np.allclose(pp_target, eff):
            err = compute_error(t_c, R_c, pp_target, R_d,
                                 thr_t_mm=THR_T_MM, thr_r_deg=THR_R_DEG)

        cmd = self.ctrl.update(err, self.q, dt=DT,
                                w0=0.1)   # faible pénalité butées
        self.q = np.clip(
            self.q + cmd.dq * DT,
            p.q_min, p.q_max,
        )

        self.traj_ee.append(_ee_pos(self.q, p) * 1000)
        return err, pp_state

    # ──────────────────────────────────────────────────────────────────────────
    # Rendu graphique
    # ──────────────────────────────────────────────────────────────────────────

    def _redraw(self, err=None, pp_state=None):
        p = self.params
        s, elbow, ee = _arm_xy(self.q, p)
        ee3   = _ee_pos(self.q, p)   # calculé une seule fois
        ex_mm = ee3[0] * 1000
        ey_mm = ee3[1] * 1000
        ez_mm = ee3[2] * 1000

        # ── Vue XY ────────────────────────────────────────────────────────────
        self._lnk1.set_data([s[0], elbow[0]], [s[1], elbow[1]])
        self._lnk2.set_data([elbow[0], ee[0]], [elbow[1], ee[1]])
        self._j_sh.set_data([s[0]],  [s[1]])
        self._j_el.set_data([elbow[0]], [elbow[1]])
        self._j_ee.set_data([ee[0]], [ee[1]])

        self._draw_gripper(ee)

        # ── Animation de l'objet saisi ─────────────────────────────────────────
        _HELD = {PickPlaceState.GRASPING, PickPlaceState.LIFTING,
                 PickPlaceState.TRANSPORT, PickPlaceState.LOWERING}
        _GONE = {PickPlaceState.RELEASING, PickPlaceState.RETURNING,
                 PickPlaceState.DONE}
        if self._pick_circ is not None and pp_state is not None:
            if pp_state in _HELD:
                self._pick_circ.center = (ex_mm, ey_mm)
                self._pick_circ.set_visible(True)
                if self._pick_lbl is not None:
                    self._pick_lbl.set_position((ex_mm, ey_mm + 28))
                    self._pick_lbl.set_visible(True)
                self._pick_xz.set_data([ex_mm], [ez_mm])
            elif pp_state in _GONE:
                self._pick_circ.set_visible(False)
                if self._pick_lbl is not None:
                    self._pick_lbl.set_visible(False)
                self._pick_xz.set_data([], [])
            # sinon (IDLE, APPROACH) : marqueur au sol, position originale

        # Trajectoire XY
        if len(self.traj_ee) > 1:
            xs = [pt[0] for pt in self.traj_ee]
            ys = [pt[1] for pt in self.traj_ee]
            self._traj_xy.set_data(xs, ys)

        # Cible séquenceur
        seq_st = self.sequencer.state
        if (seq_st not in (PickPlaceState.IDLE, PickPlaceState.DONE)
                and self.sequencer._target_pos is not None):
            tx, ty = self.sequencer._target_pos[:2] * 1000
            self._seq_tgt.set_data([tx], [ty])
        else:
            self._seq_tgt.set_data([], [])

        # ── Vue XZ ────────────────────────────────────────────────────────────
        self._col_xz.set_data([0, 0], [p.d2_base * 1000, ez_mm])
        self._arm_xz.set_data([0, ex_mm], [ez_mm, ez_mm])
        self._ee_xz.set_data([ex_mm], [ez_mm])

        if len(self.traj_ee) > 1:
            xs = [pt[0] for pt in self.traj_ee]
            zs = [pt[2] for pt in self.traj_ee]
            self._traj_xz.set_data(xs, zs)

        # ── Panneau état ──────────────────────────────────────────────────────
        if err is not None and pp_state is not None:
            self._update_state(err, pp_state)

    def _draw_gripper(self, ee_mm: np.ndarray):
        """Pince : deux traits divergents à l'EE, animation ouverture/fermeture."""
        angle    = self.q[0] + self.q[2] + self.q[3]
        progress = self.sequencer.gripper.progress
        g_state  = self.sequencer.gripper.state

        opening_mm = 18.0 * (1.0 - progress)   # 0 (fermé) → 18 mm (ouvert)
        L          = 24.0                        # longueur des doigts (mm)

        perp = np.array([-np.sin(angle),  np.cos(angle)])
        fwd  = np.array([ np.cos(angle),  np.sin(angle)]) * L

        l_base = ee_mm + perp * opening_mm
        r_base = ee_mm - perp * opening_mm

        self._grip_l.set_data(
            [l_base[0], l_base[0] + fwd[0]],
            [l_base[1], l_base[1] + fwd[1]],
        )
        self._grip_r.set_data(
            [r_base[0], r_base[0] + fwd[0]],
            [r_base[1], r_base[1] + fwd[1]],
        )

        col = (
            "#FFD700" if g_state == GripperState.CLOSED else
            "#FFA500" if g_state in (GripperState.CLOSING, GripperState.OPENING) else
            "#FFEE88"
        )
        self._grip_l.set_color(col)
        self._grip_r.set_color(col)

    def _update_state(self, err, pp_state: PickPlaceState):
        g       = self.sequencer.gripper
        t_sim   = self.step_n * DT
        et_mm   = err.norm_t_mm
        er_deg  = err.norm_r_deg
        col     = SEQ_COLORS.get(pp_state, "#FFFFFF")

        # Labels séquence
        seq_labels = {
            PickPlaceState.IDLE:      "VEILLE",
            PickPlaceState.APPROACH:  "APPROCHE (VS actif)",
            PickPlaceState.GRASPING:  "SAISIE — fermeture pince",
            PickPlaceState.LIFTING:   "LEVÉE",
            PickPlaceState.TRANSPORT: "TRANSPORT",
            PickPlaceState.LOWERING:  "ABAISSEMENT",
            PickPlaceState.RELEASING: "DÉPOSE — ouverture pince",
            PickPlaceState.RETURNING: "RETOUR HOME",
            PickPlaceState.DONE:      "CYCLE TERMINÉ ✓",
        }
        seq_lbl = seq_labels.get(pp_state, pp_state.name)
        if self._in_preapproach:
            seq_lbl = "PRÉ-APPROCHE articulaire → sol. IK"
            col     = "#4488FF"

        q = self.q
        txt = (
            f"t = {t_sim:6.1f} s   étape {self.step_n}\n"
            f"\n"
            f"|eₜ|  =  {et_mm:8.2f} mm   seuil {THR_T_MM} mm\n"
            f"|eᵣ|  =  {er_deg:8.2f} °    seuil {THR_R_DEG} °\n"
            f"\n"
            f"SÉQUENCE :  {seq_lbl}\n"
            f"PINCE    :  {g}\n"
            f"CYCLES   :  {self.sequencer.cycle_count}\n"
            f"\n"
            f"θ₁ = {np.degrees(q[0]):+6.1f}°   "
            f"d₂ = {q[1]*1000:5.1f} mm\n"
            f"θ₃ = {np.degrees(q[2]):+6.1f}°   "
            f"θ₄ = {np.degrees(q[3]):+6.1f}°"
        )
        self._stxt.set_text(txt)
        self._stxt.set_color(col)

        # Barre |et|
        frac = min(1.0, et_mm / 400.0)
        self._ebar.set_width(frac * 0.94)
        bar_col = (
            "#00FF44" if et_mm < THR_T_MM else
            "#FFD700" if et_mm < 50 else
            "#FF3333"
        )
        self._ebar.set_facecolor(bar_col)
        self._elbl.set_text(f"|et| = {et_mm:.1f} mm")

    # ──────────────────────────────────────────────────────────────────────────
    # Boucle principale
    # ──────────────────────────────────────────────────────────────────────────

    def run(self):
        plt.ion()
        self._redraw()
        self.fig.canvas.draw()

        while plt.fignum_exists(self.fig.number):
            if self.running and self.t_pick is not None:
                err, pp_state = self._step()
                self._redraw(err, pp_state)
                # Après DONE → pause automatique + message
                if pp_state == PickPlaceState.IDLE and self.sequencer.cycle_count > 0:
                    self.running = False
                    self._flash(
                        f"Cycle {self.sequencer.cycle_count} terminé  "
                        "— cliquer pour un nouvel objet ou ESPACE pour relancer",
                        "#00FF7F",
                    )

            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            plt.pause(DT)


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    gui = ScaraSimGUI()
    gui.run()
