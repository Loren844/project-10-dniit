"""
vs_controller.py
Contrôleur d'asservissement visuel PBVS pour robot SCARA 4-DOF.

Loi de commande
---------------
    v_c = -λ(t) · L_s⁺ · e

où :
    e       ∈ R^6   erreur visuelle [e_t ; e_r]
    L_s⁺    ∈ R^(6×6) pseudo-inverse de la matrice d'interaction
    λ(t)    gain adaptatif (constant ou décroissant avec la norme de e)

La vitesse cartésienne v_c = [vx, vy, vz, ωx, ωy, ωz] est convertie
en vitesses articulaires via la jacobienne inverse du SCARA :

    q̇ = J⁺(q) · v_c

J(q) est la jacobienne analytique 6×4 du SCARA 4-DOF calculée par
différentiation numérique de la FK (plus robuste que la formule analytique
aux singularités).

Saturation et sécurité
-----------------------
- Saturation des vitesses articulaires : |q̇_i| ≤ dq_max_i
- Garde de butées articulaires : pénalisation si q_i proche des limites
- Détection de singularité : alarme si σ_min(J) < seuil
- Arrêt d'urgence si la norme de v_c dépasse un seuil

Architecture
------------
    VSController
        ├── update(error, q_current) → VSCommand
        ├── reset()
        └── tune(λ, λ_min, λ_max)

    VSCommand (dataclass)
        ├── dq        : vitesses articulaires (rad/s ou m/s)
        ├── v_c       : vitesse cartésienne commandée
        ├── singular  : bool — singularité détectée
        └── saturated : bool — saturation active

Usage :
    from visual_error  import compute_error
    from vs_controller import VSController

    ctrl  = VSController(robot_params, gain=0.5)
    error = compute_error(t_cur, R_cur, t_des, R_des)
    cmd   = ctrl.update(error, q_current)
    q_new = q_current + cmd.dq * dt
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from visual_error import VisualError


# ─────────────────────────────────────────────────────────────────────────────
# Paramètres robot (copie légère depuis robot_parameters.m)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ScaraParams:
    """Paramètres géométriques et limites du SCARA 4-DOF."""
    a2:        float = 0.300   # m
    a3:        float = 0.160   # m
    d3:        float = 0.150   # m  (offset cinématique DH lien 3 – portée verticale)
    d4:        float = 0.059   # m  (offset effecteur)
    d2_base:   float = 0.200   # m  (hauteur colonne de référence)

    # Limites articulaires [θ1, d2, θ3, θ4]  (rad ou m)
    q_min: np.ndarray = field(default_factory=lambda: np.array(
        [-2.356, 0.000, -1.571, -3.142]))
    q_max: np.ndarray = field(default_factory=lambda: np.array(
        [ 2.356, 0.200,  1.571,  3.142]))

    # Vitesses maximales [rad/s, m/s, rad/s, rad/s]
    dq_max: np.ndarray = field(default_factory=lambda: np.array(
        [2.0, 0.1, 2.0, 2.0]))


# ─────────────────────────────────────────────────────────────────────────────
# Cinématique inverse analytique SCARA
# ─────────────────────────────────────────────────────────────────────────────

def ik_solutions(x_m: float, y_m: float,
                 params: ScaraParams) -> list[tuple[float, float]]:
    """
    Solutions IK analytiques du SCARA (joints θ1 et θ3) pour un point (x, y) [m].

    Retourne la liste des couples (θ1, θ3) atteignables dans les limites
    articulaires du robot. Liste vide ⟹ position hors portée.

    Utilisé pour :
    - Valider qu'un point est accessible avant de lancer le VS
    - Calculer la configuration cible de la pré-approche articulaire
    """
    a2, a3 = params.a2, params.a3
    c3 = (x_m**2 + y_m**2 - a2**2 - a3**2) / (2.0 * a2 * a3)
    if abs(c3) > 1.0:
        return []   # rayon hors portée
    solutions: list[tuple[float, float]] = []
    for sign in (+1.0, -1.0):   # coude-haut / coude-bas
        s3 = sign * np.sqrt(max(0.0, 1.0 - c3**2))
        t3 = float(np.arctan2(s3, c3))
        if not (params.q_min[2] <= t3 <= params.q_max[2]):
            continue
        t1 = np.arctan2(y_m, x_m) - np.arctan2(a3 * s3, a2 + a3 * c3)
        t1 = float(np.arctan2(np.sin(t1), np.cos(t1)))  # normalise [-π, π]
        if not (params.q_min[0] <= t1 <= params.q_max[0]):
            continue
        solutions.append((t1, t3))
    return solutions


# ─────────────────────────────────────────────────────────────────────────────
# Sortie du contrôleur
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class VSCommand:
    """Commande articulaire issue du contrôleur visuel."""
    dq:        np.ndarray         # vitesses articulaires (4,) [rad/s, m/s, rad/s, rad/s]
    v_c:       np.ndarray         # vitesse cartésienne commandée (6,)
    singular:  bool  = False      # singularité détectée
    saturated: bool  = False      # au moins un axe saturé
    gain:      float = 0.0        # gain λ appliqué
    sigma_min: float = 0.0        # valeur singulière minimale de J

    def __repr__(self) -> str:
        flags = []
        if self.singular:  flags.append("⚠ SINGULIER")
        if self.saturated: flags.append("SAT")
        flags_str = " ".join(flags)
        dq_str = np.array2string(self.dq, precision=4, suppress_small=True)
        return f"VSCommand(dq={dq_str}, λ={self.gain:.3f} {flags_str})"


# ─────────────────────────────────────────────────────────────────────────────
# Jacobienne du SCARA
# ─────────────────────────────────────────────────────────────────────────────

def scara_jacobian(q: np.ndarray, params: ScaraParams,
                   eps: float = 1e-5) -> np.ndarray:
    """
    Jacobienne analytique du SCARA 4-DOF, J ∈ R^(6×4).

    Calculée analytiquement (pas par différences finies) :
    Colonne i = [∂p/∂qi ; ∂ω/∂qi]

    Convention de pose sortie :
        p    = [px, py, pz]   position de l'effecteur (m)
        ω    = [0,  0,  θ1+θ3+θ4]  orientation (SCARA : rotation pure autour Z)

    Paramètres
    ----------
    q : [θ1, d2, θ3, θ4] — configuration courante
    """
    t1, d2, t3, t4 = q
    a2 = params.a2
    a3 = params.a3
    d3 = params.d3
    d4 = params.d4

    # Position effecteur (FK SCARA)
    # px = a2*cos(t1) + a3*cos(t1+t3)
    # py = a2*sin(t1) + a3*sin(t1+t3)
    # pz = d2 - d3 - d4          (axe Z descendant)

    s1  = np.sin(t1);         c1  = np.cos(t1)
    s13 = np.sin(t1 + t3);    c13 = np.cos(t1 + t3)

    # ∂p/∂θ1
    dp_dt1 = np.array([
        -a2 * s1 - a3 * s13,
         a2 * c1 + a3 * c13,
         0.0
    ])

    # ∂p/∂d2  (prismatique, axe Z descendant)
    dp_dd2 = np.array([0.0, 0.0, 1.0])

    # ∂p/∂θ3
    dp_dt3 = np.array([
        -a3 * s13,
         a3 * c13,
         0.0
    ])

    # ∂p/∂θ4  (θ4 n'affecte pas la position pour un SCARA standard)
    dp_dt4 = np.zeros(3)

    # Partie angulaire : SCARA — seul θ1+θ3+θ4 donne la rotation autour Z
    # ω = [0, 0, dΩ/dq_i]
    # Jacobienne angulaire (z-axis only for SCARA)
    dw_dt1 = np.array([0.0, 0.0, 1.0])
    dw_dd2 = np.zeros(3)
    dw_dt3 = np.array([0.0, 0.0, 1.0])
    dw_dt4 = np.array([0.0, 0.0, 1.0])

    J = np.column_stack([
        np.concatenate([dp_dt1, dw_dt1]),
        np.concatenate([dp_dd2, dw_dd2]),
        np.concatenate([dp_dt3, dw_dt3]),
        np.concatenate([dp_dt4, dw_dt4]),
    ])  # shape (6, 4)

    return J


def damped_pinv(J: np.ndarray, sigma_min_threshold: float = 0.01,
                lambda_dls: float = 0.01) -> tuple[np.ndarray, float, bool]:
    """
    Pseudo-inverse amortie (Damped Least Squares) pour gérer les singularités.

        J⁺ = Jᵀ (JJᵀ + λ²I)⁻¹

    Retourne (J_pinv, sigma_min, is_singular).
    """
    U, s, Vt = np.linalg.svd(J, full_matrices=False)
    sigma_min = float(s[-1])
    is_singular = sigma_min < sigma_min_threshold

    if is_singular:
        # Amortissement adaptatif
        lam = lambda_dls * (1.0 - (sigma_min / sigma_min_threshold) ** 2)
        s_damp = s / (s ** 2 + lam ** 2)
    else:
        s_damp = 1.0 / s

    J_pinv = (Vt.T * s_damp) @ U.T
    return J_pinv, sigma_min, is_singular


# ─────────────────────────────────────────────────────────────────────────────
# Contrôleur principal
# ─────────────────────────────────────────────────────────────────────────────

class VSController:
    """
    Contrôleur d'asservissement visuel PBVS pour SCARA 4-DOF.

    Paramètres
    ----------
    params           : ScaraParams
    gain             : gain λ nominal (défaut 0.5)
    gain_min         : gain minimum (adaptatif)
    gain_max         : gain maximum (adaptatif)
    adaptive         : bool — activer le gain adaptatif
    singular_thresh  : seuil σ_min pour détecter une singularité
    joint_margin     : fraction des limites articulaires pour la zone de ralentissement
    v_max_m_s        : vitesse cartésienne max de sécurité (m/s)
    """

    def __init__(
        self,
        params:          ScaraParams = None,
        gain:            float = 0.5,
        gain_min:        float = 0.05,
        gain_max:        float = 2.0,
        adaptive:        bool  = True,
        singular_thresh: float = 0.01,
        joint_margin:    float = 0.05,
        v_max_m_s:       float = 0.20,
    ):
        self.params          = params or ScaraParams()
        self._gain           = gain
        self.gain_min        = gain_min
        self.gain_max        = gain_max
        self.adaptive        = adaptive
        self.singular_thresh = singular_thresh
        self.joint_margin    = joint_margin
        self.v_max_m_s       = v_max_m_s

        # Historique (pour débogage et tracé)
        self._history: list[dict] = []

    # -------------------------------------------------------------------------
    def tune(self, gain: float = None, gain_min: float = None,
             gain_max: float = None):
        """Modifier les gains à la volée."""
        if gain     is not None: self._gain    = gain
        if gain_min is not None: self.gain_min = gain_min
        if gain_max is not None: self.gain_max = gain_max

    def reset(self):
        """Réinitialise l'historique."""
        self._history.clear()

    # -------------------------------------------------------------------------
    def _adaptive_gain(self, error_norm: float) -> float:
        """
        Gain adaptatif : λ(‖e‖) augmente quand l'erreur est grande
        et décroît pour éviter l'overshooting à l'approche.

        Modèle exponentiel (Chaumette 2006, adapté PBVS) :
            λ(‖e‖) = (λ_max - λ_min) · (1 - exp(-α·‖e‖)) + λ_min

        → grande erreur : λ ≈ λ_max (convergence rapide)
        → petite erreur : λ ≈ λ_min (approche précise, sans overshooting)

        α = ln(2)/0.05 → λ(50mm) ≈ (λ_max+λ_min)/2 (point d'inflexion)
        """
        if not self.adaptive:
            return self._gain
        alpha = np.log(2.0) / 0.010   # point d'inflexion à 10 mm (précis sous 10mm, rapide au-dessus)
        lam = (self.gain_max - self.gain_min) * (1.0 - np.exp(-alpha * error_norm)) + self.gain_min
        return float(np.clip(lam, self.gain_min, self.gain_max))

    # -------------------------------------------------------------------------
    def _joint_limit_penalty(self, q: np.ndarray) -> np.ndarray:
        """
        Gradient de la fonction de coût de répulsion des butées articulaires
        (Liegeois, 1977). Utilisé comme tâche secondaire via projection
        dans l'espace nul de J.

            H(q) = Σ (q_i - q_mid_i)² / (q_max_i - q_min_i)²
            ∂H/∂q_i = 2(q_i - q_mid_i) / (q_max_i - q_min_i)²

        Retourne ∂H/∂q (4,).
        """
        q_mid  = (self.params.q_max + self.params.q_min) / 2
        q_range = self.params.q_max - self.params.q_min
        return 2 * (q - q_mid) / (q_range ** 2)

    # -------------------------------------------------------------------------
    def update(
        self,
        error:     VisualError,
        q_current: np.ndarray,
        dt:        float = 0.033,
        w0:        float = 0.5,       # poids de la tâche secondaire (butées)
    ) -> VSCommand:
        """
        Calcule la commande articulaire pour un pas d'asservissement.

        Paramètres
        ----------
        error     : VisualError (issu de visual_error.compute_error())
        q_current : configuration articulaire courante [θ1, d2, θ3, θ4]
        dt        : période d'échantillonnage (s) — pour vérification sécurité
        w0        : gain tâche secondaire (butées articulaires)

        Retourne
        --------
        VSCommand
        """
        e = error.e                   # (6,)
        e_norm = float(np.linalg.norm(e))

        # Gain adaptatif
        lam = self._adaptive_gain(e_norm)

        # Jacobienne du SCARA à la configuration courante
        J = scara_jacobian(q_current, self.params)

        # Pseudo-inverse amortie (robuste aux singularités)
        J_pinv, sigma_min, is_singular = damped_pinv(
            J, sigma_min_threshold=self.singular_thresh)

        if is_singular:
            warnings.warn(
                f"Singularité détectée (σ_min={sigma_min:.4f} < {self.singular_thresh}). "
                "Amortissement DLS activé.", RuntimeWarning, stacklevel=2)

        # --- Vitesse cartésienne : v_c = -λ · L_s⁺ · e ---
        L_pinv = error.L_s_pinv
        v_c    = -lam * (L_pinv @ e)    # (6,)

        # Limiter la norme de v_c (sécurité)
        v_norm = np.linalg.norm(v_c[:3])
        if v_norm > self.v_max_m_s:
            v_c = v_c * (self.v_max_m_s / v_norm)

        # --- Conversion en vitesses articulaires ---
        # q̇_principal = J⁺ · v_c
        dq_principal = J_pinv @ v_c     # (4,)

        # --- Tâche secondaire : évitement des butées ---
        # q̇_sec = (I - J⁺ J) · (-w0 · ∂H/∂q)   (projection dans l'espace nul de J)
        grad_H    = self._joint_limit_penalty(q_current)
        null_proj = np.eye(4) - J_pinv @ J    # projecteur dans ker(J)
        dq_sec    = null_proj @ (-w0 * grad_H)

        dq_total = dq_principal + dq_sec

        # --- Saturation articulaire ---
        dq_sat    = np.clip(dq_total, -self.params.dq_max, self.params.dq_max)
        saturated = bool(np.any(np.abs(dq_total) > self.params.dq_max))

        # --- Enregistrement historique ---
        self._history.append({
            "e_norm":    e_norm,
            "norm_t_mm": error.norm_t_mm,
            "norm_r_deg":error.norm_r_deg,
            "gain":      lam,
            "singular":  is_singular,
            "sigma_min": sigma_min,
            "dq":        dq_sat.copy(),
            "v_c":       v_c.copy(),
        })

        return VSCommand(
            dq=dq_sat,
            v_c=v_c,
            singular=is_singular,
            saturated=saturated,
            gain=lam,
            sigma_min=sigma_min,
        )

    # -------------------------------------------------------------------------
    def plot_history(self, show: bool = True):
        """Trace l'évolution des normes d'erreur et du gain au fil des itérations."""
        if not self._history:
            print("Aucun historique disponible.")
            return

        import matplotlib.pyplot as plt

        iters     = range(len(self._history))
        norm_t    = [h["norm_t_mm"]  for h in self._history]
        norm_r    = [h["norm_r_deg"] for h in self._history]
        gains     = [h["gain"]       for h in self._history]
        sigma_min = [h["sigma_min"]  for h in self._history]

        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        fig.suptitle("Asservissement Visuel PBVS — SCARA 4-DOF", fontweight="bold")

        axes[0].plot(iters, norm_t, "b-o", ms=3, label="|eₜ| (mm)")
        axes[0].axhline(2.0, ls="--", color="r", alpha=0.5, label="Seuil 2 mm")
        axes[0].set_ylabel("Erreur translation (mm)")
        axes[0].legend(fontsize=8); axes[0].grid(True)

        axes[1].plot(iters, norm_r, "g-o", ms=3, label="|eᵣ| (°)")
        axes[1].axhline(1.0, ls="--", color="r", alpha=0.5, label="Seuil 1°")
        axes[1].set_ylabel("Erreur rotation (°)")
        axes[1].legend(fontsize=8); axes[1].grid(True)

        axes[2].plot(iters, gains,     "k-",  label="Gain λ")
        axes[2].plot(iters, sigma_min, "m--", label="σ_min Jacobienne")
        axes[2].set_ylabel("Gain / σ_min")
        axes[2].set_xlabel("Itération")
        axes[2].legend(fontsize=8); axes[2].grid(True)

        plt.tight_layout()
        if show:
            plt.show()
        return fig


# ─────────────────────────────────────────────────────────────────────────────
# Simulation autonome (test sans Phase 2)
# ─────────────────────────────────────────────────────────────────────────────

def simulate_pbvs(
    q0:         np.ndarray,
    t_desired:  np.ndarray,
    R_desired:  np.ndarray,
    dt:         float = 0.033,
    max_iter:   int   = 500,
    gain:       float = 0.5,
    adaptive:   bool  = True,
    verbose:    bool  = False,
):
    """
    Simulation de la boucle d'asservissement visuel PBVS sur le modèle SCARA.

    Le modèle direct est utilisé pour calculer la pose courante depuis q.
    Pas de dynamique — intégration Euler sur les vitesses articulaires.

    Paramètres
    ----------
    q0         : configuration initiale [θ1, d2, θ3, θ4]
    t_desired  : position désirée dans repère robot (m)
    R_desired  : orientation désirée (3×3)
    dt         : pas de temps (s)
    max_iter   : nombre maximal d'itérations
    gain       : λ nominal
    adaptive   : activer le gain adaptatif
    verbose    : afficher l'état à chaque itération

    Retourne
    --------
    history : list[dict] avec clés ['q', 't', 'norm_t_mm', 'norm_r_deg', 'gain']
    converged : bool
    """
    from visual_error import compute_error

    params = ScaraParams()
    ctrl   = VSController(params, gain=gain, adaptive=adaptive)

    # FK simplifiée pour la simulation
    def scara_fk(q):
        t1, d2, t3, t4 = q
        a2, a3, d3, d4 = params.a2, params.a3, params.d3, params.d4
        px = a2 * np.cos(t1) + a3 * np.cos(t1 + t3)
        py = a2 * np.sin(t1) + a3 * np.sin(t1 + t3)
        pz = d2 - d3 - d4
        # Rotation pure autour Z
        phi = t1 + t3 + t4
        R = np.array([[np.cos(phi), -np.sin(phi), 0],
                      [np.sin(phi),  np.cos(phi), 0],
                      [0,            0,           1]])
        return px, py, pz, R

    q = q0.copy()
    history = []
    converged = False

    for i in range(max_iter):
        px, py, pz, R_cur = scara_fk(q)
        t_cur = np.array([px, py, pz])

        err = compute_error(t_cur, R_cur, t_desired, R_desired)

        if verbose and (i % 20 == 0 or err.converged):
            print(f"  iter={i:4d} | {err} | gain={ctrl._adaptive_gain(np.linalg.norm(err.e)):.3f}")

        if err.converged:
            converged = True
            history.append({"q": q.copy(), "t": t_cur, "err": err,
                             "norm_t_mm": err.norm_t_mm, "norm_r_deg": err.norm_r_deg,
                             "gain": gain})
            if verbose:
                print(f"\n✓ Convergence à l'itération {i}")
            break

        cmd = ctrl.update(err, q, dt=dt)
        q   = q + cmd.dq * dt
        q   = np.clip(q, params.q_min, params.q_max)

        history.append({"q": q.copy(), "t": t_cur, "err": err,
                        "norm_t_mm": err.norm_t_mm, "norm_r_deg": err.norm_r_deg,
                        "gain": cmd.gain})

    return history, converged, ctrl


# ─────────────────────────────────────────────────────────────────────────────
# Test rapide
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys

    print("=== Test vs_controller.py — Simulation PBVS SCARA ===\n")

    params = ScaraParams()

    # --- Test Jacobienne ---
    q_test = np.array([0.3, 0.1, 0.2, -0.5])
    J = scara_jacobian(q_test, params)
    print(f"J(q_test) shape : {J.shape}")
    print(f"J =\n{np.round(J, 4)}")
    J_pinv, sigma, sing = damped_pinv(J)
    print(f"σ_min = {sigma:.4f}, singulier = {sing}\n")

    # --- Simulation boucle fermée ---
    # Configuration initiale (hors position désirée)
    q0 = np.array([0.5, 0.10, -0.3, 0.2])

    # Pose désirée : saisie au-dessus d'un objet
    # r = sqrt(350^2 + 50^2) ≈ 354 mm  → dans l'anneau [340, 460] mm ✓
    # d2 = pz + d3 + d4 = -0.150 + 0.150 + 0.059 = 0.059 m  ✓
    t_des = np.array([0.350, 0.050, -0.150])
    R_des = np.eye(3)   # orientation nulle (flat top-down)

    print("Simulation boucle fermée PBVS (λ adaptatif) :")
    history, converged, ctrl = simulate_pbvs(
        q0=q0, t_desired=t_des, R_desired=R_des,
        dt=0.033, max_iter=500, gain=0.5,
        adaptive=True, verbose=True
    )

    print(f"\nConvergé : {'✓ OUI' if converged else '✗ NON'} en {len(history)} itérations")
    if history:
        final = history[-1]
        print(f"Position finale : {final['t'] * 1000} mm")
        print(f"Erreur finale   : {final['norm_t_mm']:.3f} mm / {final['norm_r_deg']:.3f}°")
        print(f"Config finale q : {np.degrees(np.array([history[-1]['q'][0], 0, history[-1]['q'][2], history[-1]['q'][3]]))} ° et d2={history[-1]['q'][1]*1000:.1f} mm")

    # Tracer la convergence (optionnel si matplotlib dispo)
    try:
        import matplotlib
        matplotlib.use("Agg")  # pas de fenêtre en mode test
        fig = ctrl.plot_history(show=False)
        import os
        os.makedirs("test_images", exist_ok=True)
        fig.savefig("test_images/vs_convergence.png", dpi=100)
        print("\nGraphique sauvegardé : test_images/vs_convergence.png")
    except ImportError:
        pass

    print("\n✓ Test terminé.")
