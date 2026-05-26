"""
visual_error.py
Calcul de l'erreur visuelle et de la matrice d'interaction (jacobienne visuelle)
pour l'asservissement visuel de type PBVS (Position-Based Visual Servoing).

Stratégie PBVS
--------------
La primitive visuelle est la pose 6D de l'objet cible dans le repère robot.
L'erreur est définie dans l'espace 3D :

    e = [e_t ; e_r]   ∈ R^6

avec :
    e_t = t_courant - t_desiree          (erreur de position,  3 composantes, mètres)
    e_r = θ·u                             (erreur de rotation, représentation axe-angle, rad)

La matrice d'interaction L_s ∈ R^(6×6) relie la variation de la primitive
à la vitesse du repère effecteur :

    ė = L_s · v_c

Pour PBVS en espace cartésien, L_s ≈ I₆  (simplification valide localement).
L'implémentation plus précise utilise la matrice de transformation adjointe.

Loi de commande (intégrateur) :
    v_c = -λ · L_s⁺ · e

Références :
    Chaumette, F. & Hutchinson, S. — "Visual Servo Control" (IEEE R&A Magazine, 2006)
    Siciliano & Villani — "Robot Force Control", cap. 5

Modules :
    VisualError     : dataclass résultat du calcul
    compute_error() : calcule e et L_s depuis deux Pose6D
    axis_angle()    : représentation axe-angle d'une rotation
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# Types
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class VisualError:
    """
    Contient l'erreur visuelle e ∈ R^6 et la matrice d'interaction L_s ∈ R^(6×6).

    Attributs
    ---------
    e_t         : np.ndarray (3,)    — erreur de translation (mètres)
    e_r         : np.ndarray (3,)    — erreur de rotation θ·u (rad)
    e           : np.ndarray (6,)    — erreur complète [e_t ; e_r]
    L_s         : np.ndarray (6, 6)  — matrice d'interaction
    L_s_pinv    : np.ndarray (6, 6)  — pseudo-inverse de L_s
    norm_t_mm   : float              — norme de l'erreur de position (mm)
    norm_r_deg  : float              — norme de l'erreur de rotation (degrés)
    converged   : bool               — critère de convergence atteint
    """
    e_t:        np.ndarray
    e_r:        np.ndarray
    L_s:        np.ndarray
    thr_t_mm:   float = 2.0    # seuil translation (mm)
    thr_r_deg:  float = 1.0    # seuil rotation (degrés)

    @property
    def e(self) -> np.ndarray:
        return np.concatenate([self.e_t, self.e_r])

    @property
    def L_s_pinv(self) -> np.ndarray:
        return np.linalg.pinv(self.L_s)

    @property
    def norm_t_mm(self) -> float:
        return float(np.linalg.norm(self.e_t) * 1000)

    @property
    def norm_r_deg(self) -> float:
        return float(np.degrees(np.linalg.norm(self.e_r)))

    @property
    def converged(self) -> bool:
        return (self.norm_t_mm < self.thr_t_mm and
                self.norm_r_deg < self.thr_r_deg)

    def __repr__(self) -> str:
        return (f"VisualError(|et|={self.norm_t_mm:.2f} mm, "
                f"|er|={self.norm_r_deg:.2f}°, "
                f"{'CONVERGÉ' if self.converged else '...'} )")


# ─────────────────────────────────────────────────────────────────────────────
# Fonctions utilitaires
# ─────────────────────────────────────────────────────────────────────────────

def axis_angle(R: np.ndarray) -> np.ndarray:
    """
    Représentation axe-angle θ·u d'une matrice de rotation R (3×3).

    Retourne θ·u ∈ R^3 (norme = θ en radians).
    Utilise la formule de Rodrigues inverse :
        θ = arccos((tr(R) - 1) / 2)
        u = (1/(2 sin θ)) [R32-R23, R13-R31, R21-R12]

    Cas limites :
        θ ≈ 0 → u·θ ≈ 0
        θ ≈ π → formule de Shepperd
    """
    trace = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(trace)

    if theta < 1e-8:
        return np.zeros(3)

    if abs(theta - np.pi) < 1e-6:
        # θ ≈ π : extraire l'axe depuis la diagonale
        diag = np.array([R[0, 0], R[1, 1], R[2, 2]])
        u = np.sqrt(np.clip((diag + 1) / 2, 0, 1))
        # Signe depuis les éléments hors diagonaux
        if R[2, 1] - R[1, 2] < 0: u[0] = -u[0]
        if R[0, 2] - R[2, 0] < 0: u[1] = -u[1]
        if R[1, 0] - R[0, 1] < 0: u[2] = -u[2]
        return theta * u / (np.linalg.norm(u) + 1e-12)

    skew = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]])
    u = skew / (2 * np.sin(theta))
    return theta * u


def interaction_matrix_pbvs(R_co: np.ndarray,
                             t_co: np.ndarray,
                             R_cd: np.ndarray) -> np.ndarray:
    """
    Matrice d'interaction L_s pour PBVS (Chaumette 2006, eq. 7).

    Exprimée dans le repère objet courant :
        L_s = [ I₃      [t_co]×  ]
              [ 0₃      L_ω      ]

    où L_ω = I₃ - (θ/2)·[u]× + (1 - sinc(θ)/sinc²(θ/2)) [u]×²
             ≈ I₃  pour de petites erreurs

    Paramètres
    ----------
    R_co : rotation courant → désiré  R_cd = R_c_desired @ R_current.T
    t_co : translation d'erreur (repère robot)
    R_cd : rotation désiré dans repère caméra (non utilisée ici, pour extension future)

    Pour PBVS simplifié (erreurs faibles) on renvoie L_s = I₆,
    ce qui est équivalent à un contrôleur proportionnel en espace cartésien.
    L'implémentation complète est fournie ci-dessous.
    """
    theta_u = axis_angle(R_co)
    theta   = np.linalg.norm(theta_u)

    if theta < 1e-9:
        u = np.zeros(3)
    else:
        u = theta_u / theta

    # Matrice anti-symétrique de u
    def skew(v):
        return np.array([[ 0,    -v[2],  v[1]],
                         [ v[2],  0,    -v[0]],
                         [-v[1],  v[0],  0   ]])

    # L_ω (formule exacte)
    su = skew(u)
    sinc_theta    = np.sinc(theta / np.pi)        # sin(θ)/θ normalisé
    sinc2_theta2  = np.sinc(theta / (2 * np.pi))  # sin(θ/2)/(θ/2)

    if abs(sinc2_theta2) < 1e-12:
        coeff = 0.0
    else:
        coeff = 1.0 - sinc_theta / (sinc2_theta2 ** 2)

    L_omega = np.eye(3) - (theta / 2.0) * su + coeff * (su @ su)

    # Matrice anti-symétrique de t_co (pour le bloc translation)
    st = skew(t_co)

    # PBVS : L_s est bloc-diagonale — PAS de couplage translation/rotation
    # (contrairement à IBVS où la jacobienne image contient ce terme)
    L = np.zeros((6, 6))
    L[:3, :3] = np.eye(3)
    L[3:, 3:] = L_omega

    return L


# ─────────────────────────────────────────────────────────────────────────────
# Interface principale
# ─────────────────────────────────────────────────────────────────────────────

def compute_error(
    t_current:  np.ndarray,
    R_current:  np.ndarray,
    t_desired:  np.ndarray,
    R_desired:  np.ndarray,
    thr_t_mm:   float = 2.0,
    thr_r_deg:  float = 1.0,
) -> VisualError:
    """
    Calcule l'erreur visuelle PBVS entre pose courante et pose désirée.

    Les deux poses sont exprimées dans le repère robot.

    Paramètres
    ----------
    t_current  : position courante de l'objet (m)
    R_current  : orientation courante (3×3)
    t_desired  : position désirée / prédite par Kalman (m)
    R_desired  : orientation désirée (3×3), souvent R_desired = I pour saisie top-down

    Retourne
    --------
    VisualError
    """
    # Erreur de translation (dans repère robot)
    e_t = t_current - t_desired

    # Erreur de rotation : R_co = R_desired.T @ R_current
    R_co = R_desired.T @ R_current
    e_r  = axis_angle(R_co)

    L_s = interaction_matrix_pbvs(R_co, e_t, R_desired)

    return VisualError(e_t=e_t, e_r=e_r, L_s=L_s,
                       thr_t_mm=thr_t_mm, thr_r_deg=thr_r_deg)


# ─────────────────────────────────────────────────────────────────────────────
# Test unitaire rapide
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys

    print("=== Test visual_error.py ===\n")

    # Pose cible : objet à (300, 100, 200) mm, orienté comme le robot (I₃)
    t_des = np.array([0.300, 0.100, 0.200])
    R_des = np.eye(3)

    # Pose courante : légèrement décalée
    t_cur = np.array([0.312, 0.093, 0.198])
    R_cur = np.array([
        [ np.cos(0.05), -np.sin(0.05), 0],
        [ np.sin(0.05),  np.cos(0.05), 0],
        [0,              0,             1],
    ])

    err = compute_error(t_cur, R_cur, t_des, R_des)
    print(f"Erreur  : {err}")
    print(f"e_t (mm): {err.e_t * 1000}")
    print(f"e_r (°) : {np.degrees(err.e_r)}")
    print(f"\nL_s =\n{np.round(err.L_s, 4)}")
    print(f"\nL_s⁺ =\n{np.round(err.L_s_pinv, 4)}")

    # Test axe-angle
    print("\n--- Test axis_angle ---")
    R_test = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)  # 90° autour Z
    au = axis_angle(R_test)
    print(f"R(90°, Z) → θ·u = {np.degrees(au)} ° (attendu [0, 0, 90])")
    assert abs(np.degrees(np.linalg.norm(au)) - 90) < 0.01

    # Test cas θ ≈ 0
    au0 = axis_angle(np.eye(3))
    assert np.linalg.norm(au0) < 1e-8
    print(f"R(0°)    → θ·u = {au0} (attendu [0,0,0]) ✓")

    print("\n✓ Tous les tests passés.")
