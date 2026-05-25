"""
kalman_tracker.py
Filtre de Kalman pour prédire la position future d'un objet en mouvement
sur le tapis roulant, afin de compenser la latence du pipeline vision→robot.

Modèle d'état : vitesse constante (Constant Velocity)
    État  x = [px, py, pz, vx, vy, vz]ᵀ   (position + vitesse, repère robot)
    Mesure z = [px, py, pz]ᵀ               (position estimée par pose_estimation)

Équations du filtre :
    Prédiction  : x̂⁻ = F · x̂      P⁻ = F·P·Fᵀ + Q
    Mise à jour : K   = P⁻·Hᵀ · (H·P⁻·Hᵀ + R)⁻¹
                  x̂  = x̂⁻ + K·(z − H·x̂⁻)
                  P   = (I − K·H)·P⁻

Usage :
    from kalman_tracker import KalmanTracker, MultiObjectTracker

    # Tracker pour un seul objet
    tracker = KalmanTracker(dt=0.033)          # 30 fps
    tracker.update(np.array([0.25, 0.10, 0.0]))
    tracker.update(np.array([0.28, 0.10, 0.0]))

    # Prédire où sera l'objet dans 150 ms (latence pipeline)
    pos_future = tracker.predict_at(latency_s=0.150)

    # Tracker multi-objets (associe automatiquement les détections aux tracks)
    mot = MultiObjectTracker(dt=0.033, max_missed=5)
    mot.update([pose1, pose2])
    predictions = mot.predict_all(latency_s=0.150)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


# ---------------------------------------------------------------------------
# --- Filtre de Kalman (un seul objet) ---------------------------------------
# ---------------------------------------------------------------------------

class KalmanTracker:
    """
    Filtre de Kalman à modèle vitesse constante pour un objet 3D.

    Paramètres
    ----------
    dt          : pas de temps entre deux mesures (secondes)
    sigma_accel : écart-type du bruit de processus (accélération, m/s²)
                  Plus grand → le filtre suit mieux les changements de vitesse
                  Tapis roulant à vitesse constante : 0.5–2 m/s²
    sigma_pos   : écart-type du bruit de mesure (position, mètres)
                  Dépend de la précision de pose_estimation (~3–10 mm)
    """

    def __init__(
        self,
        dt: float = 0.033,
        sigma_accel: float = 1.0,
        sigma_pos:   float = 0.005,
    ):
        self.dt = dt
        self.initialized = False
        self.n_updates = 0

        # --- Matrice de transition d'état F (6×6) ---
        # Modèle : position ← position + vitesse × dt
        self.F = np.eye(6)
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt

        # --- Matrice d'observation H (3×6) ---
        # On mesure uniquement la position (px, py, pz)
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        # --- Bruit de processus Q (6×6) ---
        # Modèle d'accélération aléatoire (discrétisation exacte)
        dt2 = dt ** 2
        dt3 = dt ** 3
        dt4 = dt ** 4
        q = sigma_accel ** 2
        self.Q = q * np.array([
            [dt4/4, 0,     0,     dt3/2, 0,     0    ],
            [0,     dt4/4, 0,     0,     dt3/2, 0    ],
            [0,     0,     dt4/4, 0,     0,     dt3/2],
            [dt3/2, 0,     0,     dt2,   0,     0    ],
            [0,     dt3/2, 0,     0,     dt2,   0    ],
            [0,     0,     dt3/2, 0,     0,     dt2  ],
        ])

        # --- Bruit de mesure R (3×3) ---
        self.R = (sigma_pos ** 2) * np.eye(3)

        # --- État initial et covariance ---
        self.x = np.zeros(6)   # [px, py, pz, vx, vy, vz]
        self.P = np.eye(6) * 1.0   # incertitude initiale large

    # -----------------------------------------------------------------------
    def init(self, position: np.ndarray, velocity: np.ndarray = None):
        """Initialise le filtre avec une position (et optionnellement une vitesse)."""
        self.x[:3] = position
        self.x[3:] = velocity if velocity is not None else np.zeros(3)
        self.P = np.eye(6) * 1.0
        self.initialized = True
        self.n_updates = 1

    # -----------------------------------------------------------------------
    def predict(self) -> np.ndarray:
        """
        Étape de prédiction (sans nouvelle mesure).
        Appeler à chaque frame même quand l'objet n'est pas détecté.

        Retourne la position prédite [px, py, pz].
        """
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:3].copy()

    # -----------------------------------------------------------------------
    def update(self, measurement: np.ndarray) -> np.ndarray:
        """
        Étape de mise à jour avec une nouvelle mesure de position.

        Paramètres
        ----------
        measurement : np.ndarray (3,) — position mesurée [px, py, pz] en mètres

        Retourne la position estimée corrigée [px, py, pz].
        """
        if not self.initialized:
            self.init(measurement)
            return self.x[:3].copy()

        # --- Prédiction ---
        x_pred = self.F @ self.x
        P_pred = self.F @ self.P @ self.F.T + self.Q

        # --- Innovation ---
        z   = measurement.reshape(3, 1)
        y   = z - self.H @ x_pred.reshape(6, 1)          # résidu
        S   = self.H @ P_pred @ self.H.T + self.R         # covariance résidu
        K   = P_pred @ self.H.T @ np.linalg.inv(S)        # gain de Kalman

        # --- Mise à jour ---
        self.x = x_pred + (K @ y).ravel()
        self.P = (np.eye(6) - K @ self.H) @ P_pred
        self.n_updates += 1
        return self.x[:3].copy()

    # -----------------------------------------------------------------------
    def predict_at(self, latency_s: float) -> np.ndarray:
        """
        Prédit la position de l'objet dans `latency_s` secondes.

        Utilité : compenser la latence du pipeline (détection + calcul + mouvement
        robot). Si la latence est de 150 ms, le robot doit viser la position
        future et non la position actuelle.

        Paramètres
        ----------
        latency_s : latence totale en secondes à compenser

        Retourne
        --------
        position future [px, py, pz] en mètres
        """
        n_steps = max(1, round(latency_s / self.dt))

        # Propagation de l'état sans mise à jour (prédiction pure)
        F_n = np.linalg.matrix_power(self.F, n_steps)
        x_future = F_n @ self.x
        return x_future[:3].copy()

    # -----------------------------------------------------------------------
    @property
    def position(self) -> np.ndarray:
        """Position estimée courante [px, py, pz] en mètres."""
        return self.x[:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        """Vitesse estimée [vx, vy, vz] en m/s."""
        return self.x[3:].copy()

    @property
    def speed_mps(self) -> float:
        """Norme de la vitesse en m/s."""
        return float(np.linalg.norm(self.x[3:]))

    @property
    def position_uncertainty_mm(self) -> np.ndarray:
        """Écart-type de l'incertitude de position en mm (diagonale de P)."""
        return np.sqrt(np.diag(self.P)[:3]) * 1000


# ---------------------------------------------------------------------------
# --- Tracker multi-objets ---------------------------------------------------
# ---------------------------------------------------------------------------

@dataclass
class Track:
    """Un objet suivi individuellement."""
    track_id:   int
    label:      str
    tracker:    KalmanTracker
    missed:     int = 0            # Nombre de frames consécutives sans détection
    confirmed:  bool = False       # True après N_CONFIRM mises à jour

    N_CONFIRM = 3   # Mises à jour minimum avant de considérer un track fiable


@dataclass
class TrackedObject:
    """Résultat d'un track : position filtrée + prédiction."""
    track_id:       int
    label:          str
    position_m:     np.ndarray   # Position filtrée (3,)
    velocity_mps:   np.ndarray   # Vitesse estimée (3,)
    predicted_m:    np.ndarray   # Position future (3,) après latence
    uncertainty_mm: np.ndarray   # Incertitude (3,) en mm
    confirmed:      bool

    def __str__(self) -> str:
        p  = self.position_m * 1000
        v  = self.velocity_mps * 1000
        fp = self.predicted_m * 1000
        return (
            f"Track {self.track_id} [{self.label}]  "
            f"pos=({p[0]:+6.1f},{p[1]:+6.1f},{p[2]:+6.1f})mm  "
            f"vitesse={np.linalg.norm(v):.0f}mm/s  "
            f"→ prédit=({fp[0]:+6.1f},{fp[1]:+6.1f},{fp[2]:+6.1f})mm"
        )


class MultiObjectTracker:
    """
    Gère plusieurs tracks simultanément et associe les détections
    aux objets existants par plus proche voisin (distance euclidienne).

    Paramètres
    ----------
    dt              : pas de temps (s)
    sigma_accel     : bruit de processus (m/s²)
    sigma_pos       : bruit de mesure (m)
    max_missed      : frames sans détection avant suppression du track
    assoc_threshold : distance max (m) pour l'association détection→track
    latency_s       : latence pipeline à compenser pour la prédiction
    """

    def __init__(
        self,
        dt:              float = 0.033,
        sigma_accel:     float = 1.0,
        sigma_pos:       float = 0.005,
        max_missed:      int   = 5,
        assoc_threshold: float = 0.10,
        latency_s:       float = 0.150,
    ):
        self.dt              = dt
        self.sigma_accel     = sigma_accel
        self.sigma_pos       = sigma_pos
        self.max_missed      = max_missed
        self.assoc_threshold = assoc_threshold
        self.latency_s       = latency_s

        self._tracks:    dict[int, Track] = {}
        self._next_id:   int = 0

    # -----------------------------------------------------------------------
    def update(self, measurements: list[tuple[str, np.ndarray]]) -> list[TrackedObject]:
        """
        Met à jour le tracker avec les nouvelles détections.

        Paramètres
        ----------
        measurements : liste de (label, position_m)
                       position_m = np.ndarray (3,) en mètres (repère robot)

        Retourne
        --------
        liste de TrackedObject pour les tracks confirmés
        """
        # --- 1. Prédiction de tous les tracks existants ---
        for track in self._tracks.values():
            track.tracker.predict()
            track.missed += 1

        # --- 2. Association mesures ↔ tracks (plus proche voisin) ---
        assigned_track_ids   = set()
        assigned_meas_idxs   = set()

        if self._tracks and measurements:
            track_ids   = list(self._tracks.keys())
            track_preds = np.array([self._tracks[tid].tracker.position
                                    for tid in track_ids])   # (M, 3)
            meas_pos    = np.array([m[1] for m in measurements])   # (N, 3)

            # Matrice de distances (M tracks × N mesures)
            dist_matrix = np.linalg.norm(
                track_preds[:, None, :] - meas_pos[None, :, :], axis=2
            )   # (M, N)

            # Association gloutonne : paires (track, mesure) par distance croissante
            for _ in range(min(len(track_ids), len(measurements))):
                if dist_matrix.size == 0:
                    break
                i, j = np.unravel_index(dist_matrix.argmin(), dist_matrix.shape)
                if dist_matrix[i, j] > self.assoc_threshold:
                    break
                tid = track_ids[i]
                assigned_track_ids.add(tid)
                assigned_meas_idxs.add(j)

                label, pos = measurements[j]
                self._tracks[tid].tracker.update(pos)
                self._tracks[tid].missed   = 0
                self._tracks[tid].label    = label
                if self._tracks[tid].tracker.n_updates >= Track.N_CONFIRM:
                    self._tracks[tid].confirmed = True

                # Neutraliser cette ligne/colonne pour éviter double assignation
                dist_matrix[i, :] = np.inf
                dist_matrix[:, j] = np.inf

        # --- 3. Créer un nouveau track pour les mesures non assignées ---
        for j, (label, pos) in enumerate(measurements):
            if j not in assigned_meas_idxs:
                kf = KalmanTracker(self.dt, self.sigma_accel, self.sigma_pos)
                kf.init(pos)
                track = Track(track_id=self._next_id, label=label, tracker=kf)
                self._tracks[self._next_id] = track
                self._next_id += 1

        # --- 4. Supprimer les tracks perdus ---
        lost_ids = [tid for tid, t in self._tracks.items()
                    if t.missed > self.max_missed]
        for tid in lost_ids:
            del self._tracks[tid]

        # --- 5. Construire la liste de résultats ---
        results: list[TrackedObject] = []
        for track in self._tracks.values():
            if not track.confirmed:
                continue
            results.append(TrackedObject(
                track_id      = track.track_id,
                label         = track.label,
                position_m    = track.tracker.position,
                velocity_mps  = track.tracker.velocity,
                predicted_m   = track.tracker.predict_at(self.latency_s),
                uncertainty_mm= track.tracker.position_uncertainty_mm,
                confirmed      = track.confirmed,
            ))
        return results

    # -----------------------------------------------------------------------
    def predict_all(self, latency_s: Optional[float] = None) -> list[TrackedObject]:
        """Retourne les positions futures de tous les tracks confirmés."""
        if latency_s is None:
            latency_s = self.latency_s
        results = []
        for track in self._tracks.values():
            if not track.confirmed:
                continue
            results.append(TrackedObject(
                track_id      = track.track_id,
                label         = track.label,
                position_m    = track.tracker.position,
                velocity_mps  = track.tracker.velocity,
                predicted_m   = track.tracker.predict_at(latency_s),
                uncertainty_mm= track.tracker.position_uncertainty_mm,
                confirmed      = track.confirmed,
            ))
        return results

    @property
    def n_tracks(self) -> int:
        return len(self._tracks)

    @property
    def n_confirmed(self) -> int:
        return sum(1 for t in self._tracks.values() if t.confirmed)


# ---------------------------------------------------------------------------
# --- Démonstration ----------------------------------------------------------
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    print("=" * 60)
    print("  DÉMONSTRATION — Filtre de Kalman (tapis roulant)")
    print("=" * 60)

    # Simuler un objet qui se déplace sur le tapis roulant (axe X)
    # avec une vitesse réelle de 0.15 m/s et du bruit de mesure
    np.random.seed(42)
    dt          = 0.033       # 30 fps
    v_real      = 0.15        # m/s — vitesse tapis roulant
    sigma_noise = 0.006       # 6 mm de bruit de mesure
    latency_s   = 0.150       # 150 ms de latence pipeline

    N = 60   # 2 secondes
    t = np.arange(N) * dt

    # Positions réelles
    pos_real = np.column_stack([
        0.20 + v_real * t,
        0.10 * np.ones(N),
        np.zeros(N),
    ])

    # Mesures bruitées (avec 10 % de frames sans détection)
    tracker = KalmanTracker(dt=dt, sigma_accel=0.5, sigma_pos=sigma_noise)

    pos_filtered  = []
    pos_predicted = []
    velocities    = []

    for k in range(N):
        if np.random.rand() > 0.10:   # 90 % de détection
            noise = np.random.randn(3) * sigma_noise
            meas  = pos_real[k] + noise
            p_filtered = tracker.update(meas)
        else:
            p_filtered = tracker.predict()

        p_future = tracker.predict_at(latency_s)
        pos_filtered.append(p_filtered)
        pos_predicted.append(p_future)
        velocities.append(tracker.speed_mps)

    pos_filtered  = np.array(pos_filtered)
    pos_predicted = np.array(pos_predicted)

    # Rapport console
    print(f"\nVitesse réelle      : {v_real*1000:.0f} mm/s")
    print(f"Vitesse estimée     : {tracker.speed_mps*1000:.1f} mm/s  "
          f"({'OK' if abs(tracker.speed_mps - v_real) < 0.01 else 'diverge'})")

    err_filter  = np.linalg.norm(pos_filtered - pos_real, axis=1) * 1000
    err_predict = np.linalg.norm(pos_predicted - pos_real, axis=1) * 1000
    print(f"Erreur filtrée      : {err_filter.mean():.2f} mm ± {err_filter.std():.2f} mm")
    print(f"Erreur prédiction   : {err_predict.mean():.2f} mm ± {err_predict.std():.2f} mm")
    print(f"  (latence compensée = {latency_s*1000:.0f} ms)")
    print(f"Incertitude pos.    : {tracker.position_uncertainty_mm} mm (σ)")

    # Figures
    fig, axes = plt.subplots(2, 1, figsize=(10, 6))

    axes[0].plot(t, pos_real[:, 0]*1000,       'g-',  lw=2,  label='Réel')
    axes[0].plot(t, pos_filtered[:, 0]*1000,   'b-',  lw=1.5, label='Filtré (Kalman)')
    axes[0].plot(t, pos_predicted[:, 0]*1000,  'r--', lw=1.5,
                 label=f'Prédit +{latency_s*1000:.0f}ms')
    axes[0].set_xlabel('Temps (s)')
    axes[0].set_ylabel('Position X (mm)')
    axes[0].set_title('Suivi objet sur tapis roulant — Filtre de Kalman')
    axes[0].legend(); axes[0].grid(True)

    axes[1].plot(t, err_filter,  'b-',  lw=1.5, label=f'Erreur filtrée  (moy={err_filter.mean():.1f}mm)')
    axes[1].plot(t, err_predict, 'r--', lw=1.5, label=f'Erreur prédite  (moy={err_predict.mean():.1f}mm)')
    axes[1].axhline(sigma_noise*1000, color='k', ls=':', lw=1, label=f'Bruit mesure={sigma_noise*1000:.0f}mm')
    axes[1].set_xlabel('Temps (s)')
    axes[1].set_ylabel('Erreur de position (mm)')
    axes[1].set_title('Erreur de suivi')
    axes[1].legend(); axes[1].grid(True)

    plt.tight_layout()
    fig.savefig("kalman_demo.png", dpi=120)
    print("\nFigure sauvegardée : kalman_demo.png")
    print("=" * 60)
