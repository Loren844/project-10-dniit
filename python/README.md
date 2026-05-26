# Python — Vision & Asservissement Visuel SCARA

Ce dossier contient tout le code Python du projet, découpé en deux phases.

```
python/
├── phase2/    ← Reconnaissance d'objets et estimation de pose
└── phase3/    ← Contrôleur d'asservissement visuel PBVS + simulation
```

---

## Phase 2 — Reconnaissance d'objets et estimation de pose

### Objectif

Détecter les objets sur le tapis roulant depuis une caméra fixe, estimer leur pose 6D (position + orientation) dans le repère robot, et prédire leur position future pour compenser la latence du pipeline.

### Lancer

```bash
cd python/phase2
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Générer une planche de marqueurs ArUco (à imprimer ou afficher)
python generate_markers.py

# Test sur image statique
python main_phase2.py --image test_images/test_aruco_scene.jpg --method aruco

# Webcam en temps réel (ArUco)
python main_phase2.py --live --cam 1 --method aruco --size 0.08

# YOLO sur webcam
python main_phase2.py --live --cam 1 --method yolo --model yolov8n.pt

# RealSense D435 (RGB-D, calibration auto depuis firmware)
python main_phase2.py --realsense --method aruco --size 0.05

# Calibration caméra (damier physique requis, inutile avec RealSense)
python camera_calibration.py --live --rows 6 --cols 9 --size 25

# Lister les caméras disponibles
python main_phase2.py --list-cams
```

### Fichiers

#### `requirements.txt`
Dépendances Python : `numpy`, `opencv-python`, `scipy`.  
Optionnel : `ultralytics` (YOLO), `pyrealsense2` (RealSense D435).

#### `camera_calibration.py`
Calibration intrinsèque par damier (chessboard).  
Génère `calibration_data/camera_params.npz` contenant la matrice K (3×3) et les coefficients de distorsion D.  
RMS < 0.5 px = excellente qualité.  
Inutile avec une RealSense D435 (paramètres lus depuis le firmware).

```python
load_calibration("calibration_data/camera_params.npz")  # → dict {K, dist}
```

#### `detect_objects.py`
Deux méthodes de détection, toutes deux retournent des objets `Detection` normalisés :

- **ArUco** — marqueurs imprimés, temps de traitement < 5 ms, pas besoin d'apprentissage. Fournit 4 coins exacts pour solvePnP.
- **YOLO (YOLOv8)** — détection d'objets naturels, score de confiance, bounding box pixels.

```python
Detection.method        # 'aruco' | 'yolo'
Detection.bbox          # (x, y, w, h) pixels
Detection.corners       # 4×2 (ArUco seulement)
Detection.center_px     # (cx, cy)
```

#### `pose_estimation.py`
Estimation de la pose 6D dans le repère caméra.

- `estimate_pose_aruco(detections, K, D, marker_size_m)` — solvePnP exact sur les 4 coins
- `estimate_pose_yolo_rgbd(detections, K, D, depth_frame)` — déprojection 3D depuis carte de profondeur RealSense
- `estimate_pose_yolo_flat(detections, K, D, z_plane_m)` — reprojection sur plan horizontal à hauteur connue

```python
Pose6D.t_cam       # position (x, y, z) en mètres, repère caméra
Pose6D.R_cam       # matrice rotation 3×3
Pose6D.T_cam       # matrice homogène 4×4
Pose6D.euler_deg   # angles Euler ZYX en degrés
```

#### `robot_transform.py`
Transforme une `Pose6D` du repère caméra vers le repère robot via la matrice extrinsèque T (4×4).

- `RobotTransform.transform(pose_cam)` → `Pose6D` dans le repère robot
- `hand_eye_calibrate(R_list, t_list, R_gripper_list, t_gripper_list)` — calibration main-œil
- `save_transform()` / `load_transform()` — sauvegarde dans `.npz`

La matrice T est déterminée une seule fois lors de l'installation de la caméra (mesure géométrique ou calibration main-œil).

#### `kalman_tracker.py`
Filtre de Kalman à modèle vitesse constante (état 6D : position + vitesse).

**Pourquoi ?** Le pipeline vision prend ~150 ms. Sans prédiction, le robot vise l'ancienne position de l'objet. Le Kalman prédit où sera l'objet au moment où la commande sera exécutée.

- `KalmanTracker(dt, sigma_accel, sigma_pos)` — tracker mono-objet
- `tracker.update(pos_3d)` — mise à jour avec nouvelle mesure
- `tracker.predict_at(latency_s)` → position prédite à t + latency
- `MultiObjectTracker` — association automatique par plus proche voisin

#### `realsense_capture.py`
Wrapper autour de `pyrealsense2` :

- `RealSenseCapture` — flux RGB + profondeur alignés, intrinsèques firmware
- `MockRealSense` — émulation webcam (profondeur constante, pour tests sans hardware)
- `auto_camera()` — sélectionne automatiquement RealSense si connectée, sinon webcam

#### `generate_markers.py`
Génère une planche PDF/image de marqueurs ArUco (dictionnaire DICT_4X4_50) à imprimer.

#### `main_phase2.py`
Orchestre le pipeline complet en temps réel avec affichage OpenCV :
1. Capture (webcam / RealSense)
2. Détection (ArUco ou YOLO)
3. Estimation de pose
4. Transformation → repère robot
5. Mise à jour Kalman
6. Affichage HUD (pose, axes 3D, bounding box)

### Flux de données

```
Caméra
  ├── color frame (BGR)   → detect_objects → pose_estimation
  └── depth frame (mm)    ↗ (RealSense uniquement)
                              ↓
                        robot_transform → kalman_tracker → t_prédit [m]
                                                               ↓
                                                         Phase 3 : VS
```

### Résultats attendus

| Métrique | Valeur cible |
|---|---|
| Détection ArUco | 100 % sur marqueurs visibles, < 5 ms |
| Erreur solvePnP | < 1 mm (avec calibration réelle) |
| Erreur Kalman à +150 ms | < 5 mm pour v < 200 mm/s |

---

## Phase 3 — Asservissement visuel PBVS

### Objectif

Guider le robot SCARA de sa position courante vers l'objet détecté, puis exécuter le cycle pick-and-place complet (approche → saisie → levée → transport → dépose → retour). La loi de commande est un asservissement visuel PBVS (Position-Based Visual Servoing).

### Lancer

```bash
cd python/phase3
source ../phase2/.venv/bin/activate   # Phase 3 réutilise le venv Phase 2

# Simulation 2D interactive (matplotlib) — pas de hardware requis
python simulation_gui.py

# Pipeline complet avec cible fixe (bypass caméra)
python main_phase3.py --live --cam 1 --force-target 350 50 -150   # x y z en mm

# Cible nécessitant pré-approche (derrière le robot)
python main_phase3.py --live --cam 1 --force-target -300 200 -150

# Pipeline caméra réel (ArUco)
python main_phase3.py --live --cam 1 --method aruco --size 0.08

# RealSense D435
python main_phase3.py --realsense --method aruco --size 0.05

# Simulation boucle fermée sans caméra
python main_phase3.py --sim
```

### Contrôles de la GUI (simulation_gui.py)

| Touche / Action | Effet |
|---|---|
| Clic gauche | Placer l'objet (zone verte = workspace accessible par IK) |
| Clic droit | Déplacer la zone de dépose |
| ESPACE | Démarrer / pauser |
| R | Réinitialiser |

### Loi de commande PBVS

L'erreur visuelle est définie dans l'espace 3D (repère robot) :

```
e = [t_courant − t_désiré ;  θ·u]  ∈ R⁶
                  ↑ position     ↑ axe-angle

v_c = −λ(‖e‖) · Ls⁺ · e       (vitesse cartésienne)
q̇   = J⁺(q)  · v_c             (vitesses articulaires)
```

- **λ adaptatif** : sigmoïde ∈ [0.05, 1.5], augmente quand l'erreur est grande, ralentit pour le positionnement fin
- **J⁺** : pseudo-inverse DLS (amortie), robuste aux singularités
- **Tâche secondaire** : évitement des butées articulaires (méthode Liegeois, poids 0.1)

### Machine à états principale (PipelineState)

```
SEARCHING → PRE_APPROACH → TRACKING → APPROACH → CONVERGED
                ↑ (saut cible > 50 mm)               ↓
                └──────────────────────────────── EMERGENCY
```

| État | Déclencheur | Action |
|---|---|---|
| `SEARCHING` | Aucun objet détecté | Robot immobile |
| `PRE_APPROACH` | Erreur θ1 ou θ3 > 8° | Interpolation joint-space vers solution IK |
| `TRACKING` | Objet détecté, ‖e‖ > 50 mm | VS actif, λ = 1.5 |
| `APPROACH` | ‖e‖ < 50 mm | VS fin, λ = 1.0 |
| `CONVERGED` | ‖e_t‖ < 2 mm et ‖e_r‖ < 1° | Séquenceur pince activé |
| `EMERGENCY` | σ_min(J) < seuil | Arrêt d'urgence |

### Séquenceur pick-and-place (PickPlaceState)

```
IDLE → APPROACH → GRASPING → LIFTING → TRANSPORT → LOWERING → RELEASING → RETURNING → DONE
```

Chaque transition TRANSPORT et RETURNING déclenche une nouvelle `PRE_APPROACH` si la cible change de plus de 50 mm (ex. passage pick→dépose, dépose→home).

### Configuration non-singulière

Le robot démarre à `q₀ = [0°, 100 mm, −45°, 0°]`.  
θ3 = 0° est singulier (σ_min ≈ 0, r = 460 mm) et est systématiquement évité par la pré-approche.

### Fichiers

#### `visual_error.py`
Calcul de l'erreur visuelle et de la matrice d'interaction.

- `compute_error(t_cur, R_cur, t_des, R_des, params)` → `VisualError`
- `axis_angle(R)` — représentation θ·u depuis une matrice de rotation (Rodrigues inverse)
- `interaction_matrix_pbvs(R_co, t_co, R_cd)` — Ls bloc-diagonale exacte (Chaumette 2006)

```python
VisualError.e           # vecteur erreur (6,)
VisualError.norm_t_mm   # norme erreur position en mm
VisualError.converged   # bool : ‖e_t‖ < 2 mm et ‖e_r‖ < 1°
```

#### `vs_controller.py`
Contrôleur PBVS + cinématique partagée entre tous les modules.

- `ScaraParams` — paramètres géométriques et limites articulaires (miroir de `robot_parameters.m`)
  - a2=300 mm, a3=160 mm, d3=150 mm, d4=59 mm
  - q_min=[-2.356, 0, -1.571, -3.142], q_max=[2.356, 0.200, 1.571, 3.142]
  - dq_max=[2.0 rad/s, 0.1 m/s, 2.0 rad/s, 2.0 rad/s]
- `ik_solutions(x_m, y_m, params)` — IK analytique, retourne toutes les paires (θ1, θ3) valides
- `scara_jacobian(q, params)` — jacobienne analytique J ∈ R^(6×4)
- `damped_pinv(J)` — pseudo-inverse DLS, retourne σ_min et flag singularité
- `VSController.update(error, q, dt)` → `VSCommand` (dq, v_c, singular, saturated)

#### `gripper_controller.py`
Séquenceur de la tâche pick-and-place.

- `GripperController` — modèle pince (OPEN / CLOSING / CLOSED / OPENING), timers d'ouverture/fermeture
- `PickPlaceSequencer(drop_pos_m)` — machine à 9 états
  - `.update(vs_converged, object_pos_m, t_ee_m, q_current)` → (state, vs_target, gripper_close)
  - `._target_pos` — cible VS courante (change à chaque transition)
- `draw_hud(frame)` — overlay d'état en BGR sur frame caméra

#### `simulation_gui.py`
Simulation 2D complète du cycle pick-and-place (matplotlib, sans hardware).

- **Vue XY** — projection horizontale : bras + workspace pixel-exact (masque IK vectorisé sur grille 320×320)
- **Vue XZ** — profil de hauteur (levée, transport, abaissement)
- **Panneau état** — machine à états + barre d'erreur |e_t| en temps réel
- Pré-approche intégrée : déclenchée au clic ET à chaque changement de cible du séquenceur
- L'objet animé suit le bras pendant GRASPING / LIFTING / TRANSPORT / LOWERING

#### `main_phase3.py`
Pipeline complet Phase 2 + Phase 3, mode temps réel ou simulation.

- `Phase3Pipeline(cam_params, robot_tf, force_target)` — machine à états globale
  - `.process_frame(frame)` → (annotated_frame, VSCommand, PipelineState)
  - Deux chemins : `force_target` (bypass caméra) et pipeline caméra complet
  - `effective_target` calculé depuis `sequencer._target_pos` (pas l'objet brut)
  - VS s'exécute à chaque frame hors pré-approche (jamais gelé)
- CLI : `--force-target x y z`, `--method aruco|yolo`, `--realsense`, `--live`, `--sim`

### Flux de données Phase 3

```
[Phase 2] t_prédit (m)
       ↓
ik_solutions → PRE_APPROACH si θ1/θ3 loin > 8°
       ↓ (pré-approche terminée)
compute_error(t_cur, R_cur, t_effective, R_des) → e ∈ R⁶
       ↓
VSController.update(e, q) → dq (rad/s, m/s)
       ↓
PickPlaceSequencer.update → nouvelle cible → saut > 50 mm → PRE_APPROACH
       ↓
q ← q + dq × dt
```

### Résultats validés (simulation Python)

| Métrique | Valeur |
|---|---|
| Convergence depuis position éloignée | ≈ 30 frames après pré-approche |
| Erreur finale position | < 2 mm |
| Erreur finale orientation | < 1° |
| Workspace effectif | r ∈ [340, 460] mm, θ1 ∈ ±135° |
| Cycle pick-and-place complet | Validé en simulation GUI |
