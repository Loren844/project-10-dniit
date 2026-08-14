# Python — Vision & Asservissement Visuel PBVS pour robot SCARA

Ce dossier contient tout le code Python du projet, réparti en deux phases plus un dossier de tests PLC.

```
python/
├── phase2/    ← Détection d'objets et estimation de pose
├── phase3/    ← Contrôleur PBVS + simulation
└── tests/     ← Scripts de test bas-niveau (connexion PLC, axes, pince)
```

---

## Phase 2 — Détection d'objets et estimation de pose

### Objectif

Détecter des objets sur le tapis roulant depuis une caméra fixe, estimer leur pose 6D (position + orientation) dans le repère robot, et prédire leur position future pour compenser la latence du pipeline.

### Lancer

```bash
cd python/phase2
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Générer une feuille de marqueurs ArUco (à imprimer ou afficher)
python generate_markers.py

# Générer une image de test synthétique
python main_phase2.py --generate-test

# Test sur image statique
python main_phase2.py --image test_images/test_aruco_scene.jpg --method aruco

# Flux webcam (ArUco)
python main_phase2.py --live --method aruco --size 0.08

# YOLO sur webcam
python main_phase2.py --live --method yolo --model yolov8n.pt

# RealSense D435 (RGB-D, calibration automatique depuis le firmware)
python main_phase2.py --realsense --method aruco --size 0.05

# Calibration caméra (damier physique requis, inutile avec RealSense)
python camera_calibration.py --live --rows 6 --cols 9 --size 25

# Calibration main-œil interactive (marqueur ArUco sur l'effecteur)
python hand_eye_collect.py --cam 0 --marker-id 0 --marker-size 0.06

# Lister les caméras disponibles
python main_phase2.py --list-cams
```

### Fichiers

#### `requirements.txt`
Dépendances Python : `numpy`, `opencv-python`, `opencv-contrib-python`, `scipy`, `matplotlib`.
Optionnel : `ultralytics` (YOLO), `pyrealsense2` (RealSense D435).

#### `camera_calibration.py`
Calibration intrinsèque par damier.
Produit `calibration_data/camera_params.npz` contenant la matrice K (3×3), les coefficients de distorsion et le RMS de reprojection.
RMS < 0.5 px = excellente calibration.
Non nécessaire avec une RealSense D435 (paramètres lus depuis le firmware).

```python
load_calibration("calibration_data/camera_params.npz")  # → dict {K, dist, rms}
```

#### `hand_eye_collect.py`
Collecte interactive pour la calibration caméra→robot (calibration main-œil).

Protocole :
1. Fixer un marqueur ArUco sur l'effecteur.
2. Déplacer le robot dans 8–15 poses très différentes.
3. À chaque pose, entrer les angles PLC manuellement puis appuyer sur `ESPACE` pour capturer.
4. Appuyer sur `c` pour calculer et enregistrer `calibration_data/cam_to_robot.npz`.

#### `detect_objects.py`
Deux méthodes de détection, retournant des objets `Detection` normalisés :

- **ArUco** — marqueurs imprimés, < 5 ms, sans apprentissage. Fournit 4 coins exacts pour solvePnP.
- **YOLO (YOLOv8)** — détection d'objets quelconques, score de confiance, bounding box pixel.

```python
Detection.method        # 'aruco' | 'yolo'
Detection.bbox          # (x, y, w, h) pixels
Detection.corners       # 4×2 (ArUco uniquement)
Detection.center_px     # (cx, cy)
```

#### `pose_estimation.py`
Estimation de pose 6D (position + orientation) dans le repère caméra.

- `estimate_pose_aruco(detections, K, D, marker_size_m)` — solvePnP exact sur 4 coins
- `estimate_pose_yolo_rgbd(detections, K, D, depth_frame)` — position 3D depuis la carte de profondeur RealSense
- `estimate_pose_yolo_flat(detections, K, D, z_plane_m)` — rétroprojection sur plan horizontal connu

```python
Pose6D.t_cam       # position (x, y, z) en mètres, repère caméra
Pose6D.R_cam       # matrice de rotation 3×3
Pose6D.T_cam       # matrice homogène 4×4
Pose6D.euler_deg   # angles d'Euler ZYX en degrés
```

#### `robot_transform.py`
Transforme une `Pose6D` du repère caméra vers le repère robot via la matrice extrinsèque T (4×4).

- `RobotTransform.transform(pose_cam)` → `Pose6D` dans le repère robot
- `hand_eye_calibrate(R_list, t_list, R_gripper_list, t_gripper_list)` — calibration main-œil
- `save_transform()` / `load_transform()` — persistance en `.npz`

La matrice T est déterminée une fois lors de l'installation de la caméra (mesure géométrique ou calibration main-œil via `hand_eye_collect.py`).

#### `kalman_tracker.py`
Filtre de Kalman à vitesse constante (état 6D : position + vitesse).

**Pourquoi ?** Le pipeline vision prend ~150 ms. Sans prédiction, le robot cible l'ancienne position de l'objet. Le filtre de Kalman prédit où l'objet sera au moment où la commande sera exécutée.

- `KalmanTracker(dt, sigma_accel, sigma_pos)` — tracker mono-objet
- `tracker.update(pos_3d)` — mise à jour avec une nouvelle mesure
- `tracker.predict_at(latency_s)` → position prédite à t + latence
- `MultiObjectTracker` — association automatique par plus proche voisin

#### `realsense_capture.py`
Wrapper autour de `pyrealsense2` :

- `RealSenseCapture` — flux RGB + profondeur alignés, intrinsèques firmware
- `MockRealSense` — émulation webcam (profondeur constante, pour tests sans matériel)
- `auto_camera()` — sélectionne automatiquement la RealSense si connectée, sinon webcam

#### `generate_markers.py`
Génère une feuille de marqueurs ArUco (dictionnaire DICT_4X4_50) en image pour impression.

#### `main_phase2.py`
Orchestre le pipeline temps réel complet avec affichage OpenCV :
1. Capture (webcam / RealSense)
2. Détection (ArUco ou YOLO)
3. Estimation de pose
4. Transformation vers le repère robot
5. Mise à jour Kalman
6. Affichage HUD (pose, axes 3D, bounding box)

### Flux de données

```
Caméra
  ├── trame couleur (BGR)   → detect_objects → pose_estimation
  └── trame profondeur (mm) ↗ (RealSense uniquement)
                                ↓
                          robot_transform → kalman_tracker → t_predicted [m]
                                                                 ↓
                                                           Phase 3 : VS
```

### Résultats attendus

| Métrique | Valeur cible |
|---|---|
| Détection ArUco | 100% sur marqueurs visibles, < 5 ms |
| Erreur solvePnP | < 1 mm (avec calibration réelle) |
| Erreur Kalman à +150 ms | < 5 mm pour v < 200 mm/s |

---

## Phase 3 — Asservissement visuel PBVS

### Objectif

Guider le robot SCARA depuis sa position courante jusqu'à l'objet détecté, puis exécuter le cycle complet de pick-and-place (approche → saisie → levée → transport → dépose → retour). La loi de commande est un contrôleur PBVS (Position-Based Visual Servoing).

### Lancer

```bash
cd python/phase3
source ../phase2/.venv/bin/activate   # Phase 3 réutilise le venv de Phase 2

# Simulation interactive 2D (matplotlib) — aucun matériel requis
python simulation_gui.py

# Pipeline complet avec cible fixe (contournement caméra)
python main_phase3.py --live --force-target 350 50 -150   # x y z en mm

# Cible nécessitant une pré-approche (derrière le robot)
python main_phase3.py --live --force-target -300 200 -150

# Pipeline caméra réelle (ArUco)
python main_phase3.py --live --method aruco --size 0.08

# RealSense D435
python main_phase3.py --realsense --method aruco --size 0.05

# Simulation boucle fermée sans caméra
python main_phase3.py --sim
```

### Contrôles GUI (`simulation_gui.py`)

| Touche / Action | Effet |
|---|---|
| Clic gauche | Placer l'objet (zone verte = workspace atteignable par IK) |
| Clic droit | Déplacer la zone de dépose |
| ESPACE | Démarrer / pause |
| R | Réinitialiser |
| Q / Échap | Quitter |

### Loi de commande PBVS

L'erreur visuelle est définie dans l'espace 3D (repère robot) :

```
e = [t_current − t_desired ;  θ·u]  ∈ R⁶
                 ↑ position     ↑ axe-angle

v_c = −λ(‖e‖) · Ls⁺ · e       (vitesse cartésienne)
q̇   = J⁺(q)  · v_c             (vitesses articulaires)
```

- **λ adaptatif** : sigmoïde ∈ [0.05, 1.5], augmente pour les grandes erreurs, ralentit pour le positionnement fin
- **J⁺** : pseudo-inverse DLS (amorti), robuste aux singularités
- **Tâche secondaire** : évitement des butées articulaires (méthode de Liegeois, poids 0.1)

### Machine à états principale (`PipelineState`)

```
SEARCHING → PRE_APPROACH → TRACKING → APPROACH → CONVERGED
                ↑ (saut cible > 50 mm)               ↓
                └─────────────────────────────── EMERGENCY
```

| État | Déclencheur | Action |
|---|---|---|
| `SEARCHING` | Aucun objet détecté | Robot au repos |
| `PRE_APPROACH` | Erreur θ1 ou θ3 > 8° | Interpolation articulaire vers la solution IK |
| `TRACKING` | Objet détecté, ‖e‖ > 50 mm | VS actif, λ = 1.5 |
| `APPROACH` | ‖e‖ < 50 mm | VS fin, λ = 1.0 |
| `CONVERGED` | ‖e_t‖ < 2 mm et ‖e_r‖ < 1° | Séquenceur pince activé |
| `EMERGENCY` | σ_min(J) < seuil | Arrêt d'urgence |

### Séquenceur pick-and-place (`PickPlaceState`)

```
IDLE → APPROACH → GRASPING → LIFTING → TRANSPORT → LOWERING → RELEASING → RETURNING → DONE
```

Chaque transition TRANSPORT et RETURNING déclenche un nouveau `PRE_APPROACH` si la cible saute de plus de 50 mm (ex. pick→dépose, dépose→home).

### Configuration de repos non singulière

Le robot démarre à `q₀ = [0°, 100 mm, −45°, 0°]`.
θ3 = 0° est singulier (σ_min ≈ 0, r = 460 mm) et est systématiquement évité par la pré-approche.

### Fichiers

#### `visual_error.py`
Calcul de l'erreur visuelle et de la matrice d'interaction.

- `compute_error(t_cur, R_cur, t_des, R_des, params)` → `VisualError`
- `axis_angle(R)` — représentation θ·u depuis une matrice de rotation (Rodrigues inverse)
- `interaction_matrix_pbvs(R_co, t_co, R_cd)` — Ls bloc-diagonale exacte (Chaumette 2006)

```python
VisualError.e           # vecteur d'erreur (6,)
VisualError.norm_t_mm   # norme de l'erreur de position en mm
VisualError.converged   # bool : ‖e_t‖ < 2 mm et ‖e_r‖ < 1°
```

#### `vs_controller.py`
Contrôleur PBVS + cinématique partagée entre tous les modules.

- `ScaraParams` — paramètres géométriques et butées articulaires (miroir de `robot_parameters.m`)
  - a2=300 mm, a3=160 mm, d3=150 mm, d4=59 mm
  - q_min=[-2.356, 0, -1.571, -3.142], q_max=[2.356, 0.200, 1.571, 3.142]
  - dq_max=[2.0 rad/s, 0.1 m/s, 2.0 rad/s, 2.0 rad/s]
- `ik_solutions(x_m, y_m, params)` — IK analytique, retourne toutes les paires (θ1, θ3) valides
- `scara_jacobian(q, params)` — Jacobienne numérique J ∈ R^(6×4)
- `damped_pinv(J)` — pseudo-inverse DLS, retourne σ_min et flag de singularité
- `VSController.update(error, q, dt)` → `VSCommand` (dq, v_c, singular, saturated)

#### `gripper_controller.py`
Séquenceur de tâche pick-and-place.

- `GripperController` — modèle de pince (OPEN / CLOSING / CLOSED / OPENING), timers ouverture/fermeture
- `PickPlaceSequencer(drop_pos_m)` — machine à 9 états
  - `.update(vs_converged, object_pos_m, t_ee_m, q_current)` → (état, vs_target, gripper_close)
  - `._target_pos` — cible VS courante (change à chaque transition)
- `draw_hud(frame)` — superposition d'état en BGR sur la trame caméra

#### `simulation_gui.py`
Simulation 2D complète du cycle pick-and-place (matplotlib, sans matériel).

- **Vue XY** — projection horizontale : bras + workspace pixel-exact (masque IK vectorisé sur grille 320×320)
- **Vue XZ** — profil de hauteur (levée, transport, descente)
- **Panneau d'état** — machine à états temps réel + barre d'erreur |e_t|
- Pré-approche intégrée : déclenchée au clic ET à chaque changement de cible du séquenceur
- Objet animé suit le bras pendant GRASPING / LIFTING / TRANSPORT / LOWERING

#### `main_phase3.py`
Pipeline complet Phase 2 + Phase 3, temps réel ou simulation.

- `Phase3Pipeline(cam_params, robot_tf, force_target)` — machine à états globale
  - `.process_frame(frame)` → (trame annotée, VSCommand, PipelineState)
  - Deux chemins : `force_target` (contournement caméra) et pipeline caméra complet
  - `effective_target` calculé depuis `sequencer._target_pos` (pas l'objet brut)
  - VS actif à chaque trame hors pré-approche (jamais gelé)
- CLI : `--force-target x y z`, `--method aruco|yolo`, `--realsense`, `--live`, `--sim`

### Flux de données Phase 3

```
[Phase 2] t_predicted (m)
       ↓
ik_solutions → PRE_APPROACH si θ1/θ3 décalé de > 8°
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
| Convergence depuis position éloignée | ~30 trames après pré-approche |
| Erreur de position finale | < 2 mm |
| Erreur d'orientation finale | < 1° |
| Workspace effectif | r ∈ [340, 460] mm, θ1 ∈ ±135° |
| Cycle pick-and-place complet | Validé en simulation GUI |

---

## Tests PLC (`tests/`)

Scripts de diagnostic et de test bas-niveau pour la communication avec l'automate Siemens S7 via `python-snap7`.
Tous nécessitent une connexion réseau à l'automate (`192.168.0.10`).

| Fichier | Rôle |
|---|---|
| `diag_robot.py` | Scan complet de l'automate : entrées physiques, mémoire cibles, état pince |
| `ratio_test.py` | Détermine le ratio impulsions/degré (ou mm) pour chaque axe |
| `grab_test.py` | Test de la pince seule (bit M34.5 — KEP) |
| `scenario_test.py` | Séquence pick-and-place complète : HOME → APPROACH → GRAB → LIFT → PLACE |
| `test.py` | Tests unitaires divers |

```bash
cd python/tests
pip install python-snap7
python diag_robot.py       # Diagnostic général
python ratio_test.py       # Calibration des ratios d'axes
python grab_test.py        # Test pince
python scenario_test.py    # Scénario complet
```
