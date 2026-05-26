# Vision-Based Control for Autonomous Robotic Systems
**Stage M1 — DNIIT / DUT Đà Nẵng**
Superviseur : Dr. Vo Nhu Thanh

---

## Contexte

Ce projet de stage porte sur le développement d'un système d'**asservissement visuel** (*visual servoing*) pour guider un robot **SCARA 4-DOF** afin de saisir des objets en mouvement (application *pick-and-place* dynamique sur tapis roulant).

Le robot cible est un **SCARA** (*Selective Compliance Assembly Robot Arm*) à 3 liaisons rotoïdes + 1 liaison prismatique, monté au-dessus d'un tapis roulant. La caméra est positionnée au-dessus de la scène pour détecter les objets.

---

## Structure du projet

```
.
├── README.md
├── plan_stage.md              ← Plan détaillé des 5 phases du projet
├── docs/                      ← Documentation de référence (PDFs)
│   ├── offre-stage.pdf        ← Sujet de stage officiel
│   ├── Mobile_Manipulator.pdf ← Rapport technique du robot (4-DOF, vision stéréo)
│   ├── PBL6_2.pdf             ← Rapport Delta robot (tri, référence cinématique)
│   └── 1 - Thuyetminh_...pdf ← Rapport SCARA + YOLO (référence vision)
├── matlab/
│   ├── phase1/                ← Phase 1 : Simulation MATLAB
│   │   ├── main_phase1.m
│   │   ├── robot_parameters.m
│   │   ├── forward_kinematics.m
│   │   ├── inverse_kinematics.m
│   │   ├── workspace_analysis.m
│   │   ├── joint_space_trajectory.m
│   │   ├── cartesian_trajectory.m
│   │   ├── plot_trajectory.m
│   │   ├── pid_simulation.m
│   │   └── animate_robot.m
│   └── phase3/                ← Phase 3 : Simulation boucle fermée MATLAB
│       └── main_phase3.m
└── python/
    ├── phase2/                ← Phase 2 : Reconnaissance d'objets et estimation de pose
    │   ├── main_phase2.py
    │   ├── camera_calibration.py
    │   ├── detect_objects.py
    │   ├── pose_estimation.py
    │   ├── robot_transform.py
    │   ├── kalman_tracker.py
    │   ├── realsense_capture.py
    │   ├── generate_markers.py
    │   └── requirements.txt
    └── phase3/                ← Phase 3 : Contrôleur d'asservissement visuel
        ├── main_phase3.py         ← Pipeline principal (machine à états, CLI)
        ├── vs_controller.py       ← Contrôleur PBVS + cinématique SCARA
        ├── visual_error.py        ← Calcul erreur visuelle et matrice d'interaction
        ├── gripper_controller.py  ← Séquenceur pick-and-place (9 états)
        └── simulation_gui.py      ← Simulation 2D interactive (matplotlib)
```

---

## Plan du projet

Le projet est découpé en **5 phases** séquentielles. Voir [plan_stage.md](plan_stage.md) pour le détail complet.

| Phase | Thème | Statut |
|-------|-------|--------|
| **1** | Simulation MATLAB (modèle, cinématique, commande) | ✅ Complété |
| **2** | Reconnaissance d'objets et estimation de pose (Python, OpenCV) | ✅ Complété |
| **3** | Asservissement visuel PBVS (Python + MATLAB) | ✅ Complété |
| **4** | Intégration ROS2 + Gazebo Ignition Fortress — simulation pick-and-place | ✅ Complété |
| **5** | Tests expérimentaux sur robot réel | 🔲 À faire |

---

## Phase 1 — Simulation MATLAB

### Prérequis

- MATLAB R2021b ou supérieur (aucune toolbox requise — tout est implémenté from scratch)

### Lancer la simulation

```matlab
cd matlab/phase1
clear                % vider le cache des fonctions MATLAB
main_phase1          % exécution complète avec figures + animation
main_phase1(false)   % console seulement, sans figures
```

### Ce que fait `main_phase1`

Le script exécute 7 étapes dans l'ordre et affiche un rapport console à chaque étape.

---

### Description des fichiers

#### `robot_parameters.m`

Définit le modèle complet du robot SCARA sous forme d'une structure MATLAB `robot`.

Topologie : **θ1 (R) → d2 (P) → θ3 (R) → θ4 (R)**

Table DH issue du rapport PBL6 (Table 2) :

| Lien | θ | a (m) | α (°) | d (m) |
|------|---|-------|-------|-------|
| 1 | θ1 var | 0 | 0° | 0 |
| 2 | 0° fixe | **0.300** | 0° | d2 var |
| 3 | θ3 var | **0.160** | 180° | −0.150 |
| 4 | θ4 var | 0 | 0° | 0.059 |

Dimensions physiques (Table 1 du rapport) :
- Bras supérieur a2 = 300 mm, avant-bras a3 = 160 mm
- Course verticale d3 = 150 mm, offset pince d4 = 59 mm
- Portée max XY : 460 mm, zone morte : r > 340 mm

Contient aussi : butées `[θ1∈±135°, d2∈[0,200mm], θ3∈±90°, θ4∈±180°]`, gains PID, paramètres dynamiques par axe.

---

#### `forward_kinematics.m`

**Problème :** étant donné `q = [θ1, d2, θ3, θ4]`, où est l'effecteur ?

**Méthode :** composition des 4 matrices DH spécifiques au SCARA :

$$T_{0 \to 4} = A_1(\theta_1) \cdot A_2(d_2) \cdot A_3(\theta_3) \cdot A_4(\theta_4)$$

Position de l'effecteur (formules analytiques issues de la FK du rapport) :

$$P_x = a_2 \cos\theta_1 + a_3 \cos(\theta_1 + \theta_3)$$
$$P_y = a_2 \sin\theta_1 + a_3 \sin(\theta_1 + \theta_3)$$
$$P_z = d_2 - d_3 - d_4$$

```matlab
[T_end, T_all] = forward_kinematics(robot, q)
```

---

#### `inverse_kinematics.m`

**Problème :** étant donné une pose cible `(Px, Py, Pz)`, quelles commandes envoyer ?

**Méthode :** IK **analytique** — formules fermées issues du rapport PBL6 (éq. [1.8]–[1.11]) :

1. $\cos\theta_3 = \dfrac{P_x^2 + P_y^2 - a_2^2 - a_3^2}{2 a_2 a_3}$ → 2 solutions (coude haut/bas)

2. $\theta_1 = \text{atan2}\!\left(\dfrac{P_y k_1 - P_x k_2}{D},\ \dfrac{P_x k_1 + P_y k_2}{D}\right)$

3. $\theta_4 = -\theta_1 - \theta_3$ (orientation maintenue à 0°)

4. $d_2 = P_z + d_3 + d_4$

Avantage par rapport à une méthode numérique : **solution exacte en un calcul**, pas d'itération, erreur résiduelle = 0.0000 mm.

```matlab
[q_sol, success, err] = inverse_kinematics(robot, T_des, q0)
```

---

#### `workspace_analysis.m`

Visualise l'**espace de travail** du SCARA (enveloppe de tous les points atteignables par l'effecteur).

**Méthode :** Monte Carlo — 30 000 configurations aléatoires dans `[q_min, q_max]`, cinématique directe pour chacune.

Résultats attendus (cohérents avec Figure 11-12 du rapport PBL6) :
- XY : anneau toroïdal entre r = 340 mm et r = 460 mm (zone morte car θ3 ∈ ±90°)
- Z : plan de travail horizontal à hauteur variable entre −209 mm et −9 mm

Génère 4 figures : vue 3D, projection XY, projection XZ, structure du bras SCARA (bras horizontaux, colonne verticale).

```matlab
workspace_analysis(robot)
workspace_analysis(robot, 50000)  % plus d'échantillons
```

---

#### `joint_space_trajectory.m`

Génère une trajectoire **point-à-point dans l'espace articulaire** de `q_start` à `q_end`.

3 profils disponibles :

| Profil | Continuité | Particularité |
|--------|-----------|--------------|
| `cubic` | C¹ (vitesse) | Standard, simple |
| `quintic` | C² (accélération) | Mouvement le plus doux |
| `trapezoidal` | C⁰ | Proche des variateurs industriels |

Retourne `traj.q`, `traj.dq`, `traj.ddq` (positions, vitesses, accélérations).

```matlab
traj = joint_space_trajectory(q_start, q_end, T_total, dt, 'quintic')
```

---

#### `cartesian_trajectory.m`

Génère une trajectoire **linéaire dans l'espace cartésien** (l'effecteur suit une ligne droite).

- **Position :** interpolation linéaire avec lissage cubique
- **Orientation :** interpolation SLERP (*Spherical Linear Interpolation*) — l'interpolation correcte pour les rotations
- À chaque point, la cinématique inverse est appelée pour obtenir les angles articulaires

```matlab
traj = cartesian_trajectory(robot, p_start, p_end, R_start, R_end, T_total, dt)
```

---

#### `plot_trajectory.m`

Visualise une trajectoire générée (articulaire ou cartésienne).

- Trajectoire **articulaire** : 3 subplots (positions, vitesses, accélérations) en °, °/s, °/s²
- Trajectoire **cartésienne** : coordonnées XYZ vs temps + chemin 3D

```matlab
plot_trajectory(traj)
```

---

#### `pid_simulation.m`

Simule le **suivi de trajectoire en boucle fermée** par un contrôleur PID + feedforward indépendant sur chaque axe.

**Modèle dynamique (paramètres lus depuis `robot.J_eff`, `robot.B_vis`) :**

$$J_{eff,i} \cdot \ddot{q}_i = \tau_i - B_{vis,i} \cdot \dot{q}_i$$

**Loi de commande (feedforward + PID) :**

$$\tau_i = \underbrace{J_{eff,i} \cdot \ddot{q}_{ref} + B_{vis,i} \cdot \dot{q}_{ref}}_{\text{feedforward}} + K_p e_i + K_i \int e_i \, dt + K_d (\dot{q}_{ref} - \dot{q}_i)$$

Protections : anti-windup, saturation des couples, butées articulaires.
Intégration par méthode de **Heun (RK2)** pour éviter l'accumulation d'erreur de phase.

Résultats Phase 1 : RMSE < 0.5° sur tous les axes (θ1, θ3, θ4) et < 0.5 mm sur d2.

```matlab
result = pid_simulation(robot, traj, Kp, Ki, Kd)
```

---

#### `animate_robot.m`

Anime le SCARA en 3D le long d'une trajectoire. Reconstruit les points physiques du robot (colonnes verticales, bras horizontaux) pour un rendu réaliste.

```matlab
animate_robot(robot, traj)                 % vitesse normale
animate_robot(robot, traj, 'speed', 2)     % 2× plus rapide
animate_robot(robot, traj, 'trail', false) % sans trace de l'effecteur
```

---

### Flux de données entre les modules

```
robot_parameters
       │
       ├──► forward_kinematics ──► workspace_analysis
       │           │
       │           └──► inverse_kinematics ◄── cartesian_trajectory
       │
       ├──► joint_space_trajectory ──► pid_simulation
       │                    │
       │                    └──► animate_robot
       │
       └──► main_phase1  (orchestre tout)
```

### Critères de réussite Phase 1

- FK : $P_z = d_2 - d_3 - d_4$ toujours négatif (effecteur sous le plan de montage) ✓
- IK analytique : erreur résiduelle = 0.0000 mm ✓
- Espace de travail : anneau toroïdal XY, r ∈ [340, 460] mm ✓
- Suivi PID+FF : RMSE < 0.5° (axes rotoïdes) et < 0.5 mm (axe prismatique) ✓

---

---

## Phase 2 — Reconnaissance d'objets et estimation de pose

### Prérequis

```bash
cd python/phase2
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Optionnel — YOLO
pip install ultralytics

# Optionnel — Intel RealSense D4xx (RGB-D)
pip install pyrealsense2
```

### Lancer le pipeline

```bash
# Générer une planche de marqueurs ArUco à afficher sur téléphone
python generate_markers.py

# Test sur image statique
python main_phase2.py --image test_images/test_aruco_scene.jpg --method aruco

# Flux webcam temps réel (ArUco)
python main_phase2.py --live --method aruco --size 0.08  # marqueur 8 cm

# Flux RealSense D435 (RGB-D, calibration firmware automatique)
python main_phase2.py --realsense --method aruco --size 0.08

# Flux webcam avec YOLO (modèle pré-entraîné COCO)
python main_phase2.py --live --method yolo --model yolov8n.pt

# Calibration caméra (damier physique requis, inutile avec RealSense)
python camera_calibration.py --live --rows 6 --cols 9 --size 25
```

### Sélectionner la bonne caméra

```bash
# Lister toutes les caméras disponibles (webcam + RealSense)
python main_phase2.py --list-cams

# Sur macOS : index 0 = Continuity Camera (iPhone), index 1 = FaceTime
python main_phase2.py --live --cam 1 --method aruco
```

### RealSense D435 — mode RGB-D

Quand la RealSense est connectée, le pipeline :
1. Charge les **intrinsèques directement depuis le firmware** (pas besoin de calibration manuelle).
2. Aligne automatiquement le flux profondeur sur le flux couleur.
3. Active `estimate_pose_yolo_rgbd()` (YOLO) — déprojection 3D exacte pixel par pixel.
4. Fournit une carte de profondeur uint16 en mm pour `get_3d_point()`.

```bash
# Test du module RealSense seul
python realsense_capture.py               # auto_camera() — RealSense si dispo, sinon webcam
python realsense_capture.py --list        # lister les caméras RealSense
python realsense_capture.py --cam 1 --z 900  # webcam index 1, plan de travail à 900 mm
```

Si aucune RealSense n'est connectée et que `--realsense` n'est pas passé, le pipeline
bascule automatiquement sur `MockRealSense` (webcam + profondeur constante au plan de travail `--z`).

### Description des modules

#### `camera_calibration.py`
Calibration intrinsèque par damier (chessboard). Génère `calibration_data/camera_params.npz` contenant la matrice K (3×3) et les coefficients de distorsion. RMS < 0.5 px = excellente calibration.

#### `detect_objects.py`
Deux méthodes de détection :
- **ArUco** — marqueurs imprimés, pose exacte (~0.5 mm), sans apprentissage. Idéal pour tests et calibration.
- **YOLO** — détection d'objets naturels via YOLOv8 (ultralytics). Nécessite `pip install ultralytics`.

#### `pose_estimation.py`
Estimation de la pose 6D (position + orientation) dans le repère caméra :
- `estimate_pose_aruco()` — solvePnP exact sur les 4 coins du marqueur
- `estimate_pose_yolo_rgbd()` — position 3D depuis carte de profondeur (RealSense D435)
- `estimate_pose_yolo_flat()` — reprojection inverse sur plan horizontal connu

#### `robot_transform.py`
Transformation caméra → repère robot via matrice homogène T (4×4).
Calibration main-œil (`hand_eye_calibrate()`) ou mesure géométrique directe.

#### `kalman_tracker.py`
Filtre de Kalman à modèle vitesse constante. Compense la **latence du pipeline** (~150 ms) en prédisant la position future de l'objet sur le tapis roulant. Tracker multi-objets avec association par plus proche voisin.

#### `realsense_capture.py`
Wrapper autour de `pyrealsense2` pour la caméra Intel RealSense D435 (ou D415/D455) :
- `RealSenseCapture` — flux RGB + profondeur alignés, intrinsèques firmware
- `MockRealSense` — émulation avec webcam ordinaire (profondeur constante)
- `auto_camera()` — sélection automatique (RealSense si connectée, sinon webcam)

### Flux de données Phase 2

```
Caméra (webcam / RealSense D435)
       │
       ├─ color frame (BGR)
       └─ depth frame (uint16 mm) ← RealSense uniquement
       │
       ▼
[detect_objects]      → position en pixels
       │
       ▼
[pose_estimation]     → pose 6D dans repère caméra (mm)
       │              (solvePnP ArUco | déprojection RealSense | plan flat)
       ▼
[robot_transform]     → pose dans repère robot (mm)
       │
       ▼
[kalman_tracker]      → position prédite à t+150ms  ← cible réelle pour la saisie
       │
       ▼
  Phase 3 : contrôleur visual servoing
```

### Critères de réussite Phase 2

- Détection ArUco : 100 % sur marqueurs visibles, latence < 5 ms ✓
- Pose estimation : erreur résiduelle solvePnP < 1 mm (avec calibration réelle) ✓
- Kalman : erreur de prédiction à +150 ms < 5 mm pour v < 200 mm/s ✓

---

## Phase 3 — Asservissement visuel PBVS

### Stratégie : PBVS (Position-Based Visual Servoing)

L'erreur est définie dans l'espace 3D (repère robot) :

$$e = \begin{bmatrix} t_{courant} - t_{désiré} \\ \theta \cdot u \end{bmatrix} \in \mathbb{R}^6$$

Loi de commande (contrôleur intégrateur) :

$$v_c = -\lambda(\|e\|) \cdot L_s^+ \cdot e \qquad \dot{q} = J^+(q) \cdot v_c$$

- $L_s = \text{diag}(I_3,\; L_\omega)$ — matrice d'interaction PBVS bloc-diagonale
- $\lambda(\|e\|)$ — gain adaptatif sigmoïde ($\lambda \in [0.05,\, 1.5]$, inflexion à 10 mm)
- $J^+(q)$ — pseudo-inverse amortie DLS (robuste aux singularités)
- Tâche secondaire : évitement des butées articulaires (méthode Liegeois, poids $w_0 = 0.1$)

### Machine à états principale (`PipelineState`)

```
SEARCHING → PRE_APPROACH → TRACKING → APPROACH → CONVERGED
                ↑ (changement de cible >50 mm)       ↓
                └──────────────────────────────── EMERGENCY
```

| État | Déclencheur | Action |
|------|-------------|--------|
| `SEARCHING` | Aucun objet détecté | Robot en repos |
| `PRE_APPROACH` | Erreur θ1/θ3 > 8° | Interpolation joint-space vers solution IK |
| `TRACKING` | Objet détecté, \|e\| > 50 mm | VS actif, λ = 1.5 |
| `APPROACH` | \|e\| < 50 mm | VS fin, λ = 1.0, v_max = 80 mm/s |
| `CONVERGED` | \|e_t\| < 2 mm et \|e_r\| < 1° | Séquenceur pince activé |
| `EMERGENCY` | σ_min(J) < seuil | Arrêt d'urgence |

### Séquenceur pick-and-place (`PickPlaceState`)

```
IDLE → APPROACH → GRASPING → LIFTING → TRANSPORT → LOWERING → RELEASING → RETURNING → DONE
```

Déclenchement automatique dès convergence VS. Le TRANSPORT et le RETURNING déclenchent une nouvelle `PRE_APPROACH` si la cible change de plus de 50 mm.

### Configuration de repos non-singulière

Le robot démarre à $q_0 = [0°,\; 100\text{mm},\; -45°,\; 0°]$ (θ3 = −45°, r ≈ 430 mm, σ_min = 0.063). Le point θ3 = 0° est singulier (r = 460 mm, σ_min ≈ 0) et est évité par la pré-approche.

### Résultats validés (simulation Python)

- Convergence en **≈ 30 frames** depuis position éloignée (après pré-approche)
- Erreur finale : **< 2 mm** / **< 1°** ✓
- Workspace effectif : anneau $r \in [340,\; 460]$ mm, angles $\theta_1 \in [\pm 135°]$ (±163° aux limites IK)
- Cycle pick-and-place complet (APPROACH→DONE) validé en simulation GUI

### Lancer la simulation interactive (GUI matplotlib)

```bash
cd python/phase3
source ../phase2/.venv/bin/activate

# Simulation 2D interactive — clic gauche : objet, clic droit : dépose, ESPACE : start
python simulation_gui.py
```

Contrôles :
- **Clic gauche** — placer l'objet (zone verte = workspace pixel-exact par calcul IK)
- **Clic droit** — déplacer la zone de dépose
- **ESPACE** — démarrer / pause (le bras part en pré-approche puis VS)
- **R** — réinitialiser

### Lancer le contrôleur réel

```bash
# Test cible fixe (bypass caméra) — utile pour valider sans hardware
python main_phase3.py --live --cam 1 --force-target 350 50 -150

# Cible "derrière" le robot (nécessite pré-approche)
python main_phase3.py --live --cam 1 --force-target -300 200 -150

# Flux webcam temps réel (ArUco)
python main_phase3.py --live --cam 1 --method aruco --size 0.08

# Flux RealSense D435
python main_phase3.py --realsense --method aruco --size 0.05

# Simulation Python boucle fermée (sans caméra)
python main_phase3.py --sim

# Test sur image statique
python main_phase3.py --image ../phase2/test_images/test_aruco_scene.jpg
```

### Lancer la simulation MATLAB (boucle fermée)

```matlab
cd matlab/phase3
main_phase3      % Lance la simulation + affiche 6 graphiques de convergence
```

Simulation configurable (paramètres en tête du script) : `use_conveyor`, `v_conveyor`, `lambda_nom`, `dt`, `T_sim`.

### Description des modules

#### `visual_error.py`
Calcul de l'erreur visuelle $e \in \mathbb{R}^6$ et de la matrice d'interaction $L_s$ :
- `compute_error(t_cur, R_cur, t_des, R_des)` → `VisualError` (avec `.converged`)
- `axis_angle(R)` — représentation axe-angle $\theta \cdot u$ (formule de Rodrigues inverse)
- `interaction_matrix_pbvs(R_co, t_co, R_cd)` — $L_s$ bloc-diagonale exacte (Chaumette 2006)

#### `vs_controller.py`
Contrôleur complet PBVS + cinématique partagée :
- `ScaraParams` — paramètres géométriques et limites (miroir de `robot_parameters.m`)
- `ik_solutions(x_m, y_m, params)` — IK analytique SCARA, retourne toutes les solutions (θ1, θ3) dans les limites. Fonction partagée par `main_phase3.py` et `simulation_gui.py`
- `scara_jacobian(q, params)` — jacobienne analytique $J \in \mathbb{R}^{6 \times 4}$
- `damped_pinv(J)` — pseudo-inverse DLS, retourne $\sigma_{\min}$ et flag singularité
- `VSController.update(error, q, dt)` → `VSCommand` (vitesses articulaires + diagnostics)
- `simulate_pbvs(...)` — simulation Python intégrée (FK + intégrateur Euler)

#### `gripper_controller.py`
Séquenceur de la tâche pick-and-place :
- `GripperController` — modèle pince (OPEN / CLOSING / CLOSED / OPENING), timer d'ouverture/fermeture
- `PickPlaceSequencer` — machine à 9 états (IDLE → … → DONE), fournit à chaque frame la cible VS et la commande pince
- `draw_hud(frame)` — superpose l'état courant en overlay BGR sur le frame caméra
- La cible VS change automatiquement (objet → point d'approche → hauteur levée → dépose → retour home)

#### `simulation_gui.py`
Simulation 2D interactive du cycle pick-and-place complet :
- **Vue XY** — bras SCARA + workspace pixel-exact (masque calculé par IK vectorisé sur grille 320×320)
- **Vue XZ** — profil de hauteur (levée, transport, abaissement)
- **Panneau état** — machine à états en temps réel, barre d'erreur |e_t|
- Pré-approche joint-space intégrée : déclenchée au placement de l'objet *et* automatiquement à chaque changement de cible du séquenceur (pick → dépose → home)
- L'objet animé suit le bras pendant GRASPING / LIFTING / TRANSPORT / LOWERING

#### `main_phase3.py`
Orchestration complète Phase 2 + Phase 3 :
- `Phase3Pipeline` — 6 états : `SEARCHING → PRE_APPROACH → TRACKING → APPROACH → CONVERGED → EMERGENCY`
- Deux chemins d'entrée : `force_target` (bypass caméra) et pipeline caméra complet (ArUco/YOLO)
- Pré-approche dans les deux chemins : détecte les sauts de cible du séquenceur ≥ 50 mm
- `effective_target` — cible calculée depuis `sequencer._target_pos` (pas l'objet brut), évite les blocages en APPROACH lors des transitions LIFTING→TRANSPORT
- HUD live : état, |e_t|, |e_r|, θ1/d2/θ3/θ4 courants

#### `matlab/phase3/main_phase3.m`
Script MATLAB de validation en simulation boucle fermée :
- Supporte le mode tapis roulant (`use_conveyor = true`, `v_conveyor`)
- Compensation de latence Kalman ($+150$ ms)
- 6 graphiques : convergence, gain adaptatif, trajectoire XY, config. articulaires, commandes

### Flux de données Phase 3

```
[Phase 2 pipeline]
  détection + pose 6D → Kalman → t_prédit
         │
         ▼
  ik_solutions(t_prédit)
  → PRE_APPROACH si θ1/θ3 loin (> 8°) de la solution IK
         │ (pré-approche terminée)
         ▼
[visual_error.py]
  compute_error(t_cur, R_cur, t_effective, R_des)
       → e = [e_t ; e_r]  ∈ R^6
         │
         ▼
[vs_controller.py]
  VSController.update(e, q) → dq (vitesses articulaires)
         │
         ├──► [gripper_controller.py]  (séquenceur pick-and-place)
         │         → nouvelle cible → détection saut → nouvelle PRE_APPROACH
         ▼
  Robot SCARA : q ← q + dq * dt
```

### Critères de réussite Phase 3

- Convergence stable depuis toute position initiale dans le workspace ✓
- Erreur finale < 2 mm / < 1° ✓ (validé en simulation Python)
- Pré-approche joint-space : atteint θ1 = ±163° sans traverser la singularité θ3 = 0° ✓
- Cycle pick-and-place complet (9 états) validé en simulation GUI ✓
- Détection et pré-approche automatique sur chaque changement de cible du séquenceur ✓
- Détection des singularités + saturation articulaire ✓
- Support tapis roulant (objet en mouvement) via compensation Kalman ✓

---

- **1 - Thuyetminh_PBL6...pdf** — Rapport SCARA YOLO (source du modèle robot : Table 1 & 2, IK analytique, profils S-curve)
- **offre-stage.pdf** — Sujet officiel : visual servoing pour pick-and-place dynamique, YOLO + estimation de pose 6D, ROS2 + MoveIt2
- **Mobile_Manipulator.pdf** — Référence vision stéréo et mécanique bras
- Craig, J.J. — *Introduction to Robotics: Mechanics and Control*
- Chaumette, F. & Hutchinson, S. — *Visual Servo Control* (IEEE Robotics & Automation Magazine, 2006)

---

## Bilan Phase 3 et prérequis Phase 4

### Ce qui est couvert en Phase 3

| Composant | Fichier | Statut |
|-----------|---------|--------|
| Erreur visuelle PBVS + matrice d'interaction | `visual_error.py` | ✅ |
| Contrôleur VS (DLS, gain adaptatif, butées) | `vs_controller.py` | ✅ |
| IK analytique partagée | `vs_controller.ik_solutions()` | ✅ |
| Séquenceur pick-and-place 9 états | `gripper_controller.py` | ✅ |
| Pré-approche joint-space (toute position initiale) | `main_phase3.py` | ✅ |
| Pipeline caméra temps réel (ArUco / YOLO / RealSense) | `main_phase3.py` | ✅ |
| Simulation 2D interactive (workspace pixel-exact) | `simulation_gui.py` | ✅ |
| Validation MATLAB boucle fermée + tapis roulant | `matlab/phase3/main_phase3.m` | ✅ |

### Ce qu'il manque avant Phase 4

| Élément | Priorité | Notes |
|---------|----------|-------|
| **Modèle URDF/XACRO du SCARA** | Haute | Requis pour Gazebo et MoveIt 2. Peut être généré depuis les paramètres DH de `robot_parameters.m` |
| **Drivers moteurs et encodeurs** | Haute | Interface `q_current` réel → actuellement simulé dans `main_phase3.py` par intégration d'Euler |
| **Package ROS 2 (colcon)** | Haute | Wrapping de `Phase3Pipeline.process_frame()` en nœud ROS 2 |
| **`requirements.txt` phase3** | Basse | Phase 3 utilise le venv de phase2 (`../phase2/.venv`) |

---

## Phase 4 — Intégration ROS2 + MoveIt2 + Gazebo

### Vue d'ensemble

Phase 4 connecte le pipeline d'asservissement visuel (Phase 3) à ROS2 Humble via :
- **`scara_description`** — modèle URDF/XACRO du robot SCARA 4-DOF
- **`scara_moveit_config`** — configuration MoveIt2 (SRDF, IK, contrôleurs, limites)
- **`scara_visual_servoing`** — nœuds ROS2 encapsulant le pipeline Phase 3
- **Docker** — environnement ROS2 + Gazebo + MoveIt2 clé en main pour macOS

```
src/
├── scara_description/         ← URDF/XACRO + launch display
├── scara_moveit_config/       ← SRDF, kinematics.yaml, contrôleurs, launch MoveIt2
└── scara_visual_servoing/     ← Nœuds ROS2 VS + simulation Gazebo
Dockerfile
docker-compose.yml
```

Architecture des nœuds ROS2 :

```
/vs/target_pose  ──────────────────────────────────────────────► [vs_node]
/joint_states ──────────────────────────────────────────────► [vs_node]
                                                                          │
                                                                  PBVS + DLS
                                                                  Machine états
                                                                  TRACKING → CARRYING
                                                                          │
                                   /joint_trajectory_controller/joint_trajectory ◄─┘
                                             │
                                   [ros2_control / Gazebo DART]
                                             │
                              ┌──────────────┤
                         /joint_states   /vs/tcp_pose (30 Hz)
                              │               │
                         [vs_node]     [sim_target_node] → /vs/target_pose
                                             tapis roulant + set_pose Gazebo
```

### Prérequis
**Natif Linux/WSL2**  
- Ubuntu 22.04  
- ROS2 Humble Desktop Full  
- `ros-humble-moveit`, `ros-humble-gazebo-ros2-control`

---

### Build Linux / WSL2

```bash
# Installer ROS2 Humble + dépendances
sudo apt install ros-humble-moveit ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-joint-state-publisher-gui

# Depuis la racine du workspace
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y

colcon build --packages-select \
    scara_description scara_moveit_config scara_visual_servoing \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

# Vérifier que les packages sont bien installés
ros2 pkg list | grep scara
```

---

### Architecture cinématique dans le URDF

Le URDF (`scara_robot.urdf.xacro`) reflète exactement les paramètres DH de la Phase 1 :

| Joint URDF | Type | Axe | Limits | FK |
|------------|------|-----|--------|----|
| `joint1` | revolute | +Z | [−135°, +135°] | θ1 |
| `joint2` | prismatic | +Z | [0, 200 mm] | d2 (Pz ↑ quand d2 ↑) |
| `joint3` | revolute | +Z | [−90°, +90°] | θ3, origine à (a2=0.3, 0, 0) |
| `joint4` | revolute | +Z | [−180°, +180°] | θ4, origine à (a3=0.16, 0, −d3=−0.15) |
| `ee_joint` | fixed | — | — | origine à (0, 0, −d4=−0.059) |

Vérification FK intégrée :
- Px = a2·cos(θ1) + a3·cos(θ1+θ3) ✓
- Py = a2·sin(θ1) + a3·sin(θ1+θ3) ✓
- Pz = d2 − d3 − d4 = d2 − 0.209 m ✓ (EE toujours sous le plan de montage)

---

### Topics ROS2 principaux

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | → vs_node | État articulaire courant (BEST_EFFORT) |
| `/vs/target_pose` | `geometry_msgs/PoseStamped` | sim_target → vs_node | Pose cible sur le tapis |
| `/vs/deposit_pose` | `geometry_msgs/PoseStamped` | sim_target → vs_node | Point de dépôt |
| `/vs/status` | `std_msgs/String` | vs_node → | TRACKING / PICKED / CARRYING / DEPOSITED |
| `/vs/tcp_pose` | `geometry_msgs/PoseStamped` | vs_node → sim_target | Position TCP bras (pour sphère en CARRYING) |
| `/vs/target_marker` | `visualization_msgs/Marker` | sim_target → | Sphère RViz2 (cible) |
| `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | vs_node → | Commande position |

---

### Dépannage

| Problème | Solution |
|----------|----------|
| `Cannot open display :0` | `xhost + 127.0.0.1` puis relancer |
| `joint_trajectory_controller not active` | Attendre ~10 s après lancement Gazebo, ou relancer `gazebo_bridge` |
| `ModuleNotFoundError: vs_controller` | Vérifier que `python/phase3/` est accessible depuis `/ros2_ws` |
| `No module named cv_bridge` | `apt install ros-humble-cv-bridge` dans le conteneur |
| Gazebo ne démarre pas (macOS) | Essayer `headless:=true` + `ros2 launch ... use_sim_time:=true` |
| `rosdep: package not found` | `rosdep update && rosdep install ...` à l'intérieur du conteneur |
| Erreur de build colcon | `colcon build --packages-select scara_visual_servoing 2>&1 | tail -30` |

---

### Critères de réussite Phase 4

- URDF charge sans erreur dans RViz2 + Gazebo ✓
- Tous les joints contrôlables via `joint_state_publisher_gui` ✓
- VS node converge en simulation (|et| < 2 mm) ✅
- Gazebo + ros2_control : commandes position exécutées correctement ✅
- FK du URDF = FK analytique de `vs_controller.py` ✅
- Sphère rouge visible et se déplaçant sur le tapis dans Gazebo ✅
- Sphère suit le TCP du bras pendant la phase CARRYING ✅
- Cycle pick-and-place complet (TRACKING → WAIT_PICK → CARRYING → WAIT_DEP) en boucle ✅