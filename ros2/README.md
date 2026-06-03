# Phase 4 — Simulation ROS2 SCARA + Asservissement Visuel

Simulation complète d'un robot SCARA 4-DOF avec pick-and-place sur tapis roulant,
utilisant ROS2 Humble, Gazebo Ignition (Fortress) et ros2_control.

---

## Table des matières

1. [Prérequis](#prérequis)
2. [Architecture du projet](#architecture-du-projet)
3. [Paramètres du robot](#paramètres-du-robot)
4. [Installation et build](#installation-et-build)
5. [Lancement](#lancement)
6. [Description des nœuds](#description-des-nœuds)
7. [Scénario pick-and-place](#scénario-pick-and-place)
8. [Topics et interfaces](#topics-et-interfaces)
9. [Bugs corrigés](#bugs-corrigés)
10. [Synchronisation macOS → Linux](#synchronisation-macos--linux)

---

## Prérequis

### Système
- Ubuntu 22.04
- ROS2 Humble
- Gazebo Ignition Fortress 6

### Packages ROS2 requis

```bash
sudo apt install \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-robot-state-publisher \
  ros-humble-xacro
```

### Python (env virtuel `env/`)

```bash
# Déjà installé dans env/ (ultralytics, opencv, numpy, matplotlib, sympy...)
source env/bin/activate
```

---

## Architecture du projet

```
ros2/
├── src/
│   ├── scara_description/          # URDF/XACRO du robot
│   │   └── urdf/scara_robot.urdf.xacro
│   ├── scara_moveit_config/        # Configuration ros2_control + MoveIt2
│   │   └── config/ros2_controllers.yaml
│   └── scara_visual_servoing/      # Nœuds ROS2 principaux
│       ├── launch/
│       │   ├── gazebo_sim.launch.py   # Launch complet (Gazebo + VS)
│       │   └── bringup.launch.py      # Launch sans Gazebo (debug)
│       └── scara_visual_servoing/
│           ├── vs_node.py             # Nœud asservissement visuel PBVS
│           ├── sim_target_node.py     # Simulation tapis roulant
│           ├── gazebo_bridge.py       # Initialisation pose home
│           └── vision_node.py         # [Phase 5] Détection YOLO + écriture PLC snap7
├── rebuild.sh                         # Script de rebuild complet
└── README.md                          # Ce fichier
```

---

## Paramètres du robot

### Paramètres DH (SCARA 4-DOF)

| Paramètre | Valeur | Description |
|-----------|--------|-------------|
| a2        | 0.40 m | Longueur bras 1 |
| a3        | 0.30 m | Longueur bras 2 |
| d3        | 0.10 m | Offset vertical lien 3 |
| d4        | 0.15 m | Offset effecteur |

### Joints (ordre convention DH)

| Joint | Type | Limites | Init | Description |
|-------|------|---------|------|-------------|
| joint_1theta | Revolute (Z+) | ±135° | 0 rad | Rotation base |
| joint_1z     | Prismatic (Z+) | 0–0.4 m | 0.35 m | Translation verticale |
| joint3       | Revolute (Z-) | ±90° | **0.3 rad** | Rotation coude |
| joint4       | Revolute (Z-) | ±180° | 0 rad | Rotation poignet |

> **Note** : joint3 démarre à 0.3 rad (et non 0) pour éviter la singularité de configuration
> qui survient quand les deux bras sont parfaitement alignés (θ3 = 0).

### Cinématique directe (FK)

```
px = a2·cos(θ1) + a3·cos(θ1 + θ3)
py = a2·sin(θ1) + a3·sin(θ1 + θ3)
pz = d2 - d3 - d4   (d3=0.10, d4=0.15 → offset total 0.25 m)
```

Pose home [0, 0.40, 0.3, 0] → outil à z = 0.40 − 0.25 = **0.15 m**

### Espace de travail

- Rayon : r ∈ [0.10, 0.70] m
- Hauteur outil : pz ∈ [−0.25, +0.15] m (via d2 ∈ [0, 0.4])
- Zone de saisie tapis : **(0.55, 0.0, 0.09)** m — 6 cm de descente disponible depuis home

---

## Installation et build

### Première installation

```bash
cd ~/Documents/project-10/ros2

# Source ROS2
source /opt/ros/humble/setup.bash

# Build les 3 packages
colcon build --packages-select \
  scara_description \
  scara_moveit_config \
  scara_visual_servoing \
  --symlink-install

source install/setup.bash
```

### Rebuild après modification

```bash
# Rebuild complet avec nettoyage
bash rebuild.sh

# OU rebuild rapide (Python seulement, pas de changement URDF/YAML)
colcon build --packages-select scara_visual_servoing --symlink-install
source install/setup.bash
```

### Rebuild obligatoire si modification de :
- `scara_robot.urdf.xacro` → rebuild `scara_description`
- `ros2_controllers.yaml` → rebuild `scara_moveit_config`
- `*.py` → rebuild `scara_visual_servoing` (ou juste `touch` + relance avec symlink-install)

---

## Lancement

```bash
# Source (obligatoire dans chaque terminal)
source /opt/ros/humble/setup.bash
source ~/Documents/project-10/ros2/install/setup.bash

# Simulation complète (Gazebo GUI + VS + tapis roulant)
ros2 launch scara_visual_servoing gazebo_sim.launch.py

# Sans interface graphique (CI, serveur)
ros2 launch scara_visual_servoing gazebo_sim.launch.py headless:=true

# Avec MoveIt2
ros2 launch scara_visual_servoing gazebo_sim.launch.py with_moveit:=true
```

### Séquence de démarrage (timings)

| t (s) | Événement |
|-------|-----------|
| 0     | Gazebo Ignition démarre |
| 2     | Suppression des anciennes entités (si relance) |
| 4     | Spawn du robot SCARA |
| 5.5   | Spawn tapis roulant (vert, statique) |
| 6.0   | Spawn zone de dépôt (bleu, statique) |
| 6.5   | Spawn sphère objet (rouge, **dynamique**) |
| 20    | Chargement `joint_state_broadcaster` |
| 26    | Chargement `joint_trajectory_controller` |
| 28    | `sim_target_node` démarre — tapis en mouvement |

---

## Description des nœuds

### `vs_node.py` — Asservissement Visuel PBVS

Nœud principal de contrôle. Implémente une boucle d'asservissement visuel
de type PBVS (Position-Based Visual Servoing) avec :

- **Contrôleur** : `VSController` (DLS — Damped Least Squares)
- **Fréquence** : 30 Hz (dt = 0.033 s)
- **Gain** : 1.2 (gain fixe, `adaptive=True` via paramètre ROS2)
- **Sécurité** : saturation vitesse (dq_max = [2.0, 0.2, 2.0, 2.0] rad/s ou m/s)
- **Publication TCP** : `/vs/tcp_pose` (PoseStamped, 30 Hz) — utilisé par sim_target_node

**Machine à états pick-and-place** :

```
TRACKING ──(|et|<2mm)──► WAIT_PICK ──(1s)──► CARRYING ──(|et|<2mm)──► WAIT_DEP ──(1.5s)──► TRACKING
```

- `TRACKING` : robot suit la cible tapis (publiée par sim_target_node)
- `WAIT_PICK` : maintien 1 s, publie `status=PICKED`
- `CARRYING` : robot se dirige vers le dépôt `(0.0, −0.55, 0.09)`
- `WAIT_DEP` : pause 1.5 s, publie `status=DEPOSITED`

**Publication trajectoires** :
- `header.stamp = Time(sec=0)` → JTC interprète "démarrer à la réception"
- `time_from_start = max(dt×1.2, 40ms)` → fenêtre d'exécution serrée

### `sim_target_node.py` — Simulation tapis roulant

Simule un objet (sphère rouge) se déplaçant sur un tapis roulant et synchronise
sa position dans Gazebo avec l'état logique du scénario.

| Paramètre | Valeur |
|-----------|--------|
| Position X tapis | 0.55 m (face au robot, bras tendu) |
| Y entrée objet | −0.45 m |
| Y sortie objet | +0.45 m |
| Y zone de saisie | 0.0 m |
| Z centre sphère | 0.09 m (top tapis 0.05 + rayon 0.04) |
| Vitesse tapis | 0.04 m/s |
| Pause zone saisie | jusqu'à 60 s |
| Point de dépôt | (0.0, −0.55, 0.09) m |

**Déplacement sphère Gazebo** — deux approches par ordre de priorité :
1. **`ignition.transport` Python** (bindings in-process, même partition transport que Gazebo — fiable)
2. **Subprocess `ign service`** (fallback, timeout 2000 ms)

### `vision_node.py` — Détection YOLO + Interface PLC (Phase 5)

Nœud de vision pour le robot **réel**. S'abonne au flux caméra ROS2, détecte les objets
cibles avec YOLOv8, calcule leurs coordonnées dans le repère monde et envoie le résultat
directement à un **automate Siemens** (API S7) via le protocole **snap7**.

- **Abonnement** : `/camera/image_raw` (Image)
- **Publication** : `/yolo/detections` (Image annotée, 30 Hz)
- **Modèle YOLO** : `yolov8n.pt` (classes COCO utilisées : 32=ball, 47=cup, 49=orange)
- **Calcul de position monde** :

  ```
  x_cam = (cx_px − 320) × z_dist / f        (f = 554.25 px, z_dist = 0.75 m)
  y_cam = (cy_px − 240) × z_dist / f
  world_x = cam_x − y_cam                   (cam_x = 0.55 m)
  world_y = cam_y − x_cam                   (cam_y = 0.0 m)
  world_z = 0.05 m                           (hauteur tapis fixe)
  ```

- **Écriture PLC** : 3 floats big-endian (12 octets) → **DB1, offset 0** (X, Y, Z) via `snap7.client.Client`
- **Reconnexion automatique** : en cas d'échec d'écriture, tentative de reconnexion immédiate

**Prérequis supplémentaires (robot réel)** :

```bash
pip install python-snap7       # bindings snap7
# snap7 doit être compilé sur le système (libsnap7.so)
# PLC Siemens accessible à l'adresse 192.168.0.10 (rack=0, slot=1)
```

> **Note** : ce nœud est conçu pour le robot physique et ne fonctionne pas en simulation
> pure (pas de Gazebo, pas de `/vs/target_pose`). Il remplace la logique de
> `sim_target_node.py` pour fournir la position cible depuis la vision réelle.

### `gazebo_bridge.py` — Initialisation

Envoie la pose home [0, 0.35, 0.3, 0] au JTC au démarrage.

---

## Scénario pick-and-place

```
1. L'objet (sphère rouge, r=4 cm) entre sur le tapis depuis y=−0.45 m
2. L'objet se déplace en +Y à 0.04 m/s (le long du tapis x=0.55 m)
3. Quand y ≈ 0.0 m (zone de saisie) → tapis s'arrête 3 s
4. sim_target_node publie la cible (0.55, 0.0, 0.09) sur /vs/target_pose
5. vs_node converge vers la cible : |et| < 2 mm (descente de 6 cm depuis home z=0.15)
6. Maintien 1 s → statut "PICKED" publié → sphère suit le TCP du bras
7. vs_node se dirige vers le dépôt (0.0, −0.55, 0.09) avec status=CARRYING
8. À convergence : maintien 1.5 s → statut "DEPOSITED" publié
9. Nouvel objet sur le tapis depuis y=−0.45 → cycle recommence
```

---

## Topics et interfaces

### Topics publiés

| Topic | Type | Nœud | Description |
|-------|------|------|-------------|
| `/joint_trajectory_controller/joint_trajectory` | `JointTrajectory` | vs_node | Commandes articulaires |
| `/vs/status` | `String` | vs_node | TRACKING / PICKED / DEPOSITED / SINGULAR |
| `/vs/error` | `Vector3` | vs_node | Norme erreur (mm, deg, 0) |
| `/vs/target_pose` | `PoseStamped` | sim_target_node | Position cible pick |
| `/vs/deposit_pose` | `PoseStamped` | sim_target_node | Position dépôt |
| `/vs/target_marker` | `Marker` | sim_target_node | Visualisation RViz (sphère) |

### Topics abonnés

| Topic | Type | Nœud | QoS |
|-------|------|------|-----|
| `/joint_states` | `JointState` | vs_node | **BEST_EFFORT** (obligatoire) |
| `/vs/tcp_pose` | `PoseStamped` | sim_target_node | RELIABLE |
| `/vs/status` | `String` | sim_target_node | RELIABLE |

> **Important** : `joint_state_broadcaster` publie en BEST_EFFORT.
> S'abonner en RELIABLE → aucun message reçu.

---

## Bugs corrigés

### 1. Robot invisible dans Gazebo
**Cause** : Spawn trop tôt, Gazebo pas encore prêt.  
**Fix** : `TimerAction(period=4.0)` avant le spawn.

### 2. Pas de `/joint_states` reçu
**Cause** : QoS mismatch — vs_node s'abonnait en RELIABLE, JSB publie en BEST_EFFORT.  
**Fix** : `QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT)`.

### 3. Singularité permanente (robot ne bouge pas)
**Cause** : joint3 initial = 0 rad → bras alignés → Jacobien de rang insuffisant.  
**Fix** : `initial_value = 0.3 rad` dans le URDF + `self.q = [0, 0.40, 0.3, 0]` dans vs_node.

### 4. Trajectoires rejetées "ends in the past"
**Cause** : `header.stamp = now()` + `time_from_start = 50ms` → trajectoire expirée à la réception.  
**Fix** : `header.stamp = Time(sec=0, nanosec=0)` → JTC interprète "démarrer à la réception" + `time_from_start = 40ms`.

### 5. JTC "Failed to activate" (timeout spawner)
**Cause** : Spawner trop court pour la confirmation `switch_controller`.  
**Fix** : `--controller-manager-timeout 30` + `open_loop_control: true`.

### 6. Objet bloqué dans la zone de saisie (timeout)
**Cause** : Après timeout, l'objet entrait immédiatement de nouveau dans la zone.  
**Fix** : Timeout 60 s + saut `PICK_ZONE_Y + 0.10` après timeout.

### 7. Paramètres numériques traités comme string
**Cause** : `DeclareLaunchArgument` sans `value_type=float`.  
**Fix** : `value_type=float` pour gain, target_x/y/z.

### 8. Mauvais noms de joints
**Cause** : Héritage Phase 3.  
**Fix** : `JOINT_NAMES = ['joint_1theta', 'joint_1z', 'joint3', 'joint4']`.

### 9. Sphère spawne à (0, 0, 0) au lieu de la position voulue
**Cause** : En Ignition Fortress, `ros_gz_sim create` peut ignorer le tag `<pose>` interne au SDF.  
**Fix** : Toujours passer `-x -y -z` comme arguments CLI de `create` (prioritaires sur le SDF).

### 10. Sphère statique / kinematic ignorée par `set_pose`
**Cause** : En Ignition Fortress + DART, `set_pose` sur les corps `static=true` ou `kinematic=true` est silencieusement ignoré.  
**Fix** : Corps **dynamique** (ni static, ni kinematic) + `<collision>` → la sphère repose sur le tapis par contact physique et répond à `set_pose`.

### 11. `<gravity>false</gravity>` ignoré par DART
**Cause** : Le moteur physique DART (défaut Ignition Fortress) ne supporte pas la désactivation de gravité par lien. La balise est ignorée → la sphère tombait à travers le sol.  
**Fix** : Corps dynamique avec collision (sphère r=0.04 m, tapis statique avec collision) — la physique de contact maintient la sphère sur le tapis.

### 12. `set_pose` subprocess timeout systématique
**Cause** : `ign service` lancé en subprocess ne peut pas atteindre le réseau de transport Ignition de Gazebo (isolation de découverte multicast dans le container Docker).  
**Fix** : Utiliser les **bindings Python `ignition.transport`** directement dans le processus `sim_target_node` — même partition transport que Gazebo → communication garantie. Subprocess conservé en fallback avec timeout 2000 ms.

### 13. Robot qui se réinitialise en boucle (cible changeante)
**Cause** : Chaque publication sur `/vs/target_pose` réinitialisait le flag `_converged`, même si la cible ne bougeait que de quelques mm (bruit de publication).  
**Fix** : `_cb_target_pose` ne réinitialise `_converged` que si la cible saute de plus de 50 mm.

---

## Synchronisation macOS → Linux

Les fichiers sont développés sur macOS (`/Users/lorenzo/test/ros2/`) et synchronisés manuellement vers Linux (`~/Documents/project-10/ros2/`).

### Fichiers à synchroniser après chaque modification

```bash
# Depuis macOS vers Linux (exemple avec scp)
scp src/scara_description/urdf/scara_robot.urdf.xacro        linux:~/Documents/project-10/ros2/src/scara_description/urdf/
scp src/scara_moveit_config/config/ros2_controllers.yaml      linux:~/Documents/project-10/ros2/src/scara_moveit_config/config/
scp src/scara_visual_servoing/scara_visual_servoing/vs_node.py linux:~/Documents/project-10/ros2/src/scara_visual_servoing/scara_visual_servoing/
scp src/scara_visual_servoing/scara_visual_servoing/sim_target_node.py linux:~/Documents/project-10/ros2/src/scara_visual_servoing/scara_visual_servoing/
scp src/scara_visual_servoing/launch/gazebo_sim.launch.py     linux:~/Documents/project-10/ros2/src/scara_visual_servoing/launch/
```

### Après synchronisation

```bash
cd ~/Documents/project-10/ros2

# Forcer la détection des fichiers modifiés
find src/scara_description src/scara_moveit_config src/scara_visual_servoing -exec touch {} +

# Rebuild
colcon build --packages-select scara_description scara_moveit_config scara_visual_servoing --symlink-install

source install/setup.bash
```

### Vérifications rapides sur Linux

```bash
# joint3 initial = 0.3 (anti-singularité)
grep -A10 'name="joint3"' src/scara_description/urdf/scara_robot.urdf.xacro | grep initial_value
# Attendu : <param name="initial_value">0.3</param>

# Trajectoires avec stamp=0
grep "Time(sec=0" src/scara_visual_servoing/scara_visual_servoing/vs_node.py
# Attendu : msg.header.stamp = Time(sec=0, nanosec=0)

# JTC open loop
grep "open_loop" src/scara_moveit_config/config/ros2_controllers.yaml
# Attendu : open_loop_control: true
```

---

## Commandes utiles en cours d'exécution

```bash
# Voir les joints en temps réel
ros2 topic echo /joint_states

# Voir le statut VS
ros2 topic echo /vs/status

# Voir l'erreur de convergence (en mm et degrés)
ros2 topic echo /vs/error

# Voir les trajectoires envoyées
ros2 topic echo /joint_trajectory_controller/joint_trajectory

# État des contrôleurs
ros2 control list_controllers

# Envoyer manuellement une trajectoire de test (home)
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [joint_1theta, joint_1z, joint3, joint4],
    points: [{positions: [0.3, 0.30, 0.3, 0.0], time_from_start: {sec: 2}}]}'
```
