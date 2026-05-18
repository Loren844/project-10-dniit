# MATLAB — Simulation robot SCARA 4-DOF

Ce dossier contient toute la simulation MATLAB du projet, découpée en deux phases.

```
matlab/
├── phase1/    ← Modélisation, cinématique, trajectoires, simulation PID
└── phase3/    ← Validation en boucle fermée du contrôleur PBVS
```

**Prérequis** : MATLAB R2021b ou supérieur. Aucune toolbox requise — tout est implémenté from scratch (pas de Robotics Toolbox).

---

## Phase 1 — Modélisation et simulation du robot

### Objectif

Construire de zéro le modèle complet du SCARA 4-DOF : cinématique directe/inverse, analyse du workspace, génération de trajectoires, simulation PID en boucle fermée, animation 3D.

### Lancer

```matlab
cd matlab/phase1
clear                % vider le cache des fonctions MATLAB
main_phase1          % exécution complète : 7 étapes + rapport console + figures
main_phase1(false)   % console seulement, sans figures
```

Durée typique : ~5 secondes. Génère 8 figures + rapport console structuré.

### Modèle robot (DH)

Topologie : **θ1 (R) → d2 (P) → θ3 (R) → θ4 (R)**

| Lien | θ | a [m] | α [°] | d [m] |
|------|---|-------|-------|-------|
| 1 | θ1 var | 0 | 0° | 0 |
| 2 | 0° fixe | **0.300** | 0° | d2 var |
| 3 | θ3 var | **0.160** | 180° | −0.150 |
| 4 | θ4 var | 0 | 0° | 0.059 |

Butées articulaires :

| Joint | Min | Max |
|---|---|---|
| θ1 | −135° | +135° |
| d2 | 0 mm | 200 mm |
| θ3 | −90° | +90° |
| θ4 | −180° | +180° |

### Fichiers

#### `robot_parameters.m`
Définit la structure `robot` complète utilisée par tous les autres scripts.

Contient : paramètres DH (a, alpha, d, theta_offset), limites articulaires, gains PID par axe, paramètres dynamiques (inertie effective `J_eff`, frottement visqueux `B_vis`).

```matlab
robot = robot_parameters();
% robot.a2, robot.a3, robot.d3, robot.d4
% robot.q_min, robot.q_max
% robot.Kp, robot.Ki, robot.Kd  (4×1 vecteurs)
```

#### `forward_kinematics.m`
Cinématique directe par composition des matrices DH.

Calcule la position et l'orientation de l'effecteur pour une configuration `q = [θ1, d2, θ3, θ4]`.

Formules analytiques :
```
Px = a2·cos(θ1) + a3·cos(θ1+θ3)
Py = a2·sin(θ1) + a3·sin(θ1+θ3)
Pz = d2 − d3 − d4
```

```matlab
[T_end, T_all] = forward_kinematics(robot, q)
% T_end : matrice homogène 4×4 effecteur→base
% T_all : {T01, T02, T03, T04} (toutes les transformations intermédiaires)
```

#### `inverse_kinematics.m`
Cinématique inverse analytique — formules fermées (pas d'itération).

1. cos(θ3) = (Px² + Py² − a2² − a3²) / (2·a2·a3) → 2 solutions coude haut/bas
2. θ1 = atan2(...) depuis les deux solutions θ3
3. θ4 = −θ1 − θ3 (orientation finale maintenue à 0°)
4. d2 = Pz + d3 + d4

Erreur résiduelle = 0.0000 mm (solution exacte).

```matlab
[q_sol, success, err_mm] = inverse_kinematics(robot, T_des, q0)
% q_sol : [θ1, d2, θ3, θ4] en rad/m
% success : bool
% err_mm : erreur de position résiduelle en mm
```

#### `workspace_analysis.m`
Visualise l'espace de travail par Monte Carlo (30 000 configurations aléatoires).

Résultats attendus :
- **XY** : anneau toroïdal r ∈ [340, 460] mm (zone morte si |θ3| > 90°)
- **Z** : plan horizontal variable z ∈ [−209, −9] mm

Génère 4 figures : vue 3D, projection XY, projection XZ, structure SCARA.

```matlab
workspace_analysis(robot)
workspace_analysis(robot, 50000)  % plus d'échantillons
```

#### `joint_space_trajectory.m`
Trajectoire point-à-point dans l'espace articulaire.

Trois profils disponibles :

| Profil | Continuité | Usage |
|---|---|---|
| `cubic` | C¹ (vitesse) | Standard, simple |
| `quintic` | C² (accélération) | Mouvement le plus doux |
| `trapezoidal` | C⁰ | Proche variateurs industriels |

```matlab
traj = joint_space_trajectory(q_start, q_end, T_total, dt, 'quintic')
% traj.q    : (N×4) positions
% traj.dq   : (N×4) vitesses
% traj.ddq  : (N×4) accélérations
```

#### `cartesian_trajectory.m`
Trajectoire linéaire dans l'espace cartésien (l'effecteur suit une ligne droite).

- Position : interpolation linéaire avec lissage cubique
- Orientation : interpolation SLERP (Spherical Linear Interpolation)
- IK appelée à chaque point pour obtenir les angles articulaires

```matlab
traj = cartesian_trajectory(robot, p_start, p_end, R_start, R_end, T_total, dt)
```

#### `plot_trajectory.m`
Visualise une trajectoire générée.

- Trajectoire **articulaire** : 3 subplots (positions °, vitesses °/s, accélérations °/s²)
- Trajectoire **cartésienne** : coordonnées XYZ vs temps + chemin 3D

```matlab
plot_trajectory(traj)
```

#### `pid_simulation.m`
Simulation boucle fermée avec contrôleur PID + feedforward sur chaque axe.

Modèle dynamique simplifié (1er ordre par axe) :
```
J_eff · q̈ = τ − B_vis · q̇
```

Loi de commande :
```
τ = J_eff·q̈_ref + B_vis·q̇_ref   (feedforward)
  + Kp·e + Ki·∫e·dt + Kd·(q̇_ref − q̇)   (PID)
```

Protections : anti-windup, saturation couple, butées articulaires.  
Intégration par méthode de Heun (RK2).

```matlab
result = pid_simulation(robot, traj, Kp, Ki, Kd)
% result.q_actual   : positions simulées
% result.e          : erreurs de suivi
% result.rmse       : RMSE par axe
```

Résultats Phase 1 : RMSE < 0.5° (axes rotoïdes), < 0.5 mm (axe prismatique).

#### `animate_robot.m`
Animation 3D du SCARA le long d'une trajectoire.

Reconstruit les segments physiques (colonne verticale, bras horizontaux) pour un rendu réaliste. Affiche la trace de l'effecteur.

```matlab
animate_robot(robot, traj)
animate_robot(robot, traj, 'speed', 2)      % 2× plus rapide
animate_robot(robot, traj, 'trail', false)  % sans trace
```

#### `main_phase1.m`
Orchestre les 7 étapes dans l'ordre avec rapport console structuré :

1. Chargement paramètres robot
2. Test cinématique directe (validation formules analytiques)
3. Test cinématique inverse (erreur résiduelle = 0 mm)
4. Analyse workspace (Monte Carlo)
5. Génération trajectoire articulaire quintic
6. Simulation PID boucle fermée
7. Animation 3D

### Flux de données

```
robot_parameters
   ├── forward_kinematics → workspace_analysis
   │         └── inverse_kinematics ← cartesian_trajectory
   ├── joint_space_trajectory → pid_simulation → animate_robot
   └── main_phase1  (orchestre tout)
```

---

## Phase 3 — Validation MATLAB du contrôleur PBVS

### Objectif

Valider en simulation MATLAB la convergence de la boucle d'asservissement visuel PBVS avant déploiement. Tester les cas difficiles (tapis roulant, compensation Kalman, gain adaptatif).

Ce script est **complémentaire** à la simulation Python (`python/phase3/simulation_gui.py`) : il permet de valider les mêmes algorithmes dans l'environnement MATLAB avec un affichage de convergence détaillé.

### Lancer

```matlab
cd matlab/phase3
% Ajouter phase1 au path (IK, FK partagés)
addpath('../phase1')
main_phase3
```

Durée typique : ~2 secondes. Génère 6 figures de convergence.

### Configurer la simulation (en tête du script)

| Paramètre | Défaut | Description |
|---|---|---|
| `dt` | 0.033 s | Pas de temps (≈ 30 Hz) |
| `T_sim` | 15 s | Durée maximale |
| `lambda_nom` | 0.5 | Gain nominal VS |
| `lambda_min/max` | 0.05 / 2.0 | Bornes du gain adaptatif |
| `adaptive` | true | Gain sigmoïde selon ‖e‖ |
| `thr_t_mm` | 2.0 mm | Seuil convergence position |
| `thr_r_deg` | 1.0° | Seuil convergence orientation |
| `q0` | [0.5, 0.10, −0.30, 0.20] | Configuration initiale (rad/m) |
| `t_des` | [0.350, 0.050, −0.150] m | Position cible |
| `use_conveyor` | false | Activer le tapis roulant |
| `v_conveyor` | [0.05, 0, 0] m/s | Vitesse du tapis |

### Fichier

#### `main_phase3.m`
Script autonome (toutes les fonctions en bas du fichier ou importées depuis `phase1/`).

Étapes de la simulation :
1. Chargement paramètres robot (depuis `phase1/robot_parameters.m`)
2. Initialisation contrôleur PBVS (gain adaptatif, matrice d'interaction)
3. Boucle de simulation (FK → erreur → commande VS → saturation → intégration)
4. Si `use_conveyor = true` : la cible se déplace + compensation Kalman (+150 ms)
5. Affichage 6 figures : convergence e_t/e_r, gain adaptatif, trajectoire XY, configurations articulaires, commandes dq

**Note sur la convention DH** : avec la table DH du SCARA (link 3 : α = 180°), la FK produit toujours R avec trace(R) = −1 quelle que soit la configuration. L'erreur de rotation est donc calculée dans un repère adapté à cette convention (voir commentaires dans le script).

### Résultats attendus

| Métrique | Valeur |
|---|---|
| Convergence position ‖e_t‖ < 2 mm | En ~30 itérations (≈ 1 s à 30 Hz) |
| Convergence orientation ‖e_r‖ < 1° | Simultanée |
| Compensation tapis roulant (Kalman) | Erreur résiduelle < 5 mm à v = 50 mm/s |
| Pas de dépassement des butées articulaires | Garanti par saturation |
