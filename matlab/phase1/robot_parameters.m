%% robot_parameters.m
% Définition des paramètres du robot SCARA 4-DOF
% Basé sur le rapport PBL6 SCARA (DNIIT / DUT) — Dr. Vo Nhu Thanh
%
% Topologie SCARA : 3 liaisons rotoïdes (θ1, θ3, θ4) + 1 liaison prismatique (d2)
%
% Convention DH (Denavit-Hartenberg) standard :
%   T_i = Rz(θ_i) · Tz(d_i) · Tx(a_i) · Rx(α_i)
%
% Table DH (issue du rapport, Table 2) :
%
%  Lien | θ_i (var) | a_i (m) | α_i (°) | d_i (m)
%  -----|-----------|---------|---------|--------
%    1  |   θ1      |   0     |   0°    |   0
%    2  |   0°      |  a2     |   0°    |   d2*  (prismatique, variable)
%    3  |   θ3      |  a3     | 180°    |  -d3
%    4  |   θ4      |   0     |   0°    |   d4
%
% Dimensions réelles du robot (Table 1 du rapport) :
%   Lien 1 : 300 mm (= a2)
%   Lien 2 : 160 mm (= a3)
%   Lien 3 : 150 mm (= d3, offset cinématique DH – porte verticale)
%   Lien 4 :  59 mm (= d4)

function robot = robot_parameters()

    robot.name     = 'SCARA Robot 4-DOF';
    robot.n_joints = 4;

    %% --- Longueurs des segments (m) ---
    robot.a2 = 0.300;   % Longueur liaison 1 → 2 (bras supérieur)
    robot.a3 = 0.160;   % Longueur liaison 2 → 3 (avant-bras)
    robot.d3 = 0.150;   % Offset cinématique DH lien 3 (portée verticale, m)
    robot.d4 = 0.059;   % Offset vertical effecteur (m)

    % d2_base : hauteur de la colonne (offset vertical de référence)
    robot.d2_base = 0.200;   % valeur nominale en position haute (m)

    %% --- Table DH : [a (m), alpha (rad), d (m), theta_offset (rad)]
    % Pour les liaisons rotoïdes  : d est fixe, θ est variable
    % Pour la liaison prismatique : θ = 0°,      d est variable (→ géré dans FK)
    % Codage : chaque ligne = [a, alpha, d_nominal, theta_offset]
    robot.DH = [
        0,          0,       0,          0;   % Lien 1 : θ1 var, d1=0, a1=0, α1=0°
        robot.a2,   0,       robot.d2_base, 0; % Lien 2 : θ2=0°, d2 var (prismatique), α2=0°
        robot.a3,   pi,     -robot.d3,   0;   % Lien 3 : θ3 var, d3 fixe, α3=180°
        0,          0,       robot.d4,   0;   % Lien 4 : θ4 var, d4 fixe, α4=0°
    ];
    % Colonne d'une table DH standard : [a, alpha, d, theta_offset]

    %% --- Type de chaque liaison ('R' = rotoïde, 'P' = prismatique) ---
    robot.joint_type = ['R', 'P', 'R', 'R'];

    %% --- Limites articulaires ---
    % q = [θ1(rad), d2(m), θ3(rad), θ4(rad)]
    robot.q_min = [deg2rad(-135),  0.000, deg2rad(-90),  deg2rad(-180)];
    robot.q_max = [deg2rad( 135),  0.200, deg2rad( 90),  deg2rad( 180)];

    %% --- Vitesses maximales ---
    % [rad/s, m/s, rad/s, rad/s]
    robot.dq_max = [2.0, 0.1, 2.0, 2.0];

    %% --- Masses des segments (kg) ---
    robot.mass = [0.5, 0.4, 0.3, 0.1];

    %% --- Modèle dynamique (pour la simulation PID) ---
    % Inertie effective par axe [kg·m², kg, kg·m², kg·m²]
    robot.J_eff   = [0.08, 2.0, 0.04, 0.01];
    % Amortissement visqueux [N·m·s/rad, N·s/m, N·m·s/rad, N·m·s/rad]
    robot.B_vis   = [0.5,  5.0, 0.3,  0.1];
    % Couple/force max [N·m, N, N·m, N·m]
    robot.tau_max = [2.0,  30,  1.5,  0.5];

end
