%% main_phase1.m
% =========================================================================
%  PHASE 1 — Simulation MATLAB du robot mobile manipulateur 4-DOF
%  Stage : Vision-Based Control for Autonomous Robotic Systems
%  DNIIT / DUT — Superviseur : Dr. Vo Nhu Thanh
% =========================================================================
%
% Ce script exécute séquentiellement toutes les étapes de la Phase 1 :
%
%   ÉTAPE 1 — Initialisation du modèle robot (paramètres DH)
%   ÉTAPE 2 — Validation de la cinématique directe
%   ÉTAPE 3 — Validation de la cinématique inverse
%   ÉTAPE 4 — Analyse de l'espace de travail
%   ÉTAPE 5 — Planification de trajectoires (articulaire + cartésien)
%   ÉTAPE 6 — Simulation de la commande PID en boucle fermée
%   ÉTAPE 7 — Bilan et rapport de la Phase 1
%
% Usage :
%   >> main_phase1          % Exécution complète
%   >> main_phase1(false)   % Sans affichage des figures

function main_phase1(show_plots)

    if nargin < 1
        show_plots = true;
    end

    clc;
    fprintf('=========================================================\n');
    fprintf('  PHASE 1 — Simulation MATLAB : Mobile Manipulator 4-DOF \n');
    fprintf('=========================================================\n\n');

    %% --- Ajout du dossier courant au path MATLAB ---
    addpath(fileparts(mfilename('fullpath')));

    % =====================================================================
    %  ÉTAPE 1 — Initialisation du modèle robot
    % =====================================================================
    print_step(1, 'Initialisation du modèle robot (paramètres DH)');

    robot = robot_parameters();

    fprintf('  Robot         : %s\n', robot.name);
    fprintf('  Topologie     : θ1(R) — d2(P) — θ3(R) — θ4(R)\n');
    fprintf('  Segments      : a2=%.0fmm, a3=%.0fmm, d3_max=%.0fmm, d4=%.0fmm\n', ...
            robot.a2*1000, robot.a3*1000, robot.d3*1000, robot.d4*1000);
    fprintf('  Portée max XY : %.0f mm\n', (robot.a2 + robot.a3)*1000);
    fprintf('  Limites       : θ1∈[%.0f°,%.0f°], d2∈[%.0f,%.0f]mm, θ3∈[%.0f°,%.0f°]\n', ...
            rad2deg(robot.q_min(1)), rad2deg(robot.q_max(1)), ...
            robot.q_min(2)*1000, robot.q_max(2)*1000, ...
            rad2deg(robot.q_min(3)), rad2deg(robot.q_max(3)));
    fprintf('\n');

    % =====================================================================
    %  ÉTAPE 2 — Cinématique directe
    % =====================================================================
    print_step(2, 'Validation de la cinématique directe');

    % Configurations de test SCARA : q = [θ1(rad), d2(m), θ3(rad), θ4(rad)]
    test_configs = {
        [0,          0.100,  0,          0       ], 'Config zéro (bras tendu)';
        [pi/4,       0.100,  pi/3,      -pi/12  ], 'Config intermédiaire';
        [pi/2,       0.050, -pi/3,       0      ], 'Config étendue gauche';
        [deg2rad(90),0.080, deg2rad(90),-deg2rad(180)], 'Limites θ max';
    };

    fprintf('  %-30s | Position effecteur (mm)\n', 'Configuration');
    fprintf('  %s\n', repmat('-', 1, 60));
    for i = 1:size(test_configs, 1)
        q_test = test_configs{i, 1};
        label  = test_configs{i, 2};
        T = forward_kinematics(robot, q_test);
        p = T(1:3, 4) * 1000;   % en mm
        fprintf('  %-32s | [%7.2f, %7.2f, %7.2f] mm\n', label, p(1), p(2), p(3));
    end
    fprintf('\n');

    % =====================================================================
    %  ÉTAPE 3 — Cinématique inverse
    % =====================================================================
    print_step(3, 'Validation de la cinématique inverse');

    % Test aller-retour FK → IK → FK
    % q_true = [θ1(rad), d2(m), θ3(rad), θ4(rad)]
    q_true = [deg2rad(45), 0.080, deg2rad(30), deg2rad(-75)];
    T_des  = forward_kinematics(robot, q_true);
    q0     = [0, 0.100, 0, 0];   % Point de départ différent

    fprintf('  Config vraie : θ1=%.1f°  d2=%.0fmm  θ3=%.1f°  θ4=%.1f°\n', ...
            rad2deg(q_true(1)), q_true(2)*1000, ...
            rad2deg(q_true(3)), rad2deg(q_true(4)));
    fprintf('  Pose cible   : Px=%.2fmm  Py=%.2fmm  Pz=%.2fmm\n', ...
            T_des(1,4)*1000, T_des(2,4)*1000, T_des(3,4)*1000);

    [q_sol, success, err] = inverse_kinematics(robot, T_des, q0);

    if success
        T_sol = forward_kinematics(robot, q_sol);
        pos_err = norm(T_sol(1:3,4) - T_des(1:3,4)) * 1000;
        fprintf('  Solution IK  : θ1=%.1f°  d2=%.0fmm  θ3=%.1f°  θ4=%.1f°\n', ...
                rad2deg(q_sol(1)), q_sol(2)*1000, ...
                rad2deg(q_sol(3)), rad2deg(q_sol(4)));
        fprintf('  Erreur résiduelle : %.4f mm  ✓\n', pos_err);
    else
        fprintf('  ⚠ Hors espace de travail (err = %.4e m)\n', err);
    end
    fprintf('\n');

    % =====================================================================
    %  ÉTAPE 4 — Analyse de l'espace de travail
    % =====================================================================
    print_step(4, 'Analyse de l''espace de travail');

    if show_plots
        workspace_analysis(robot, 30000);
    else
        fprintf('  (visualisation désactivée)\n');
    end
    fprintf('\n');

    % =====================================================================
    %  ÉTAPE 5 — Planification de trajectoire
    % =====================================================================
    print_step(5, 'Planification de trajectoires');

    T_total = 3.0;   % Durée (s)
    dt      = 0.01;  % Pas de temps (s)

    % q = [θ1(rad), d2(m), θ3(rad), θ4(rad)]
    q_start = [0,          0.100, 0,           0       ];
    q_end   = [deg2rad(60), 0.050, deg2rad(45), deg2rad(-105)];

    fprintf('  Trajectoire articulaire (profil quintic) :\n');
    fprintf('    q_start : θ1=%.1f°  d2=%.0fmm  θ3=%.1f°  θ4=%.1f°\n', ...
            rad2deg(q_start(1)), q_start(2)*1000, rad2deg(q_start(3)), rad2deg(q_start(4)));
    fprintf('    q_end   : θ1=%.1f°  d2=%.0fmm  θ3=%.1f°  θ4=%.1f°\n', ...
            rad2deg(q_end(1)), q_end(2)*1000, rad2deg(q_end(3)), rad2deg(q_end(4)));
    fprintf('    Durée   = %.1f s,  dt = %.3f s\n', T_total, dt);

    traj_joint = joint_space_trajectory(q_start, q_end, T_total, dt, 'quintic');

    % Trajectoire cartésienne
    T_start_cart = forward_kinematics(robot, q_start);
    T_end_cart   = forward_kinematics(robot, q_end);
    p_start_cart = T_start_cart(1:3, 4);
    p_end_cart   = T_end_cart(1:3, 4);
    R_start_cart = T_start_cart(1:3, 1:3);
    R_end_cart   = T_end_cart(1:3, 1:3);

    fprintf('  Trajectoire cartésienne linéaire :\n');
    fprintf('    Départ  = [%s] mm\n', num2str(p_start_cart'*1000, '%.2f '));
    fprintf('    Arrivée = [%s] mm\n', num2str(p_end_cart'*1000, '%.2f '));

    traj_cart = cartesian_trajectory(robot, p_start_cart, p_end_cart, ...
                                     R_start_cart, R_end_cart, T_total, dt);

    if show_plots
        plot_trajectory(traj_joint);
        plot_trajectory(traj_cart);
        animate_robot(robot, traj_joint);
    end
    fprintf('\n');

    % =====================================================================
    %  ÉTAPE 6 — Simulation commande PID
    % =====================================================================
    print_step(6, 'Simulation de la commande PID en boucle fermée');

    % Gains PID SCARA — axes très différents (rotation/translation/rotation/rotation)
    % q2 est prismatique (unité N, pas N.m) → gains numériquement différents
    Kp = [60,  20,  40,  20];
    Ki = [4,   2,   3,   1 ];
    Kd = [8,   1,   5,   2 ];

    fprintf('  Gains PID :\n');
    fprintf('    Kp = [%s]\n', num2str(Kp));
    fprintf('    Ki = [%s]\n', num2str(Ki));
    fprintf('    Kd = [%s]\n', num2str(Kd));

    result_pid = pid_simulation(robot, traj_joint, Kp, Ki, Kd);

    if show_plots
        % résultats déjà affichés dans pid_simulation
    end

    % Vérification des critères de réussite
    % Critère : RMSE < 0.5° pour les axes rotoïdes (1,3,4), < 0.5mm pour d2
    rmse_deg = rad2deg(result_pid.rmse);
    rmse_check = [rmse_deg(1), result_pid.rmse(2)*1000, rmse_deg(3), rmse_deg(4)];
    all_ok = rmse_check(1) < 0.5 && rmse_check(2) < 0.5 && ...
             rmse_check(3) < 0.5 && rmse_check(4) < 0.5;
    if all_ok
        fprintf('\n  ✓ Critère de suivi satisfait (RMSE < 0.5° sur tous les axes)\n');
    else
        fprintf('\n  ⚠ Certains axes dépassent le seuil de 0.5° — ajuster les gains.\n');
    end
    fprintf('\n');

    % =====================================================================
    %  ÉTAPE 7 — Bilan Phase 1
    % =====================================================================
    print_step(7, 'Bilan de la Phase 1');

    fprintf('  Résumé des validations :\n');
    fprintf('    [OK] Modèle DH du robot 4-DOF défini\n');
    fprintf('    [OK] Cinématique directe : calcul matriciel DH\n');
    if success
        fprintf('    [OK] Cinématique inverse : convergence DLS (err = %.4f mm)\n', ...
                norm(T_sol(1:3,4) - T_des(1:3,4))*1000);
    else
        fprintf('    [KO] Cinématique inverse : non convergée — vérifier la pose cible\n');
    end
    fprintf('    [OK] Espace de travail visualisé\n');
    fprintf('    [OK] Trajectoire articulaire et cartésienne générées\n');
    if all_ok
        fprintf('    [OK] Contrôleur PID : critère de suivi satisfait\n');
    else
        fprintf('    [KO] Contrôleur PID : gains à re-régler\n');
    end
    fprintf('\n=========================================================\n\n');
end

%% -----------------------------------------------------------------------
function print_step(n, title)
    fprintf('─────────────────────────────────────────────────────────\n');
    fprintf('  Étape %d — %s\n', n, title);
    fprintf('─────────────────────────────────────────────────────────\n');
end
