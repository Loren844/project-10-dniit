%% main_phase3.m
% Script principal — Phase 3 : Asservissement visuel PBVS en simulation MATLAB
%
% Objectif : valider en simulation la convergence de la boucle d'asservissement
% visuel sur le modèle SCARA 4-DOF avant déploiement sur le robot réel.
%
% Structure de la simulation :
%   1. Modèle FK/IK (Phase 1)
%   2. Générateur de pose cible (objet sur tapis roulant)
%   3. Contrôleur PBVS (loi de commande + Jacobienne)
%   4. Boucle fermée avec saturation + butées articulaires
%   5. Tracé des résultats
%
% Usage :
%   >> main_phase3
%   ou pour configurer :
%   >> main_phase3  (éditer les paramètres de la section CONFIGURATION)
%
% Dépendances : robot_parameters.m, forward_kinematics.m, inverse_kinematics.m
%               (repertoire matlab/phase1/ — ajouter au path)

clear; close all; clc;
addpath('../phase1');   % accès aux modules Phase 1

fprintf('╔══════════════════════════════════════════════════════╗\n');
fprintf('║  Phase 3 — Asservissement Visuel PBVS  (SCARA 4-DOF) ║\n');
fprintf('╚══════════════════════════════════════════════════════╝\n\n');

%% ─────────────────────────────────────────────────────────────────────────
%% CONFIGURATION
%% ─────────────────────────────────────────────────────────────────────────

robot = robot_parameters();

% --- Paramètres de la simulation ---
dt       = 0.033;     % Pas de temps [s] (≈ 30 Hz, cadence caméra)
T_sim    = 15.0;      % Durée maximale [s]
N        = round(T_sim / dt);

% --- Paramètres du contrôleur PBVS ---
lambda_nom = 0.5;     % Gain nominal
lambda_min = 0.05;    % Gain minimum (éviter oscillations)
lambda_max = 2.0;     % Gain maximum
adaptive   = true;    % Gain adaptatif selon ‖e‖

% --- Seuils de convergence ---
thr_t_mm  = 2.0;      % Erreur position max [mm]
thr_r_deg = 1.0;      % Erreur rotation max [°]

% --- Configuration initiale du robot ---
q0 = [0.5,  0.10, -0.30,  0.20];   % [θ1(rad), d2(m), θ3(rad), θ4(rad)]

% --- Pose désirée (objet sur tapis, coordonnées repère robot) ---
% r = sqrt(350^2 + 50^2) ≈ 354 mm → dans l'anneau [340, 460] mm ✓
% d2 = pz + d3 + d4 = -0.150 + 0.150 + 0.059 = 0.059 m ✓
t_des = [0.350; 0.050; -0.150];     % [m]

% IMPORTANT : R_des = eye(3) n'est PAS atteignable avec la convention DH
% de ce SCARA (link 3 : α=π → Rx(π) baked in).
% La FK exacte produit toujours trace(R) = -1 quel que soit q.
% On calcule R_des via IK+FK pour obtenir l'orientation réellement atteignable.
T_ik_input = [eye(3), t_des; 0 0 0 1];   % L'IK n'utilise que t_des
[q_des_ik, ik_ok, ~] = inverse_kinematics(robot, T_ik_input, q0);
if ~ik_ok
    error('IK échouée pour t_des — position hors espace de travail ?');
end
T_fk_des = forward_kinematics(robot, q_des_ik);
R_des = T_fk_des(1:3, 1:3);
fprintf('  R_des calculé depuis IK+FK (trace=%.2f, attendu ≈1)····', trace(R_des));

% --- Mode tapis roulant (objet en mouvement) ---
use_conveyor = false;
v_conveyor   = [0.05; 0.0; 0.0];    % Vitesse tapis [m/s] (selon X)

% --- Latence de compensation Kalman ---
latency_s = 0.150;    % [s]

fprintf('Configuration :\n');
fprintf('  dt = %.1f ms | T_sim = %.1f s | N = %d itérations\n', ...
        dt*1e3, T_sim, N);
fprintf('  q0 = [%.2f rad, %.1f mm, %.2f rad, %.2f rad]\n', ...
        q0(1), q0(2)*1e3, q0(3), q0(4));
fprintf('  t_des = [%.1f, %.1f, %.1f] mm\n', t_des*1e3);
fprintf('  λ = %.2f (adaptatif: %d)\n\n', lambda_nom, adaptive);

%% ─────────────────────────────────────────────────────────────────────────
%% INITIALISATION
%% ─────────────────────────────────────────────────────────────────────────

% Tableaux d'historique
hist_t    = zeros(3, N);   % Positions effecteur
hist_q    = zeros(4, N);   % Configurations
hist_et   = zeros(1, N);   % Norme erreur translation (mm)
hist_er   = zeros(1, N);   % Norme erreur rotation (°)
hist_dq   = zeros(4, N);   % Vitesses articulaires
hist_lam  = zeros(1, N);   % Gain appliqué
hist_tdes = zeros(3, N);   % Pose cible (peut bouger avec tapis)

q   = q0(:);               % Configuration courante (colonne)
converged   = false;
iter_conv   = N;
t_sim_total = 0;

%% ─────────────────────────────────────────────────────────────────────────
%% BOUCLE DE SIMULATION
%% ─────────────────────────────────────────────────────────────────────────

fprintf('Début simulation...\n');
tic;

for k = 1:N

    t_sim_total = k * dt;

    %% — Mise à jour de la cible (tapis roulant) ———————————————————————————
    if use_conveyor
        % Position prédite à t + latence (compensation Kalman)
        t_target = t_des + v_conveyor * (t_sim_total + latency_s);
    else
        t_target = t_des;
    end

    %% — FK : position courante de l'effecteur ——————————————————————————
    T_cur = forward_kinematics(robot, q');
    t_cur = T_cur(1:3, 4);
    R_cur = T_cur(1:3, 1:3);

    %% — Calcul de l'erreur visuelle PBVS ——————————————————————————————
    e_t = t_cur - t_target;                 % Erreur de translation (m)

    % Convention PBVS correcte pour ce SCARA (lien 3 : α=π) :
    %   R_co = R_cur^T · R_des  ("de courant vers désiré")
    % ⇒ e_rz = (θ1+θ3-θ4) - cst_des  et  d(e_rz)/dt = ω_z  (jacobienne)
    % ⇒ loi  v_c = -λ e  donne  d(e_rz)/dt = -λ e_rz (convergence)
    %
    % ATTENTION : l'ordre inversé (R_des'*R_cur) donne d(e_rz)/dt = +λ e_rz
    %             ce qui diverge — c'est le bug originel.
    R_co = R_cur' * R_des;

    % Ce SCARA ne peut tourner que autour de Z : R_co est toujours Rz(α).
    % Extraction directe du lacet — valide pour tout α ∈ (-π, π], sans
    % singularité à θ=π (contrairement à la représentation axe-angle).
    e_rz = atan2(R_co(2,1), R_co(1,1));    % = angle de Rz directement
    e_r  = [0; 0; e_rz];

    norm_t_mm  = norm(e_t) * 1e3;
    norm_r_deg = norm(e_r) * (180/pi);

    %% — Gain adaptatif —————————————————————————————————————————————————
    % Grande erreur → grand gain (convergence rapide)
    % Petite erreur → petit gain (approche précise, sans overshoot)
    % Point d'inflexion à 50 mm
    e_norm = norm([e_t; e_r]);
    if adaptive
        alpha  = log(2) / 0.05;   % inflexion à 50 mm
        lambda = (lambda_max - lambda_min) * (1 - exp(-alpha * e_norm)) + lambda_min;
        lambda = min(max(lambda, lambda_min), lambda_max);
    else
        lambda = lambda_nom;
    end

    %% — Matrice d'interaction L_s ———————————————————————————————————————
    L_s = interaction_matrix_pbvs(R_co, e_t);
    %% — Vitesse cartésienne commandée ——————————————————————————————————
    e6   = [e_t; e_r];
    Lp   = pinv(L_s);       % pseudo-inverse (L_s ∈ R^(6×6) ici)
    v_c  = -lambda * (Lp * e6);

    % Limiter la vitesse de translation max
    v_norm = norm(v_c(1:3));
    if v_norm > 0.20
        v_c = v_c * (0.20 / v_norm);
    end

    %% — Jacobienne du SCARA + conversion articulaire ———————————————————
    J = scara_jacobian(q, robot);           % (6×4)
    [J_pinv, sigma_min, is_sing] = damped_pinv_mat(J, 0.01, 0.01);

    % Commande principale
    dq_main = J_pinv * v_c;

    % Tâche secondaire : évitement des butées articulaires
    q_mid   = (robot.q_max(:) + robot.q_min(:)) / 2;
    q_range = robot.q_max(:) - robot.q_min(:);
    grad_H  = 2 * (q - q_mid) ./ (q_range.^2);
    N_J     = eye(4) - J_pinv * J;
    dq_sec  = N_J * (-0.5 * grad_H);

    dq_total = dq_main + dq_sec;

    %% — Saturation articulaire ——————————————————————————————————————————
    dq_sat = max(min(dq_total, robot.dq_max(:)), -robot.dq_max(:));

    %% — Intégration Euler ——————————————————————————————————————————————
    q_new = q + dq_sat * dt;
    q_new = max(min(q_new, robot.q_max(:)), robot.q_min(:));

    %% — Enregistrement ——————————————————————————————————————————————————
    hist_q(:, k)    = q;
    hist_t(:, k)    = t_cur;
    hist_et(k)      = norm_t_mm;
    hist_er(k)      = norm_r_deg;
    hist_dq(:, k)   = dq_sat;
    hist_lam(k)     = lambda;
    hist_tdes(:, k) = t_target;

    %% — Critère de convergence ——————————————————————————————————————————
    if norm_t_mm < thr_t_mm && norm_r_deg < thr_r_deg
        fprintf('  ✓ Convergence atteinte à l''itération %d (t=%.2f s)\n', ...
                k, t_sim_total);
        fprintf('    |e_t| = %.3f mm  |e_r| = %.3f°\n', norm_t_mm, norm_r_deg);
        converged  = true;
        iter_conv  = k;
        hist_q     = hist_q(:, 1:k);
        hist_t     = hist_t(:, 1:k);
        hist_et    = hist_et(1:k);
        hist_er    = hist_er(1:k);
        hist_dq    = hist_dq(:, 1:k);
        hist_lam   = hist_lam(1:k);
        hist_tdes  = hist_tdes(:, 1:k);
        break;
    end

    q = q_new;

end

elapsed = toc;

if ~converged
    fprintf('  ✗ Pas de convergence en %d itérations.\n', N);
    fprintf('    Erreur finale : |e_t|=%.2f mm  |e_r|=%.2f°\n', ...
            hist_et(end), hist_er(end));
end

fprintf('\nTemps de calcul : %.3f s (%.1f µs/iter)\n', ...
        elapsed, elapsed/N*1e6);

%% ─────────────────────────────────────────────────────────────────────────
%% AFFICHAGE DES RÉSULTATS
%% ─────────────────────────────────────────────────────────────────────────

time_vec = (1:length(hist_et)) * dt;

figure('Name','Phase 3 — Convergence PBVS','NumberTitle','off', ...
       'Position', [100 100 1100 800]);

%% Fig 1 : Erreurs de convergence
subplot(3,2,[1 2]);
yyaxis left
plot(time_vec, hist_et, 'b-', 'LineWidth', 1.5); hold on;
yline(thr_t_mm, 'b--', sprintf('Seuil %.0f mm', thr_t_mm), ...
      'LabelHorizontalAlignment','left');
ylabel('|e_t| (mm)', 'Color','b');
yyaxis right
plot(time_vec, hist_er, 'r-', 'LineWidth', 1.5);
yline(thr_r_deg, 'r--', sprintf('Seuil %.0f°', thr_r_deg), ...
      'LabelHorizontalAlignment','left');
ylabel('|e_r| (°)', 'Color','r');
xlabel('Temps (s)');
title('Erreur visuelle en fonction du temps');
grid on; legend('|e_t| (mm)','','|e_r| (°)','', 'Location','northeast');

if converged
    xline(iter_conv * dt, 'g--', 'Convergence', 'LabelVerticalAlignment','bottom');
end

%% Fig 2 : Gain adaptatif
subplot(3,2,3);
plot(time_vec, hist_lam, 'k-', 'LineWidth', 1.5);
xlabel('Temps (s)'); ylabel('Gain λ');
title('Gain adaptatif'); grid on;
yline(lambda_min, 'r--'); yline(lambda_max, 'r--');

%% Fig 3 : Trajectoire XY de l'effecteur
subplot(3,2,4);
plot(hist_t(1,:)*1e3, hist_t(2,:)*1e3, 'b-o', 'MarkerSize', 3); hold on;
plot(t_target(1)*1e3, t_target(2)*1e3, 'r*', 'MarkerSize', 12, 'LineWidth', 2);
plot(hist_t(1,1)*1e3, hist_t(2,1)*1e3, 'go', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X (mm)'); ylabel('Y (mm)');
title('Trajectoire effecteur (vue de dessus)');
legend('Trajectoire','Cible','Départ','Location','best');
grid on; axis equal;

%% Fig 4 : Configurations articulaires
subplot(3,2,5);
plot(time_vec, rad2deg(hist_q(1,:)), 'b-', 'LineWidth',1.2); hold on;
plot(time_vec, rad2deg(hist_q(3,:)), 'g-', 'LineWidth',1.2);
plot(time_vec, rad2deg(hist_q(4,:)), 'r-', 'LineWidth',1.2);
plot(time_vec, hist_q(2,:)*1e3,     'm--','LineWidth',1.2);
xlabel('Temps (s)');
ylabel('θ (°) | d2 (mm)');
title('Évolution articulaire');
legend('θ₁ (°)','θ₃ (°)','θ₄ (°)','d₂ (mm)','Location','best');
grid on;

%% Fig 5 : Vitesses articulaires
subplot(3,2,6);
labels_dq = {'dθ₁ (rad/s)','dd₂ (m/s)','dθ₃ (rad/s)','dθ₄ (rad/s)'};
colors_dq = {'b','m','g','r'};
hold on;
for i = 1:4
    plot(time_vec, hist_dq(i,:), 'Color', colors_dq{i}, 'LineWidth', 1.0);
end
xlabel('Temps (s)'); ylabel('Vitesses articulaires');
title('Commandes articulaires');
legend(labels_dq, 'Location','best');
grid on;

sgtitle(sprintf('Asservissement Visuel PBVS — SCARA 4-DOF   (λ adaptatif, convergeé=%d, iter=%d)', ...
        converged, iter_conv), 'FontWeight','bold');

fprintf('\n--- Résumé Phase 3 ---\n');
fprintf('  Convergence : %s\n', mat2str(converged));
fprintf('  Itérations  : %d / %d\n', min(iter_conv, N), N);
fprintf('  Temps sim.  : %.2f s\n', min(iter_conv, N) * dt);
fprintf('  |e_t| final : %.3f mm (seuil %.1f mm)\n', hist_et(end), thr_t_mm);
fprintf('  |e_r| final : %.3f °  (seuil %.1f °)\n',  hist_er(end), thr_r_deg);


%% ─────────────────────────────────────────────────────────────────────────
%% SÉQUENCE PICK-AND-PLACE (continuation après convergence VS)
%% ─────────────────────────────────────────────────────────────────────────

if ~converged
    fprintf('\n[WARN] VS non convergé — séquence pick-and-place ignorée.\n');
else

    fprintf('\n╔═══════════════════════════════════════════╗\n');
    fprintf('║  Séquence Pick-and-Place (post-convergence)║\n');
    fprintf('╚═══════════════════════════════════════════╝\n\n');

    %% --- Paramètres pick-and-place ---
    t_drop        = [0.200; -0.280; -0.130];  % Position de dépose [m]
    lift_height   = 0.080;                    % Hauteur de levée [m]
    place_height  = 0.020;                    % Offset Z au-dessus de la surface de dépose
    t_close_grip  = 0.30;                     % Durée fermeture pince [s]
    t_open_grip   = 0.25;                     % Durée ouverture  pince [s]
    q_home        = [0.0; 0.10; 0.0; 0.0];   % Configuration de veille

    % Noms des états (pour affichage)
    PP_NAMES = {'GRASP_CLOSE','LIFT','TRANSPORT','LOWER','PLACE_OPEN','RETURN','DONE'};
    % Codes numérique :  0=fermeture pince, 1=levée, 2=transport XY,
    %                    3=descente dépose, 4=ouverture pince, 5=retour home, 6=done

    %% --- Positions cibles de chaque étape ---
    q_conv  = hist_q(:, end);            % Config. à la convergence
    T_conv  = forward_kinematics(robot, q_conv');
    t_conv  = T_conv(1:3, 4);           % Position EE à la convergence

    t_lift      = t_conv + [0; 0; lift_height];   % Levée depuis la saisie
    t_transport = [t_drop(1:2); t_lift(3)];        % Transport à hauteur sécurité
    t_lower     = t_drop + [0; 0; place_height];   % Abaissement sur la dépose
    T_home      = forward_kinematics(robot, q_home');
    t_home      = T_home(1:3,4) + [0; 0; lift_height]; % Home à hauteur sécurité

    fprintf('  Saisie    : [%.0f, %.0f, %.0f] mm\n', t_conv*1e3);
    fprintf('  Levée     : z = %.0f mm\n', t_lift(3)*1e3);
    fprintf('  Transport : [%.0f, %.0f] mm (XY)\n', t_drop(1:2)*1e3);
    fprintf('  Dépose    : [%.0f, %.0f, %.0f] mm\n', t_lower*1e3);
    fprintf('  Retour    : home\n\n');

    %% --- Initialisation ---
    N_pp     = 2000;
    q_pp     = q_conv;
    pp_state = 0;        % 0 = GRASP_CLOSE (départ)
    pp_k     = 0;
    pp_t     = 0.0;
    grip_tmr = 0.0;      % timer fermeture/ouverture
    grip_prg = 0.0;      % progression pince (0=ouvert, 1=fermé)

    pp_hist_t  = zeros(3, N_pp);
    pp_hist_q  = zeros(4, N_pp);
    pp_hist_st = zeros(1, N_pp);
    pp_hist_gp = zeros(1, N_pp);

    tic;

    %% --- Boucle pick-and-place ---
    for k = 1:N_pp
        pp_k = pp_k + 1;
        pp_t = pp_k * dt;

        T_c  = forward_kinematics(robot, q_pp');
        t_c  = T_c(1:3, 4);
        R_c  = T_c(1:3, 1:3);
        dq   = zeros(4, 1);

        switch pp_state

            case 0  %% GRASP_CLOSE : fermeture pince (simulation timer)
                grip_tmr = grip_tmr + dt;
                grip_prg = min(1.0, grip_tmr / t_close_grip);
                if grip_tmr >= t_close_grip
                    pp_state = 1;  grip_tmr = 0;
                    fprintf('  [t=%.2fs] PINCE FERMÉE ✓ → LEVÉE\n', pp_t);
                end

            case 1  %% LIFT : VS vers position de levée
                dq = pp_vs_step(q_pp, t_c, R_c, t_lift, R_des, robot, lambda_max);
                if norm(t_c - t_lift)*1e3 < 5.0
                    pp_state = 2;
                    fprintf('  [t=%.2fs] LEVÉE OK → TRANSPORT [%.0f, %.0f] mm\n', ...
                            pp_t, t_drop(1)*1e3, t_drop(2)*1e3);
                end

            case 2  %% TRANSPORT : VS XY vers la zone de dépose
                dq = pp_vs_step(q_pp, t_c, R_c, t_transport, R_des, robot, lambda_max);
                if norm(t_c(1:2) - t_drop(1:2))*1e3 < 5.0
                    pp_state = 3;
                    fprintf('  [t=%.2fs] TRANSPORT OK → ABAISSEMENT z=%.0f mm\n', ...
                            pp_t, t_lower(3)*1e3);
                end

            case 3  %% LOWER : descente vers hauteur de dépose
                dq = pp_vs_step(q_pp, t_c, R_c, t_lower, R_des, robot, lambda_max);
                if norm(t_c - t_lower)*1e3 < 5.0
                    pp_state = 4;  grip_tmr = 0;
                    fprintf('  [t=%.2fs] EN POSITION → OUVERTURE PINCE\n', pp_t);
                end

            case 4  %% PLACE_OPEN : ouverture pince
                grip_tmr = grip_tmr + dt;
                grip_prg = max(0.0, 1 - grip_tmr / t_open_grip);
                if grip_tmr >= t_open_grip
                    pp_state = 5;  grip_tmr = 0;
                    fprintf('  [t=%.2fs] PINCE OUVERTE ✓ → RETOUR HOME\n', pp_t);
                end

            case 5  %% RETURN : retour à la position de veille
                dq = pp_vs_step(q_pp, t_c, R_c, t_home, R_des, robot, lambda_max);
                if norm(t_c(1:2) - t_home(1:2))*1e3 < 15.0
                    pp_state = 6;
                    fprintf('  [t=%.2fs] RETOUR HOME ✓ — CYCLE TERMINÉ\n', pp_t);
                end

            case 6  %% DONE
                break;
        end

        %% Saturation + intégration Euler
        dq     = max(min(dq, robot.dq_max(:)), -robot.dq_max(:));
        q_pp   = max(min(q_pp + dq * dt, robot.q_max(:)), robot.q_min(:));

        pp_hist_t(:, k) = t_c;
        pp_hist_q(:, k) = q_pp;
        pp_hist_st(k)   = pp_state;
        pp_hist_gp(k)   = grip_prg;
    end

    pp_elapsed = toc;
    pp_hist_t  = pp_hist_t(:, 1:pp_k);
    pp_hist_st = pp_hist_st(1:pp_k);
    pp_hist_gp = pp_hist_gp(1:pp_k);
    pp_tvec    = (1:pp_k) * dt;

    fprintf('\n  Durée séquence : %.2f s\n', pp_k * dt);

    %% --- Tracé pick-and-place ---
    figure('Name','Phase 3 — Séquence Pick-and-Place','NumberTitle','off', ...
           'Position',[200 200 1000 700]);

    subplot(2,2,[1 2]);
    % Trajectoire complète : VS + pick-and-place
    all_t = [hist_t, pp_hist_t];
    plot(all_t(1,:)*1e3, all_t(2,:)*1e3, 'b-', 'LineWidth', 1.2); hold on;

    jalons = [t_conv, t_lift, t_transport, t_lower, t_home] * 1e3;
    labels = {'Saisie','Levée','Transport','Dépose','Home'};
    markers_pp = {'rs','m^','cd','gp','ko'};
    for ji = 1:5
        plot(jalons(1,ji), jalons(2,ji), markers_pp{ji}, ...
             'MarkerSize',12,'LineWidth',2,'DisplayName',labels{ji});
    end
    plot(t_des(1)*1e3, t_des(2)*1e3, 'r*','MarkerSize',14,'LineWidth',2, ...
         'DisplayName','Cible VS');
    xlabel('X (mm)'); ylabel('Y (mm)');
    title('Trajectoire complète : asservissement + pick-and-place (vue dessus)');
    legend('Location','best'); grid on; axis equal;

    subplot(2,2,3);
    plot(pp_tvec, pp_hist_t(3,:)*1e3, 'b-','LineWidth',1.5); hold on;
    yline(t_des(3)*1e3,  'r--', 'Saisie Z');
    yline(t_lift(3)*1e3, 'm--', 'Sécurité Z');
    yline(t_drop(3)*1e3, 'g--', 'Dépose Z');
    xlabel('Temps (s)'); ylabel('Z effecteur (mm)');
    title('Profil de hauteur (pick-and-place)'); grid on;

    subplot(2,2,4);
    yyaxis left
    stairs(pp_tvec, pp_hist_st, 'k-','LineWidth',1.5);
    yticks(0:6);
    yticklabels(PP_NAMES);
    ylabel('État séquence');
    yyaxis right
    plot(pp_tvec, pp_hist_gp, 'm-','LineWidth',1.5);
    ylim([-0.1, 1.3]);
    ylabel('Fermeture pince (0=ouvert, 1=fermé)');
    xlabel('Temps (s)');
    title('Machine à états + état pince'); grid on;

    sgtitle('Séquence Pick-and-Place complète — SCARA 4-DOF','FontWeight','bold');

end  % if converged (pick-and-place)


%% ═══════════════════════════════════════════════════════════════════════════
%% FONCTIONS LOCALES
%% ═══════════════════════════════════════════════════════════════════════════

function dq = pp_vs_step(q, t_cur, R_cur, t_target, R_des, robot, lambda)
    % Une étape de VS pour la séquence pick-and-place (réutilise les fonctions locales)
    e_t  = t_cur - t_target;
    R_co = R_cur' * R_des;
    e_rz = atan2(R_co(2,1), R_co(1,1));
    e_r  = [0; 0; e_rz];
    e6   = [e_t; e_r];

    % Gain adaptatif réduit pendant pick-and-place (mouvements lents)
    e_norm = norm(e6);
    lam    = lambda * min(1.0, e_norm / 0.05);   % linéaire entre 0 et lambda_max

    L_s  = interaction_matrix_pbvs(R_co, e_t);
    v_c  = -lam * (pinv(L_s) * e6);
    v_norm = norm(v_c(1:3));
    if v_norm > 0.15, v_c = v_c * (0.15/v_norm); end

    J   = scara_jacobian(q, robot);
    dq  = pinv(J) * v_c;
end



function au = axis_angle_vec(R)
    % Représentation axe-angle θ·u d'une matrice de rotation (3×3)
    trace_val = (trace(R) - 1) / 2;
    trace_val = max(-1.0, min(1.0, trace_val));
    theta = acos(trace_val);

    if theta < 1e-8
        au = zeros(3,1);
        return;
    end

    if abs(theta - pi) < 1e-6
        % θ ≈ π
        diag_vals = [R(1,1); R(2,2); R(3,3)];
        u = sqrt(max(0, (diag_vals + 1) / 2));
        if R(3,2) - R(2,3) < 0, u(1) = -u(1); end
        if R(1,3) - R(3,1) < 0, u(2) = -u(2); end
        if R(2,1) - R(1,2) < 0, u(3) = -u(3); end
        au = theta * u / (norm(u) + 1e-12);
        return;
    end

    skew_vec = [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
    u = skew_vec / (2 * sin(theta));
    au = theta * u;
end


function L = interaction_matrix_pbvs(R_co, t_co)
    % Matrice d'interaction L_s ∈ R^(6×6) pour PBVS
    % (Chaumette & Hutchinson, IEEE RAM 2006)
    theta_u = axis_angle_vec(R_co);
    theta   = norm(theta_u);

    if theta < 1e-9
        u = zeros(3,1);
    else
        u = theta_u / theta;
    end

    su = skew_mat(u);

    % Calcul de L_omega
    sinc_th   = sinc(theta / pi);             % sin(θ)/θ
    sinc2_th2 = sinc(theta / (2*pi));         % sin(θ/2)/(θ/2)

    if abs(sinc2_th2) < 1e-12
        coeff = 0;
    else
        coeff = 1 - sinc_th / sinc2_th2^2;
    end

    L_omega = eye(3) - (theta/2) * su + coeff * (su * su);

    % PBVS : L_s bloc-diagonale, PAS de terme de couplage skew(t_co)
    L = [eye(3),   zeros(3);
         zeros(3), L_omega];
end


function J = scara_jacobian(q, robot)
    % Jacobienne analytique du SCARA 4-DOF, J ∈ R^(6×4)
    t1  = q(1);  t3  = q(3);
    a2  = robot.a2;  a3 = robot.a3;

    s1   = sin(t1);      c1   = cos(t1);
    s13  = sin(t1+t3);   c13  = cos(t1+t3);

    dp_dt1 = [-a2*s1 - a3*s13;  a2*c1 + a3*c13;  0];
    dp_dd2 = [0; 0; 1];
    dp_dt3 = [-a3*s13;  a3*c13;  0];
    dp_dt4 = [0; 0; 0];

    dw_dt1 = [0; 0; 1];
    dw_dd2 = [0; 0; 0];
    dw_dt3 = [0; 0; 1];
    % z_3 = R_03 · [0,0,1] = Rz(θ1+θ3) · Rx(π) · [0,0,1]
    %      = Rz(θ1+θ3) · [0,0,-1] = [0,0,-1]
    % Le lien 3 (α=π) retourne l'axe Z → joint 4 tourne dans le sens opposé.
    dw_dt4 = [0; 0; -1];

    J = [[dp_dt1; dw_dt1], [dp_dd2; dw_dd2], ...
         [dp_dt3; dw_dt3], [dp_dt4; dw_dt4]];
end


function [Jp, sigma_min, is_sing] = damped_pinv_mat(J, thr, lam_dls)
    % Pseudo-inverse amortie (DLS) — robuste aux singularités
    [U, S, V] = svd(J, 'econ');
    s_vec     = diag(S);
    sigma_min = s_vec(end);
    is_sing   = sigma_min < thr;

    if is_sing
        lam   = lam_dls * (1 - (sigma_min/thr)^2);
        s_inv = s_vec ./ (s_vec.^2 + lam^2);
    else
        s_inv = 1 ./ s_vec;
    end

    Jp = V * diag(s_inv) * U';
end


function S = skew_mat(v)
    % Matrice anti-symétrique 3×3 du vecteur v
    S = [  0,    -v(3),  v(2);
           v(3),  0,    -v(1);
          -v(2),  v(1),  0   ];
end
