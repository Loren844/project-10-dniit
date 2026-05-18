%% pid_simulation.m
% Simulation de commande PID articulaire en boucle fermée.
%
% Usage :
%   result = pid_simulation(robot, traj, Kp, Ki, Kd)
%   result = pid_simulation(robot, traj)   % gains par défaut

function result = pid_simulation(robot, traj, Kp, Ki, Kd)

    n  = robot.n_joints;
    dt = traj.t(2) - traj.t(1);
    N  = length(traj.t);

    %% --- Gains PID par défaut ---
    if nargin < 3 || isempty(Kp), Kp = 80 * ones(1, n); end
    if nargin < 4 || isempty(Ki), Ki = 5  * ones(1, n); end
    if nargin < 5 || isempty(Kd), Kd = 12 * ones(1, n); end

    %% --- Initialisation ---
    q      = zeros(N, n);
    dq     = zeros(N, n);
    torque = zeros(N, n);

    q(1, :)  = traj.q(1, :);
    dq(1, :) = zeros(1, n);

    e_int  = zeros(1, n);

    %% --- Modèle dynamique (lu depuis la structure robot) ---
    J_eff   = robot.J_eff;
    B_vis   = robot.B_vis;
    tau_max = robot.tau_max;

    %% --- Boucle de simulation ---
    for k = 1:N-1
        % Référence au pas k+1 : le couple calculé à k doit amener le robot à k+1.
        % Utiliser k+1 élimine le retard d'un pas inhérent au contrôle discret.
        k_ff = min(k+1, N);
        q_ref   = traj.q(k_ff, :);
        dq_ref  = traj.dq(k_ff, :);
        ddq_ref = traj.ddq(k_ff, :);

        e     = q_ref - q(k, :);
        e_int = max(-10, min(10, e_int + e * dt));   % Anti-windup

        % Terme dérivé : erreur de vitesse analytique (pas de différence finie)
        % → évite le bruit numérique et le retard d'un échantillon du terme Kd
        vel_err = dq_ref - dq(k, :);

        % Feedforward : couple nominal pour suivre la trajectoire planifiée
        tau_ff = J_eff .* ddq_ref + B_vis .* dq_ref;
        tau    = tau_ff + Kp .* e + Ki .* e_int + Kd .* vel_err;
        tau    = max(-tau_max, min(tau_max, tau));

        % Méthode de Heun (RK2) pour l'intégration du modèle du robot.
        % Euler explicite : erreur globale O(dt)  → accumulation de lag de phase.
        % Heun             : erreur globale O(dt²) → 100× plus précis à dt égal.
        %
        % Étape 1 (pente en k) :
        ddq_1   = (tau - B_vis .* dq(k, :)) ./ J_eff;
        dq_pred = dq(k, :) + ddq_1 * dt;
        % Étape 2 (pente en k+1 avec même tau) :
        ddq_2      = (tau - B_vis .* dq_pred) ./ J_eff;
        % Moyenne des deux pentes (règle des trapèzes) :
        dq(k+1, :) = dq(k, :) + 0.5 * (ddq_1 + ddq_2) * dt;
        q(k+1, :)  = max(robot.q_min, min(robot.q_max, ...
                       q(k, :) + 0.5 * (dq(k,:) + dq(k+1,:)) * dt));

        torque(k, :) = tau;
    end

    %% --- Calcul des métriques ---
    e_tracking = traj.q - q;
    rmse_joint = sqrt(mean(e_tracking.^2));
    max_error  = max(abs(e_tracking));

    result.t         = traj.t;
    result.q_ref     = traj.q;
    result.q         = q;
    result.dq        = dq;
    result.torque    = torque;
    result.e         = e_tracking;
    result.rmse      = rmse_joint;
    result.max_error = max_error;
    result.Kp        = Kp;
    result.Ki        = Ki;
    result.Kd        = Kd;

    %% --- Affichage console ---
    fprintf('\n=== Performances du contrôleur PID ===\n');
    fprintf('%-14s %14s %14s\n', 'Articulation', 'RMSE', 'Err. max');
    fprintf('%s\n', repmat('-', 1, 46));
    for i = 1:n
        if robot.joint_type(i) == 'P'
            fprintf('  q_%d            %10.4f mm   %10.4f mm\n', i, ...
                    rmse_joint(i)*1000, max_error(i)*1000);
        else
            fprintf('  q_%d            %10.4f °    %10.4f °\n', i, ...
                    rad2deg(rmse_joint(i)), rad2deg(max_error(i)));
        end
    end

    %% --- Figures ---
    plot_pid_results(result, n);
end

%% -----------------------------------------------------------------------
function plot_pid_results(result, n)

    colors = lines(n);

    figure('Name', 'Suivi PID — Positions articulaires', 'NumberTitle', 'off');
    for i = 1:n
        subplot(ceil(n/2), 2, i);
        plot(result.t, rad2deg(result.q_ref(:,i)), '--', ...
             'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', 'Référence');
        hold on;
        plot(result.t, rad2deg(result.q(:,i)), '-', ...
             'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', 'Réponse');
        xlabel('Temps (s)'); ylabel('Angle (°)');
        title(sprintf('Articulation q_%d', i));
        legend('Location', 'best'); grid on; hold off;
    end
    sgtitle('Suivi PID — Positions articulaires');

    figure('Name', 'Suivi PID — Erreurs', 'NumberTitle', 'off');
    for i = 1:n
        subplot(ceil(n/2), 2, i);
        plot(result.t, rad2deg(result.e(:,i)), 'r-', 'LineWidth', 1.5);
        yline(0, 'k--');
        xlabel('Temps (s)'); ylabel('Erreur (°)');
        title(sprintf('Erreur q_%d  (RMSE = %.3f°)', i, rad2deg(result.rmse(i))));
        grid on;
    end
    sgtitle('Suivi PID — Erreurs de suivi');

    figure('Name', 'Suivi PID — Couples moteurs', 'NumberTitle', 'off');
    for i = 1:n
        subplot(ceil(n/2), 2, i);
        plot(result.t(1:end-1), result.torque(1:end-1, i), 'b-', 'LineWidth', 1.5);
        xlabel('Temps (s)'); ylabel('Couple (N.m)');
        title(sprintf('Couple \\tau_%d', i));
        grid on;
    end
    sgtitle('Suivi PID — Couples moteurs');
end
