%% workspace_analysis.m
% Analyse et visualisation de l'espace de travail du bras manipulateur 4-DOF
%
% Méthode : échantillonnage Monte Carlo des configurations articulaires
% pour tracer l'enveloppe atteignable de l'effecteur.
%
% Usage :
%   workspace_analysis(robot)
%   workspace_analysis(robot, n_samples)

function workspace_analysis(robot, n_samples)

    if nargin < 2
        n_samples = 50000;   % Nombre de configurations aléatoires
    end

    fprintf('=== Analyse de l''espace de travail ===\n');
    fprintf('Nombre d''échantillons : %d\n', n_samples);

    n = robot.n_joints;

    %% Échantillonnage aléatoire dans les limites articulaires
    q_rand = rand(n_samples, n);
    for i = 1:n
        range  = robot.q_max(i) - robot.q_min(i);
        q_rand(:, i) = robot.q_min(i) + q_rand(:, i) * range;
    end

    %% Calcul des positions de l'effecteur (cinématique directe)
    points = zeros(n_samples, 3);
    for k = 1:n_samples
        T = forward_kinematics(robot, q_rand(k, :));
        points(k, :) = T(1:3, 4)';
    end

    %% -- Statistiques de l'espace de travail --
    x_range = [min(points(:,1)), max(points(:,1))];
    y_range = [min(points(:,2)), max(points(:,2))];
    z_range = [min(points(:,3)), max(points(:,3))];

    reach_max = max(sqrt(sum(points.^2, 2)));
    reach_min = min(sqrt(sum(points.^2, 2)));

    fprintf('\n--- Étendue de l''espace de travail ---\n');
    fprintf('  X : [%.3f, %.3f] m\n', x_range(1), x_range(2));
    fprintf('  Y : [%.3f, %.3f] m\n', y_range(1), y_range(2));
    fprintf('  Z : [%.3f, %.3f] m\n', z_range(1), z_range(2));
    fprintf('  Portée max : %.3f m\n', reach_max);
    fprintf('  Portée min : %.3f m\n', reach_min);

    %% -- Visualisation 3D --
    figure('Name', 'Espace de travail — Vue 3D', 'NumberTitle', 'off');
    scatter3(points(:,1), points(:,2), points(:,3), 1, points(:,3), '.');
    colorbar;
    colormap(jet);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title(sprintf('Espace de travail du %s\n(%d échantillons)', ...
          robot.name, n_samples));
    axis equal; grid on; view(45, 30);

    %% -- Vue de dessus (plan XY) --
    figure('Name', 'Espace de travail — Vue XY', 'NumberTitle', 'off');
    scatter(points(:,1), points(:,2), 1, points(:,3), '.');
    colorbar; colormap(jet);
    xlabel('X (m)'); ylabel('Y (m)');
    title('Projection XY de l''espace de travail');
    axis equal; grid on;

    %% -- Vue de côté (plan XZ) --
    figure('Name', 'Espace de travail — Vue XZ', 'NumberTitle', 'off');
    scatter(points(:,1), points(:,3), 1, points(:,2), '.');
    colorbar; colormap(jet);
    xlabel('X (m)'); ylabel('Z (m)');
    title('Projection XZ de l''espace de travail');
    axis equal; grid on;

    %% -- Visualisation de la structure du bras (config zéro) --
    figure('Name', 'Structure du bras SCARA (config zéro)', 'NumberTitle', 'off');
    plot_robot(robot, [0, robot.d2_base, 0, 0]);
    title(sprintf('Configuration zéro — %s', robot.name));
end

%% -----------------------------------------------------------------------
function plot_robot(robot, q)
% Trace la structure 3D du robot SCARA avec bras horizontaux et colonnes verticales.

    theta1 = q(1);
    d2     = q(2);
    theta3 = q(3);
    a2 = robot.a2;
    a3 = robot.a3;
    d3 = robot.d3;
    d4 = robot.d4;

    % Points physiques (même logique que animate_robot)
    p0     = [0, 0, 0];
    p_col  = [0, 0, d2];
    p_shldr = [a2*cos(theta1), a2*sin(theta1), d2];
    p_step  = [a2*cos(theta1), a2*sin(theta1), d2 - d3];
    cx = a2*cos(theta1) + a3*cos(theta1 + theta3);
    cy = a2*sin(theta1) + a3*sin(theta1 + theta3);
    p_elbow = [cx, cy, d2 - d3];
    p_ee    = [cx, cy, d2 - d3 - d4];

    pts = [p0; p_col; p_shldr; p_step; p_elbow; p_ee] * 1000;  % → mm
    labels = {'Base', 'Haut col.', 'Épaule', 'Coude haut', 'Coude', 'Effecteur'};

    hold on;

    % Segments
    plot3(pts(:,1), pts(:,2), pts(:,3), ...
          'b-o', 'LineWidth', 3, 'MarkerSize', 8, ...
          'MarkerFaceColor', [1 0.6 0], 'MarkerEdgeColor', 'k');

    % Repère de la base
    L = 50;   % mm
    quiver3(0,0,0, L,0,0, 'r', 'LineWidth', 2, 'AutoScale', 'off');
    quiver3(0,0,0, 0,L,0, 'g', 'LineWidth', 2, 'AutoScale', 'off');
    quiver3(0,0,0, 0,0,L, 'b', 'LineWidth', 2, 'AutoScale', 'off');

    % Labels
    for i = 1:length(labels)
        text(pts(i,1)+5, pts(i,2)+5, pts(i,3)+5, labels{i}, ...
             'FontSize', 9, 'Color', [0.9 0.9 0.9]);
    end

    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    set(gca, 'Color', [0.15 0.15 0.15], ...
             'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7], 'ZColor', [0.7 0.7 0.7]);
    axis equal; grid on; view(45, 30);
    hold off;
end
