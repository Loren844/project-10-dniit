%% animate_robot.m
% Anime le robot SCARA 4-DOF le long d'une trajectoire articulaire.
%
% Usage :
%   animate_robot(robot, traj)
%   animate_robot(robot, traj, 'speed', 2.0)   % 2× plus rapide
%   animate_robot(robot, traj, 'trail', false)  % sans trace de l'effecteur
%
% Entrées :
%   robot  - structure robot SCARA (voir robot_parameters.m)
%   traj   - trajectoire articulaire (traj.q [N×4], traj.t [1×N])
%            q = [θ1(rad), d2(m), θ3(rad), θ4(rad)]

function animate_robot(robot, traj, varargin)

    %% --- Options ---
    p = inputParser();
    addParameter(p, 'speed', 1.0, @(x) isnumeric(x) && x > 0);
    addParameter(p, 'trail', true, @islogical);
    parse(p, varargin{:});
    speed_factor = p.Results.speed;
    show_trail   = p.Results.trail;

    N  = size(traj.q, 1);
    dt = (traj.t(end) - traj.t(1)) / (N - 1);
    pause_ms = max(1, round(dt * 1000 / speed_factor));

    %% --- Pré-calcul des positions articulaires ---
    % joints_pos(k, :, :) = [6×3] : 6 points physiques du SCARA
    joints_pos = zeros(N, 6, 3);
    for k = 1:N
        pts = get_joint_positions(robot, traj.q(k, :));
        joints_pos(k, :, :) = pts;
    end

    %% --- Figure ---
    fig = figure('Name', 'Animation — Robot SCARA 4-DOF', 'NumberTitle', 'off', ...
                 'Color', [0.12 0.12 0.12]);
    ax = axes('Parent', fig, 'Color', [0.08 0.08 0.08], ...
              'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7], ...
              'ZColor', [0.7 0.7 0.7], 'GridColor', [0.3 0.3 0.3]);
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    view(ax, 45, 30);

    reach = (robot.a2 + robot.a3) * 1000 + 50;  % mm
    % Z : joint2 monte jusqu'à d2_max, l'effecteur descend jusqu'à d2-d3-d4
    z_max =  robot.d2_base * 1000 + 50;           % mm — colonne + marge
    z_min = -(robot.d3 + robot.d4) * 1000 - 30;   % mm — descente max + marge
    xlim(ax, [-reach, reach]);
    ylim(ax, [-reach, reach]);
    zlim(ax, [z_min, z_max]);
    xlabel(ax, 'X (mm)'); ylabel(ax, 'Y (mm)'); zlabel(ax, 'Z (mm)');

    % Plan de base
    [xg, yg] = meshgrid(linspace(-reach, reach, 5));
    surf(ax, xg, yg, zeros(size(xg)), ...
         'FaceColor', [0.2 0.2 0.2], 'EdgeColor', [0.3 0.3 0.3], ...
         'FaceAlpha', 0.4, 'EdgeAlpha', 0.3);

    % Colonne verticale (trait pointillé)
    col_h = robot.d2_base * 1000;
    plot3(ax, [0,0], [0,0], [0, col_h], 'w--', 'LineWidth', 1);

    % Bras
    h_arm = plot3(ax, 0, 0, 0, '-o', ...
                  'Color',          [0.2 0.6 1.0], ...
                  'LineWidth',       3.5, ...
                  'MarkerSize',      8, ...
                  'MarkerFaceColor', [1.0 0.8 0.2], ...
                  'MarkerEdgeColor', [0.9 0.7 0.1]);

    % Effecteur
    h_ee = plot3(ax, 0, 0, 0, 's', ...
                 'MarkerSize',      12, ...
                 'MarkerFaceColor', [0.1 0.9 0.3], ...
                 'MarkerEdgeColor', [0.0 0.7 0.2]);

    % Trace
    h_trail = plot3(ax, NaN, NaN, NaN, '-', ...
                    'Color', [0.9 0.4 0.1], 'LineWidth', 1.2);

    % Repère de base
    L = reach * 0.15;
    plot3(ax, [0,L],[0,0],[0,0], 'r-', 'LineWidth', 2);
    plot3(ax, [0,0],[0,L],[0,0], 'g-', 'LineWidth', 2);
    plot3(ax, [0,0],[0,0],[0,L], 'b-', 'LineWidth', 2);

    h_title = title(ax, '', 'Color', [0.9 0.9 0.9], 'FontSize', 10);

    %% --- Boucle d'animation ---
    trail_x = NaN(1, N);
    trail_y = NaN(1, N);
    trail_z = NaN(1, N);

    for k = 1:N
        if ~ishandle(fig), break; end

        pts = squeeze(joints_pos(k, :, :)) * 1000;  % m → mm

        set(h_arm, 'XData', pts(:,1), 'YData', pts(:,2), 'ZData', pts(:,3));
        set(h_ee,  'XData', pts(end,1), 'YData', pts(end,2), 'ZData', pts(end,3));

        if show_trail
            trail_x(k) = pts(end,1);
            trail_y(k) = pts(end,2);
            trail_z(k) = pts(end,3);
            set(h_trail, 'XData', trail_x(1:k), ...
                         'YData', trail_y(1:k), ...
                         'ZData', trail_z(1:k));
        end

        q = traj.q(k, :);
        set(h_title, 'String', ...
            sprintf('t=%.2fs  θ1=%.1f°  d2=%.0fmm  θ3=%.1f°  θ4=%.1f°', ...
                    traj.t(k), rad2deg(q(1)), q(2)*1000, ...
                    rad2deg(q(3)), rad2deg(q(4))));
        drawnow;
        pause(pause_ms / 1000);
    end
end

%% -----------------------------------------------------------------------
function pts = get_joint_positions(robot, q)
% Retourne [6×3] — points physiques pour un dessin correct du SCARA :
%   base → haut colonne → épaule → descente coude → coude → effecteur
% Les bras sont ainsi horizontaux et les colonnes verticales.

    theta1 = q(1);
    d2     = q(2);
    theta3 = q(3);

    a2 = robot.a2;
    a3 = robot.a3;
    d3 = robot.d3;
    d4 = robot.d4;

    % 1. Base
    p0 = [0, 0, 0];
    % 2. Haut de la colonne (axe de rotation θ1) — montée verticale
    p_col = [0, 0, d2];
    % 3. Épaule : bras supérieur s'étend horizontalement à hauteur d2
    p_shldr = [a2*cos(theta1),  a2*sin(theta1),  d2];
    % 4. Descente verticale de d3 à l'articulation du coude
    p_step  = [a2*cos(theta1),  a2*sin(theta1),  d2 - d3];
    % 5. Coude : avant-bras s'étend horizontalement à hauteur d2-d3
    cx = a2*cos(theta1) + a3*cos(theta1 + theta3);
    cy = a2*sin(theta1) + a3*sin(theta1 + theta3);
    p_elbow = [cx, cy, d2 - d3];
    % 6. Effecteur : descente verticale finale de d4
    p_ee = [cx, cy, d2 - d3 - d4];

    pts = [p0; p_col; p_shldr; p_step; p_elbow; p_ee];
end
