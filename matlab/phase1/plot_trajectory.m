%% plot_trajectory.m
% Visualise une trajectoire articulaire ou cartésienne.
%
% Usage :
%   plot_trajectory(traj)

function plot_trajectory(traj)

    switch traj.type
        case 'joint_space'
            n = size(traj.q, 2);
            figure('Name', sprintf('Trajectoire articulaire (%s)', traj.profile), ...
                   'NumberTitle', 'off');

            subplot(3,1,1);
            plot(traj.t, rad2deg(traj.q)); grid on;
            xlabel('Temps (s)'); ylabel('Position (°)');
            title('Positions articulaires');
            legend(arrayfun(@(i) sprintf('q_%d', i), 1:n, 'UniformOutput', false));

            subplot(3,1,2);
            plot(traj.t, rad2deg(traj.dq)); grid on;
            xlabel('Temps (s)'); ylabel('Vitesse (°/s)');
            title('Vitesses articulaires');
            legend(arrayfun(@(i) sprintf('dq_%d', i), 1:n, 'UniformOutput', false));

            subplot(3,1,3);
            plot(traj.t, rad2deg(traj.ddq)); grid on;
            xlabel('Temps (s)'); ylabel('Accélération (°/s²)');
            title('Accélérations articulaires');
            legend(arrayfun(@(i) sprintf('ddq_%d', i), 1:n, 'UniformOutput', false));

        case 'cartesian'
            figure('Name', 'Trajectoire cartésienne', 'NumberTitle', 'off');

            subplot(1,2,1);
            plot(traj.t, traj.p * 1000); grid on;
            xlabel('Temps (s)'); ylabel('Position (mm)');
            title('Coordonnées cartésiennes vs temps');
            legend('X', 'Y', 'Z');

            subplot(1,2,2);
            plot3(traj.p(:,1), traj.p(:,2), traj.p(:,3), 'b-', 'LineWidth', 2);
            hold on;
            plot3(traj.p(1,1),   traj.p(1,2),   traj.p(1,3),   'gs', ...
                  'MarkerSize', 10, 'MarkerFaceColor', 'g');
            plot3(traj.p(end,1), traj.p(end,2), traj.p(end,3), 'r^', ...
                  'MarkerSize', 10, 'MarkerFaceColor', 'r');
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            title('Chemin cartésien 3D'); grid on; axis equal;
            legend('Chemin', 'Départ', 'Arrivée');
            hold off;

        otherwise
            warning('plot_trajectory: type de trajectoire inconnu : %s', traj.type);
    end
end
