%% cartesian_trajectory.m
% Génère une trajectoire linéaire cartésienne avec interpolation SLERP.
%
% Usage :
%   traj = cartesian_trajectory(robot, p_start, p_end, R_start, R_end, T_total, dt)

function traj = cartesian_trajectory(robot, p_start, p_end, R_start, R_end, T_total, dt)

    t = 0:dt:T_total;
    N = length(t);

    traj.t    = t;
    traj.p    = zeros(N, 3);
    traj.R    = cell(N, 1);
    traj.q    = zeros(N, robot.n_joints);
    traj.type = 'cartesian';

    T0 = [R_start, p_start; 0 0 0 1];
    [q_init, ~, ~] = inverse_kinematics(robot, T0, []);

    for k = 1:N
        s  = t(k) / T_total;
        sm = 3*s^2 - 2*s^3;   % Lissage cubique

        p_k = p_start + sm * (p_end - p_start);
        traj.p(k, :) = p_k';

        R_k = slerp_rotation(R_start, R_end, sm);
        traj.R{k} = R_k;

        T_k = [R_k, p_k; 0 0 0 1];
        [q_k, ok, ~] = inverse_kinematics(robot, T_k, q_init);
        if ok
            q_init = q_k;
        end
        traj.q(k, :) = q_k;
    end
end

%% -----------------------------------------------------------------------
function R_interp = slerp_rotation(R1, R2, t)
    R_rel = R1' * R2;
    theta = acos(max(-1, min(1, (trace(R_rel) - 1) / 2)));
    if abs(theta) < 1e-8
        R_interp = R1;
    else
        R_interp = R1 * expm(t * theta / (2*sin(theta)) * (R_rel - R_rel'));
    end
end
