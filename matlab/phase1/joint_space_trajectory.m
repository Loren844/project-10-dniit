%% joint_space_trajectory.m
% Génère une trajectoire articulaire point-à-point.
%
% Usage :
%   traj = joint_space_trajectory(q_start, q_end, T_total, dt)
%   traj = joint_space_trajectory(q_start, q_end, T_total, dt, profile)
%
% Entrées :
%   q_start  - configuration initiale [1 x n] (rad)
%   q_end    - configuration finale   [1 x n] (rad)
%   T_total  - durée totale (s)
%   dt       - pas de temps (s)
%   profile  - 'cubic' (défaut) | 'quintic' | 'trapezoidal'
%
% Sorties :
%   traj.t   - vecteur temps
%   traj.q   - positions articulaires  [N x n]
%   traj.dq  - vitesses articulaires   [N x n]
%   traj.ddq - accélérations           [N x n]

function traj = joint_space_trajectory(q_start, q_end, T_total, dt, profile)

    if nargin < 5, profile = 'cubic'; end

    t = 0:dt:T_total;
    N = length(t);
    n = length(q_start);

    traj.t       = t;
    traj.q       = zeros(N, n);
    traj.dq      = zeros(N, n);
    traj.ddq     = zeros(N, n);
    traj.type    = 'joint_space';
    traj.profile = profile;

    for i = 1:n
        delta_q = q_end(i) - q_start(i);

        switch profile
            case 'cubic'
                for k = 1:N
                    s = t(k) / T_total;
                    traj.q(k, i)   = q_start(i) + delta_q * (3*s^2 - 2*s^3);
                    traj.dq(k, i)  = delta_q / T_total * (6*s - 6*s^2);
                    traj.ddq(k, i) = delta_q / T_total^2 * (6 - 12*s);
                end

            case 'quintic'
                for k = 1:N
                    s = t(k) / T_total;
                    traj.q(k, i)   = q_start(i) + delta_q * (10*s^3 - 15*s^4 + 6*s^5);
                    traj.dq(k, i)  = delta_q / T_total * (30*s^2 - 60*s^3 + 30*s^4);
                    traj.ddq(k, i) = delta_q / T_total^2 * (60*s - 180*s^2 + 120*s^3);
                end

            case 'trapezoidal'
                t_acc = T_total / 3;
                v_max = 1.5 * delta_q / T_total;

                for k = 1:N
                    tk = t(k);
                    if tk <= t_acc
                        a = v_max / t_acc;
                        traj.q(k, i)   = q_start(i) + 0.5 * a * tk^2;
                        traj.dq(k, i)  = a * tk;
                        traj.ddq(k, i) = a;
                    elseif tk <= T_total - t_acc
                        traj.q(k, i)   = q_start(i) + v_max * (tk - t_acc/2);
                        traj.dq(k, i)  = v_max;
                        traj.ddq(k, i) = 0;
                    else
                        a    = -v_max / t_acc;
                        dt_  = tk - (T_total - t_acc);
                        q_br = q_start(i) + v_max * (T_total - t_acc - t_acc/2);
                        traj.q(k, i)   = q_br + v_max * dt_ + 0.5 * a * dt_^2;
                        traj.dq(k, i)  = v_max + a * dt_;
                        traj.ddq(k, i) = a;
                    end
                end

            otherwise
                error('joint_space_trajectory: profil inconnu "%s". Choisir cubic|quintic|trapezoidal.', profile);
        end
    end
end
