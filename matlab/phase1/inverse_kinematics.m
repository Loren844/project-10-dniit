%% inverse_kinematics.m
% Cinématique inverse analytique du robot SCARA 4-DOF.
%
% Formules issues du rapport PBL6 (équations [1.8], [1.9], [1.10], [1.11]) :
%
%   cos(θ3) = (Px² + Py² - a2² - a3²) / (2·a2·a3)
%   θ3      = atan2(±√(1 - cos²θ3), cos θ3)
%
%   sin θ1  = (Py·(a2 + a3·cos θ3) - Px·a3·sin θ3) / D
%   cos θ1  = (Px·(a2 + a3·cos θ3) + Py·a3·sin θ3) / D
%   θ1      = atan2(sin θ1, cos θ1)
%
%   θ4 = -θ1 - θ3   (orientation à 0°, simplification du rapport)
%
%   d2* = Pz + d3 + d4
%
% Usage :
%   [q_sol, success, err] = inverse_kinematics(robot, T_des, q0)

function [q_sol, success, err] = inverse_kinematics(robot, T_des, q0)

    if nargin < 3 || isempty(q0), q0 = [0, 0.1, 0, 0]; end

    Px = T_des(1, 4);
    Py = T_des(2, 4);
    Pz = T_des(3, 4);

    a2 = robot.a2;
    a3 = robot.a3;
    d3 = robot.d3;
    d4 = robot.d4;

    success = false;
    q_sol   = q0;
    err     = inf;

    %% --- Calcul de theta3 ---
    cos_t3 = (Px^2 + Py^2 - a2^2 - a3^2) / (2 * a2 * a3);

    if abs(cos_t3) > 1.0
        return;   % Hors espace de travail
    end

    sin_t3_pos = sqrt(max(0, 1 - cos_t3^2));
    sin_t3_neg = -sin_t3_pos;

    % Choisir la branche la plus proche de q0(3)
    t3_pos = atan2(sin_t3_pos, cos_t3);
    t3_neg = atan2(sin_t3_neg, cos_t3);
    if abs(t3_pos - q0(3)) <= abs(t3_neg - q0(3))
        theta3 = t3_pos;
        sin_t3 = sin_t3_pos;
    else
        theta3 = t3_neg;
        sin_t3 = sin_t3_neg;
    end

    %% --- Calcul de theta1 ---
    k1 = a2 + a3 * cos_t3;
    k2 = a3 * sin_t3;
    D  = k1^2 + k2^2;

    if D < 1e-12
        return;   % Singularite
    end

    sin_t1 = (Py * k1 - Px * k2) / D;
    cos_t1 = (Px * k1 + Py * k2) / D;
    theta1 = atan2(sin_t1, cos_t1);

    %% --- Calcul de theta4 ---
    theta4 = -theta1 - theta3;

    %% --- Calcul de d2 ---
    d2 = Pz + d3 + d4;

    %% --- Solution ---
    q_sol = [theta1, d2, theta3, theta4];
    q_sol = max(robot.q_min, min(robot.q_max, q_sol));

    %% --- Erreur residuelle ---
    T_check = forward_kinematics(robot, q_sol);
    err     = norm(T_check(1:3, 4) - T_des(1:3, 4));
    success = (err < 1e-3);

end
