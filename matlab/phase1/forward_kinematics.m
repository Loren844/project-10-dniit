%% forward_kinematics.m
% Cinématique directe du robot SCARA 4-DOF.
%
% Topologie : θ1 (R) → d2 (P) → θ3 (R) → θ4 (R)
%
% Vecteur d'état : q = [θ1 (rad), d2 (m), θ3 (rad), θ4 (rad)]
%
% Table DH (voir robot_parameters.m) :
%   Lien 1 : a=0,   α=0°,   d=0,     θ=θ1    → rotation de la colonne
%   Lien 2 : a=a2,  α=0°,   d=d2,    θ=0°    → translation verticale + décalage a2
%   Lien 3 : a=a3,  α=180°, d=-d3,   θ=θ3    → rotation horizontale + plongée
%   Lien 4 : a=0,   α=0°,   d=d4,    θ=θ4    → rotation du poignet
%
% Usage :
%   [T_end, T_all] = forward_kinematics(robot, q)
%
% Sorties :
%   T_end  — matrice homogène 4×4 de l'effecteur dans le repère de base
%   T_all  — cell {4×1} des matrices T_{0→i} pour i = 1..4

function [T_end, T_all] = forward_kinematics(robot, q)

    assert(length(q) == 4, 'forward_kinematics (SCARA): q doit avoir 4 composantes.');

    theta1 = q(1);
    d2     = q(2);
    theta3 = q(3);
    theta4 = q(4);

    a2 = robot.a2;
    a3 = robot.a3;
    d3 = robot.d3;
    d4 = robot.d4;

    %% --- Matrice DH lien 1 : θ=θ1, d=0, a=0, α=0 ---
    A1 = dh_matrix(theta1, 0, 0, 0);

    %% --- Matrice DH lien 2 : θ=0°, d=d2 (prismatique), a=a2, α=0 ---
    A2 = dh_matrix(0, d2, a2, 0);

    %% --- Matrice DH lien 3 : θ=θ3, d=-d3, a=a3, α=π (180°) ---
    A3 = dh_matrix(theta3, -d3, a3, pi);

    %% --- Matrice DH lien 4 : θ=θ4, d=d4, a=0, α=0 ---
    A4 = dh_matrix(theta4, d4, 0, 0);

    %% --- Composition T_{0→i} ---
    T_all    = cell(4, 1);
    T_all{1} = A1;
    T_all{2} = A1 * A2;
    T_all{3} = T_all{2} * A3;
    T_all{4} = T_all{3} * A4;

    T_end = T_all{4};
end

%% -----------------------------------------------------------------------
function T = dh_matrix(theta, d, a, alpha)
% Matrice homogène DH standard : Rz(θ)·Tz(d)·Tx(a)·Rx(α)
    ct = cos(theta);  st = sin(theta);
    ca = cos(alpha);  sa = sin(alpha);

    T = [ct,  -st*ca,   st*sa,   a*ct;
         st,   ct*ca,  -ct*sa,   a*st;
          0,      sa,      ca,      d;
          0,       0,       0,      1];
end
