close all; clear; clc;
format short G
%#ok<*NOPTS>

%% robot model

% load robot model parameters (dh, mass, ..)
run("robot_param.m")
run("helpers.m"); % load helpers functions (dhtf(), t(), R(), ..)

% kinematics
T0  = eye(4);
T01 = dhtf(d1, q1, 0, pi/2)
T12 = dhtf(0, q2, a2, 0)
T23 = dhtf(0, q3, a3, 0)

T02 = T01*T12
T03 = T01*T12*T23

% fk = vpa(subs(T03, q', [0 0 0]), 4)
% fk = r.fkine([0 0 0])

% center of mass (in base frame)
p1 = dhom(T01 * [c1 ; 1])
p2 = dhom(T01 * T12 * [c2 ; 1])
p3 = dhom(T01 * T12 * T23 * [c3 ; 1])

% transformation matrix operations
% z(T) = T(1:3, 3) | t(T) = T(1:3, 4) | R(T) = T(1:3, 1:3)

% link jacobians (in base frame)
JP1 = [cross(z(T0), p1 - t(T0)) [0 0 0]' [0 0 0]']
JP2 = [cross(z(T0), p2 - t(T0)) cross(z(T01), p2 - t(T01)) [0 0 0]']
JP3 = [cross(z(T0), p3 - t(T0)) cross(z(T01), p3 - t(T01)) cross(z(T02), p3 - t(T02))]

JO1 = [z(T0) [0 0 0]' [0 0 0]']
JO2 = [z(T0) z(T01) [0 0 0]']
JO3 = [z(T0) z(T01) z(T02)]

% jacobian (in base frame, for end-effector)
JP = [cross(z(T0), t(T03) - t(T0)) cross(z(T01), t(T03) - t(T01)) cross(z(T02), t(T03) - t(T02))];
JO = [z(T0) z(T01) z(T02)];
J  = [JP ; JO]

% vpa(subs(J, q', [0 0 0]), 4)
% vpa(subs(r.jacob0(q), q', [0 0 0]), 4)

% link inertia tensors (in base frame)
I01 = R(T01) * I1 * R(T01)'
I02 = R(T02) * I2 * R(T02)'
I03 = R(T03) * I3 * R(T03)'

% potential energy
g0 = [0 0 -9.82];   
% PE = -1 * ([m1 * g0 * p1] + ...
%            [m2 * g0 * p2] + ...
%            [m3 * g0 * p3])
PE = -g0 * [p1 p2 p3] * [m1 m2 m3]'

% mass matrix (inertia tensor)
M = [m1 * (JP1' * JP1) + JO1' * I01 * JO1] + ...
    [m2 * (JP2' * JP2) + JO2' * I02 * JO2] + ...
    [m3 * (JP3' * JP3) + JO3' * I03 * JO3]

% kinetic energy
KE = 1/2 * dq' * M * dq

% Lagrangian
L = KE - PE

% compute d/dt * d(L)/d(dq1)
% chain rule = F(q(t)) -> dF/dt = dF/dq * dq/dt
% F = d(L)/d(dq1) -> F(q(t), dq(t)) -> two contributions (q, dq)
% dF/dt = dF/dq * dq/dt + dF/ddq * dq/dt

% first row (q1, dq1), where dq1 = dq1/dt
dL_ddq1 = diff(L, dq1); % d(L)/d(dq1) = F(q, dq)
ddt_dL_ddq1 = diff(dL_ddq1, q1) * dq1 + diff(dL_ddq1, dq1) * ddq1
% ddt_dL_ddq1 = diff(dL_ddq1, t) % not possible, no 't' since no 'q(t)'

% diff() for single row, jacobian() for all rows
% dL_ddq1 == dLddq(1) and ddt_dL_ddq1 == ddt_dLdq(1)
dLddq = jacobian(L, dq);
ddt_dLdq = jacobian(dLddq, q) * dq + jacobian(dLddq, dq) * ddq;
QL = ddt_dLdq - jacobian(L, q)'

% QL = jacobian(dLddq, q) * dq + jacobian(dLddq, dq) * ddq - jacobian(L, q)'

% equations of motion (M, C, g)
M = simplify(M)
C = coriolis(M)
g = jacobian(PE, q)'
Q = M * ddq + C * dq + g
