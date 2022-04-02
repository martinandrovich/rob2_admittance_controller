clc;

th_c = pi/4; r_c = [1 0 0]';
eta_c = cos(th_c/2);
eps_c = sin(th_c/2) * r_c;

q_c = UnitQuaternion(eta_c, eps_c)

th_d = pi/4 + 0.1; r_d = [1 0 0]';
eta_d = cos(th_d/2);
eps_d = sin(th_d/2) * r_d;

q_d = UnitQuaternion(eta_d, eps_d)

% normality test
eta_c^2 + eps_c' * eps_c
eta_d^2 + eps_d' * eps_d

eps_d_cd = eta_d * eps_c - eta_c * eps_d - skew(eps_d) * eps_c
% eps_d_cd = eta_d * eps_c - eta_c * eps_d - skew(eps_c) * eps_d

q_d_cd = inv(q_d) * q_c

eps_c
inv(eta_d * eye(3) - skew(eps_d)) * (eps_d_cd + eta_c * eps_d)
q_c_ = q_d * q_d_cd