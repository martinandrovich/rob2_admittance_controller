close all; clear; clc;
format short G
%#ok<*NOPTS>

% load model
run("model.m")

%% forward dynamics

% forward dynamics
% fd(q, dq) = ddq = inv(M(q)) * (tau - (C(q, dq) * dq + g(q)))
% using dampening: tau = -D * dq

D = eye(size(C)) * 5;
tau = -D * dq;
fd = inv(M) * (tau - (C * dq + g)); % M is full rank, use inv()
fd = simplify(fd)

fd_fun = @(q_, dq_) double(subs(fd, [q' dq'], [q_' dq_']));
% fd_fun = matlabFunction(fd) % scuffed

%% simulate

% initial conditions + timespan
q0 = [1 pi/3 pi/3]; dq0 = [0 0 0];
tspan = [0 1];

% define function handle for ode45
% dydt = odefun(t,y) where y = [q dq] and dydt = [dq ddq]
dydt = @(t, y) [y(4:6) ; fd_fun(y(1:3), y(4:6))];

% simulate
[t, y] = simulate_ode23(dydt, tspan, [q0 dq0])
tau_d = y(:,4:6) * -D

%% plot

close all;
plot_sim(t, y, tau_d);
