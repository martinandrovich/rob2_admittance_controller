close all; clear; clc;
format short G
%#ok<*NOPTS>

% load model
% uncomment symbolic defs in "robot_param.m" for simulation
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
tspan = [0 5];

% define function handle for ode45
% dydt = odefun(t,y) where y = [q dq] and dydt = [dq ddq]
dydt = @(t, y) [y(4:6) ; fd_fun(y(1:3), y(4:6))];

% simulate
[t, y] = simulate_ode23(dydt, tspan, [q0 dq0])
tau_d = y(:,4:6) * -D

%% plots

% figure defaults
close all;
run("plot_config.m");

% joint position
f = figure;
colororder(COLOR.MAP)
plot(t, y(:,1))
hold on; grid on;
plot(t, y(:,2))
plot(t, y(:,3))
xlabel("Time [s]"); ylabel("Joint position [rad]")
legend(["$q_1$", "$q_2$", "$q_3$"])

exportgraphics(f, PATH_IMG + "/sim_dampening_q.pdf");

% joint velocity
f = figure;
colororder(COLOR.MAP)
plot(t, y(:,4))
hold on; grid on;
plot(t, y(:,5))
plot(t, y(:,6))
xlabel("Time [s]"); ylabel("Joint velocity [rad/s]")
legend(["$\dot{q}_1$", "$\dot{q}_2$", "$\dot{q}_3$"])

exportgraphics(f, PATH_IMG + "/sim_dampening_dq.pdf");

% joint torque
f = figure;
colororder(COLOR.MAP)
plot(t, tau_d(:,1))
hold on; grid on;
plot(t, tau_d(:,2))
plot(t, tau_d(:,3))
xlabel("Time [s]"); ylabel("Joint torque [N m]")
legend(["$\tau_1$", "$\tau_2$", "$\tau_3$"])

exportgraphics(f, PATH_IMG + "/sim_dampening_tau.pdf");
