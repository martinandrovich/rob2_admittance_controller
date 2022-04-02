close all; clear; clc;
format short G
%#ok<*NOPTS>

% simulation
model = "admittance_controller_full";
sys = load_system(model);
tspan = [0 25]; dt = 0.01;
t = linspace(tspan(1), tspan(2), tspan(2)/dt);
set_param(model, "StopTime", num2str(tspan(2)))

% input wrench
h_0e = [
	zeros(5/dt, 6) ;                            % nothing for 5 sec
	repmat([1 2 3], 5/dt, 1) zeros(5/dt, 3) ;   % force for 5 sec
	zeros(5/dt, 6) ;                            % nothing for 5 sec
	zeros(5/dt, 3) repmat([1 0.5 1], 5/dt, 1) ; % moment for 5 sec
	zeros(5/dt, 6)                              % nothing for 5 sec
];
h_0es = timeseries(h_0e, t);

% impedance parameters
% SET THESE TO REAL VALUES
Kp = eye(3) * 16;
Kd = eye(3) * 4;
Md = eye(3) * 1;

zeta = 1;

% trajectory
T_0 = trvec2tform([0.2 -0.3 0.3]) * eul2tform([-0.2 -0.5 -0.6], "XYZ");
T_d = trvec2tform([0.6    0 0.4]) * eul2tform([-1.2 1 -1.2], "XYZ");
[Ts, vel, acc] = transformtraj(T_0, T_d, tspan, t);
R_ds = Ts(1:3,1:3,:);

% pose(s)
T_0ds = timeseries(Ts, t);
T_d0s = arrayfun(@(i) inv(Ts(:, :, i)), 1:size(Ts,3), 'UniformOutput', false); % -> cell array
T_d0s = timeseries(cat(3, T_d0s{:}), t); % cuz who needs for loops

% position(s)
p_ds = timeseries(squeeze(Ts(1:3,4,:))', t);

% euler angle(s)
phi_ds = timeseries(rotm2eul(R_ds, "XYZ"), t);

% quaternion(s)
q_ds = timeseries(rotm2quat(R_ds), t);

% initial conditions
p_cd0 = [0 0 0]'; dp_cd0 = [0 0 0]';
phi_cd0 = [0 0 0]'; dphi_cd0 = [0 0 0]';
q_d_cd0 = [1 0 0 0]; w_d_cd0 = [0 0 0]';

%% simulate

out = sim(model)
t_sim = out.tout(1:length(t));
p_c = out.p_c(:,:,1:length(t));
phi_c = out.phi_c(:,:,1:length(t));
q_c = out.q_c(:,:,1:length(t));
mu_d = out.mu_d(:,:,1:length(t));

%% plots

% figure defaults
close all;
run("plot_config.m");

% force
f_x = h_0e(:, 1); f_y = h_0e(:, 2); f_z = h_0e(:, 3);

f = figure;
colororder(COLOR.MAP)
plot(t, f_x)
hold on; grid on;
plot(t, f_y)
plot(t, f_z)
legend(["$f_x$", "$f_y$", "$f_z$"])
xlabel("Time [s]"); ylabel("Force [N]");

exportgraphics(f, PATH_IMG + "/sim_admittance_controller_f.pdf");

% moment
mu_x = h_0e(:, 4); mu_y = h_0e(:, 5); mu_z = h_0e(:, 6);

f = figure;
colororder(COLOR.MAP)
plot(t, mu_x)
hold on; grid on;
plot(t, mu_y)
plot(t, mu_z)
legend(["$\mu_x$", "$\mu_y$", "$\mu_z$"])
xlabel("Time [s]"); ylabel("Torque [N m]");

exportgraphics(f, PATH_IMG + "/sim_admittance_controller_mu.pdf");

% moment (in desired frame)
mu_d_x = squeeze(mu_d(1,1,:)); mu_d_y = squeeze(mu_d(2,1,:)); mu_d_z = squeeze(mu_d(3,1,:));

f = figure;
colororder(COLOR.MAP)
plot(t_sim, mu_d_x)
hold on; grid on;
plot(t_sim, mu_d_y)
plot(t_sim, mu_d_z)
legend(["$\mu^{d}_{x}$", "$\mu^{d}_{y}$", "$\mu^{d}_{z}$"])
xlabel("Time [s]"); ylabel("Torque (in desired frame) [N m]");

exportgraphics(f, PATH_IMG + "/sim_admittance_controller_mu_d.pdf");

% position
x_d = squeeze(Ts(1,4,:)); y_d = squeeze(Ts(2,4,:)); z_d = squeeze(Ts(3,4,:));
x_c = squeeze(p_c(1,1,:)); y_c = squeeze(p_c(2,1,:)); z_c = squeeze(p_c(3,1,:));

f = figure;
colororder(COLOR.MAP)
tl = tiledlayout(3,1);

nexttile
plot(t, x_d)
hold on; grid on;
plot(t_sim, x_c)
legend(["$x_d$", "$x_c$"])
ylabel("$x$ [m]")

nexttile
plot(t, y_d)
hold on; grid on;
plot(t_sim, y_c)
legend(["$y_d$", "$y_c$"])
ylabel("$y$ [m]")

nexttile
plot(t, z_d)
hold on; grid on;
plot(t_sim, z_c)
legend(["$z_d$", "$z_c$"])
ylabel("$z$ [m]")

xlabel(tl, "Time [s]")

exportgraphics(f, PATH_IMG + "/sim_admittance_controller_p_c.pdf");

% orientation (euler)

phi_d = rotm2eul(R_ds, "XYZ");
varphi_d = phi_d(:, 1); th_d = phi_d(:, 2); psi_d = phi_d(:, 3);
varphi_c = squeeze(phi_c(1, 1, :)); th_c = squeeze(phi_c(2, 1, :)); psi_c = squeeze(phi_c(3, 1, :));

f = figure;
colororder(COLOR.MAP)
tl = tiledlayout(3,1);

nexttile
plot(t, varphi_d)
hold on; grid on;
plot(t_sim, varphi_c)
legend(["$\varphi_d$", "$\varphi_c$"])
ylabel("$\varphi$ [rad]")

nexttile
plot(t, th_d)
hold on; grid on;
plot(t_sim, th_c)
legend(["$\vartheta_d$", "$\vartheta_c$"])
ylabel("$\vartheta$ [rad]")

nexttile
plot(t, psi_d)
hold on; grid on;
plot(t_sim, psi_c)
legend(["$\psi_d$", "$\psi_c$"])
ylabel("$\psi$ [rad]")

xlabel(tl, "Time [s]")

exportgraphics(f, PATH_IMG + "/sim_admittance_controller_phi_c.pdf");

% orientation (quat)

q_d = rotm2quat(R_ds);
eps_d_x = q_d(:, 2); eps_d_y = q_d(:, 3); eps_d_z = q_d(:, 4);
eps_c_x = squeeze(q_c(1, 2, :)); eps_c_y = squeeze(q_c(1, 3, :)); eps_c_z = squeeze(q_c(1, 4, :));

f = figure;
colororder(COLOR.MAP)
tl = tiledlayout(3,1);

nexttile
plot(t, eps_d_x)
hold on; grid on;
plot(t_sim, eps_c_x)
legend(["$\epsilon_{d,x}$", "$\epsilon_{c,x}$"])
ylabel("$\epsilon_x$ ")

nexttile
plot(t, eps_d_y)
hold on; grid on;
plot(t_sim, eps_c_y)
legend(["$\epsilon_{d,y}$", "$\epsilon_{c,y}$"])
ylabel("$\epsilon_y$")

nexttile
plot(t, eps_d_z)
hold on; grid on;
plot(t_sim, eps_c_z)
legend(["$\epsilon_{d,z}$", "$\epsilon_{c,z}$"])
ylabel("$\epsilon_z$")

xlabel(tl, "Time [s]")

exportgraphics(f, PATH_IMG + "/sim_admittance_controller_eps_c.pdf");

%% symbolic expression for T(phi) matrix for XYZ-euler

syms phi th psi
dphi_ = eye(3); dphi_ = dphi_(:, 1);
dth_  = eye(3) * rotx(phi); dth_ = dth_(:, 2);
dpsi_ = eye(3) * rotx(phi) * roty(th); dpsi_ = dpsi_(:, 3);
T_ = [dphi_ dth_ dpsi_]
T_fcn = @(vec) double(subs(T_, [phi th psi], vec'));
