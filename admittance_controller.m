close all; clear; clc;
format short G
set(0,'DefaultLineLineWidth',2)
%#ok<*NOPTS>

%% impedance parameters
clc;

F_ext = 10; % [N]
X_err_d = 1; % [m]

md = 5; % [kg]
kp = F_ext/X_err_d; % due to desired maximum displacement
kd = sqrt(4 * md * kp);

wn = sqrt(kp/md)
zeta = kd / (2 * sqrt(kp * md))

% impedance parameters
n = 3;
Md = eye(n)*md;
Kd = eye(n)*kd;
Kp = eye(n)*kp;

%% input

f   = [1 2 3]'
mu  = [1 0.5 1]'

h_0e = [f ; mu]

%% translational

% initial conditions for p_cd, dp_cd
p_cd0 = [0 0 0]'; dp_cd0 = [0 0 0]';

% desired
p_d = [1 2 3]'
dp_d = [0 0 0]'
ddp_d = [0 0 0]'

%% euler

% XYZ-euler as φ = [ϕ ϑ ψ]' = [phi th psi]'
% Rx(phi) * Ry'(th) * Rz''(psi)

% initial conditions for phi_cd, dphi_cd
phi_cd0 = [0 0 0]'; dphi_cd0 = [0 0 0]';

% desired
phi_d = [pi/2 0 pi/4]'; % [roll pitch yaw]
dphi_d = [0 0 0]';
ddphi_d = [0 0 0]';

R_d = eul2rotm(phi_d', "XYZ") % test
eul_d = rotm2eul(R_d, "XYZ")

% symbolic expression for T(phi) matrix for XYZ-euler
syms phi th psi
dphi_ = eye(3); dphi_ = dphi_(:, 1);
dth_  = eye(3) * rotx(phi); dth_ = dth_(:, 2);
dpsi_ = eye(3) * rotx(phi) * roty(th); dpsi_ = dpsi_(:, 3);
T_ = [dphi_ dth_ dpsi_]
T_fcn = @(vec) double(subs(T_, [phi th psi], vec'));

%% quaternion + angular velocity

% q = [cos(th/2) r * sin(th/2)] = [eta, eps] = [a b c d]
% th = acos(a)*2 = acos(eta)*2
% r  = 1/sin(th/2) * [b c d] = 1/sin(th/2) * eps
% r  = unit([b c d]) = unit(eps) = eps/norm(eps)

% initial conditions
q_d_cd0 = quaternion([0 0 0], "euler", "XYZ", "frame").compact();
w_d_cd0 = [0 0 0]';

% desired
q_d = quaternion(phi_d', "euler", "XYZ", "frame").compact();
w_d = [0 0 0]';
dw_d = [0 0 0]';


%% trajectory
close all;

% t = 1:100;

tspan = [0 1];
t = linspace(tspan(1), tspan(2), 100);

T_0 = eye(4);
% T_d = SE3(eul2rotm(phi_d', "XYZ"), p_d).T
T_d = trvec2tform(p_d') * eul2tform(phi_d', "XYZ")
[Ts, vel, acc] = transformtraj(T_0, T_d, tspan, t);

%% plot

% position
x_d = squeeze(Ts(1,4,:)); y_d = squeeze(Ts(2,4,:)); z_d = squeeze(Ts(3,4,:));

figure("Name", "Position");
plot(t, x_d);
hold on
plot(t, y_d);
plot(t, z_d);
legend(["x_d", "y_d", "z_d"])

% velocity
dp_d = vel(4:6, :)';
dx_d = dp_d(:, 1); dy_d = dp_d(:, 2); dz_d = dp_d(:, 3);

figure("Name", "Velocity");
plot(t, dx_d);
hold on
plot(t, dy_d);
plot(t, dz_d);
legend(["dx_d", "dy_d", "dz_d"])

% orientation (euler via slerp)
R_d = squeeze(Ts(1:3, 1:3,:)); eul_d = rotm2eul(R_d, "XYZ");
vphi_d = eul_d(:, 1); th_d = eul_d(:, 2); psi_d = eul_d(:, 3);

figure("Name", "Orientation");
plot(t, vphi_d);
hold on
plot(t, th_d);
plot(t, psi_d);
legend(["\phi_d", "\theta_d", "\psi_d"])

% angular velocity
w_d = vel(1:3, :)';
w_x_d = w_d(:, 1); w_y_d = w_d(:, 2); w_z_d = w_d(:, 3);

figure("Name", "Angular Velocity");
plot(t, w_x_d);
hold on
plot(t, w_y_d);
plot(t, w_z_d);
legend(["\omega_x_d", "\omega_y_d", "\omega_z_d"])

% euler velocity
% w = T(phi) * dphi -> dphi = inv(T(phi)) * w

R_ds = Ts(1:3,1:3,:);
phi_ds = rotm2eul(R_ds, "XYZ");

dphi = zeros(size(phi_ds));
for i=1:size(phi_ds, 1)
	dphi(i,:) = inv(T_fcn(phi_ds(i,:)')) * w_d(i,:)';
end

dvphi_d = dphi(:, 1); dth_d = dphi(:, 2); dpsi_d = dphi(:, 3);

figure("Name", "Euler Velocity");
plot(t, dvphi_d);
hold on
plot(t, dth_d);
plot(t, dpsi_d);
legend(["d\phi_d",  "d\theta_d", "d\psi_d"])


