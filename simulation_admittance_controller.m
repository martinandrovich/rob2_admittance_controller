close all; clear; clc;
format short G
%#ok<*NOPTS>

% todo:
% inv(T_0d) in workspace
% select proper gains
% output data for plots
% make plots

% simulation timing
tspan = [0 25]; dt = 0.01;
t = linspace(tspan(1), tspan(2), tspan(2)/dt);
set_param("admittance_controller_full", "StopTime", num2str(tspan(2)))

% input wrench
data = [
	zeros(5/dt, 6) ;                            % nothing for 5 sec
	repmat([1 2 3], 5/dt, 1) zeros(5/dt, 3) ;   % force for 5 sec
	zeros(5/dt, 6) ;                            % nothing for 5 sec
	zeros(5/dt, 3) repmat([1 0.5 1], 5/dt, 1) ; % moment for 5 sec
	zeros(5/dt, 6)                              % nothing for 5 sec
];
h_0e = timeseries(data, t);

% impedance parameters
% SET THESE TO REAL VALUES
Kp = eye(3) * 16;
Kd = eye(3) * 4;
Md = eye(3) * 1;

% trajectory
T_0 = trvec2tform([0.2 -0.3 0.3]) * eul2tform([-0.2 -0.5 -0.6], "XYZ")
T_d = trvec2tform([0.6    0 0.4]) * eul2tform([-1.2 1 -1.2], "XYZ")
[Ts, vel, acc] = transformtraj(T_0, T_d, tspan, t);
R_ds = Ts(1:3,1:3,:);

% euler angles
data = [squeeze(Ts(1:3,4,:))' rotm2eul(R_ds, "XYZ")]; % [p_d phi_d]
p_d = timeseries(data, t);

% data = [vel(4:6, :)' zeros(length(t), 3)]; % [dp_d dphi_d]
% dp_d = timeseries(data, t);

% quaternions
data = rotm2quat(R_ds);
q_d = timeseries(data, t); % [eta eps]
T_0d = timeseries(Ts, t); % [4x4]'s

% initial conditions
p_cd0 = [0 0 0]'; dp_cd0 = [0 0 0]'; % trans
phi_cd0 = [0 0 0]'; dphi_cd0 = [0 0 0]'; % euler
q_d_cd0 = [1 0 0 0]; w_d_cd0 = [0 0 0]'; % quat

% symbolic expression for T(phi) matrix for XYZ-euler
syms phi th psi
dphi_ = eye(3); dphi_ = dphi_(:, 1);
dth_  = eye(3) * rotx(phi); dth_ = dth_(:, 2);
dpsi_ = eye(3) * rotx(phi) * roty(th); dpsi_ = dpsi_(:, 3);
T_ = [dphi_ dth_ dpsi_]
T_fcn = @(vec) double(subs(T_, [phi th psi], vec'));

