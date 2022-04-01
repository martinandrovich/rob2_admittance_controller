close all; clear; clc;
format short G
%#ok<*NOPTS>

% load model
run("model.m")

%% forward dynamics

% forward dynamics
fd = simplify(inv(M) * (tau - (C * dq + g)));
fd_fun = @(q_, dq_, tau_) double(subs(fd, [q' dq' tau'], [q_' dq_' tau_']));

%% simulation

% initial conditions + timespan
q0 = [0 pi/3 pi/3]; dq0 = [0 0 0];
tspan = [0 1];

% setpoints [q_d dq_d ddq_d]
ref = [[1 2 3]' [0 0 0]' [0 0 0]'];

% simulate
[t, y] = simulate_ode23(@(t, y) odefcn(t, y, fd_fun, ref), tspan, [q0 dq0])
tau_d = y(:,4:6) * -eye(3)

%% plot

close all;
plot_sim(t, y, tau_d);

%%

% ode function (main loop)
function dydt = odefcn(t, y, fd_fun, ref)

	% pos and vel from robot
	q = y(1:3); dq = y(4:6);

	% setpoints
	q_d = ref(:, 1); dq_d = ref(:, 2); ddq_d = ref(:, 3);

	% compute desired torque
	tau_d = invdyn_ctrl(q, dq, q_d, dq_d, ddq_d);
	
	% forward dynamics
	ddq = fd_fun(q, dq, tau_d);
	
	% simulation output
	dydt = [dq ; ddq];

end

% controller (compute desired torque)
function tau = invdyn_ctrl(q, dq, q_d, dq_d, ddq_d)

	% gains
	Kp = eye(3)*200; Kd = eye(3)*50;

	% robot model parameters from workspace
	g = double(subs(evalin("base", "g"), [sym("q", [3 1])'                  ], [q'    ]));
	M = double(subs(evalin("base", "M"), [sym("q", [3 1])'                  ], [q'    ]));
	C = double(subs(evalin("base", "C"), [sym("q", [3 1])' sym("dq", [3 1])'], [q' dq']));
	
	% desired torque
	y = ddq_d + Kd * (dq_d - dq) + Kp * (q_d - q);
	tau = M * y + C * dq + g;
	
end
