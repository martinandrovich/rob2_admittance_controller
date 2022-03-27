% close all; clear; clc;
format short G
%#ok<*NOPTS>

% load parameters (dh, mass, inertia, ..)
run("robot_param.m");
run("helpers.m"); % load helpers functions (mat(), dhtf(), ..)

% model
r = SerialLink(dh, "name", "Johnny")

% kinematics
T01 = r.A(1:1, q);
T12 = r.A(1:2, q);
T23 = r.A(2:3, q);
T03 = r.A(1:3, q);

% dynamics (cannot be used for Newton-Euler)
r.links(1).m = m1; r.links(1).r = c1; r.links(1).I = I1; r.links(1).Jm = 3.3e-05; r.links(1).B = 3.3e-05; r.links(1).G = 1;
r.links(2).m = m2; r.links(2).r = c2; r.links(2).I = I2; r.links(2).Jm = 3.3e-05; r.links(2).B = 3.3e-05; r.links(2).G = 1;
r.links(3).m = m3; r.links(3).r = c3; r.links(3).I = I3; r.links(3).Jm = 3.3e-05; r.links(3).B = 3.3e-05; r.links(3).G = 1;

% forward kinematics test
T_ee = r.fkine([1 0 0])
double(subs(T03.T, q', [1 0 0]))

% plots
figure
r.plot([0 0 0], "jvec", "nobase"); hold on
trplot(SE3(), "rgb", "thick", 2)

figure
plot_frame = @(T, link) trplot(T, "frame", link, "rgb", "thick", 2, "framelabeloffset", -1, "axis", [-2 2 -2 2 -1 2]);
plot_frame(SE3(), "0"); hold on
plot_frame(r.A(1:1, [0 0 0]), "1")
plot_frame(r.A(1:2, [0 0 0]), "2")
plot_frame(r.A(1:3, [0 0 0]), "2")

figure
r.teach()

