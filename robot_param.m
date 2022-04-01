% load by running 'run("robot_param.m");'
% https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/

% symbols
syms   q [3 1] real
syms  dq [3 1] real
syms ddq [3 1] real
syms tau [3 1] real

% link lengths [m]
d1 = 0.1625;
a2 = -0.425;
a3 = -0.3922;

% dh params, [q d a α]
dh = [0 d1  0 pi/2 ;
      0  0 a2    0
      0  0 a3    0 ];

% center of mass (in link frame)
c1 = [0 -0.02561 0.00193]';
c2 = [0.2125 0 0.11336]';
c3 = [0.15 0 0.0265]';

% inertia (in link frame)
I1 = diag([0.0084 0.0064 0.0084]);
I2 = diag([0.0078 0.21 0.21]);
I3 = diag([0.0016 0.0462 0.0462]);

% masses [kg]
m1 = 3.761;
m2 = 8.058;
m3 = 2.846;

% make everything symbolic (except alpha)
syms d1 a2 a3 real
syms m c1 c2 c3 [3 1] real
% syms I1 I2 I3 [3 3] real
I1 = diag(diag(sym("I1", [3 3], "real")));
I2 = diag(diag(sym("I2", [3 3], "real")));
I3 = diag(diag(sym("I3", [3 3], "real")));
