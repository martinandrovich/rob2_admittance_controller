format short G
%#ok<*NOPTS>

% load model and dynamics first, then load test() from helpers.m
run("helpers.m")
% test = @(x) vpa(subs(x, [q' dq' ddq'], [q_' dq_' ddq_']), 4);

% sub values for test()
q_ = [1 2 3]'; dq_ = [1 2 3]'; ddq_ = [1 2 3]';

%% equations of motion
clc;

L_ = test(L)
M_ = test(M)
C_ = test(C)
g_ = test(g)
Q_ = test(Q)
QL_ = test(QL)

%% dynamics
% clc;

fd_ = test(fd)

%% kinematics
clc;

T01_ = test(T01)
T12_ = test(T12)
T23_ = test(T23)

T02_ = test(T02)
T03_ = test(T03)

p1_ = test(p1)
p2_ = test(p2)
p3_ = test(p3)

%% inertia + jacobians
clc;

I01_ = test(I01)
I02_ = test(I02)
I03_ = test(I03)

JP1_ = test(JP1)
JP2_ = test(JP2)
JP3_ = test(JP3)
JO1_ = test(JO1)
JO2_ = test(JO2)
JO3_ = test(JO3)

J_   = test(J)