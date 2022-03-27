% handles

coriolis = @coriolis_;
dhom = @dhom_;
simulate_ode23 = @simulate_ode23_;
plot_sim = @plot_sim_;

% inline functions

t = @(T) T(1:3, 4);
R = @(T) T(1:3, 1:3);
z = @(T) T(1:3, 3);

% https://se.mathworks.com/matlabcentral/answers/651683-how-to-drop-small-terms-in-symbolic-expression#answer_557288
mat = @(T) vpa(mapSymType(vpa(simplify(T), 3), 'vpareal', @(x) piecewise(abs(x)<=1e-10, 0, x)), 3);

test = @(x) vpa(subs(subs(subs(x, q, q_), dq, dq_), ddq, ddq_), 4);
test = @(x) vpa(subs(x, [q' dq' ddq'], [q_' dq_' ddq_']), 4);

dhtf = @(d, theta, a, alpha) [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta) ; 
                              sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta) ;
                                       0             sin(alpha)             cos(alpha)            d ;
                                       0                      0                      0            1];

% implementations

function p = dhom_(p)
    % dhom = @(x) subsref(x/x(end), struct('type','()','subs',{{1:length(x)-1}}));
    p = p/p(end);
    p = p(1:end-1)
end

function C = coriolis_(M)
    n = height(M);
    C = sym(zeros(size(M)));
    for k = 1:n % rows
        for j = 1:n % cols
            % C(k,j) = Σ(i,n) 1/2 * [dM(k,j)/dq(i) + dM(k,i)/dq(j) + dM(i,j)/dq(k)] * dq(i)
            C(k,j) = sum(arrayfun(@(i) 1/2 * [ diff(M(k,j), sym("q"+i)) + diff(M(k,i), sym("q"+j)) + diff(M(i,j), sym("q"+k)) ] * sym("dq"+i), 1:n));
        end
    end
end

function [t, y] = simulate_ode23_(odefcn, tspan, y0)
    f = waitbar(0,"0","name","Simulating ODE...", "WindowStyle", "Modal");
    ode_stats = @(t, y, flag) double(waitbar(str2double(mat2str(t))/tspan(2), f, sprintf("%.3f / %.3f seconds", str2double(mat2str(t)), tspan(2))))*0; % must return 0 for ODE
    % ode_stats = @(t, y, flag) fprintf("%s / %f\n", mat2str(t(1)), tspan(2))*0
    [t, y] = ode23(odefcn, tspan, y0, odeset("OutputFcn", ode_stats));
    close(f);
end

function plot_sim_(t, y)
    set(0,'DefaultLineLineWidth',2)

    figure;
    plot(t,y(:,1))
    hold on;
    plot(t,y(:,2))
    plot(t,y(:,3))
    xlabel("time [s]"); ylabel("joint position [rad]")
    legend(["q_1", "q_2", "q_3"])
    
    figure
    plot(t,y(:,4))
    hold on
    plot(t,y(:,5))
    plot(t,y(:,6))
    xlabel("time [s]"); ylabel("joint velocity [rad/s]")
    legend(["dq_1", "dq_2", "dq_3"])
end