% File: bicycle_task4_follow_line.m
% -------------------------------------------------------------
% Task 4: Follow a straight line ax + by + c = 0 (bicycle model)
% Steering law: gamma = -Kd*d + Kth*angdiff(theta_star, theta)
% -------------------------------------------------------------
clc; clear; close all;

%% Vehicle & controller parameters
L          = 2.5;                 % wheelbase [m]
v_ref      = 1.2;                 % constant forward speed [m/s]
Kd         = 1.5;                 % lateral error gain (>0)
Kth        = 2.0;                 % heading alignment gain (>0)
gamma_max  = deg2rad(35);         % steering saturation [rad]

%% Line definition  ax + by + c = 0
a = 1;  b = -2;  c = 4;           % <- try other lines later
den = hypot(a,b);                 % sqrt(a^2+b^2)
theta_star = atan2(-a, b);        % line tangent direction

%% Initial pose (off the line & misaligned)
x = 0; y = 0; theta = deg2rad(90);  % start facing up

%% Simulation setup
dt = 0.01;  T = 20;  N = round(T/dt);
t  = (0:N-1)'*dt;

% Logs
X = zeros(N,1); Y = zeros(N,1); TH = zeros(N,1);
D = zeros(N,1); Gcmd = zeros(N,1);

%% Main loop
for k = 1:N
    % Lateral distance to line (signed, + on normal side)
    d = (a*x + b*y + c)/den;

    % Steering law
    gamma_cmd = -Kd*d + Kth*angdiff_wrap(theta_star, theta);
    gamma = clamp(gamma_cmd, -gamma_max, gamma_max);

    % Constant forward speed
    v = v_ref;

    % Bicycle kinematics
    xdot  = v*cos(theta);
    ydot  = v*sin(theta);
    thdot = (v/L)*tan(gamma);

    % Integrate
    x = x + xdot*dt;
    y = y + ydot*dt;
    theta = theta + thdot*dt;

    % Log
    X(k)=x; Y(k)=y; TH(k)=theta; D(k)=d; Gcmd(k)=gamma;
end

%% ---- Plots ----
figure; hold on; grid on; axis equal; xlabel('x [m]'); ylabel('y [m]');
title('Line following (path and target line)');

% plot the line robustly
xr = linspace(min(X)-1, max(X)+1, 200);
if abs(b) > 1e-12
    yr = (-(a*xr + c))/b;
    plot(xr, yr, 'k--', 'LineWidth', 1.5);
else
    xline(-c/a, 'k--', 'LineWidth', 1.5);
end
plot(X, Y, 'b-', 'LineWidth', 2);
legend('target line','robot path', 'Location','best');

figure;
subplot(2,1,1);
plot(t, D, 'LineWidth', 1.6); grid on; ylabel('d(t) [m]');
title('Lateral distance to line');
subplot(2,1,2);
plot(t, rad2deg(Gcmd), 'LineWidth', 1.6); grid on;
ylabel('\gamma [deg]'); xlabel('time [s]');

% Report steady errors (last 2 seconds)
idx = t >= (T-2);
d_ss   = mean(D(idx));
head_err_ss = mean(angdiff_wrap(theta_star, TH(idx)));
fprintf('Steady-state lateral error d ≈ %.3f m\n', d_ss);
fprintf('Steady-state heading error ≈ %.3f deg\n', rad2deg(head_err_ss));

%% ---- helpers ----
function d = angdiff_wrap(a, b)
    d = atan2(sin(a-b), cos(a-b));   % in (-pi, pi]
end
function y = clamp(u, lo, hi)
    y = min(max(u, lo), hi);
end
