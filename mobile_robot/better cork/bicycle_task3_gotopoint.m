% File: bicycle_task3_gotopoint.m
% -------------------------------------------------------------
% Task 3: Go-to-Point control for the bicycle kinematic model
% -------------------------------------------------------------
clc; clear; close all;

%% Vehicle & controller parameters
L          = 2.5;                 % wheelbase [m]
kv         = 1.0;                 % speed gain (>0)
kth        = 2.0;                 % steering gain (>0)
v_max      = 2.0;                 % speed limit [m/s]
gamma_max  = deg2rad(35);         % steering angle limit [rad]
tol_pos    = 0.05;                % stop when within 5 cm

%% Initial state and goal
x = 0;  y = 0;  th = deg2rad(0);  % start pose
xg = 5; yg = 5;                   % goal point

%% Simulation setup
dt = 0.01;  T = 20;  N = round(T/dt);
t  = (0:N-1)'*dt;

% Preallocate logs
X = zeros(N,1); Y = zeros(N,1); TH = zeros(N,1);
Vcmd = zeros(N,1); Gcmd = zeros(N,1);

k_end = N;   % will store the last valid index actually used

%% Main loop
for k = 1:N
    % --- Geometry to goal ---
    dx = xg - x;           dy = yg - y;
    rho = hypot(dx, dy);   th_des = atan2(dy, dx);

    % --- Control laws (with saturation) ---
    v_cmd     = kv * rho;
    gamma_cmd = kth * angdiff_wrap(th_des, th);
    v     = clamp(v_cmd,     -v_max,  v_max);
    gamma = clamp(gamma_cmd, -gamma_max, gamma_max);

    % Stop near goal
    if rho < tol_pos
        v = 0;
    end

    % --- Bicycle kinematics ---
    xdot  = v*cos(th);
    ydot  = v*sin(th);
    thdot = (v/L) * tan(gamma);

    % --- Integrate (Euler) ---
    x  = x  + xdot*dt;   y  = y  + ydot*dt;   th = th + thdot*dt;

    % --- Log ---
    X(k)=x; Y(k)=y; TH(k)=th; Vcmd(k)=v; Gcmd(k)=gamma;

    % Early exit if settled for a while
    if rho < tol_pos && k > 50
        k_end = k;
        break;
    end
end

%% --- Crop all logs to the same length ---
X = X(1:k_end); Y = Y(1:k_end); TH = TH(1:k_end);
Vcmd = Vcmd(1:k_end); Gcmd = Gcmd(1:k_end); t = t(1:k_end);

%% Plots
figure;
plot(X, Y, 'b-', 'LineWidth', 2); hold on; grid on; axis equal;
plot(xg, yg, 'rx', 'MarkerSize', 12, 'LineWidth', 2);
xlabel('x [m]'); ylabel('y [m]'); title('Go-to-Point Trajectory');
legend('path','goal', 'Location','best');

figure;
subplot(2,1,1); plot(t, Vcmd, 'LineWidth', 1.8); grid on;
ylabel('v [m/s]'); title('Commands');
subplot(2,1,2); plot(t, rad2deg(Gcmd), 'LineWidth', 1.8); grid on;
ylabel('\gamma [deg]'); xlabel('time [s]');

% Report final error
pos_err = hypot(xg - X(end), yg - Y(end));
fprintf('Final position error = %.3f m\n', pos_err);

%% ---- helpers ----
function d = angdiff_wrap(a, b)
% Angle difference in (-pi, pi]
    d = atan2(sin(a-b), cos(a-b));
end

function y = clamp(u, lo, hi)
    y = min(max(u, lo), hi);
end
