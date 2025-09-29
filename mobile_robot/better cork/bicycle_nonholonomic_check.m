% File: bicycle_nonholonomic_check.m
% -------------------------------------------------------------
% Task 2: Verify the nonholonomic (no-sideways) constraint:
%          ydot*cos(theta) - xdot*sin(theta) = 0
% We simulate the bicycle model and compute the residual.
% -------------------------------------------------------------

clc; clear; close all;

%% Vehicle parameters
L = 2.5;                     % wheelbase [m]

%% Input profiles (you can tweak these)
v_fun     = @(t) 1.5 + 0.5*sin(0.3*t);        % forward speed [m/s]
gamma_fun = @(t) deg2rad(15)*sin(0.5*t);      % steering angle [rad]

%% Simulation settings
dt = 0.01;   T = 15;   N = round(T/dt);
t = (0:N-1)'*dt;

% State: [x y theta]
x = 0; y = 0; th = 0;

% Storage
X  = zeros(N,1);  Y  = zeros(N,1);  TH = zeros(N,1);
Xd = zeros(N,1);  Yd = zeros(N,1);  THd = zeros(N,1);  % time derivatives
E  = zeros(N,1);                                    % constraint residual

%% Simulation loop (forward Euler)
for k = 1:N
    v     = v_fun(t(k));
    gamma = gamma_fun(t(k));

    % Bicycle kinematics
    xdot  = v*cos(th);
    ydot  = v*sin(th);
    thdot = (v/L)*tan(gamma);

    % Save derivatives (for residual check)
    Xd(k)  = xdot;   Yd(k)  = ydot;   THd(k) = thdot;

    % Constraint residual (should be ~0)
    E(k) = ydot*cos(th) - xdot*sin(th);

    % Integrate state
    x  = x  + xdot*dt;
    y  = y  + ydot*dt;
    th = th + thdot*dt;

    % Store state
    X(k) = x;  Y(k) = y;  TH(k) = th;
end

%% Plots
figure;
plot(X, Y, 'LineWidth', 2); axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Trajectory (time-varying v and \gamma)');

figure;
subplot(3,1,1); plot(t, arrayfun(v_fun,t), 'LineWidth', 1.5); grid on;
ylabel('v [m/s]'); title('Inputs and Heading Rate');
subplot(3,1,2); plot(t, rad2deg(arrayfun(gamma_fun,t)), 'LineWidth', 1.5); grid on;
ylabel('\gamma [deg]');
subplot(3,1,3); plot(t, THd, 'LineWidth', 1.5); grid on;
ylabel('\theta dot [rad/s]'); xlabel('time [s]');

figure;
plot(t, E, 'LineWidth', 2); grid on;
xlabel('time [s]'); ylabel('e(t) = ydot cos\theta - xdot sin\theta');
title('Nonholonomic residual (should be ~ 0)');
yline(0,'k--');

%% Sanity check: v = 0 segment -> thetadot should be 0
% Find times where v is (almost) zero and report max |thetadot|
v_vals = arrayfun(v_fun, t);
idx_zero_v = v_vals < 1e-3;
fprintf('Max |thetadot| when v ~ 0: %.3e rad/s\n', max(abs(THd(idx_zero_v))+eps));
fprintf('Max |constraint residual| over sim: %.3e\n', max(abs(E)));
