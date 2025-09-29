% File: bicycle_model_sim.m
% ---------------------------------------------
% Task 1: Bicycle Model Simulation
% This script simulates a simple bicycle kinematic model
% with constant velocity and steering angle inputs.
% ---------------------------------------------

clc; clear; close all;

%% Parameters of the vehicle
L = 2.5;          % wheelbase [m]
v = 2.0;          % forward velocity [m/s]
gamma = deg2rad(20);  % steering angle [rad]

%% Simulation setup
dt = 0.01;        % time step [s]
T = 10;           % total simulation time [s]
N = T/dt;         % number of steps

% State vector: [x, y, theta]
x = 0; y = 0; theta = 0;   % initial condition

% For saving results
X = zeros(N,1);
Y = zeros(N,1);
Theta = zeros(N,1);
time = (0:N-1)'*dt;

%% Simulation loop (Euler integration)
for k = 1:N
    % --- Bicycle model equations ---
    dx = v * cos(theta);
    dy = v * sin(theta);
    dtheta = (v/L) * tan(gamma);

    % --- Update state (Euler method) ---
    x = x + dx*dt;
    y = y + dy*dt;
    theta = theta + dtheta*dt;

    % --- Save results ---
    X(k) = x;
    Y(k) = y;
    Theta(k) = theta;
end

%% Plot results
figure;
plot(X, Y, 'b-', 'LineWidth', 2);
xlabel('X [m]'); ylabel('Y [m]');
title('Bicycle Model Trajectory');
grid on; axis equal;

figure;
plot(time, Theta, 'r-', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('\theta [rad]');
title('Heading Angle vs Time');
grid on;
