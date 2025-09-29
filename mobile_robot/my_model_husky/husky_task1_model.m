% HUSKY_TASK1_MODEL  — Task 1: Unicycle (diff-drive) kinematic simulation
% -------------------------------------------------------------------------
% What this script does
%   • Simulates a Husky-like robot using unicycle kinematics:
%       x_dot = v * cos(theta)
%       y_dot = v * sin(theta)
%       th_dot = omega
%   • Demonstrates three command profiles:
%       1) 'line'   : straight line (omega = 0)
%       2) 'circle' : constant-curvature motion (v, omega constant)
%       3) 'sine'   : time-varying omega = A*sin(2*pi*f*t)
%   • Plots XY path, heading theta(t), and commanded v(t), omega(t)
%
% Requirements
%   • MATLAB R2021a+
%   • husky_utils.m on your MATLAB path
%
% Usage
%   • Edit the block "USER SETTINGS" then run the script.
%   • Switch 'profile' to 'line', 'circle', or 'sine' to try different demos.
%
% Notes
%   • All arrays share the same time base T to avoid plot length mismatches.
%   • States are wrapped with wrapToPi() to keep angles in (-pi, pi].
%
% -------------------------------------------------------------------------

clear; clc; close all;

%% USER SETTINGS -----------------------------------------------------------
profile = 'circle';   % options: 'line' | 'circle' | 'sine'
x0      = [0; 0; 0];  % initial pose [x; y; theta] (meters, radians)

% Simulation timing
dt  = 0.01;           % [s] time step
Tf  = 20.0;           % [s] total duration
T   = 0:dt:Tf;        % time vector
N   = numel(T);

% Robot parameters (edit to match your platform)
P = husky_utils.params('r',0.165,'W',0.55,'vMax',1.0,'wMax',1.5);

% Demo command parameters (only used by some profiles)
v_line   = 0.6;       % [m/s] for straight line
v_circle = 0.6;       % [m/s] for circle
w_circle = 0.3;       % [rad/s] for circle  (Radius = v/w)
v_sine   = 0.6;       % [m/s] for sine demo
A_omega  = 0.6;       % [rad/s] amplitude for sine omega
f_omega  = 0.1;       % [Hz] freq for sine omega

%% PRE-ALLOCATE ------------------------------------------------------------
X    = zeros(3, N);   % states: [x;y;theta]
Vcmd = zeros(1, N);   % commanded linear speed v(t)
Wcmd = zeros(1, N);   % commanded angular speed omega(t)

X(:,1) = x0;

%% MAIN SIMULATION LOOP ----------------------------------------------------
for k = 1:N-1
    t = T(k);

    % Command profile (v, omega)
    switch profile
        case 'line'    % constant v, zero omega
            v = v_line; 
            w = 0.0;

        case 'circle'  % constant v and omega => circle of radius R = v/w
            v = v_circle;
            w = w_circle;

        case 'sine'    % constant v, sinusoidal omega
            v = v_sine;
            w = A_omega * sin(2*pi*f_omega*t);

        otherwise
            error('Unknown profile "%s". Choose: line | circle | sine', profile);
    end

    % Clip to robot limits, then save commands
    v = husky_utils.sat(v, -P.vMax, P.vMax);
    w = husky_utils.sat(w, -P.wMax, P.wMax);
    Vcmd(k) = v;  Wcmd(k) = w;

    % Integrate one step
    X(:,k+1) = husky_utils.rk4step_unicycle(X(:,k), v, w, dt);
end

% Repeat last command to match lengths with time/state arrays
Vcmd(N) = Vcmd(N-1);
Wcmd(N) = Wcmd(N-1);

%% OPTIONAL: PRINT SOME STATS ---------------------------------------------
if strcmp(profile,'circle')
    R = v_circle / w_circle;
    fprintf('[INFO] Circle demo: v=%.2f m/s, w=%.2f rad/s -> Radius R=%.2f m\n', ...
        v_circle, w_circle, R);
end
fprintf('[INFO] Final pose: [x=%.3f, y=%.3f, theta=%.3f rad]\n', X(1,end), X(2,end), X(3,end));

%% PLOTS -------------------------------------------------------------------
figure('Name','Task 1 — Husky Unicycle Simulation','Color','w');

% (1) XY Path
subplot(2,2,[1 3]);
plot(X(1,:), X(2,:), 'LineWidth', 1.8); grid on; axis equal;
hold on;
plot(X(1,1), X(2,1), 'ko', 'MarkerFaceColor','g', 'DisplayName','Start');
plot(X(1,end), X(2,end), 'ks', 'MarkerFaceColor','r', 'DisplayName','End');
xlabel('x [m]'); ylabel('y [m]'); title(sprintf('Path (%s profile)', profile));
legend('Path','Start','End','Location','best');

% (2) Heading vs time
subplot(2,2,2);
plot(T, X(3,:), 'LineWidth', 1.6); grid on;
xlabel('t [s]'); ylabel('\theta [rad]'); title('Heading \theta(t)');

% (3) Commands vs time
subplot(2,2,4);
plot(T, Vcmd, 'LineWidth', 1.6); hold on;
plot(T, Wcmd, 'LineWidth', 1.6);
grid on; xlabel('t [s]'); ylabel('Command');
legend('v(t) [m/s]', '\omega(t) [rad/s]','Location','best');
title('Commanded v(t), \omega(t)');
ylim([0 1])
%% (Optional) Save figure
% saveas(gcf, sprintf('husky_task1_profile_%s.png', profile));
