% HUSKY_TASK4_LINEFOLLOW — Task 4: Follow a line ax + by + c = 0
% -------------------------------------------------------------------------
% Goal
%   Make the Husky (unicycle model) converge to and track an infinite line
%   defined by ax + by + c = 0 using a simple lateral + heading feedback:
%
%   Cross-track distance (signed):
%       d = (a*x + b*y + c) / sqrt(a^2 + b^2)
%
%   Desired line tangent angle (constant for a straight line):
%       theta_star = atan2(-a, b)      % tangent vector t = [b, -a]
%
%   Heading error:
%       theta_e = wrapToPi(theta_star - theta)
%
%   Control (unicycle):
%       v = v0 (constant forward speed, clipped to [0, vMax])
%       omega = -K_d * d + K_theta * theta_e   (clipped to [-wMax, wMax])
%
% What this script does
%   • Simulates convergence to the chosen line from an off-line start pose.
%   • Plots XY path with the line drawn, cross-track and heading errors,
%     and the commanded v(t) and omega(t).
%   • Prints performance metrics (max/mean/RMS cross-track error).
%
% Requirements
%   • MATLAB R2021a+
%   • husky_utils.m on your MATLAB path
%
% -------------------------------------------------------------------------

clear; clc; close all;

%% USER SETTINGS -----------------------------------------------------------
% Line: ax + by + c = 0
% Example 1: horizontal line y = 1  -> a=0, b=1, c=-1
% Example 2: vertical line   x = 2  -> a=1, b=0, c=-2
% Example 3: slanted line    x - y + 0.5 = 0 -> a=1, b=-1, c=0.5
a = 1;  b = 1;  c = -6;     % <-- EDIT HERE for your desired line

% Initial pose [x; y; theta] (meters, radians)
x0 = [-2.0; -0.5; 0.2];

% Timing
dt = 0.01;       % [s]
Tf = 25.0;       % [s]
T  = 0:dt:Tf;
N  = numel(T);

% Robot parameters
P = husky_utils.params('r',0.165,'W',0.55,'vMax',1.0,'wMax',1.5);

% Controller parameters
v0      = 0.6;   % [m/s] constant forward speed
K_d     = 1.0;   % [1/s] cross-track gain
K_theta = 2.0;   % [1/s] heading gain

% Optional: enable small integral action on d (helps reject constant bias)
use_integrator = false;
Ki_d           = 0.0;  % [1/s^2], only used if use_integrator=true
i_d            = 0.0;  % integrator state

%% PRE-COMPUTATIONS --------------------------------------------------------
den = hypot(a, b);                 % sqrt(a^2 + b^2)
assert(den > eps, 'Line is degenerate: (a,b) cannot both be zero.');
theta_star = atan2(-a, b);         % tangent angle (constant)

%% PRE-ALLOCATE ------------------------------------------------------------
X     = zeros(3, N);
Vcmd  = zeros(1, N);
Wcmd  = zeros(1, N);
D     = zeros(1, N);               % cross-track distance d(t)
TH_E  = zeros(1, N);               % heading error theta_e(t)

X(:,1) = x0;

%% MAIN LOOP ---------------------------------------------------------------
for k = 1:N-1
    % Cross-track distance and heading error
    x = X(1,k);  y = X(2,k);  th = X(3,k);
    d = (a*x + b*y + c) / den;
    theta_e = husky_utils.wrapToPi(theta_star - th);

    % Optional PI on d
    if use_integrator
        i_d = i_d + dt * d;
    end

    % Control commands
    v = v0;
    w = -K_d * d + K_theta * theta_e + (use_integrator * Ki_d * i_d);

    % Saturation
    v = husky_utils.sat(v, 0, P.vMax);           % forward-only
    w = husky_utils.sat(w, -P.wMax, P.wMax);

    % Save
    Vcmd(k) = v;  Wcmd(k) = w;  D(k) = d;  TH_E(k) = theta_e;

    % Integrate dynamics
    X(:,k+1) = husky_utils.rk4step_unicycle(X(:,k), v, w, dt);
end
% Last-sample bookkeeping
Vcmd(N) = Vcmd(N-1);
Wcmd(N) = Wcmd(N-1);
x = X(1,N); y = X(2,N); th = X(3,N);
D(N)    = (a*x + b*y + c) / den;
TH_E(N) = husky_utils.wrapToPi(theta_star - th);

%% METRICS -----------------------------------------------------------------
d_abs   = abs(D);
d_rms   = sqrt(mean(D.^2));
d_mean  = mean(d_abs);
d_max   = max(d_abs);
fprintf('[INFO] Cross-track error |d|: mean = %.3f m, RMS = %.3f m, max = %.3f m\n', d_mean, d_rms, d_max);

%% PLOTS -------------------------------------------------------------------
figure('Name','Task 4 — Line Following','Color','w');

% (1) XY Path + line
subplot(2,2,[1 3]);
plot(X(1,:), X(2,:), 'LineWidth', 1.8, 'DisplayName','Path'); hold on; grid on; axis equal;
plot(X(1,1), X(2,1), 'ko', 'MarkerFaceColor','g', 'DisplayName','Start');

% Draw the line across the plot extents
% First determine tentative bounds from the trajectory with a margin
minx = min(X(1,:)); maxx = max(X(1,:));
miny = min(X(2,:)); maxy = max(X(2,:));
mx = (maxx-minx); my = (maxy-miny);
if mx < 1, mx = 1; end; if my < 1, my = 1; end
minx = minx - 0.2*mx;  maxx = maxx + 0.2*mx;
miny = miny - 0.2*my;  maxy = maxy + 0.2*my;

if abs(b) > 1e-12
    % y = -(a*x + c)/b
    xx = linspace(minx, maxx, 200);
    yy = -(a*xx + c)/b;
else
    % vertical line x = -c/a
    xx = -c/a * ones(1,200);
    yy = linspace(miny, maxy, 200);
end
plot(xx, yy, 'c--', 'LineWidth', 1.2, 'DisplayName', sprintf('Line: %gx + %gy + %g = 0', a, b, c));

xlabel('x [m]'); ylabel('y [m]'); title('XY Path with target line');
legend('Location','best');

% (2) Errors vs time
subplot(2,2,2);
plot(T, D, 'LineWidth', 1.6, 'DisplayName','d(t) [m]'); hold on;
plot(T, TH_E, 'LineWidth', 1.6, 'DisplayName','\theta_e(t) [rad]');
grid on; xlabel('t [s]'); ylabel('Error');
title('Cross-track and heading errors');
legend('Location','best');
ylim([-2 2])

% (3) Commands
subplot(2,2,4);
plot(T, Vcmd, 'LineWidth', 1.6, 'DisplayName','v(t) [m/s]'); hold on;
plot(T, Wcmd, 'LineWidth', 1.6, 'DisplayName','\omega(t) [rad/s]');
grid on; xlabel('t [s]'); ylabel('Command');
title('Commanded v(t), \omega(t)');
legend('Location','best');
ylim([-1 1])
%% (Optional) Save figure
% saveas(gcf, 'husky_task4_linefollow.png');
