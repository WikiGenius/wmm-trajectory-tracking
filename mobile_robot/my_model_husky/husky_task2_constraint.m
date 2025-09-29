% HUSKY_TASK2_CONSTRAINT — Task 2: Verify nonholonomic (no-sideways) constraint
% -------------------------------------------------------------------------
% Goal
%   Show that a differential-drive (unicycle) model has zero lateral
%   velocity in the body frame, i.e. the nonholonomic constraint holds.
%
% Theory
%   Unicycle kinematics:
%       xdot = v * cos(theta)
%       ydot = v * sin(theta)
%       thdot = omega
%   Lateral velocity in world components:
%       v_lat_world = ydot*cos(theta) - xdot*sin(theta)  ->  0
%   Body-frame velocity (R(-theta)*[xdot; ydot]):
%       [vx_b; vy_b] = [ cos th   sin th; -sin th  cos th ] [xdot; ydot]
%       => vy_b = -sin(theta)*xdot + cos(theta)*ydot       ->  0
%
% What this script does
%   • Simulates a short trajectory (line/circle/sine profile).
%   • Computes the residual using model derivatives (xdot, ydot from v,theta).
%   • Also computes residual using finite differences of the simulated states.
%   • Plots the residuals and reports max |residual|.
%
% Requirements
%   • MATLAB R2021a+
%   • husky_utils.m on your MATLAB path
%
% -------------------------------------------------------------------------

clear; clc; close all;

%% USER SETTINGS -----------------------------------------------------------
profile = 'circle';   % 'line' | 'circle' | 'sine'
x0      = [0; 0; 0];  % initial pose [x; y; theta]

% Simulation timing
dt  = 0.01;           % [s]
Tf  = 20.0;           % [s]
T   = 0:dt:Tf;        % time vector
N   = numel(T);

% Robot parameters (edit to match your platform)
P = husky_utils.params('r',0.165,'W',0.55,'vMax',1.0,'wMax',1.5);

% Command profile parameters (same as Task 1 defaults)
v_line   = 0.6;       % [m/s]
v_circle = 0.6;       % [m/s]
w_circle = 0.3;       % [rad/s]
v_sine   = 0.6;       % [m/s]
A_omega  = 0.6;       % [rad/s]
f_omega  = 0.1;       % [Hz]

%% PRE-ALLOCATE ------------------------------------------------------------
X     = zeros(3, N);     % states [x;y;theta]
Vcmd  = zeros(1, N);     % v(t)
Wcmd  = zeros(1, N);     % w(t)
XdotM = zeros(1, N);     % xdot (model)
YdotM = zeros(1, N);     % ydot (model)

X(:,1) = x0;

%% SIMULATION + MODEL DERIVATIVES -----------------------------------------
for k = 1:N-1
    t = T(k);

    % Command (v, omega) per profile
    switch profile
        case 'line'
            v = v_line; w = 0.0;
        case 'circle'
            v = v_circle; w = w_circle;
        case 'sine'
            v = v_sine;  w = A_omega * sin(2*pi*f_omega*t);
        otherwise
            error('Unknown profile "%s". Choose: line | circle | sine', profile);
    end

    % Clip to robot limits
    v = husky_utils.sat(v, -P.vMax, P.vMax);
    w = husky_utils.sat(w, -P.wMax, P.wMax);
    Vcmd(k) = v;  Wcmd(k) = w;

    % Model-based derivatives at current state (for residual)
    th = X(3,k);
    XdotM(k) = v * cos(th);
    YdotM(k) = v * sin(th);

    % Integrate one step
    X(:,k+1) = husky_utils.rk4step_unicycle(X(:,k), v, w, dt);
end

% Last-sample commands & model derivatives (t = T(end))
switch profile
    case 'line'
        v = v_line; w = 0.0;
    case 'circle'
        v = v_circle; w = w_circle;
    case 'sine'
        v = v_sine;  w = A_omega * sin(2*pi*f_omega*T(end));
end
v = husky_utils.sat(v, -P.vMax, P.vMax);
w = husky_utils.sat(w, -P.wMax, P.wMax);
Vcmd(N) = v; Wcmd(N) = w;
XdotM(N) = v * cos(X(3,N));
YdotM(N) = v * sin(X(3,N));

%% RESIDUALS ---------------------------------------------------------------
theta = X(3,:);

% World-components residual using model derivatives (length N)
res_model = YdotM .* cos(theta) - XdotM .* sin(theta);

% Body-frame lateral velocity using model derivatives (should equal res_model)
vy_body_model = -sin(theta).*XdotM + cos(theta).*YdotM;

% Finite-difference derivatives (length N-1)
Xd_fd = diff(X(1,:)) ./ diff(T);
Yd_fd = diff(X(2,:)) ./ diff(T);
th_fd = theta(1:end-1);  % align with forward differences

% Residual from finite differences (length N-1)
res_fd = Yd_fd .* cos(th_fd) - Xd_fd .* sin(th_fd);
vy_body_fd = -sin(th_fd).*Xd_fd + cos(th_fd).*Yd_fd;

% Stats
max_res_model = max(abs(res_model));
max_res_fd    = max(abs(res_fd));

fprintf('[INFO] Max |residual| (model derivatives):    %.3e\n', max_res_model);
fprintf('[INFO] Max |residual| (finite differences):   %.3e\n', max_res_fd);

%% PLOTS -------------------------------------------------------------------
figure('Name','Task 2 — Nonholonomic Constraint Check','Color','w');

% (1) XY Path
subplot(2,2,1);
plot(X(1,:), X(2,:), 'LineWidth', 1.6); grid on; axis equal;
hold on;
plot(X(1,1), X(2,1), 'ko', 'MarkerFaceColor','g', 'DisplayName','Start');
plot(X(1,end), X(2,end), 'ks', 'MarkerFaceColor','r', 'DisplayName','End');
xlabel('x [m]'); ylabel('y [m]'); title(sprintf('Path (%s profile)', profile));
legend('Path','Start','End','Location','best');

% (2) Residual (world-components): model vs finite-difference
subplot(2,2,2);
plot(T, res_model, 'LineWidth', 1.6, 'DisplayName','model deriv');
hold on;
plot(T(1:end-1), res_fd, '--', 'LineWidth', 1.2, 'DisplayName','finite diff');
grid on; xlabel('t [s]'); ylabel('residual = ydot cos\theta - xdot sin\theta');
title('Nonholonomic residual'); legend('Location','best');
ylim([-10*eps, 10*eps])

% (3) Body-frame lateral velocity v_y^body (should be ~0)
subplot(2,2,3);
plot(T, vy_body_model, 'LineWidth', 1.6, 'DisplayName','model deriv');
hold on;
plot(T(1:end-1), vy_body_fd, '--', 'LineWidth', 1.2, 'DisplayName','finite diff');
grid on; xlabel('t [s]'); ylabel('v_y^{body} [m/s]');
title('Body-frame lateral velocity'); legend('Location','best');
ylim([-10*eps, 10*eps])

% (4) Commands
subplot(2,2,4);
plot(T, Vcmd, 'LineWidth', 1.6, 'DisplayName','v(t)'); hold on;
plot(T, Wcmd, 'LineWidth', 1.6, 'DisplayName','\omega(t)');
grid on; xlabel('t [s]'); ylabel('Command');
title('Commanded v(t), \omega(t)'); legend('Location','best');
ylim([0, 0.8])
%% (Optional) Save figure
% saveas(gcf, sprintf('husky_task2_profile_%s.png', profile));

% Optional assertion (may fail only due to numerical diff noise)
% tol = 1e-6;
% assert(max_res_model < tol, 'Model residual too large');
