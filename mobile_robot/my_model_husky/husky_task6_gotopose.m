% HUSKY_TASK6_GOTOPOSE — Task 6: Go-to-Pose (x_g, y_g, theta_g)
% -------------------------------------------------------------------------
% Goal
%   Drive the robot from (x0, y0, theta0) to a desired pose (xg, yg, thg).
%   Polar-form unicycle controller (forward motion, Samson-style shaping):
%
%       dx    = xg - x,   dy = yg - y
%       rho   = sqrt(dx^2 + dy^2)
%       alpha = wrap(atan2(dy, dx) - theta)
%       beta  = wrap(thg - theta - alpha)
%
%   Control law:
%       v = k_rho * rho * cos(alpha)            (forward-only, then clipped)
%       w = k_alpha * alpha + k_beta * beta     (then clipped)
%
%   Classic stability conditions:
%       k_rho > 0,   k_beta < 0,   k_alpha > k_rho
%
% Features
%   • Forward-only velocity with cos(alpha) shaping (turn-in-place if cos<0)
%   • Dwell-based stop on both distance and final heading
%   • Array truncation to keep plot vectors consistent
%   • Clean plots of XY path, polar errors (rho, alpha, beta), and commands
%
% Requirements
%   • MATLAB R2021a+
%   • husky_utils.m on your MATLAB path
%
% -------------------------------------------------------------------------

clear; clc; close all;

%% USER SETTINGS -----------------------------------------------------------
x0  = [-2.0; -1.5;  2.8];        % initial pose [x; y; theta]
xg  = [ 2.0;  1.0; -0.5];        % goal pose [xg; yg; thg]

% Timing
dt = 0.01;    % [s]
Tf = 50.0;    % [s]
T  = 0:dt:Tf; N = numel(T);

% Robot parameters
P = husky_utils.params('r',0.165,'W',0.55,'vMax',1.0,'wMax',1.5);

% Controller gains (satisfy k_rho>0, k_beta<0, k_alpha>k_rho)
k_rho   = 1.0;
k_alpha = 2.0;
k_beta  = -1.2;

% Stop tolerances and dwell
rho_tol   = 0.03;      % [m] distance tolerance
th_tol    = 2*pi/180;  % [rad] ~2 degrees heading tolerance
dwell_N   = 50;        % consecutive steps within tolerances

% Option: minimum forward speed near target to avoid stalling (set 0 to off)
v_min = 0.0;

%% PRE-ALLOCATE ------------------------------------------------------------
X     = zeros(3, N); X(:,1) = x0;
Vcmd  = zeros(1, N);
Wcmd  = zeros(1, N);
RHO   = zeros(1, N);
ALPHA = zeros(1, N);
BETA  = zeros(1, N);

%% MAIN LOOP ---------------------------------------------------------------
within = 0;
k_end = N;

for k = 1:N-1
    x = X(1,k); y = X(2,k); th = X(3,k);
    dx = xg(1) - x;   dy = xg(2) - y;
    rho   = hypot(dx, dy);
    alpha = husky_utils.wrapToPi(atan2(dy, dx) - th);
    beta  = husky_utils.wrapToPi(xg(3) - th - alpha);

    % Store errors
    RHO(k)   = rho;
    ALPHA(k) = alpha;
    BETA(k)  = beta;

    % Control law (forward-only with Samson shaping)
    v = k_rho * rho * cos(alpha);
    if v_min > 0
        v = max(v, (rho > rho_tol) * v_min); % only apply v_min if not within distance tol
    end
    v = max(0, v);  % forward-only
    w = k_alpha * alpha + k_beta * beta;

    % Saturate
    v = husky_utils.sat(v, 0, P.vMax);
    w = husky_utils.sat(w, -P.wMax, P.wMax);

    % Save commands
    Vcmd(k) = v;  Wcmd(k) = w;

    % Integrate
    X(:,k+1) = husky_utils.rk4step_unicycle(X(:,k), v, w, dt);

    % Stop condition (both position and heading within tolerances)
    th_err = abs(husky_utils.wrapToPi(X(3,k+1) - xg(3)));
    if (rho < rho_tol) && (th_err < th_tol)
        within = within + 1;
        if within >= dwell_N
            k_end = k+1;
            break;
        end
    else
        within = 0;
    end
end

% Close arrays and truncate
Vcmd(k_end) = Vcmd(max(1, k_end-1));
Wcmd(k_end) = Wcmd(max(1, k_end-1));
RHO(k_end)  = RHO(max(1, k_end-1));
ALPHA(k_end)= ALPHA(max(1, k_end-1));
BETA(k_end) = BETA(max(1, k_end-1));

T     = T(1:k_end);
X     = X(:,1:k_end);
Vcmd  = Vcmd(1:k_end);
Wcmd  = Wcmd(1:k_end);
RHO   = RHO(1:k_end);
ALPHA = ALPHA(1:k_end);
BETA  = BETA(1:k_end);

%% REPORT ------------------------------------------------------------------
th_err_final = husky_utils.wrapToPi(X(3,end) - xg(3));
fprintf('[INFO] Final pose:   x=%.3f, y=%.3f, th=%.3f rad\n', X(1,end), X(2,end), X(3,end));
fprintf('[INFO] Final errors: rho=%.4f m, alpha=%.4f rad, beta=%.4f rad, |th_err|=%.4f rad\n', ...
    RHO(end), ALPHA(end), BETA(end), abs(th_err_final));

%% PLOTS -------------------------------------------------------------------
figure('Name','Task 6 — Go-to-Pose Controller','Color','w');

% (1) XY Path with start and goal pose indicators
subplot(2,2,[1 3]);
plot(X(1,:), X(2,:), 'LineWidth', 1.8, 'DisplayName','Path'); grid on; axis equal; hold on;
plot(X(1,1), X(2,1), 'ko', 'MarkerFaceColor','g', 'DisplayName','Start');
plot(X(1,end), X(2,end), 'ks', 'MarkerFaceColor','r', 'DisplayName','End');
% Draw goal pose arrow
quiver(xg(1), xg(2), 0.4*cos(xg(3)), 0.4*sin(xg(3)), 0, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 2, 'DisplayName','Goal heading', 'Color','c');
plot(xg(1), xg(2), 'kp', 'MarkerSize',10,'MarkerFaceColor','y', 'DisplayName','Goal pos');
xlabel('x [m]'); ylabel('y [m]'); title('Go-to-Pose: XY path');
legend('Location','best');

% (2) Polar errors vs time
subplot(2,2,2);
plot(T, RHO, 'LineWidth', 1.6, 'DisplayName','\rho(t) [m]'); hold on;
plot(T, ALPHA, 'LineWidth', 1.6, 'DisplayName','\alpha(t) [rad]');
plot(T, BETA, 'LineWidth', 1.6, 'DisplayName','\beta(t) [rad]');
yline(rho_tol, ':', 'Color',[.3 .3 .3], 'DisplayName','\rho tol');
yline( th_tol, ':', 'Color',[.6 .6 .6], 'DisplayName','\theta tol');
grid on; xlabel('t [s]'); ylabel('Error'); title('Polar-form errors');
legend('Location','best');

% (3) Commands vs time
subplot(2,2,4);
plot(T, Vcmd, 'LineWidth', 1.6, 'DisplayName','v(t) [m/s]'); hold on;
plot(T, Wcmd, 'LineWidth', 1.6, 'DisplayName','\omega(t) [rad/s]');
grid on; xlabel('t [s]'); ylabel('Command'); title('Commanded v(t), \omega(t)');
legend('Location','best');
ylim([-2, 2])
%% (Optional) Save figure
% saveas(gcf, 'husky_task6_gotopose.png');
