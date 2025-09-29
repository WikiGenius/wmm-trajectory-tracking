% HUSKY_TASK3_GOTOPOINT — Task 3: Go-to-Point (x, y) with a unicycle controller
% -------------------------------------------------------------------------
% Goal
%   Drive the robot from an initial pose (x0, y0, theta0) to a target point
%   (xg, yg) using a simple proportional controller:
%       rho   = distance to goal
%       alpha = bearing error (goal bearing minus heading)
%       v     = k_rho * rho      (clipped to [0, vMax])
%       w     = k_alpha * alpha  (clipped to [-wMax, wMax])
%
% Features
%   • Saturation on v and w (forward-only v >= 0).
%   • Dwell-based stop condition to avoid early termination due to noise.
%   • Pre-allocated arrays with SAFE truncation at the end to keep vectors
%     the same length in plots (prevents "vectors must be same length").
%   • Clear plots: XY path with start/goal markers, errors vs time, commands.
%
% Requirements
%   • MATLAB R2021a+
%   • husky_utils.m on your MATLAB path
%
% -------------------------------------------------------------------------

clear; clc; close all;

%% USER SETTINGS -----------------------------------------------------------
x0   = [-2.0; -1.0;  0.3];     % initial pose [x; y; theta] (rad)
xg   = [ 2.0;  1.0];           % goal position [xg; yg]

% Simulation timing
dt   = 0.01;                   % [s] step
Tf   = 40.0;                   % [s] max duration
T    = 0:dt:Tf;                % time array
N    = numel(T);

% Robot parameters
P = husky_utils.params('r',0.165,'W',0.55,'vMax',1.0,'wMax',1.5);

% Controller gains
k_rho   = 1.0;                 % distance gain
k_alpha = 2.0;                 % heading gain

% Stop logic
rho_tol     = 0.03;            % [m] stop when distance < rho_tol
dwell_steps = 50;              % require this many consecutive steps within tol

% Optional: speed shaping (uncomment to use cos(alpha) shaping)
use_cos_shaping = true;        % if true, v = k_rho * rho * cos(alpha), clipped to [0, vMax]

%% PRE-ALLOCATE ------------------------------------------------------------
X     = zeros(3, N);           % states [x; y; theta]
Vcmd  = zeros(1, N);           % v(t)
Wcmd  = zeros(1, N);           % w(t)
RHO   = zeros(1, N);           % distance-to-goal history
ALPHA = zeros(1, N);           % bearing error history

X(:,1) = x0;

%% MAIN LOOP ---------------------------------------------------------------
within_tol_count = 0;
k_end = N;  % will update if we break early

for k = 1:N-1
    % Compute errors to goal
    dx   = xg(1) - X(1,k);
    dy   = xg(2) - X(2,k);
    rho  = hypot(dx, dy);
    alpha = husky_utils.wrapToPi(atan2(dy, dx) - X(3,k));

    % Store errors
    RHO(k)   = rho;
    ALPHA(k) = alpha;

    % Controller (forward-only v)
    if use_cos_shaping
        % Shaping reduces v when the goal is far behind the robot
        v = k_rho * rho * cos(alpha);
        v = max(0, v);  % enforce forward-only
    else
        v = k_rho * rho;
    end
    w = k_alpha * alpha;

    % Saturation
    v = husky_utils.sat(v, 0, P.vMax);               % forward-only
    w = husky_utils.sat(w, -P.wMax, P.wMax);

    % Save commands
    Vcmd(k) = v;  Wcmd(k) = w;

    % Integrate one step
    X(:,k+1) = husky_utils.rk4step_unicycle(X(:,k), v, w, dt);

    % Stop condition with dwell
    if rho < rho_tol
        within_tol_count = within_tol_count + 1;
        if within_tol_count >= dwell_steps
            k_end = k + 1;   % last valid state index
            break;
        end
    else
        within_tol_count = 0;
    end
end

% Fill last sample for commands/errors to keep lengths consistent
if k_end < N
    final_idx = k_end;
else
    final_idx = N;
end
% Compute final errors at final_idx
dx_f   = xg(1) - X(1,final_idx);
dy_f   = xg(2) - X(2,final_idx);
RHO(final_idx)   = hypot(dx_f, dy_f);
ALPHA(final_idx) = husky_utils.wrapToPi(atan2(dy_f, dx_f) - X(3,final_idx));

% Repeat last command for final sample
Vcmd(final_idx) = Vcmd(max(1, final_idx-1));
Wcmd(final_idx) = Wcmd(max(1, final_idx-1));

% Truncate arrays to consistent lengths [1:final_idx]
T     = T(1:final_idx);
X     = X(:,1:final_idx);
Vcmd  = Vcmd(1:final_idx);
Wcmd  = Wcmd(1:final_idx);
RHO   = RHO(1:final_idx);
ALPHA = ALPHA(1:final_idx);

%% REPORT ------------------------------------------------------------------
fprintf('[INFO] Final pose:   x=%.3f, y=%.3f, th=%.3f rad\n', X(1,end), X(2,end), X(3,end));
fprintf('[INFO] Final errors: rho=%.4f m, alpha=%.4f rad\n', RHO(end), ALPHA(end));

%% PLOTS -------------------------------------------------------------------
figure('Name','Task 3 — Go-to-Point Controller','Color','w');

% (1) XY Path
subplot(2,2,[1 3]);
plot(X(1,:), X(2,:), 'LineWidth', 1.8, 'DisplayName','Path'); grid on; axis equal;
hold on;
plot(X(1,1), X(2,1), 'ko', 'MarkerFaceColor','g', 'DisplayName','Start');
plot(xg(1), xg(2), 'kp', 'MarkerSize',10,'MarkerFaceColor','y','DisplayName','Goal');
plot(X(1,end), X(2,end), 'ks', 'MarkerFaceColor','r', 'DisplayName','End');
xlabel('x [m]'); ylabel('y [m]'); title('Go-to-Point: XY path');
legend('Location','best');

% (2) Errors vs time
subplot(2,2,2);
plot(T, RHO, 'LineWidth', 1.6, 'DisplayName','\rho(t) [m]'); hold on;
plot(T, ALPHA, 'LineWidth', 1.6, 'DisplayName','\alpha(t) [rad]');
yline(rho_tol, ':', 'Color','r', 'DisplayName','\rho tol');
grid on; xlabel('t [s]'); ylabel('Error'); title('Errors to goal');
legend('Location','best');

% (3) Commands vs time
subplot(2,2,4);
plot(T, Vcmd, 'LineWidth', 1.6, 'DisplayName','v(t) [m/s]'); hold on;
plot(T, Wcmd, 'LineWidth', 1.6, 'DisplayName','\omega(t) [rad/s]');
grid on; xlabel('t [s]'); ylabel('Command'); title('Commanded v(t), \omega(t)');
legend('Location','best');

%% (Optional) Save figure
% saveas(gcf, 'husky_task3_gotopoint.png');
