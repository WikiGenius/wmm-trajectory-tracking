% HUSKY_TASK5_PUREPURSUIT — Task 5: Pure Pursuit Path Following
% -------------------------------------------------------------------------
% Goal
%   Follow a polyline path using the pure-pursuit law for a unicycle:
%       alpha = wrap(atan2(y_t - y, x_t - x) - theta)
%       kappa = 2*sin(alpha)/Ld
%       omega = v * kappa
%   with constant forward speed v (clipped to [0, vMax]).
%
% Features
%   • Robust nearest-point on polyline and signed cross-track error
%   • Look-ahead target chosen by arclength marching (Ld) from nearest point
%   • Dwell stop logic when near the final waypoint
%   • Clean plotting and tracking metrics (RMS/mean/max cross-track)
%
% Requirements
%   • MATLAB R2021a+
%   • husky_utils.m on your MATLAB path
%
% -------------------------------------------------------------------------

clear; clc; close all;

%% USER SETTINGS -----------------------------------------------------------
% Waypoints (edit these as you like). Example: a smooth 'S' polyline.
Wpts = [ ...
    -3.0   -1.5;
    -2.0   -1.0;
    -1.0   -0.2;
     0.0    0.5;
     1.0    0.8;
     2.0    0.6;
     3.0    0.0;
     4.0   -0.5;
     5.0   -0.8 ];
% If you prefer a rectangle or circle, you can redefine Wpts accordingly.

% Initial pose [x; y; theta]
x0 = [-3.5; -1.8; 0.0];

% Timing
dt = 0.01;              % [s]
Tf = 12.6;              % [s]
T  = 0:dt:Tf; N = numel(T);

% Robot parameters
P = husky_utils.params('r',0.165,'W',0.55,'vMax',1.0,'wMax',1.5);

% Controller parameters
v0       = 0.8;         % [m/s] constant forward speed
Ld       = 0.8;         % [m]   look-ahead distance (try 0.5..2.0)
pos_tol  = 0.05;        % [m]   stop when close to final waypoint
dwell_N  = 50;          % consecutive steps within tol before stop

% (Optional) slow down slightly in sharp bends (based on curvature proxy)
use_speed_slowdown = true;
v_min = 0.3; v_max = min(P.vMax, v0);
curv_gain = 0.6;  % larger -> more slowdown on large alpha

%% SANITY CHECKS -----------------------------------------------------------
assert(size(Wpts,2)==2 && size(Wpts,1)>=2, 'Wpts must be Nx2 with N>=2');
seg = diff(Wpts,1,1);
seg_len = hypot(seg(:,1), seg(:,2));
assert(all(seg_len>0), 'Consecutive waypoints must not be identical.');
cum_s = [0; cumsum(seg_len)];   % cumulative arclength

%% PRE-ALLOCATE ------------------------------------------------------------
X     = zeros(3, N); X(:,1) = x0;
Vcmd  = zeros(1, N);
Wcmd  = zeros(1, N);
Dxt   = zeros(1, N);    % signed cross-track distance
Alpha = zeros(1, N);    % bearing error
progress_idx = zeros(1,N); % nearest segment index for info

%% MAIN LOOP ---------------------------------------------------------------
within_tol = 0;
k_end = N;

% Start search index near the first segment for speed
seg_idx_hint = 1; seg_t_hint = 0.0;

for k = 1:N-1
    x = X(1,k); y = X(2,k); th = X(3,k);

    % ---- Nearest point on path (segment index + projection) -------------
    [seg_i, t_proj, p_proj] = nearestPointOnPolyline([x;y], Wpts, seg_idx_hint);
    progress_idx(k) = seg_i;
    seg_idx_hint = seg_i; seg_t_hint = t_proj; % remember for next step

    % Signed cross-track distance (normal based sign)
    [d_signed, t_hat, n_hat] = crossTrackSigned([x;y], seg_i, t_proj, Wpts);
    Dxt(k) = d_signed;

    % ---- Look-ahead target point by arclength marching ------------------
    target = advanceAlongPolyline(seg_i, t_proj, Ld, Wpts, seg_len);
    xt = target(1); yt = target(2);

    % ---- Pure pursuit ----------------------------------------------------
    alpha = husky_utils.wrapToPi(atan2(yt - y, xt - x) - th);
    Alpha(k) = alpha;
    if use_speed_slowdown
        v = max(v_min, v_max * exp(-curv_gain * abs(alpha))); % simple slowdown
    else
        v = v0;
    end
    v = husky_utils.sat(v, 0, P.vMax);
    w = husky_utils.sat( v * (2*sin(alpha)/Ld), -P.wMax, P.wMax );

    Vcmd(k) = v; Wcmd(k) = w;

    % ---- Integrate -------------------------------------------------------
    X(:,k+1) = husky_utils.rk4step_unicycle(X(:,k), v, w, dt);

    % ---- Stop when we reach the final waypoint (with dwell) -------------
    dx_end = Wpts(end,1) - X(1,k+1);
    dy_end = Wpts(end,2) - X(2,k+1);
    if hypot(dx_end, dy_end) < pos_tol && seg_i == size(Wpts,1)-1
        within_tol = within_tol + 1;
        if within_tol >= dwell_N
            k_end = k+1;
            break;
        end
    else
        within_tol = 0;
    end
end

% Close arrays (repeat last commands) and truncate
Vcmd(k_end) = Vcmd(max(1,k_end-1));
Wcmd(k_end) = Wcmd(max(1,k_end-1));
Dxt(k_end)  = Dxt(max(1,k_end-1));
Alpha(k_end)= Alpha(max(1,k_end-1));

T    = T(1:k_end);
X    = X(:,1:k_end);
Vcmd = Vcmd(1:k_end);
Wcmd = Wcmd(1:k_end);
Dxt  = Dxt(1:k_end);
Alpha= Alpha(1:k_end);
progress_idx = progress_idx(1:k_end);

%% METRICS -----------------------------------------------------------------
d_abs  = abs(Dxt);
d_rms  = sqrt(mean(Dxt.^2));
d_mean = mean(d_abs);
d_max  = max(d_abs);
fprintf('[INFO] Cross-track |d|: mean = %.3f m, RMS = %.3f m, max = %.3f m\n', d_mean, d_rms, d_max);
fprintf('[INFO] Final pose: x=%.3f, y=%.3f, th=%.3f rad\n', X(1,end), X(2,end), X(3,end));

%% PLOTS -------------------------------------------------------------------
figure('Name','Task 5 — Pure Pursuit Path Following','Color','w');

% (1) XY Path + waypoints + trajectory
subplot(2,2,[1 3]);
plot(Wpts(:,1), Wpts(:,2), 'c--o', 'LineWidth', 1.2, 'DisplayName','Path'); hold on; grid on; axis equal;
plot(X(1,:), X(2,:), 'b-', 'LineWidth', 1.7, 'DisplayName','Robot path');
plot(X(1,1), X(2,1), 'co', 'MarkerFaceColor','g', 'DisplayName','Start');
plot(Wpts(end,1), Wpts(end,2), 'kp', 'MarkerSize',10, 'MarkerFaceColor','y', 'DisplayName','Goal');
xlabel('x [m]'); ylabel('y [m]'); title('Pure Pursuit: XY path');
legend('Location','best');

% (2) Errors vs time
subplot(2,2,2);
plot(T, Dxt, 'LineWidth', 1.6, 'DisplayName','cross-track d(t) [m]'); hold on;
plot(T, Alpha, 'LineWidth', 1.6, 'DisplayName','bearing \alpha(t) [rad]');
grid on; xlabel('t [s]'); ylabel('Error'); title('Tracking errors');
legend('Location','best');

% (3) Commands vs time
subplot(2,2,4);
plot(T, Vcmd, 'LineWidth', 1.6, 'DisplayName','v(t) [m/s]'); hold on;
plot(T, Wcmd, 'LineWidth', 1.6, 'DisplayName','\omega(t) [rad/s]');
grid on; xlabel('t [s]'); ylabel('Command'); title('Commanded v(t), \omega(t)');
legend('Location','best');

%% (Optional) Save figure
% saveas(gcf, 'husky_task5_purepursuit.png');

% -------------------------------------------------------------------------
% Helper functions
% -------------------------------------------------------------------------
function [seg_i, t_proj, p_proj] = nearestPointOnPolyline(p, W, seg_hint)
% Find nearest point on polyline W (Nx2). Returns segment index seg_i,
% param t in [0,1] along segment, and projection point p_proj.
    Nw = size(W,1);
    if nargin < 3, seg_hint = 1; end
    % Search a small window around hint (for speed), but fall back to full
    win = 3;
    i0 = max(1, seg_hint - win);
    i1 = min(Nw-1, seg_hint + win);
    best_d2 = inf; seg_i = 1; t_proj = 0; p_proj = W(1,:).';
    for i = i0:i1
        a = W(i,:).'; b = W(i+1,:).';
        [t, proj] = projectPointToSegment(p, a, b);
        d2 = sum((p - proj).^2);
        if d2 < best_d2
            best_d2 = d2; seg_i = i; t_proj = t; p_proj = proj;
        end
    end
    % If the window missed the best, do full search (rare)
    if ~isfinite(best_d2) || best_d2 > 1e6
        best_d2 = inf;
        for i = 1:Nw-1
            a = W(i,:).'; b = W(i+1,:).';
            [t, proj] = projectPointToSegment(p, a, b);
            d2 = sum((p - proj).^2);
            if d2 < best_d2
                best_d2 = d2; seg_i = i; t_proj = t; p_proj = proj;
            end
        end
    end
end

function [t, proj] = projectPointToSegment(p, a, b)
% Orthogonal projection of point p onto segment [a,b]. Returns parameter t
% in [0,1] such that proj = a + t*(b-a).
    v = b - a; vv = dot(v,v);
    if vv < eps
        t = 0; proj = a;
        return;
    end
    t = dot(p - a, v) / vv;
    t = max(0, min(1, t));
    proj = a + t * v;
end

function target = advanceAlongPolyline(seg_i, t_proj, Ld, W, seg_len)
% Advance a look-ahead distance Ld along polyline starting at projection
% (seg_i, t_proj). Returns target point [xt; yt].
    % Remaining distance on current segment
    v = (1 - t_proj) * seg_len(seg_i);
    i = seg_i; t = t_proj;
    rem = Ld;
    if rem <= v + eps
        % Stay on the same segment
        a = W(i,:).'; b = W(i+1,:).';
        target = a + (t + rem/seg_len(i)) * (b - a);
        return;
    end
    % Move to the next segments
    rem = rem - v;
    i = i + 1;
    while i <= size(W,1)-1 && rem > seg_len(i) + eps
        rem = rem - seg_len(i);
        i = i + 1;
    end
    % If we ran past the end, clamp to last waypoint
    if i > size(W,1)-1
        target = W(end,:).';
        return;
    end
    % Interpolate on segment i
    a = W(i,:).'; b = W(i+1,:).';
    target = a + (rem/seg_len(i)) * (b - a);
end

function [d_signed, t_hat, n_hat] = crossTrackSigned(p, seg_i, t_proj, W)
% Signed cross-track distance: + if p is to the left of segment direction.
    a = W(seg_i,:).'; b = W(seg_i+1,:).';
    v = b - a;
    L = norm(v);
    if L < eps
        t_hat = [1;0]; n_hat = [0;1];
        d_signed = 0;
        return;
    end
    t_hat = v / L;
    n_hat = [ -t_hat(2); t_hat(1) ];  % left-hand normal
    proj = a + t_proj * v;
    d_signed = dot(p - proj, n_hat);
end
