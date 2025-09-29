% File: bicycle_task5_pure_pursuit.m
% -------------------------------------------------------------
% Task 5: Pure Pursuit path following (bicycle model)
% Teaches: look-ahead geometry, curvature -> steering mapping.
% -------------------------------------------------------------
clc; clear; close all;

%% Vehicle params
L         = 2.5;               % wheelbase [m]
v_ref     = 1.5;               % constant forward speed [m/s]
gamma_max = deg2rad(35);       % steering saturation [rad]
tol_goal  = 0.10;              % stop when within 10 cm of final waypoint

%% Pure Pursuit param
Ld = 2.0;                      % look-ahead distance [m] (try 1.0, 3.0)

%% Path (polyline of waypoints) - smooth-ish "road"
W = [ 0  0;
      5  0;
     10  2;
     15  5;
     20  5;
     25  3;
     30  0];                   % you can edit or load from file

% Precompute segment lengths and cumulative arc-length
seg = W(2:end,:) - W(1:end-1,:);
seg_len = sqrt(sum(seg.^2,2));
cumlen  = [0; cumsum(seg_len)];
path_len = cumlen(end);

%% Initial pose (off the path)
x = -2;  y = -1;  th = deg2rad(15);

%% Sim setup
dt = 0.01;  T = 60;  N = round(T/dt);
t = (0:N-1)'*dt;

% Logs
X = zeros(N,1); Y = zeros(N,1); TH = zeros(N,1);
G = zeros(N,1); ALPHA = zeros(N,1); S_cl = zeros(N,1);
k_end = N;

%% Main loop
for k = 1:N
    % 1) nearest point on path -> arc-length s_cl
    [p_cl, s_cl] = closestPointOnPath([x y], W, seg_len, cumlen);
    S_cl(k) = s_cl;

    % 2) target arc-length Ld ahead (clamped to end)
    s_tgt = min(s_cl + Ld, path_len);
    p_tgt = pointOnPathAtS(s_tgt, W, seg_len, cumlen);

    % 3) heading to target & pure-pursuit curvature
    alpha = angdiff_wrap( atan2(p_tgt(2)-y, p_tgt(1)-x), th ); % in (-pi,pi]
    kappa = 2*sin(alpha) / max(Ld,1e-6);                      % curvature
    gamma_cmd = atan(L * kappa);                               % steering from curvature
    gamma = clamp(gamma_cmd, -gamma_max, gamma_max);

    % 4) kinematics (constant v)
    xdot = v_ref*cos(th);
    ydot = v_ref*sin(th);
    thdot = (v_ref/L)*tan(gamma);

    % 5) integrate
    x = x + xdot*dt;  y = y + ydot*dt;  th = th + thdot*dt;

    % 6) log
    X(k)=x; Y(k)=y; TH(k)=th; G(k)=gamma; ALPHA(k)=alpha;

    % 7) goal condition (close to final waypoint & we've passed most of path)
    if norm([x y]-W(end,:)) < tol_goal && s_cl > 0.95*path_len
        k_end = k; break;
    end
end

% crop logs
X=X(1:k_end); Y=Y(1:k_end); TH=TH(1:k_end); G=G(1:k_end); ALPHA=ALPHA(1:k_end); t=t(1:k_end);

%% Plots
figure; hold on; grid on; axis equal;
plot(W(:,1), W(:,2), 'c--', 'LineWidth',1.5);          % path
plot(X, Y, 'b-', 'LineWidth',2);                       % vehicle path
plot(W(1,1), W(1,2), 'go', 'MarkerSize',8,'LineWidth',1.5);
plot(W(end,1), W(end,2), 'rx', 'MarkerSize',10,'LineWidth',2);
xlabel('x [m]'); ylabel('y [m]'); title('Pure Pursuit Path Following');
legend('target path','robot path','start','goal','Location','best');

figure;
subplot(2,1,1); plot(t, rad2deg(ALPHA), 'LineWidth',1.6); grid on;
ylabel('\alpha [deg]'); title('Look-ahead bearing \alpha');
subplot(2,1,2); plot(t, rad2deg(G), 'LineWidth',1.6); grid on;
ylabel('\gamma [deg]'); xlabel('time [s]'); title('Steering command');

% Print a small report
fprintf('Stopped at (x,y) = (%.2f, %.2f) m\n', X(end), Y(end));
fprintf('Distance to final waypoint = %.3f m\n', norm([X(end) Y(end)]-W(end,:)));

%% ---------- helpers ----------
function y = clamp(u, lo, hi), y = min(max(u,lo),hi); end
function d = angdiff_wrap(a,b), d = atan2(sin(a-b), cos(a-b)); end

function [p_cl, s_cl] = closestPointOnPath(p, W, seg_len, cumlen)
% Return closest point p_cl on polyline W and its arc length s_cl.
    nseg = size(W,1)-1;
    best_d2 = inf; p_cl = W(1,:); s_cl = 0;
    for i = 1:nseg
        a = W(i,:); b = W(i+1,:);
        ab = b - a; L2 = dot(ab,ab);
        if L2 < eps
            proj = 0; q = a;
        else
            proj = max(0, min(1, dot(p-a,ab)/L2)); % clamp to segment
            q = a + proj*ab;
        end
        d2 = sum((p - q).^2);
        if d2 < best_d2
            best_d2 = d2;
            p_cl = q;
            s_cl = cumlen(i) + proj*seg_len(i);
        end
    end
end

function q = pointOnPathAtS(s, W, seg_len, cumlen)
% Return point q at arc-length s along polyline W (clamped to end).
    s = max(0, min(s, cumlen(end)));
    % find segment where cumlen(i) <= s <= cumlen(i+1)
    idx = find(cumlen <= s, 1, 'last');
    if idx == numel(cumlen)
        q = W(end,:); return; % at the very end
    end
    ds = s - cumlen(idx);
    if seg_len(idx) < eps
        q = W(idx,:); return;
    end
    dir = (W(idx+1,:) - W(idx,:)) / seg_len(idx);
    q = W(idx,:) + ds * dir;
end
