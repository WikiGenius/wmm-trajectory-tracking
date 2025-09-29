% File: bicycle_task6_gotopose.m
% -------------------------------------------------------------
% Task 6: Drive a bicycle-model robot to a desired pose (xg, yg, thetag)
% Uses polar errors (rho, alpha, beta) and a linear steering law.
% -------------------------------------------------------------
clc; clear; close all;

%% Vehicle params
L         = 2.5;                 % wheelbase [m]
v_max     = 2.0;                 % speed limit [m/s]
gamma_max = deg2rad(35);         % steering limit [rad]

%% Controller gains  (choose k_rho>0, k_beta<0, k_alpha>k_rho)
k_rho   = 1.0;
k_alpha = 1.6;      % > k_rho
k_beta  = -0.8;     % < 0

pos_tol   = 0.05;                % [m]
yaw_tol   = deg2rad(3);          % [rad]

%% Start pose and goal pose
x = 0;  y = 0;  th = deg2rad(0);                     % start
xg = 5; yg = 5; thg = deg2rad(45);                   % goal

%% Sim setup
dt = 0.01;  T = 20;  N = round(T/dt);
t = (0:N-1)'*dt;

% logs
X=zeros(N,1); Y=zeros(N,1); TH=zeros(N,1);
RHO=zeros(N,1); ALPHA=zeros(N,1); BETA=zeros(N,1);
Vcmd=zeros(N,1); Gcmd=zeros(N,1);
k_end = N;

%% Main loop
for k = 1:N
    % --- relative pose ---
    dx = xg - x;  dy = yg - y;
    rho   = hypot(dx, dy);                     % distance to goal
    alpha = angdiff_wrap( atan2(dy, dx), th ); % angle to goal in vehicle frame
    beta  = angdiff_wrap( thg, th + alpha );   % final heading error
    
    % If goal is "behind", prefer reversing: flip direction for continuity
    s = 1;   % +1 forward, -1 reverse
    if abs(alpha) > pi/2
        s = -1;
        alpha = angdiff_wrap(alpha - pi*sign(alpha), 0);
        beta  = angdiff_wrap(beta  - pi*sign(beta),  0);
    end

    % --- control laws with saturation ---
    v_cmd     = s * k_rho * rho;
    gamma_cmd = k_alpha*alpha + k_beta*beta;

    v     = clamp(v_cmd,     -v_max,  v_max);
    gamma = clamp(gamma_cmd, -gamma_max, gamma_max);

    % --- bicycle kinematics ---
    xdot  = v*cos(th);
    ydot  = v*sin(th);
    thdot = (v/L) * tan(gamma);

    % integrate
    x = x + xdot*dt;  y = y + ydot*dt;  th = th + thdot*dt;

    % log
    X(k)=x; Y(k)=y; TH(k)=th;
    RHO(k)=rho; ALPHA(k)=alpha; BETA(k)=beta;
    Vcmd(k)=v; Gcmd(k)=gamma;

    % stop if both position and heading are met
    if (rho < pos_tol) && (abs(angdiff_wrap(thg, th)) < yaw_tol) && k > 50
        k_end = k; break;
    end
end

% crop logs
X=X(1:k_end); Y=Y(1:k_end); TH=TH(1:k_end); t=t(1:k_end);
RHO=RHO(1:k_end); ALPHA=ALPHA(1:k_end); BETA=BETA(1:k_end);
Vcmd=Vcmd(1:k_end); Gcmd=Gcmd(1:k_end);

%% Plots
figure; hold on; grid on; axis equal;
plot(X, Y, 'b-', 'LineWidth', 2);
plot(xg, yg, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('x [m]'); ylabel('y [m]'); title('Go-to-Pose Trajectory');
legend('path','goal','Location','best');

figure;
subplot(3,1,1); plot(t, RHO, 'LineWidth',1.6); grid on; ylabel('\rho [m]');
title('Polar errors and commands');
subplot(3,1,2); plot(t, rad2deg(ALPHA), 'LineWidth',1.6); grid on; ylabel('\alpha [deg]');
subplot(3,1,3); plot(t, rad2deg(BETA), 'LineWidth',1.6); grid on; ylabel('\beta [deg]'); xlabel('time [s]');

figure;
subplot(2,1,1); plot(t, Vcmd, 'LineWidth',1.6); grid on; ylabel('v [m/s]'); title('Commands');
subplot(2,1,2); plot(t, rad2deg(Gcmd), 'LineWidth',1.6); grid on; ylabel('\gamma [deg]'); xlabel('time [s]');

% report
pe = hypot(xg - X(end), yg - Y(end));
ye = abs(angdiff_wrap(thg, TH(end)));
fprintf('Final pos err = %.3f m, final yaw err = %.2f deg\n', pe, rad2deg(ye));

%% --------- helpers ----------
function d = angdiff_wrap(a, b)
% angle difference wrap to (-pi, pi]
    d = atan2(sin(a-b), cos(a-b));
end
function y = clamp(u, lo, hi), y = min(max(u,lo),hi); end
