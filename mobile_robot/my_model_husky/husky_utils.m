classdef husky_utils
%HUSKY_UTILS  Utilities for Husky (differential-drive / unicycle) modeling.
%
%   Quick start:
%       P = husky_utils.params('r',0.165,'W',0.55,'vMax',1.0,'wMax',1.5);
%       x = [0;0;0];  dt = 0.01;  v = 0.5;  w = 0.2;
%       for k = 1:1000
%           x = husky_utils.rk4step_unicycle(x, v, w, dt);
%       end
%
%   Key helpers you'll use in later tasks:
%       wrapToPi, angdiff, sat
%       uni2wheels, wheels2uni, wheelAngularSpeeds
%       rk4step_unicycle, eulerstep_unicycle
%       lateralVelocityWorld (for the nonholonomic check in Task 2)

    methods(Static)
        %% ----- Parameters ------------------------------------------------
        function P = params(varargin)
        %PARAMS  Make a parameter struct (override via name-value pairs).
        % Fields:
        %   r    - wheel radius [m]
        %   W    - track width (wheel center-to-center) [m]
        %   vMax - linear speed limit [m/s]
        %   wMax - angular speed limit [rad/s]
            ip = inputParser;
            addParameter(ip, 'r',    0.165);   % Example value; set to your robot
            addParameter(ip, 'W',    0.55);    % Example value; set to your robot
            addParameter(ip, 'vMax', 1.0);
            addParameter(ip, 'wMax', 1.5);
            parse(ip, varargin{:});
            P = ip.Results;
        end

        %% ----- Angles & clipping ----------------------------------------
        function a = wrapToPi(a)
        %WRAPTOPI  Wrap angle(s) to (-pi, pi].
            a = mod(a + pi, 2*pi) - pi;
        end

        function d = angdiff(a, b)
        %ANGDIFF  Smallest signed angle a - b in (-pi, pi].
            d = husky_utils.wrapToPi(a - b);
        end

        function y = sat(x, xmin, xmax)
        %SAT  Saturate x element-wise to [xmin, xmax].
            y = min(max(x, xmin), xmax);
        end

        %% ----- Unicycle <-> Wheel conversions ---------------------------
        function [vR, vL] = uni2wheels(v, w, W)
        %UNI2WHEELS  From (v, w) to right/left wheel linear speeds.
            vR = v + 0.5*W*w;
            vL = v - 0.5*W*w;
        end

        function [v, w] = wheels2uni(vR, vL, W)
        %WHEELS2UNI  From wheel linear speeds to (v, w).
            v = 0.5*(vR + vL);
            w = (vR - vL)/W;
        end

        function [wR, wL] = wheelAngularSpeeds(vR, vL, r)
        %WHELANGULARSPEEDS  Wheel angular speeds [rad/s] from rim speeds.
            wR = vR / r;
            wL = vL / r;
        end

        %% ----- Simple rate limit / clipping bundle (optional) -----------
        function [vR, vL] = clipWheelsByVW(v, w, P)
        %CLIPWHEELSBYVW  Clip (v,w) by P.vMax/P.wMax, then map to wheels.
            v  = husky_utils.sat(v, -P.vMax, P.vMax);
            w  = husky_utils.sat(w, -P.wMax, P.wMax);
            [vR, vL] = husky_utils.uni2wheels(v, w, P.W);
        end

        %% ----- Kinematics integrators -----------------------------------
        function xnext = eulerstep_unicycle(x, v, w, dt)
        %EULERSTEP_UNICYCLE  One Euler step for x=[x;y;theta].
            xnext = x + dt * [v*cos(x(3)); v*sin(x(3)); w];
            xnext(3) = husky_utils.wrapToPi(xnext(3));
        end

        function xnext = rk4step_unicycle(x, v, w, dt)
        %RK4STEP_UNICYCLE  One RK4 step for x=[x;y;theta].
            f = @(x_) [v*cos(x_(3)); v*sin(x_(3)); w];
            k1 = f(x);
            k2 = f(x + 0.5*dt*k1);
            k3 = f(x + 0.5*dt*k2);
            k4 = f(x + dt*k3);
            xnext = x + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
            xnext(3) = husky_utils.wrapToPi(xnext(3));
        end

        %% ----- Lateral-velocity residual (for Task 2) -------------------
        function v_lat = lateralVelocityWorld(xdot, ydot, theta)
        %LATERALVELOCITYWORLD  ydot*cos(theta) - xdot*sin(theta).
            v_lat = ydot.*cos(theta) - xdot.*sin(theta);
        end
    end
end
