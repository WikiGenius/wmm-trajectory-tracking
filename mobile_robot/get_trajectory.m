function [vR, vL] = get_trajectory(trajectory_case)
    if trajectory_case == 1
        % Straight line
        vR = @(t) 1.0 .*(t>=0);
        vL = @(t) 1.0 .*(t>=0);
    elseif trajectory_case == 2
        % Circle
        vR = @(t) 1.0 .*(t>=0);
        vL = @(t) 0.5 .*(t>=0); 
    elseif trajectory_case == 3
        % Square path
        vR = @(t) 1.0 .*(t>=0);
        vL = @(t) 1.0 .*(t>=0 & t<5) -1.0 .*(t>=5 & t<7) + 1.0 .*(t>=7 & t<12) - 1.0 .*(t>=12); 
    elseif trajectory_case == 4
        % Sine-wave trajectory
        vR = @(t) 1 + 0.5*sin(0.5*t);
        vL = @(t) 1 - 0.5*sin(0.5*t); 
    elseif trajectory_case == 5
        % Figure-8 trajectory
        vR = @(t) 1 + 0.5*sin(0.5*t);
        vL = @(t) 1 + 0.5*sin(0.5*t + pi);
    elseif trajectory_case == 6
        vR = @(t) 1.0 .*(t>=0 & t<5) + 0.0 .*(t>=5 & t<15);
        vL = @(t) 0.0 .*(t>=0 & t<5) + 1.0 .*(t>=5 & t<15); 
    end

end