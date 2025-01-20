%   Manipulability
%   @author         Mirroyal Ismayilov
%   @organisation   King's College London
%   @module         Applied Medical Robotics
%   @year           2024

close all
clear all
clc

% Parameters for robot arm
r1 = 1.20;    % Length of first link
r2 = 0.92;    % Length of second link
t1 = 90.02;   % Initial angle of joint 1
t2 = -0.81;      % Initial angle of joint 2
%divide all length parameters and desired coordaintees by 100

% Desired position
x_d = [0.76; 0.70]; % Desired x and y coordinates of the end-effector

% Control parameters
damping_factor = 0.01;   % Damping factor for DLS
step_limit = 0.1;        % Maximum step size for target clamping
gain = 0.3;              % Gain for Jacobian transpose method
tolerance = 1e-3;        % Position error tolerance
max_iterations = 2000;   % Maximum number of iterations, default 1000, previous 10000
dt = 0.2;               % Time step for plotting manipulability
ellipsoid_scale = 0.25;   % Scaling factor for ellipsoids

% Initialize array to store manipulability values
manipulability_values = zeros(max_iterations, 1);

figure;
hold on;
axis equal;
xlim([-2.5, 2.5]);
ylim([-0.5, 2.5]);
xlabel('X');
ylabel('Y');
title('Robot Arm Motion and Manipulability Ellipsoids');

for i = 1:max_iterations
    % Calculate forward kinematics to get the current end-effector position
   T = forward_kinematics(r1, r2, t1, t2);


    x = T(1, 4);
    y = T(2, 4);
    current_position = [x; y];
    
    % Calculate position error between desired and current end-effector position
    error = x_d - current_position;
    
    % Target clamping: Limit the step size to avoid large jumps
    if norm(error) > step_limit
        error=(error/norm(error))*step_limit;
    end
    
    % Compute Jacobian for the current joint angles
    J = ik_jacobian(r1, r2, t1, t2);
    inv_J= inv(J);
    
    % Calculate manipulability as the square root of the determinant of (J*J')
    manipulability =sqrt(det(J*J'));
    manipulability_values(i) = manipulability;
    
    % Damped Least Squares (DLS) method for stable inverse calculation
    inv_J_damped = J'*inv(J*J'+damping_factor^(2)*eye(size(J, 2)));
    delta_q_dls = inv_J_damped * error;
    delta_q_inv = inv_J*error;

    % Jacobian Transpose method as an alternative to inverse Jacobian
    delta_q_transpose = gain*J'*error;
    
    % Select method: uncomment one of the two lines below
    % delta_q = delta_q_dls;     % Damped Least Squares method
    % delta_q = delta_q_transpose; % Jacobian Transpose method
    delta_q = delta_q_inv;
    
    % Update joint angles using calculated change
    t1 = t1 + delta_q(1);
    t2 = t2 + delta_q(2);
    
    % Plot current robot arm position
    plot([0, r1*cosd(t1), r1*cosd(t1) + r2*cosd(t1 + t2)], ...
         [0, r1*sind(t1), r1*sind(t1) + r2*sind(t1 + t2)], '-o');
    
    % Plot the manipulability ellipsoid at the current configuration
    [U, S, V] = svd(J);  % Singular Value Decomposition of the Jacobian
    theta = linspace(0, 2*pi, 100);
    ellipse_x = ellipsoid_scale * S(1,1) * cos(theta); %.....apply scaling;  
    ellipse_y = ellipsoid_scale * S(2,2) * sin(theta); %.....apply scaling;  
    ellipse = U * [ellipse_x; ellipse_y]; %Rotate ellipse based on current Jacobian ...* [ellipse_x; ellipse_y]; 
    
    % Plot the ellipsoid around the current end-effector position
    plot(ellipse(1, :) + x, ellipse(2, :) + y, 'm');
    
    pause(0.01);  % Pause for visualization
    
    % Check if the end-effector is within the desired tolerance
    if norm(error) < tolerance
        disp('Desired position reached!');
        disp (t1);
        disp (t2);
        break;
    end
end

% Plot manipulability over the trajectory
figure;
plot(manipulability_values(1:i));
xlabel('Iteration');
ylabel('Manipulability');
title('Manipulability across the trajectory');

