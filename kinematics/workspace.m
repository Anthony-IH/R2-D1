% Communicaiton between Arduino and MATLAB
%   @author         Alejandro Granados
%   @organisation   King's College London
%   @module         Medical Robotics Hardware Development
%   @year           2024

close all
clear all

% declare global variables
%   hPlot       plot widget
%   hFig        figure widget
%   c           counter
%   x           x coordinate of points
%   y           y coordinate of points
global hPlot hFig c x y 

% Create GUI
hFig = figure;

% Create plot area
hPlot = axes('Position', [0.2, 0.35, 0.6, 0.6]);

% Set up and initialise variables for real-time plotting
c = 0;
x = [];
y = [];

% TODO initialise geometry of 2-arm robotic system
pen_r = 5; %radius of the pen
r1 = 120;
r2 = 87 + pen_r;

% TODO Specify the angle resolution you want the workspace to be plotted at
resolution = 20;    % [1..50]
angle1_range = linspace(0, 180, resolution);  % [0..180] degrees given the resolution
angle2_range = linspace(0, 360, resolution);  % [0..360] degrees given the resolution

% TODO Iterate through the given resolution of both angles
for t1 = 1:resolution
    for t2 = 1:resolution
        % increase counter to save information 
        c = c+1;

        % TODO compute the homogeneous transformation via
        % forward_kinematics() function given the geometry and angles
        T = forward_kinematics(r1, r2, angle1_range(t1), angle2_range(t2));
        
        % TODO retrieve end effector position and save for plotting
        x(c) = T(1,4);
        y(c) = T(2,4);
    end
end

% real-time plotting
colour = linspace(1,10,length(x));
scatter(hPlot, x, y, 20, colour, 'filled');
xlim([-(r1+r2),r1+r2]);
ylim([-(r1+r2),r1+r2]);

hold on
x_square=[-78, 78, 78, -78, -78];
y_square=[0, 0, 156, 156, 0];
plot(x_square, y_square, 'r');
% Close GUI
%delete(hFig);