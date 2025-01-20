% Communicaiton between Arduino and MATLAB
%   @author         Alejandro Granados
%   @organisation   King's College London
%   @module         Medical Robotics Hardware Development
%   @year           2024

function [T] = forward_kinematics(r1, r2, t1, t2)


% TODO replace with the values of an homogenous transformation matrix
A01=[cosd(t1), -sind(t1), 0, r1*cosd(t1);
   sind(t1), cosd(t1), 0, r1*sind(t1);
   0 0 1 0; 
   0 0 0 1];

A12=[cosd(t2), -sind(t2), 0, r2*cosd(t2);
   sind(t2), cosd(t2), 0, r2*sind(t2);
   0 0 1 0; 
   0 0 0 1];

T=A01*A12;


t1=87;
t2=152;
r1=120;
r2=92;

x = T(1,4)
y = T(2,4)
end

