% Compute Jacobian
%   @author         Mirroyal Ismayilov
%   @organisation   King's College London
%   @module         Applied Medical Robotics
%   @year           2024
function J = ik_jacobian(r1,r2,t1,t2)
N = 2;  % Number of joints
J = zeros(2, N);  % Initialize the Jacobian matrix

J = [-r1*sind(t1)-r2*sind(t1+t2), -r2*sind(t1+t2);
    r1*cosd(t1)+r2*cosd(t1+t2), r2*cosd(t1+t2)];
end
