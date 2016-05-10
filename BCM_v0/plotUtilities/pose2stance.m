% Function to structure robot pose in arena
% 
% Written by Ali Raza, (c) 2013
% ali.raza@ymail.com
%
% University of Engineering and Technology

% input variables:
% x= x-coordinate, y= y-coordinate
% theta = the pose angle.
% SF = scaling factor.

% Output Variables:
% P = a matrix of 12x2
% ========================================================================


function P = pose2stance(x,y,theta,SF)

R = [cos(theta) -sin(theta);
    sin(theta) cos(theta)];

P(1,:) = [0.6 -0.7];
P(2,:) = [0.6 1];
P(3,:) = [-0.6 0.7];
P(4,:) = [-0.6 -0.7];
P(5,:) = [0.25 0.7];
P(6,:) = [0.25 0.8];
P(7,:) = [-0.25 0.8];
P(8,:) = [-0.25 0.7];
P(9,:) = [0.25 -0.8];
P(10,:) = [0.25 -0.7];
P(11,:) = [-0.25 -0.7];
P(12,:) = [-0.25 -0.8];
P = P*SF;

% here it is now calculating the rotation matrix of the robot.
% the returned matrix is of 12 x 2 size.
for I=1:12
    
    P(I,:) = (R*(P(I,:))')';
    P(I,:) = P(I,:) + [x y];
    
end