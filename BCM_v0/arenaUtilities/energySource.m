% The function to generate a source of energy (sun) in the arena
%
% Written by Ali Raza, (c) 2013
% ali.raza@ymail.com
%
% University of Engineering and Technology
% ===============================================================
function [Z] = energySource(esLocX,esLocY)
A = 1; x0 = esLocX; y0 = esLocY;
sigma_x = 5; sigma_y = 5;
 
theta = 0;
a = cos(theta)^2/2/sigma_x^2 + sin(theta)^2/2/sigma_y^2;
b = -sin(2*theta)/4/sigma_x^2 + sin(2*theta)/4/sigma_y^2 ;
c = sin(theta)^2/2/sigma_x^2 + cos(theta)^2/2/sigma_y^2;

% [X, Y] = meshgrid(x0-15:.1:x0+15, y0-15:.1:y0+15);
[X, Y] = meshgrid(1:1:84, 1:1:60);
Z = A*exp( - (a*(X-x0).^2 + 2*b*(X-x0).*(Y-y0) + c*(Y-y0).^2)) ;
% contour(Z);
contour(X,Y,Z);
% shading interp;view(-36,36);axis equal;drawnow