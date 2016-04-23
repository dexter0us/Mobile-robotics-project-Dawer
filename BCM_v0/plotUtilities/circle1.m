% Function to plot circle
% 
% Written by Ali Raza, (c) 2013
% ali.raza@ymail.com
%
% University of Engineering and Technology
% ========================================================================

function circle1(center,radius,points,style)

THETA = linspace(0,2*pi,points);
RHO = ones(1,points)*radius;

[X,Y] = pol2cart(THETA,RHO);

X = X+center(1);
Y = Y+center(2);

fill(X,Y,style);
%plot(X,Y,style)